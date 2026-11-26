"""Canopy trouble spots: an early heads-up before places where Mark keeps taking the wheel back.

Forge builds the spots from his takeovers (graph/build.py, trouble_spot lessons): 2+ separate passes
with a takeover at the same place and heading, not parking, tight turns, lane changes, crawling or the
first and last 300 m of a drive. A spot fades after 3 clean passes. Forge pushes the live ones to
/data/canopy/trouble_spots.json after each drive:
  {"version": 1, "spots": [{"id": .., "lat": .., "lon": .., "heading": .., "count": .., "label": ..}]}
No file, no spots, nothing happens.

Live: one chime and Zia "Heads up, tricky spot." (slot trouble_spot), plus a "Tricky spot ahead"
banner, once per spot per REARM_S, only while engaged and above MIN_MPH, when the spot is ahead on
the same heading within max(MIN_LEAD_M, v * LEAD_S).

Shadow only: the set-speed trim. It works out what it would target going in (TRIM_MPH under the set
speed or the map limit, whichever is lower; it only ever lowers, never sets above the limit, and the
brake wins) and logs one row per approach next to what he actually did, to
/data/canopy/trouble_shadow.jsonl. Nothing here presses a button or changes a target.

Path mode: the comma's own Look Ahead view (/dev/shm/canopy_lookahead.json, written once a second by
canopy_hudd) says where his road actually goes. When a spot sits within LA_MATCH_M of a point on that
path and that point is between MIN_AHEAD_M and reach along it, the heads-up may fire even around a
curve or a turn, where the straight-ahead bearing check fails. This only goes live when the gate
("look_ahead_comma", scoped by the path source) says so; otherwise each approach logs one shadow row
to /data/canopy/lookahead_cue_shadow.jsonl and nothing is posted.
"""
import json
import math
import os
import time

from openpilot.sunnypilot.canopy import banner

SPOTS_PATH = "/data/canopy/trouble_spots.json"
GARAGE_PATH = "/data/canopy/garage.json"
SHADOW_PATH = "/data/canopy/trouble_shadow.jsonl"
SOURCE = "trouble"
TEXT = "Tricky spot ahead"
MS_TO_MPH = 2.23694
MPH_TO_MS = 1 / MS_TO_MPH

MIN_MPH = 15.0
LEAD_S = 9.0            # heads-up this many seconds out
MIN_LEAD_M = 120.0
MIN_AHEAD_M = 25.0      # closer than this it is too late to be useful
BEARING_TOL_DEG = 30.0  # spot must be roughly straight ahead
HEADING_TOL_DEG = 40.0  # and we must be going the way he was when he took over
REARM_S = 300.0
ZONE_PAST_M = 50.0      # the approach ends this far past the spot
ZONE_TIMEOUT_S = 60.0
TRIM_MPH = 3.0
GARAGE_SKIP_M = 150.0
RELOAD_S = 30.0
BANNER_S = 8.0

LOOKAHEAD_PATH = "/dev/shm/canopy_lookahead.json"
LOOKAHEAD_OFF = "/data/canopy/lookahead_off"
LA_SHADOW_PATH = "/data/canopy/lookahead_cue_shadow.jsonl"
LA_FEATURE = "look_ahead_comma"
LA_MAX_AGE_S = 3.0
LA_MATCH_M = 40.0
LA_RECHECK_S = 0.5      # hudd rewrites the view once a second; stat it at most this often


def dist_m(lat1, lon1, lat2, lon2) -> float:
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp, dl = p2 - p1, math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * 6371000.0 * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2) -> float:
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dl = math.radians(lon2 - lon1)
  y = math.sin(dl) * math.cos(p2)
  x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
  return math.degrees(math.atan2(y, x)) % 360.0


def angle_diff(a: float, b: float) -> float:
  return abs((a - b + 180.0) % 360.0 - 180.0)


def would_target(v_cruise: float, limit: float | None) -> float:
  """The trim it would set going in, m/s. Only lowers; never above the map limit."""
  cap = v_cruise if limit is None or limit <= 0 else min(v_cruise, limit)
  return max(0.0, min(v_cruise, cap - TRIM_MPH * MPH_TO_MS))


class TroubleSpots:
  def __init__(self, path: str = SPOTS_PATH, garage_path: str = GARAGE_PATH, shadow_path: str | None = SHADOW_PATH,
               post=banner.post, lookahead_path: str = LOOKAHEAD_PATH, off_flag: str = LOOKAHEAD_OFF,
               la_shadow_path: str | None = LA_SHADOW_PATH, gates=None):
    self.path, self.garage_path, self.shadow_path = path, garage_path, shadow_path
    self._post = post
    self.lookahead_path, self.off_flag, self.la_shadow_path = lookahead_path, off_flag, la_shadow_path
    if gates is None:
      try:
        from openpilot.sunnypilot.canopy.gates import Gates
        gates = Gates()
      except Exception:
        gates = None
    self.gates = gates
    self.spots = []
    self._mtime = None
    self._check_t = -1e9
    self._last_fire = {}
    self.zone = None        # the approach being watched
    self.last_fire = None   # dict for the caller to log
    self.last_row = None    # finished shadow row
    self._la_cond = None    # the gate cell at the last live path fire
    self._la_view = None    # the last parsed Look Ahead view
    self._la_mtime = None   # and the file mtime it came from
    self._la_stat_t = -1e9  # when we last stat'ed the file

  def _load(self, now):
    if now - self._check_t < RELOAD_S:
      return
    self._check_t = now
    try:
      mt = os.stat(self.path).st_mtime
    except OSError:
      self.spots, self._mtime = [], None
      return
    if mt == self._mtime:
      return
    self._mtime = mt
    garage = None
    try:
      with open(self.garage_path) as f:
        g = json.load(f)
      garage = (float(g["lat"]), float(g["lon"]))
    except (OSError, ValueError, KeyError, TypeError):
      pass
    spots = []
    try:
      with open(self.path) as f:
        for s in json.load(f).get("spots", []):
          lat, lon, hdg = float(s["lat"]), float(s["lon"]), float(s["heading"]) % 360.0
          if garage and dist_m(lat, lon, *garage) <= GARAGE_SKIP_M:
            continue
          spots.append({"id": str(s["id"]), "lat": lat, "lon": lon, "heading": hdg, "count": int(s.get("count", 0)),
                        "label": str(s.get("label", ""))})
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
      spots = []
    self.spots = spots

  def _finish(self, reason, wall):
    z, self.zone = self.zone, None
    row = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in z.items() if not k.startswith("_")}
    row.update(end=reason, t_end=round(wall, 1))
    self.last_row = row
    if self.shadow_path:
      try:
        os.makedirs(os.path.dirname(self.shadow_path), exist_ok=True)
        with open(self.shadow_path, "a") as f:
          f.write(json.dumps(row, separators=(",", ":")) + "\n")
      except OSError:
        pass
    if z.get("via") == "path" and z.get("took_over") is True:
      if self.gates is not None and self._la_cond is not None:
        try:
          self.gates.demote(LA_FEATURE, self._la_cond, "takeover")
        except Exception:
          pass
    self._la_cond = None

  def _path_view(self, now, wall):
    """The comma's last Look Ahead view, or None if off, stale or unreadable.

    update() runs at the planner rate and hudd rewrites the file once a second, so stat it at most
    every LA_RECHECK_S and re-parse only when the mtime moved. The off flag and the age check still
    run on every call, against the cached view.
    """
    try:
      if os.path.exists(self.off_flag):
        return None
    except OSError:
      return None
    if now - self._la_stat_t >= LA_RECHECK_S:
      self._la_stat_t = now
      try:
        mt = os.stat(self.lookahead_path).st_mtime
      except OSError:
        self._la_view, self._la_mtime = None, None
        return None
      if mt != self._la_mtime:
        self._la_mtime = mt
        try:
          with open(self.lookahead_path) as f:
            view = json.load(f)
        except (OSError, ValueError, TypeError):
          view = None
        self._la_view = view if isinstance(view, dict) else None
    view = self._la_view
    if not isinstance(view, dict):
      return None
    try:
      ts = float(view["ts"])
    except (KeyError, TypeError, ValueError):
      return None
    if wall - ts > LA_MAX_AGE_S:
      return None
    return view

  def _path_hits(self, view, reach):
    """Spots the path runs past, with how far along the path they sit."""
    for s in self.spots:
      for it in view.get("items") or []:
        if not isinstance(it, dict):
          continue
        la, lo = it.get("lat"), it.get("lon")
        if la is None or lo is None:
          continue
        try:
          if dist_m(s["lat"], s["lon"], float(la), float(lo)) > LA_MATCH_M:
            continue
        except (TypeError, ValueError):
          continue
        try:
          along = float(it.get("along_m"))
        except (TypeError, ValueError):
          break
        if MIN_AHEAD_M <= along <= reach:
          yield s, along
        break

  def _log_path_shadow(self, s, along, view, v_ego, wall):
    if not self.la_shadow_path:
      return
    row = {"t": round(wall, 1), "spot": s["id"], "along_m": round(along), "source": view.get("source"),
           "path_conf": view.get("path_conf"), "mph": round(v_ego * MS_TO_MPH, 1), "live": False}
    try:
      os.makedirs(os.path.dirname(self.la_shadow_path), exist_ok=True)
      with open(self.la_shadow_path, "a") as f:
        f.write(json.dumps(row, separators=(",", ":")) + "\n")
    except OSError:
      pass

  def _watch(self, gps, car_state, v_ego, engaged, lat_active, now, wall):
    z = self.zone
    d = dist_m(gps.latitude, gps.longitude, z["_lat"], z["_lon"])
    ahead = angle_diff(bearing_deg(gps.latitude, gps.longitude, z["_lat"], z["_lon"]), gps.bearingDeg) <= 90.0
    z["min_mph"] = min(z["min_mph"], v_ego * MS_TO_MPH)
    z["min_dist_m"] = min(z["min_dist_m"], d)
    if getattr(car_state, "brakePressed", False):
      z["brake"] = True
    if getattr(car_state, "steeringPressed", False):
      z["wheel"] = True
    if z["engaged_at_fire"] and (not engaged or not lat_active):
      z["took_over"] = True
    if (not ahead and d >= ZONE_PAST_M) or now - z["_t"] > ZONE_TIMEOUT_S:
      self._finish("passed" if not ahead else "timeout", wall)
      return True
    return False

  def _fire(self, s, d, via, v_ego, v_cruise, limit, engaged, lat_active, now, wall):
    """Arm the approach watch, note the fire and post the banner."""
    self._last_fire[s["id"]] = now
    wt = would_target(v_cruise, limit) if v_cruise > 0 else None
    self.zone = {"spot": s["id"], "count": s["count"], "label": s["label"], "via": via,
                 "t_fire": round(wall, 1),
                 "dist_at_fire_m": d, "mph_at_fire": v_ego * MS_TO_MPH, "set_mph": v_cruise * MS_TO_MPH,
                 "limit_mph": (limit * MS_TO_MPH) if limit else None,
                 "would_set_mph": (wt * MS_TO_MPH) if wt is not None else None,
                 "min_mph": v_ego * MS_TO_MPH, "min_dist_m": d, "brake": False, "wheel": False,
                 "took_over": False, "engaged_at_fire": bool(engaged and lat_active),
                 "_lat": s["lat"], "_lon": s["lon"], "_t": now}
    self.last_fire = {"spot": s["id"], "count": s["count"], "label": s["label"], "via": via,
                      "dist_m": round(d), "mph": round(v_ego * MS_TO_MPH, 1)}
    try:
      self._post(SOURCE, TEXT, "heads_up", BANNER_S)
    except Exception:
      pass

  def update(self, gps, car_state, v_ego: float, engaged: bool, lat_active: bool, v_cruise: float,
             limit: float | None, now: float | None = None, wall: float | None = None) -> bool:
    """True on the frame a heads-up fires (see last_fire). last_row is set when an approach ends.
    v_cruise and limit are m/s (limit None when the map has none)."""
    now = time.monotonic() if now is None else now
    wall = time.time() if wall is None else wall
    self.last_fire = None
    self.last_row = None
    self._load(now)
    if gps is None or not getattr(gps, "hasFix", True):
      return False
    if self.zone is not None:
      self._watch(gps, car_state, v_ego, engaged, lat_active, now, wall)
      return False
    if not self.spots or not engaged or v_ego * MS_TO_MPH < MIN_MPH:
      return False

    reach = max(MIN_LEAD_M, v_ego * LEAD_S)
    lat, lon, hdg = gps.latitude, gps.longitude, gps.bearingDeg
    for s in self.spots:
      if abs(s["lat"] - lat) > 0.02:
        continue
      d = dist_m(lat, lon, s["lat"], s["lon"])
      if not (MIN_AHEAD_M <= d <= reach):
        continue
      if angle_diff(hdg, s["heading"]) > HEADING_TOL_DEG:
        continue
      if angle_diff(bearing_deg(lat, lon, s["lat"], s["lon"]), hdg) > BEARING_TOL_DEG:
        continue
      if now - self._last_fire.get(s["id"], -1e9) < REARM_S:
        continue
      self._fire(s, d, "line", v_ego, v_cruise, limit, engaged, lat_active, now, wall)
      return True

    # Nothing straight ahead: let the local Look Ahead path carry the heads-up round a curve or a turn.
    view = self._path_view(now, wall)
    if view is None or self.gates is None:
      return False
    try:
      cond = self.gates.conditions(lat, lon, wall, v_ego * MS_TO_MPH, (limit * MS_TO_MPH) if limit else None)
      cond = dict(cond)
    except Exception:
      return False
    cond["path_source"] = view.get("source")
    try:
      live = bool(self.gates.is_live(LA_FEATURE, cond, float(view.get("path_conf") or 0.0)))
    except Exception:
      live = False
    for s, along in self._path_hits(view, reach):
      if now - self._last_fire.get(s["id"], -1e9) < REARM_S:
        continue
      if live:
        self._la_cond = cond
        self._fire(s, along, "path", v_ego, v_cruise, limit, engaged, lat_active, now, wall)
        return True
      self._log_path_shadow(s, along, view, v_ego, wall)
      self._last_fire[s["id"]] = now
    return False

