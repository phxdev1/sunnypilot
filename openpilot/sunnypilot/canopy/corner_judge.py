"""Canopy corner judge: steer through signaled turns, except at corners where Mark keeps taking over.

By default the car steers through a signaled turn (Lane turn gives the model the turn under 19 mph).
Forge learns hand-back corners from his drives (drives/bin/corners.py): a signaled turn under 20 mph
where he fought the wheel while openpilot was steering, on 2+ separate passes within 50 m, same
approach heading and same turn side. A corner fades after 3 clean passes. Forge pushes the live ones
to /data/canopy/handback_corners.json:
  {"version": 1, "corners": [{"id": .., "lat": .., "lon": .., "heading": .., "side": "L"|"R", "count": ..}]}
No file, no corners, nothing happens.

At a hand-back corner (blinker on that side, under MAX_MPH, steering on, corner just ahead or at it,
same approach heading): one chime and Zia "Your turn." (slot your_turn) and a "Your turn" banner,
every time (he always hears it when steering lets go), then steering pauses for that turn only. The pause is a file on tmpfs that this keeps touching while
the turn lasts; controlsd treats it as live while it is under HOLD_S old (handback_active()). It ends
when the blinker goes off, the car passes TURN_END_MPH, or after MAX_TURN_S. If plannerd stops, the
file goes stale and steering comes back, so a fault never leaves steering off.
"""
import json
import math
import os
import time

from openpilot.sunnypilot.canopy import banner

CORNERS_PATH = "/data/canopy/handback_corners.json"
SHM_DIR = os.environ.get("CANOPY_BANNER_DIR", "/dev/shm/canopy")
HANDBACK_PATH = os.path.join(SHM_DIR, "corner_handback")
GARAGE_PATH = "/data/canopy/garage.json"
SOURCE = "corner"
TEXT = "Your turn"
MS_TO_MPH = 2.23694

MAX_MPH = 20.0          # signaled turns only (lane changes need 40+ here)
TURN_END_MPH = 25.0     # back over this: the turn is over
REACH_M = 60.0          # corner this close ahead
AT_M = 20.0             # this close, no bearing check (stopped at the corner, GPS jitter)
HEADING_TOL_DEG = 45.0  # approaching the way he was when he fought it
BEARING_TOL_DEG = 50.0  # corner roughly ahead
MAX_TURN_S = 45.0       # a long red with the blinker on still ends
REARM_S = 20.0          # every hand back gets the cue ("always cue"); this only stops a double cue on a blinker flicker
HOLD_S = 1.0            # controlsd: steering stays paused this long after the last touch
RELOAD_S = 30.0
GARAGE_SKIP_M = 150.0
BANNER_S = 6.0


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


def handback_active(path: str = HANDBACK_PATH, now: float | None = None) -> bool:
  """For controlsd: True while the corner judge is handing the current turn back."""
  try:
    age = (time.time() if now is None else now) - os.stat(path).st_mtime
  except OSError:
    return False
  return 0.0 <= age < HOLD_S


class CornerJudge:
  def __init__(self, path: str = CORNERS_PATH, handback_path: str = HANDBACK_PATH, garage_path: str = GARAGE_PATH,
               post=banner.post):
    self.path, self.handback_path, self.garage_path = path, handback_path, garage_path
    self._post = post
    self.corners = []
    self._mtime = None
    self._check_t = -1e9
    self._last_cue = {}
    self.turn = None        # {"id", "side", "t"} while handing a turn back
    self.last_fire = None   # dict for the caller to log
    self.last_end = None    # dict when a hand back ends

  def _load(self, now):
    if now - self._check_t < RELOAD_S:
      return
    self._check_t = now
    try:
      mt = os.stat(self.path).st_mtime
    except OSError:
      self.corners, self._mtime = [], None
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
    corners = []
    try:
      with open(self.path) as f:
        for c in json.load(f).get("corners", []):
          side = str(c["side"]).upper()
          if side not in ("L", "R"):
            continue
          lat, lon = float(c["lat"]), float(c["lon"])
          if garage and dist_m(lat, lon, *garage) <= GARAGE_SKIP_M:
            continue
          corners.append({"id": str(c["id"]), "lat": lat, "lon": lon, "heading": float(c["heading"]) % 360.0,
                          "side": side, "count": int(c.get("count", 0))})
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
      corners = []
    self.corners = corners

  def _touch(self):
    try:
      os.makedirs(os.path.dirname(self.handback_path), exist_ok=True)
      with open(self.handback_path, "a"):
        os.utime(self.handback_path, None)
    except OSError:
      pass

  def _end(self, reason, now):
    t, self.turn = self.turn, None
    self.last_end = {"corner": t["id"], "side": t["side"], "secs": round(now - t["t"], 1), "end": reason}
    try:
      os.unlink(self.handback_path)
    except OSError:
      pass

  def update(self, gps, car_state, v_ego: float, lat_active: bool, now: float | None = None) -> bool:
    """True on the frame the "Your turn" cue fires (see last_fire). Keeps the hand back alive while the turn lasts."""
    now = time.monotonic() if now is None else now
    self.last_fire = None
    self.last_end = None
    left, right = bool(getattr(car_state, "leftBlinker", False)), bool(getattr(car_state, "rightBlinker", False))
    side = "L" if left and not right else ("R" if right and not left else None)
    mph = v_ego * MS_TO_MPH

    if self.turn is not None:
      if side != self.turn["side"]:
        self._end("blinker_off", now)
      elif mph > TURN_END_MPH:
        self._end("speed", now)
      elif now - self.turn["t"] > MAX_TURN_S:
        self._end("timeout", now)
      else:
        self._touch()
      return False

    self._load(now)
    if not self.corners or side is None or mph >= MAX_MPH or not lat_active:
      return False
    if gps is None or not getattr(gps, "hasFix", True):
      return False
    lat, lon, hdg = gps.latitude, gps.longitude, gps.bearingDeg
    for c in self.corners:
      if c["side"] != side or abs(c["lat"] - lat) > 0.01:
        continue
      d = dist_m(lat, lon, c["lat"], c["lon"])
      if d > REACH_M:
        continue
      if angle_diff(hdg, c["heading"]) > HEADING_TOL_DEG:
        continue
      if d > AT_M and angle_diff(bearing_deg(lat, lon, c["lat"], c["lon"]), hdg) > BEARING_TOL_DEG:
        continue
      self.turn = {"id": c["id"], "side": side, "t": now}
      self._touch()
      cue = now - self._last_cue.get(c["id"], -1e9) >= REARM_S
      if cue:
        self._last_cue[c["id"]] = now
        try:
          self._post(SOURCE, TEXT, "heads_up", BANNER_S)
        except Exception:
          pass
      self.last_fire = {"corner": c["id"], "side": side, "count": c["count"], "dist_m": round(d),
                        "mph": round(mph, 1), "cue": cue}
      return cue
    return False
