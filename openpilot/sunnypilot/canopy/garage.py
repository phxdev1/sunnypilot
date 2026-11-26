"""Canopy garage parking: know when he is pulling into the garage at home, show it, log it.

Shadow only. Nothing here steers, brakes or presses a button. It puts "Garage parking in
progress" on the comma screen (canopy/banner.py) so he can see when Canopy thinks it is
parking, and logs each attempt for the parking planner shadow (Drex plan vs what he did).

Starts when all hold for START_HOLD_S:
  in Drive, under MAX_MPH, within ZONE_M of the garage spot, nose within HEADING_TOL_DEG
  of the garage heading (GPS bearing, only trusted above MIN_BEARING_MPH; slower, the last
  trusted bearing stands).
Ends on Park (parked), or when he drives off: out past EXIT_M, over EXIT_MPH, or Reverse
for REVERSE_S (backing out, not parking).

The garage spot and heading are not in the repo: /data/canopy/garage.json
  {"lat": .., "lon": .., "heading_deg": .., "zone_m": 40}
No file, no banner.
"""
import json
import math
import time

from openpilot.sunnypilot.canopy import banner

GARAGE_PATH = "/data/canopy/garage.json"
SOURCE = "garage"
TEXT = "Garage parking in progress"
MS_TO_MPH = 2.23694

ZONE_M = 40.0
EXIT_M = 70.0
MAX_MPH = 8.0
EXIT_MPH = 15.0
MIN_BEARING_MPH = 1.5
HEADING_TOL_DEG = 50.0
START_HOLD_S = 1.0
REVERSE_S = 2.0
REFRESH_S = 20.0
BANNER_TIMEOUT_S = 45.0
CONFIG_RETRY_S = 30.0


def dist_m(lat1, lon1, lat2, lon2) -> float:
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp, dl = p2 - p1, math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * 6371000.0 * math.asin(math.sqrt(a))


def angle_diff(a: float, b: float) -> float:
  return abs((a - b + 180.0) % 360.0 - 180.0)


def _gear(car_state) -> str:
  try:
    return str(car_state.gearShifter)
  except Exception:
    return "unknown"


class GaragePark:
  def __init__(self, path: str = GARAGE_PATH, post=banner.post, clear=banner.clear):
    self.path = path
    self._post, self._clear = post, clear
    self.cfg = None
    self._cfg_t = -1e9
    self.active = False
    self.started_at = None
    self._cand_since = None
    self._rev_since = None
    self._bearing = None
    self._last_post = -1e9
    self.min_dist = None
    self.last_event = None  # ("start" | "end", dict) for the caller to log

  def _load(self, now):
    if self.cfg is not None or now - self._cfg_t < CONFIG_RETRY_S:
      return
    self._cfg_t = now
    try:
      with open(self.path) as f:
        c = json.load(f)
      self.cfg = {"lat": float(c["lat"]), "lon": float(c["lon"]), "heading": float(c["heading_deg"]) % 360.0,
                  "zone": float(c.get("zone_m", ZONE_M))}
    except (OSError, ValueError, KeyError, TypeError):
      self.cfg = None

  def _end(self, reason, now, wall):
    self.active = False
    self._cand_since = self._rev_since = None
    self._clear(SOURCE)
    self.last_event = ("end", {"reason": reason, "secs": round(now - (self.started_at or now), 1),
                               "min_dist_m": None if self.min_dist is None else round(self.min_dist, 1), "wall": round(wall, 1)})

  def update(self, gps, car_state, v_ego: float, now: float | None = None, wall: float | None = None) -> bool:
    """Returns True on the frame a start or an end happened (see last_event)."""
    now = time.monotonic() if now is None else now
    wall = time.time() if wall is None else wall
    self.last_event = None
    self._load(now)
    if self.cfg is None or gps is None or not getattr(gps, "hasFix", True):
      return False

    mph = v_ego * MS_TO_MPH
    gear = _gear(car_state)
    d = dist_m(gps.latitude, gps.longitude, self.cfg["lat"], self.cfg["lon"])
    if mph >= MIN_BEARING_MPH:
      self._bearing = gps.bearingDeg
    nose_ok = self._bearing is not None and angle_diff(self._bearing, self.cfg["heading"]) <= HEADING_TOL_DEG

    if not self.active:
      if d > self.cfg["zone"] * 1.5:
        self._bearing = None  # a bearing from out on the street says nothing about the driveway
      cand = gear == "drive" and mph < MAX_MPH and d <= self.cfg["zone"] and nose_ok
      if not cand:
        self._cand_since = None
        return False
      if self._cand_since is None:
        self._cand_since = now
      if now - self._cand_since < START_HOLD_S:
        return False
      self.active, self.started_at, self.min_dist = True, now, d
      self._rev_since = None
      self._post(SOURCE, TEXT, "info", BANNER_TIMEOUT_S)
      self._last_post = now
      self.last_event = ("start", {"dist_m": round(d, 1), "mph": round(mph, 1), "wall": round(wall, 1)})
      return True

    self.min_dist = min(self.min_dist, d)
    if gear == "park":
      self._end("parked", now, wall)
      return True
    if d > EXIT_M or mph > EXIT_MPH:
      self._end("drove_off", now, wall)
      return True
    if gear == "reverse":
      self._rev_since = self._rev_since or now
      if now - self._rev_since >= REVERSE_S:
        self._end("backed_out", now, wall)
        return True
    else:
      self._rev_since = None
    if now - self._last_post >= REFRESH_S:
      self._post(SOURCE, TEXT, "info", BANNER_TIMEOUT_S)
      self._last_post = now
    return False
