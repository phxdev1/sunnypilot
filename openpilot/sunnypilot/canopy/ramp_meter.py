"""Canopy ramp meter hold.

ProPILOT will roll through a red ramp meter when there is no car ahead to stop for, and merge up
would even ask it to speed toward the freeway. So when a metered on-ramp's light is coming up:
chime once, and hold the set speed (no merge up, no speed increases) until the car is past the light.

Meters come from OpenStreetMap (traffic_signals=ramp_meter), precomputed on Forge into
/data/canopy/ramp_meters.json as [lat, lon, bearing_deg] rows, bearing being the direction traffic
flows through the meter. On-ramp meters sit 10 to 30 m beside the freeway lanes, running the same
way, so position and heading alone would chime on the freeway too. A meter only counts when the
map says we are not on a 60+ mph road, or, with no map limit, when we are under 55 mph.
"""
import json
import math
import os
import time

from openpilot.common.constants import CV

METERS_PATH = "/data/canopy/ramp_meters.json"
AHEAD_M = 300.      # start caring this far before the light
PAST_M = 5.         # the light is behind us once we are this far beyond it
CROSS_M = 12.       # how far off our line the light may sit (GPS error plus lane width)
HEADING_DEG = 30.   # our heading vs the ramp's direction through the light
FREEWAY_MPH = 60.   # a map limit at or over this means we are on the freeway, not the ramp
NO_MAP_MAX_MPH = 55.
MIN_SPEED = 3.      # m/s, GPS heading is noise below this
LOST_HOLD_S = 3.    # keep holding this long if the light drops out of view (GPS jitter)
M_PER_DEG_LAT = 110540.


class RampMeters:
  def __init__(self, path: str = METERS_PATH):
    self.path = path
    self.mtime = None
    self.meters: list[tuple[float, float, float]] = []
    self.refresh()

  def refresh(self) -> None:
    try:
      mtime = os.path.getmtime(self.path)
    except OSError:
      self.meters, self.mtime = [], None
      return
    if mtime == self.mtime:
      return
    try:
      with open(self.path) as f:
        self.meters = [(float(a), float(b), float(c)) for a, b, c in json.load(f)["meters"]]
      self.mtime = mtime
    except (OSError, ValueError, KeyError, TypeError):
      self.meters = []

  def ahead(self, lat: float, lon: float, heading: float):
    """(index, along-track metres) of the nearest meter on our path, or None."""
    best = None
    m_per_deg_lon = 111320. * math.cos(math.radians(lat))
    h = math.radians(heading)
    fx, fy = math.sin(h), math.cos(h)  # east, north
    span = (AHEAD_M + 50.) / M_PER_DEG_LAT
    for i, (mlat, mlon, mbrg) in enumerate(self.meters):
      if abs(mlat - lat) > span or abs(mlon - lon) * m_per_deg_lon > AHEAD_M + 50.:
        continue
      if abs((heading - mbrg + 180.) % 360. - 180.) > HEADING_DEG:
        continue
      dx, dy = (mlon - lon) * m_per_deg_lon, (mlat - lat) * M_PER_DEG_LAT
      along, cross = dx * fx + dy * fy, abs(dx * fy - dy * fx)
      if -PAST_M <= along <= AHEAD_M and cross <= CROSS_M and (best is None or along < best[1]):
        best = (i, along)
    return best


class RampMeterHold:
  def __init__(self, meters: RampMeters | None = None):
    self.meters = meters if meters is not None else RampMeters()
    self.holding = False
    self.chime = False        # true for one update when a new light comes up
    self.meter = None         # index of the light we are holding for
    self.distance = 0.
    self._last_seen = 0.
    self._chimed = None

  def update(self, gps, v_ego: float, map_limit: float) -> bool:
    """gps: a gpsLocation(External) reader or None. map_limit in m/s, 0 when the map has none."""
    self.chime = False
    now = time.monotonic()
    hit = None
    if gps is not None and v_ego > MIN_SPEED and getattr(gps, "hasFix", True):
      on_ramp = map_limit * CV.MS_TO_MPH < FREEWAY_MPH if map_limit > 0. else v_ego * CV.MS_TO_MPH < NO_MAP_MAX_MPH
      if on_ramp or self.holding:  # once holding, a freeway limit ahead doesn't let go of the light
        hit = self.meters.ahead(gps.latitude, gps.longitude, gps.bearingDeg)

    if hit is not None:
      self.meter, self.distance, self._last_seen = hit[0], max(hit[1], 0.), now
      self.holding = True
      if self._chimed != self.meter:
        self._chimed, self.chime = self.meter, True
    elif self.holding:
      if v_ego <= MIN_SPEED:
        self._last_seen = now  # stopped at the light: keep holding, GPS heading means nothing here
      elif gps is None or now - self._last_seen > LOST_HOLD_S:
        self.holding, self.meter = False, None  # past the light (or it dropped out for good)
    return self.holding
