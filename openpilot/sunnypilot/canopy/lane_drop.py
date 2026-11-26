"""Canopy lane drop heads-up.

In the right lane, coming up on a spot where the freeway's right lane ends or turns into an exit:
chime once, about 20 seconds out, so Mark can tap the left blinker in good time. Canopy can't light
the blinker itself. Quiet when he's already signalling, or not in the right lane.

Drop points come from OpenStreetMap lane counts (a freeway stretch handing over to one with fewer
lanes; "exits" when a ramp leaves there), precomputed on Forge into /data/canopy/lane_drops.json as
[lat, lon, freeway_bearing_deg, kind] rows, kind 0 = lane ends, 1 = lane exits.
When Zia's route plan reaches the comma, its lane drops and exits replace this guesswork on routed
drives (and an exit the route takes won't prompt).
"""
import json
import os

from openpilot.common.constants import CV
from openpilot.sunnypilot.canopy.ramp_meter import RampMeters
from openpilot.sunnypilot.canopy.move_over import lane_position

DROPS_PATH = "/data/canopy/lane_drops.json"
PROMPT_FROM_M = 650.
PROMPT_UNTIL_M = 200.
MIN_MPH = 40.
MIN_FREEWAY_LIMIT_MPH = 55.
MAX_CROSS_M = 20.      # drop points sit on the freeway's centreline in OSM, a few lanes from us


class LaneDrops(RampMeters):
  def __init__(self, path: str = DROPS_PATH):
    self.kinds: list[int] = []
    super().__init__(path)

  def refresh(self) -> None:
    try:
      mtime = os.path.getmtime(self.path)
    except OSError:
      self.meters, self.kinds, self.mtime = [], [], None
      return
    if mtime == self.mtime:
      return
    try:
      with open(self.path) as f:
        rows = json.load(f)["drops"]
      self.meters = [(float(a), float(b), float(c)) for a, b, c, _ in rows]
      self.kinds = [int(k) for *_, k in rows]
      self.mtime = mtime
    except (OSError, ValueError, KeyError, TypeError):
      self.meters, self.kinds = [], []


class LaneDrop:
  ENDS, EXITS = 0, 1

  def __init__(self, drops: LaneDrops | None = None):
    self.drops = drops if drops is not None else LaneDrops()
    self.chime = False
    self.kind = self.ENDS
    self.distance = 0.
    self._done = None

  def update(self, gps, model, car_state, v_ego: float, map_limit: float) -> bool:
    self.chime = False
    if gps is None or not getattr(gps, "hasFix", True) or v_ego * CV.MS_TO_MPH < MIN_MPH:
      return False
    if 0. < map_limit * CV.MS_TO_MPH < MIN_FREEWAY_LIMIT_MPH:
      return False
    hit = self.drops.ahead(gps.latitude, gps.longitude, gps.bearingDeg, PROMPT_FROM_M, MAX_CROSS_M)
    if hit is None or hit[0] == self._done:
      return False
    i, along = hit
    if along < PROMPT_UNTIL_M:
      self._done = i
      return False
    if car_state is None or car_state.leftBlinker or car_state.rightBlinker:
      return False
    in_right_lane, lane_to_left = lane_position(model)
    if not (in_right_lane and lane_to_left):
      return False
    self._done, self.distance, self.chime = i, along, True
    self.kind = self.drops.kinds[i] if i < len(self.drops.kinds) else self.ENDS
    return True
