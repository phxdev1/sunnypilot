"""Canopy move-over prompt.

Coming up on an on-ramp that merges into the freeway from the right, in the right lane, with a lane
to the left and the left side clear: chime once so Mark can tap the blinker and give the mergers
room. Canopy can't light the blinker itself; his tap starts the usual lane change, blind spot gate
and all.

Merge points come from OpenStreetMap (a ramp joining the freeway from the right, skipping ramps that
bring their own new lane), precomputed on Forge into /data/canopy/onramp_merges.json as
[lat, lon, freeway_bearing_deg] rows.

The blind spot can't tell sides apart yet (0x3F0 warns for either side), so "left clear" means the
car reports no blind spot warning at all. That only ever skips a prompt, never gives a wrong one.
"""
import json
import os

from openpilot.common.constants import CV
from openpilot.sunnypilot.canopy.ramp_meter import RampMeters

MERGES_PATH = "/data/canopy/onramp_merges.json"
PROMPT_FROM_M = 450.   # first chance to prompt, before the merge point
PROMPT_UNTIL_M = 120.  # too late to be useful after this
MIN_MPH = 45.
MIN_FREEWAY_LIMIT_MPH = 55.
LANE_W = 3.7           # m
RIGHT_EDGE_MAX_M = 6.  # right road edge this close: no lane to our right
PROB = 0.5
MAX_CROSS_M = 15.     # merge points sit on the right edge; we're in the right lane


class OnrampMerges(RampMeters):
  """Same file shape and lookup as the ramp meters, under the key 'merges'."""

  def __init__(self, path: str = MERGES_PATH):
    super().__init__(path)

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
        self.meters = [(float(a), float(b), float(c)) for a, b, c in json.load(f)["merges"]]
      self.mtime = mtime
    except (OSError, ValueError, KeyError, TypeError):
      self.meters = []


def _near_y(line, n: int = 5) -> float | None:
  """Average lateral offset (right-positive, model frame) over the first few points."""
  try:
    ys = list(line.y)[:n]
  except AttributeError:
    return None
  return sum(ys) / len(ys) if ys else None


def lane_position(model) -> tuple[bool, bool]:
  """(in_right_lane, lane_to_left) from the driving model's lane lines and road edges."""
  try:
    probs = list(model.laneLineProbs)
    edges = list(model.roadEdges)
  except AttributeError:
    return False, False
  if len(probs) < 4 or len(edges) < 2:
    return False, False
  right_edge, left_edge = _near_y(edges[1]), _near_y(edges[0])
  in_right_lane = probs[3] < PROB and right_edge is not None and 0. < right_edge < RIGHT_EDGE_MAX_M
  lane_to_left = probs[0] > PROB and probs[1] > PROB and left_edge is not None and -left_edge > LANE_W * 1.4
  return in_right_lane, lane_to_left


class MoveOver:
  def __init__(self, merges: OnrampMerges | None = None):
    self.merges = merges if merges is not None else OnrampMerges()
    self.chime = False
    self.distance = 0.
    self._done = None  # merge point already prompted (or passed while waiting)

  def update(self, gps, model, car_state, v_ego: float, map_limit: float) -> bool:
    self.chime = False
    if gps is None or not getattr(gps, "hasFix", True) or v_ego * CV.MS_TO_MPH < MIN_MPH:
      return False
    if 0. < map_limit * CV.MS_TO_MPH < MIN_FREEWAY_LIMIT_MPH:
      return False
    hit = self.merges.ahead(gps.latitude, gps.longitude, gps.bearingDeg, PROMPT_FROM_M, MAX_CROSS_M)
    if hit is None or hit[0] == self._done:
      return False
    i, along = hit
    if along < PROMPT_UNTIL_M:
      self._done = i  # missed the window: stay quiet for this ramp
      return False
    if car_state is None or car_state.leftBlindspot or car_state.leftBlinker or car_state.rightBlinker:
      return False
    in_right_lane, lane_to_left = lane_position(model)
    if not (in_right_lane and lane_to_left):
      return False
    self._done, self.distance, self.chime = i, along, True
    return True
