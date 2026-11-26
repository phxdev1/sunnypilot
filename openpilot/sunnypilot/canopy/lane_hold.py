"""Canopy hold-to-merge and the blinker-off chime.

Default: a held blinker makes exactly one lane change (patch 03 tap lane change, blind spot gate,
1.5 s abort). When that change ends, for any reason, and the blinker is still on 2 s later, one
chime to turn it off, and once more 10 s after that. Never a second change on the same hold.

Route lane: in the last ~1.1 mi before one of the loop's system-ramp exits (101 to 202, 202 to I-10,
I-10 to US 60, US 60 to 101; /data/canopy/hold_targets.json, built on Forge from OSM), a held RIGHT
blinker keeps changing lanes, one at a time, until the car is in the rightmost lane, then chimes.
The rightmost lane is the one fact the model gives reliably (road edge right beside the lane), so
that is the only target. Left holds, and everything off the loop, stay one change per hold.

This runs in plannerd. It tells DesireHelper (modeld) to allow the next change by writing "right" to
/dev/shm/canopy_hold while the car is in an exit zone and not yet in the rightmost lane; DesireHelper
still needs the blinker held, a clear blind spot (either side warning blocks) and no road edge on
that side, and stops after MAX_HOLD_CHANGES.

On screen: while in an exit zone, a small Canopy banner says the exit lane is known ("hold right
for the exit lane", then "in the exit lane"). No banner means a held blinker gets one change only.
hudd streams the same banner to the head unit.
"""
import json
import math
import os

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.canopy import banner

TARGETS_PATH = "/data/canopy/hold_targets.json"
HOLD_FILE = "/dev/shm/canopy_hold"
ZONE_M = 1800.          # how far before the gore a held right blinker may make more than one change
ZONE_CROSS_M = 45.
ZONE_HEADING_DEG = 35.
MIN_SPEED = 13.         # m/s (~30 mph)
EDGE_ROOM_M = 4.0       # rightmost lane: road edge within this of the lane's right line
EDGE_ROOM_FADED_M = 5.5 # ... or this, when the model barely sees a lane line beyond it
FADED_PROB = 0.15
EDGE_CONFIRM = int(0.5 / DT_MDL)
BLINKER_HOLD = int(0.8 / DT_MDL)  # the Leaf's signal flashes; treat it as on through the dark phase
CHIME_AFTER_S = 2.0
CHIME_AGAIN_S = 10.0
HOLD_WAIT_S = 9.0       # in an exit zone, no next change for this long: it has stopped, chime
M_PER_DEG_LAT = 110540.
BANNER_ZONE = "Exit lane known: hold right to reach it"
BANNER_EDGE = "In the exit lane"
NAV_LANES = os.path.join(banner.BANNER_DIR, "nav_lanes.json")  # hudd streams it to the head unit
NAV_EVERY = 10          # frames between nav_lanes.json writes while in a zone (hudd drops it after 2 s)

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


def _near_y(line, n: int = 5):
  try:
    ys = list(line.y)[:n]
  except AttributeError:
    return None
  return sum(ys) / len(ys) if ys else None


def in_rightmost_lane(model) -> bool:
  try:
    lines, edges, probs = list(model.laneLines), list(model.roadEdges), list(model.laneLineProbs)
  except AttributeError:
    return False
  if len(lines) < 4 or len(edges) < 2 or len(probs) < 4:
    return False
  ll2, re1 = _near_y(lines[2]), _near_y(edges[1])
  if ll2 is None or re1 is None:
    return False
  room = re1 - ll2
  return room < EDGE_ROOM_M or (probs[3] < FADED_PROB and room < EDGE_ROOM_FADED_M)


class HoldTargets:
  def __init__(self, path: str = TARGETS_PATH):
    self.path, self.mtime, self.rows = path, None, []
    self.refresh()

  def refresh(self) -> None:
    try:
      mtime = os.path.getmtime(self.path)
    except OSError:
      self.rows, self.mtime = [], None
      return
    if mtime == self.mtime:
      return
    try:
      with open(self.path) as f:
        self.rows = [(float(a), float(b), float(c)) for a, b, c in json.load(f)["targets"]]
      self.mtime = mtime
    except (OSError, ValueError, KeyError, TypeError):
      self.rows = []

  def in_zone(self, lat: float, lon: float, heading: float) -> bool:
    m_per_deg_lon = 111320. * math.cos(math.radians(lat))
    h = math.radians(heading)
    fx, fy = math.sin(h), math.cos(h)
    for tlat, tlon, tbrg in self.rows:
      if abs(tlat - lat) * M_PER_DEG_LAT > ZONE_M + 100. or abs(tlon - lon) * m_per_deg_lon > ZONE_M + 100.:
        continue
      if abs((heading - tbrg + 180.) % 360. - 180.) > ZONE_HEADING_DEG:
        continue
      dx, dy = (tlon - lon) * m_per_deg_lon, (tlat - lat) * M_PER_DEG_LAT
      along, cross = dx * fx + dy * fy, abs(dx * fy - dy * fx)
      if 0. <= along <= ZONE_M and cross <= ZONE_CROSS_M:
        return True
    return False


class LaneHold:
  def __init__(self, targets: HoldTargets | None = None, hold_file: str = HOLD_FILE, post=banner.post, clear=banner.clear):
    self._post, self._clear, self._banner = post, clear, ""
    self.targets = targets if targets is not None else HoldTargets()
    self.hold_file = hold_file
    self.side = ""            # what DesireHelper is told: "right" or ""
    self.in_zone = False
    self.at_edge = False
    self._edge_frames = 0
    self._blink = [0, 0]
    self._prev_state = LaneChangeState.off
    self._dir = ""            # direction of the change under way
    self._pending = None      # direction of the change that just ended while the blinker stayed on
    self._t = 0.
    self._chimes = 0
    self._frame = 0
    self._nav_on = False
    self.merge_cue = ""       # "left" or "right" on the frame a lane change starts, else ""
    self._write("")

  def _write(self, side: str) -> None:
    self.side = side
    try:
      tmp = self.hold_file + ".tmp"
      with open(tmp, "w") as f:
        f.write(side)
      os.replace(tmp, self.hold_file)
    except OSError:
      pass

  def _write_nav(self) -> None:
    """The lane strip for the head unit. The model knows only whether this is the rightmost lane, so
    lanes stays null; target is the rightmost lane, current is filled in once the car is in it."""
    self._nav_on = True
    d = {"lanes": None, "current": "rightmost" if self.at_edge else None, "target": "rightmost",
         "confident": True, "reason": "exit lane"}
    try:
      os.makedirs(os.path.dirname(NAV_LANES), exist_ok=True)
      tmp = NAV_LANES + ".tmp"
      with open(tmp, "w") as f:
        json.dump(d, f)
      os.replace(tmp, NAV_LANES)
    except OSError:
      pass

  def update(self, gps, model, car_state, v_ego: float) -> bool:
    """True for one update when the blinker-off chime should play."""
    self._frame += 1
    if self._frame % 100 == 0:
      self.targets.refresh()

    for i, lit in enumerate((bool(car_state.leftBlinker), bool(car_state.rightBlinker))):
      self._blink[i] = BLINKER_HOLD if lit else max(0, self._blink[i] - 1)
    left_on, right_on = self._blink[0] > 0, self._blink[1] > 0

    self._edge_frames = self._edge_frames + 1 if in_rightmost_lane(model) else 0
    self.at_edge = self._edge_frames >= EDGE_CONFIRM
    has_fix = gps is not None and getattr(gps, "hasFix", True)
    self.in_zone = bool(has_fix and v_ego > MIN_SPEED and self.targets.in_zone(gps.latitude, gps.longitude, gps.bearingDeg))
    side = "right" if self.in_zone and not self.at_edge else ""
    if side != self.side:
      self._write(side)
    if not self.in_zone:
      self._nav_on = False
    text = (BANNER_EDGE if self.at_edge else BANNER_ZONE) if self.in_zone else ""
    if text != self._banner or (text and self._frame % 40 == 0):
      try:
        if text:
          self._post("lane_hold", text, "info", 4.0)
        elif self._banner:
          self._clear("lane_hold")
      except OSError:
        pass
      self._banner = text

    if self.in_zone and (self._frame % NAV_EVERY == 0 or not self._nav_on):
      self._write_nav()

    meta = model.meta
    state, direction = meta.laneChangeState, meta.laneChangeDirection
    chime = False
    self.merge_cue = ""
    if state == LaneChangeState.laneChangeStarting and direction != LaneChangeDirection.none:
      d = "left" if direction == LaneChangeDirection.left else "right"
      if self._prev_state != LaneChangeState.laneChangeStarting:
        self.merge_cue = d
      self._dir = d
    elif self._prev_state == LaneChangeState.laneChangeStarting and self._dir:
      self._pending, self._t, self._chimes = self._dir, 0., 0
    self._prev_state = state

    if self._pending is not None:
      still_on = left_on if self._pending == "left" else right_on
      if not still_on:
        self._pending = None
      elif state == LaneChangeState.laneChangeStarting:
        self._t = 0.  # another change is under way
      else:
        self._t += DT_MDL
        # still heading for the rightmost lane: give the next change time to start (blind spot, settle)
        wait = HOLD_WAIT_S if (self._pending == "right" and self.side == "right") else CHIME_AFTER_S
        if self._chimes == 0 and self._t >= wait:
          chime, self._chimes, self._t = True, 1, 0.
        elif self._chimes == 1 and self._t >= CHIME_AGAIN_S:
          chime, self._chimes = True, 2
    return chime
