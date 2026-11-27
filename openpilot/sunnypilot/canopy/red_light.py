"""Canopy red light warning.

ProPILOT only stops for a car ahead. With nobody ahead, it will carry the set speed into a red light.
The driving model does see the light: its plan slows to a stop at the line. So when a mapped traffic
signal is coming up, cruise is on, there is no lead car to stop behind, and the model's plan comes to
a stop at the signal, chime once so Mark brakes.

Two sources must agree before it chimes: the map (a signal is really there) and the model (it wants
to stop for it). Neither alone is enough: the map doesn't know the light's color, and the model
also slows for curves and turns.

Signals come from OpenStreetMap (highway=traffic_signals), precomputed on Forge into
/data/canopy/traffic_signals.json as [lat, lon] rows. Most carry no direction, so a signal counts
when it sits on our line ahead, whichever way it faces.
"""
import json
import math
import os
import time

from openpilot.common.constants import CV

SIGNALS_PATH = "/data/canopy/traffic_signals.json"
CELL_DEG = 0.005       # grid cell for the signal index (~550 m)
CROSS_M = 15.          # how far off our line the signal node may sit (GPS error, wide roads)
MIN_MPH = 20.          # below this he is already slowing or in a lot
MIN_AHEAD_M = 30.      # closer than this the chime is too late to help
MIN_AHEAD_S = 2.5      # ...or closer than this many seconds out
MAX_AHEAD_M = 250.
MAX_AHEAD_S = 12.
STOP_V = 2.5           # m/s: the model plan is "stopped" under this
MIN_DECEL = 0.4        # m/s^2: an unfinished plan slowing less than this isn't stopping
STOP_BEFORE_M = 45.    # signal nodes often sit mid-intersection; the stop line is up to ~40 m short
STOP_PAST_M = 15.
STOP_HOLD_S = 1.0      # the model must want to stop this long before we chime (was 0.5)
CHIME_MIN_DECEL = 0.8  # m/s^2: a plan that ends still moving must brake this hard to count for the chime
STABLE_M = 15.         # the planned stop point may wander this much during the hold, or the hold restarts
# Nov 26: 5 of 7 red reads on the lap were wrong (the car went through). Reads that only extrapolate a
# gentle slow-down, or flicker on for a frame or two, were the cheap ones to cut, so the chime now
# needs a firm or finished stop in the plan, held steady for a full second. Still advisory only.
LEAD_MARGIN_M = 10.    # a lead car closer than the signal plus this: ProPILOT stops for it, stay quiet
COOLDOWN_S = 20.       # one chime per intersection (intersections have several signal nodes)
M_PER_DEG_LAT = 110540.


class TrafficSignals:
  def __init__(self, path: str = SIGNALS_PATH):
    self.path = path
    self.mtime = None
    self.signals: list[tuple[float, float]] = []
    self.grid: dict[tuple[int, int], list[int]] = {}
    self.refresh()

  def refresh(self) -> None:
    try:
      mtime = os.path.getmtime(self.path)
    except OSError:
      self.signals, self.grid, self.mtime = [], {}, None
      return
    if mtime == self.mtime:
      return
    try:
      with open(self.path) as f:
        signals = [(float(a), float(b)) for a, b in json.load(f)["signals"]]
    except (OSError, ValueError, KeyError, TypeError):
      self.signals, self.grid = [], {}
      return
    grid: dict[tuple[int, int], list[int]] = {}
    for i, (lat, lon) in enumerate(signals):
      grid.setdefault((math.floor(lat / CELL_DEG), math.floor(lon / CELL_DEG)), []).append(i)
    self.signals, self.grid, self.mtime = signals, grid, mtime

  def ahead(self, lat: float, lon: float, heading: float, max_ahead: float = MAX_AHEAD_M, max_cross: float = CROSS_M):
    """(index, along-track metres) of the nearest signal on our line ahead, or None."""
    if not self.grid:
      return None
    m_per_deg_lon = 111320. * math.cos(math.radians(lat))
    h = math.radians(heading)
    fx, fy = math.sin(h), math.cos(h)  # east, north
    ci, cj = math.floor(lat / CELL_DEG), math.floor(lon / CELL_DEG)
    best = None
    for di in (-1, 0, 1):
      for dj in (-1, 0, 1):
        for i in self.grid.get((ci + di, cj + dj), ()):
          slat, slon = self.signals[i]
          dx, dy = (slon - lon) * m_per_deg_lon, (slat - lat) * M_PER_DEG_LAT
          along, cross = dx * fx + dy * fy, abs(dx * fy - dy * fx)
          if 0. < along <= max_ahead and cross <= max_cross and (best is None or along < best[1]):
            best = (i, along)
    return best


def plan_stop_point(model, min_decel: float = MIN_DECEL) -> float | None:
  """Where (metres ahead) the model plan comes to a stop, or None if it isn't stopping."""
  try:
    xs, vs = list(model.position.x), list(model.velocity.x)
  except AttributeError:
    return None
  return stop_point(xs, vs, min_decel)


def stop_point(xs, vs, min_decel: float = MIN_DECEL) -> float | None:
  """The plan's stop, from plain x and v lists (the same maths plan_stop_point uses).

  The plan only looks ~10 s ahead, which at 40 mph ends short of a light 150 m out. When it ends
  still moving but braking, carry its last deceleration on to a stop."""
  if len(xs) < 5 or len(xs) != len(vs):
    return None
  for x, v in zip(xs, vs):
    if v < STOP_V:
      return x
  x0, x1, v0, v1 = xs[-5], xs[-1], vs[-5], vs[-1]
  if x1 - x0 < 1.:
    return None
  decel = (v0 * v0 - v1 * v1) / (2. * (x1 - x0))
  if decel < min_decel:
    return None
  return x1 + v1 * v1 / (2. * decel)


class RedLight:
  def __init__(self, signals: TrafficSignals | None = None):
    self.signals = signals if signals is not None else TrafficSignals()
    self.signal = None       # index of the signal ahead, if any
    self.distance = 0.
    self._stop_since = None
    self._stop_at0 = None
    self._chimed = None
    self._last_chime = -1e9

  def update(self, gps, model, car_state, lead, v_ego: float, now: float | None = None) -> bool:
    """True once when the warning should chime. lead: radarState.leadOne or None."""
    now = time.monotonic() if now is None else now
    self.signal = None
    hit = None
    cruise_on = bool(getattr(getattr(car_state, "cruiseState", None), "enabled", False))
    if (gps is not None and getattr(gps, "hasFix", True) and cruise_on and v_ego * CV.MS_TO_MPH >= MIN_MPH
        and not getattr(car_state, "brakePressed", False)):
      hit = self.signals.ahead(gps.latitude, gps.longitude, gps.bearingDeg)
    if hit is not None:
      idx, along = hit
      if not (max(MIN_AHEAD_M, v_ego * MIN_AHEAD_S) <= along <= min(MAX_AHEAD_M, v_ego * MAX_AHEAD_S)):
        hit = None
      elif lead is not None and getattr(lead, "present", False) and lead.dRel < along + LEAD_MARGIN_M:
        hit = None  # ProPILOT will stop behind the car ahead

    if hit is None:
      self._stop_since = None
      return False
    # the stop point is measured from the car, so it shrinks as we close in: compare in track terms
    stop_shift = v_ego * (now - self._stop_since) if self._stop_since is not None else 0.

    self.signal, self.distance = hit
    stop_at = plan_stop_point(model, CHIME_MIN_DECEL)
    if stop_at is None or not (self.distance - STOP_BEFORE_M <= stop_at <= self.distance + STOP_PAST_M):
      self._stop_since = None
      return False
    if self._stop_since is not None and abs(stop_at + stop_shift - self._stop_at0) > STABLE_M:
      self._stop_since = None  # the plan's stop point jumped: start the hold again from here
    if self._stop_since is None:
      self._stop_since, self._stop_at0 = now, stop_at
    if now - self._stop_since < STOP_HOLD_S:
      return False
    if self._chimed == self.signal or now - self._last_chime < COOLDOWN_S:
      return False
    self._chimed, self._last_chime = self.signal, now
    return True
