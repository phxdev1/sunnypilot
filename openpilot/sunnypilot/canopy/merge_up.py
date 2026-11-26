"""Canopy: merge up. Stock speed limit sync only looks ahead for lower limits, so on an on-ramp
it keeps the ramp's (or the street's) limit and only steps up once the car is on the freeway.
When a much higher freeway-class limit is just ahead, take it now so the merge happens at speed.

ProPILOT does not slow for curves, so never raise the set speed while the road ahead is curved
too tightly for the freeway speed (a loop ramp): the model's planned path decides."""
import time

from openpilot.common.constants import CV

MIN_AHEAD_MPH = 60.0   # freeway-class limit ahead
MIN_JUMP_MPH = 15.0    # big step up, not a 35 -> 45 change on a surface street
MAX_DIST_M = 600.0     # about the length of an on-ramp
MAX_LAT_ACCEL = 1.5    # m/s^2 at the target speed on the tightest point of the planned path
PLAN_SECONDS = 6.0     # how far along the model's plan to look for curves
CURVE_HOLD_S = 5.0     # once a curve blocks it, stay blocked a little so the set speed doesn't flap


def plan_max_curvature(model) -> float:
  """Tightest curvature (1/m) on the model's plan over the next PLAN_SECONDS."""
  try:
    ts, yaw, vx = model.position.t, model.orientationRate.z, model.velocity.x
  except AttributeError:
    return float("inf")  # no plan to look at: don't raise the speed
  k = 0.
  for t, r, v in zip(ts, yaw, vx):
    if t > PLAN_SECONDS:
      break
    k = max(k, abs(r) / max(v, 3.))
  return k


class MergeUp:
  def __init__(self):
    self.blocked_until = 0.
    self.active = False

  def update(self, speed_limit: float, next_speed_limit: float, distance_ahead: float, max_curvature: float) -> bool:
    """Limits in m/s (0 = unknown, typical for an untagged ramp); distance in m. True = use the limit ahead now."""
    ahead, now = next_speed_limit * CV.MS_TO_MPH, speed_limit * CV.MS_TO_MPH
    wanted = ahead >= MIN_AHEAD_MPH and distance_ahead <= MAX_DIST_M and (now <= 0. or ahead - now >= MIN_JUMP_MPH)
    t = time.monotonic()
    if wanted and max_curvature * next_speed_limit ** 2 > MAX_LAT_ACCEL:
      self.blocked_until = t + CURVE_HOLD_S
    self.active = wanted and t >= self.blocked_until
    return self.active
