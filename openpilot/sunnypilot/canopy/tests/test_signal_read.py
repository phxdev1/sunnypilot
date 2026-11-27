"""Tests for canopy/signal_read.py: the read only claims what a plan can prove, and it holds."""
from openpilot.sunnypilot.canopy import signal_read
from openpilot.sunnypilot.canopy.signal_read import SignalRead


class FakeSignals:
  """Stands in for red_light.TrafficSignals: one signal, at a fixed distance ahead (or none)."""

  def __init__(self, along=None, index=0):
    self.along = along
    self.index = index
    self.signals = [(33.41520, -111.74420), (33.41600, -111.74500)]

  def ahead(self, lat, lon, heading, max_ahead=0.0):
    if self.along is None:
      return None
    return self.index, self.along


def plan(stop_at=None, reach=120.0):
  """A model plan: 11 points 20 m apart, either rolling to the reach or easing to a stop."""
  xs = [20.0 * i for i in range(11)]
  if stop_at is None:
    return xs, [15.0] * len(xs)
  vs = []
  for x in xs:
    if x >= stop_at:
      vs.append(0.0)
    else:
      d = max(stop_at - x, 1.0)
      vs.append(min(15.0, (2.0 * 3.0 * d) ** 0.5))
  return xs, vs


def at(sr, along, mph, stop_at=None, lead=None, t=0.0, reach=120.0):
  xs, vs = plan(stop_at, reach)
  return sr.update(33.4152, -111.7442, 90.0, mph / 2.23694, xs, vs, lead_m=lead, now=t)


def test_nothing_without_a_fix_or_a_signal():
  sr = SignalRead(signals=FakeSignals(along=None))
  assert sr.update(None, None, None, 10.0, [], [], now=0.0) is None
  assert at(sr, None, 30) is None
  assert sr.state == "unknown"


def test_red_needs_the_plan_to_stop_at_the_line_and_a_moment_to_hold():
  sr = SignalRead(signals=FakeSignals(along=100.0))
  assert at(sr, 100.0, 30.0, stop_at=95.0, t=0.0)["state"] == "unknown"   # first frame: not held yet
  assert at(sr, 100.0, 30.0, stop_at=95.0, t=0.3)["state"] == "unknown"   # still inside HOLD_S
  out = at(sr, 100.0, 30.0, stop_at=95.0, t=0.6)
  assert out["state"] == "red" and out["conf"] == signal_read.RED_PLAN
  assert out["lat"] == 33.4152 and out["lon"] == -111.7442


def test_standing_at_the_line_is_the_strong_red():
  sr = SignalRead(signals=FakeSignals(along=25.0))
  for t in (0.0, 1.0, 2.1, 2.6):
    out = at(sr, 25.0, 0.0, stop_at=20.0, t=t)
  assert out["state"] == "red" and out["conf"] == signal_read.RED_STOP


def test_green_at_a_standstill_when_the_plan_opens_past_the_light():
  sr = SignalRead(signals=FakeSignals(along=25.0))
  for t in (0.0, 1.0, 2.1, 2.6):
    out = at(sr, 25.0, 0.0, stop_at=None, reach=90.0, t=t)
  assert out["state"] == "green" and out["conf"] == signal_read.GREEN_STOP


def test_rolling_green_needs_speed_and_a_plan_that_does_not_stop_short():
  sr = SignalRead(signals=FakeSignals(along=60.0))
  out = at(sr, 60.0, 30.0, stop_at=None, reach=120.0, t=0.0)
  assert out["state"] == "unknown"                                        # HOLD_S not reached
  out = at(sr, 60.0, 30.0, stop_at=None, reach=120.0, t=0.6)
  assert out["state"] == "green" and out["conf"] == signal_read.GREEN_ROLL
  slow = SignalRead(signals=FakeSignals(along=60.0))
  for t in (0.0, 0.6):
    out = at(slow, 60.0, 4.0, stop_at=None, reach=120.0, t=t)
  assert out["state"] == "unknown"                                        # too slow to claim green


def test_a_plan_that_does_not_reach_the_light_claims_nothing():
  sr = SignalRead(signals=FakeSignals(along=180.0))
  for t in (0.0, 0.6, 1.2):
    out = at(sr, 180.0, 30.0, stop_at=None, reach=140.0, t=t)
  assert out["state"] == "unknown"


def test_a_lead_car_is_the_reason_and_not_the_light():
  sr = SignalRead(signals=FakeSignals(along=25.0))
  for t in (0.0, 1.0, 2.1, 2.6):
    out = at(sr, 25.0, 0.0, stop_at=20.0, lead=18.0, t=t)
  assert out["state"] == "unknown" and out["conf"] == 0.0


def test_a_new_signal_starts_the_hold_again():
  sr = SignalRead(signals=FakeSignals(along=100.0, index=0))
  at(sr, 100.0, 30.0, stop_at=95.0, t=0.0)
  assert at(sr, 100.0, 30.0, stop_at=95.0, t=0.6)["state"] == "red"
  sr.signals.index = 1
  assert at(sr, 100.0, 30.0, stop_at=95.0, t=0.7)["state"] == "unknown"
  assert at(sr, 100.0, 30.0, stop_at=95.0, t=1.3)["state"] == "red"


def test_a_flicker_never_reaches_the_screen():
  sr = SignalRead(signals=FakeSignals(along=100.0))
  at(sr, 100.0, 30.0, stop_at=95.0, t=0.0)
  assert at(sr, 100.0, 30.0, stop_at=95.0, t=0.6)["state"] == "red"
  assert at(sr, 100.0, 30.0, stop_at=None, reach=140.0, t=0.8)["state"] == "red"   # one frame, then gone
  assert at(sr, 100.0, 30.0, stop_at=None, reach=140.0, t=1.4)["state"] == "green"


def test_the_window_is_the_chime_window():
  sr = SignalRead(signals=FakeSignals(along=100.0))
  # a stop 60 m short of the signal is a queue, not the light
  for t in (0.0, 0.6):
    out = at(sr, 100.0, 30.0, stop_at=40.0, t=t)
  assert out["state"] == "unknown"
