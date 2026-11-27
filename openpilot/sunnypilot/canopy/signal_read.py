"""Canopy signal read: the light the comma is closing on, from what the model plans.

The head unit paints every signal on the route from a timing guess. This is the read it can trust
instead: a mapped traffic signal ahead, and the driving model's plan at that signal.

  red     the plan comes to a stop at the signal's stop line. The same evidence the red light
          chime uses, and strongest once the car is actually standing still inside the window.
  green   the car is standing at the line and the plan opens past it (the green read auto_resume
          logs), or the car is rolling through at speed with nothing in the plan that stops short.
  unknown everything else: the signal is too far for the plan to reach, a car ahead is the reason
          we are slowing, or the plan is a curve or a turn. Nothing is guessed from timing.

Signal positions come from the same OSM table the chime uses (/data/canopy/traffic_signals.json),
so the read and the chime never disagree about which light they mean.

Watch only: nothing in the planner reads this. hudd publishes it on the bus; the head unit draws
it as a detection, over the top of its prediction.
"""
import time

from openpilot.sunnypilot.canopy.red_light import STOP_BEFORE_M, STOP_PAST_M, TrafficSignals, stop_point

MS_TO_MPH = 2.23694
MAX_AHEAD_M = 200.0     # the read is only about the light being closed on, not the next one
MIN_AHEAD_M = 3.0
LEAD_MARGIN_M = 10.0    # a car this close to the signal is why we are stopping, not the light
HOLD_S = 0.5            # a new state must survive this long before it is published (no flicker)
STANDSTILL_V = 0.15     # m/s
STANDSTILL_S = 2.0      # the same "stopped" the green read uses
GREEN_X_M = 30.0        # the plan must reach this far past the signal to count as an open light
GREEN_MIN_MPH = 12.0    # a rolling green needs this much speed (mirrors the head unit's green pass)
RED_PLAN = 0.55         # the plan stops at the line, still rolling
RED_STOP = 0.8          # standing still inside the window
GREEN_STOP = 0.7        # standing at the line and the plan opens
GREEN_ROLL = 0.5        # rolling through at speed (clears a camera-detection bar of 0.5)


class SignalRead:
  def __init__(self, signals=None, clock=time.monotonic):
    self.signals = signals if signals is not None else TrafficSignals()
    self._clock = clock
    self.signal = None
    self.state = "unknown"
    self.conf = 0.0
    self._cand = None
    self._cand_since = None
    self._stop_since = None

  def _reset(self):
    self.signal = None
    self.state, self.conf = "unknown", 0.0
    self._cand, self._cand_since, self._stop_since = None, None, None

  def call(self, stood_s, mph, stop_at, along, xs):
    """(state, confidence) for one sample. Pure: every input is a number or a list."""
    at_line = along - STOP_BEFORE_M <= (stop_at if stop_at is not None else -1e9) <= along + STOP_PAST_M
    opens_past = bool(xs) and max(xs) >= along + GREEN_X_M
    if at_line:
      return ("red", RED_STOP) if stood_s >= STANDSTILL_S else ("red", RED_PLAN)
    if stood_s >= STANDSTILL_S and opens_past:
      return "green", GREEN_STOP
    if mph >= GREEN_MIN_MPH and stop_at is None and opens_past:
      return "green", GREEN_ROLL
    return "unknown", 0.0

  def update(self, lat, lon, heading, v_ego, xs, vs, lead_m=None, now=None):
    """One model frame. Returns the read {m, state, conf, lat, lon} for the signal ahead, or None."""
    now = self._clock() if now is None else now
    hit = None
    if lat is not None and lon is not None and heading is not None:
      hit = self.signals.ahead(lat, lon, heading, MAX_AHEAD_M)
    if hit is None or hit[1] < MIN_AHEAD_M:
      self._reset()
      return None
    idx, along = hit
    if self.signal is not None and idx != self.signal:
      # a different light: the old read says nothing about this one
      self._cand, self._cand_since, self._stop_since = None, None, None
      self.state, self.conf = "unknown", 0.0
    self.signal = idx

    if lead_m is not None and lead_m < along + LEAD_MARGIN_M:
      self._cand, self._cand_since = None, None
      self.state, self.conf = "unknown", 0.0
      return self._out(along)

    if v_ego < STANDSTILL_V:
      if self._stop_since is None:
        self._stop_since = now
    else:
      self._stop_since = None
    stood = 0.0 if self._stop_since is None else now - self._stop_since

    state, conf = self.call(stood, v_ego * MS_TO_MPH, stop_point(xs, vs), along, xs)
    if state == "unknown":
      self.state, self.conf = "unknown", 0.0
      self._cand, self._cand_since = None, None
    else:
      if self._cand != (state, conf):
        self._cand, self._cand_since = (state, conf), now
      if self._cand_since is not None and (state == self.state or now - self._cand_since >= HOLD_S):
        self.state, self.conf = self._cand
    return self._out(along)

  def _out(self, along):
    lat = lon = None
    try:
      lat, lon = self.signals.signals[self.signal]
    except (IndexError, TypeError):
      pass
    return {"m": round(float(along), 1), "state": self.state, "conf": round(float(self.conf), 2),
            "lat": None if lat is None else round(float(lat), 5),
            "lon": None if lon is None else round(float(lon), 5)}
