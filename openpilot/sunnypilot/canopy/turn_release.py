"""Canopy turn release: stop fighting the driver in slow turns.

At low speed the model often ignores the turn signal and goes straight (the car swings
wide), or the wheel lags and the model then asks for a much tighter turn. Either way he
grabs the wheel and fights the LKAS. When he holds the wheel during a slow turn, release
lateral until the turn is done and the wheel is back near center.

Shadow first: until the gate for FEATURE is live it only logs what it would have done.
Each episode writes one decision row with its own outcome to the canopy decision log, so
the gate can count clean events. A release that he has to correct right after re-engage
demotes the gate cell.
"""
import json
import os
import threading
import time

FEATURE = "turn_release"
LOG_PATH = "/data/canopy/turn_release.jsonl"
DECISIONS_PATH = "/data/canopy/decisions.jsonl"
OFF_FLAG = "/data/canopy/turn_release.off"
MAX_MPH = 19.0
MPH_TO_MS = 0.44704
TURN_ANGLE_DEG = 60.0   # |steering angle| at or above this counts as a turn in progress
HOLD_S = 0.5            # press must be held this long before a release
CENTER_DEG = 30.0       # re-engage only when |angle| below this
QUIET_S = 1.0           # and no press and no blinker for this long
MAX_EPISODE_S = 30.0
REFIGHT_DEG = 45.0      # and the press moved the wheel this far from where it was at re-engage
REFIGHT_S = 2.0         # a press this soon after re-engage marks the release as fail
TURN_DONE_DEG = 90.0    # peak |angle| in an episode at or above this means a real turn was made

MAX_DECISIONS_BYTES = 20 * 1024 * 1024


class TurnRelease:
  def __init__(self, gates=None, log_path=LOG_PATH, decisions_path=DECISIONS_PATH, off_flag=OFF_FLAG,
               clock=time.monotonic, wall=time.time):
    self.log_path = log_path
    self.decisions_path = decisions_path
    self.off_flag = off_flag
    self.clock = clock
    self.wall = wall
    self._off = os.path.exists(off_flag)
    self.gates = gates
    if gates is None:
      try:
        from openpilot.sunnypilot.canopy.gates import Gates
        g = Gates()
        threading.Thread(target=g.snapshot, daemon=True).start()
        self.gates = g
      except Exception:
        self.gates = None
    self.press_since = None
    self.episode = None
    self.released = False
    self.quiet_since = None
    self.reengaged_at = None
    self.reengage_angle = 0.0
    self.last_live_episode = None
    self._last_now = None

  def update(self, v_ego, left_blinker, right_blinker, steering_pressed, steering_angle_deg,
             model_curvature, lat_active) -> bool:
    """True only when lateral must be released now. Never raises."""
    try:
      now = self.clock()

      mph = v_ego / MPH_TO_MS
      one_blinker = left_blinker != right_blinker
      angle = abs(steering_angle_deg)
      turn_context = mph < MAX_MPH and (one_blinker or angle >= TURN_ANGLE_DEG)

      # a press right after a release means the release was wrong
      if self.reengaged_at is not None:
        if self.press_since is not None and now - self.press_since >= HOLD_S and self.press_since - self.reengaged_at <= REFIGHT_S and not turn_context and abs(steering_angle_deg - self.reengage_angle) >= REFIGHT_DEG and self.last_live_episode is not None:
          self._demote(self.last_live_episode, "refight")
          self.reengaged_at = None
        elif now - self.reengaged_at > REFIGHT_S + HOLD_S:
          self.reengaged_at = None

      if steering_pressed:
        self.press_since = self.press_since or now
      else:
        self.press_since = None

      if self.episode is None:
        if lat_active and turn_context and self.press_since is not None and now - self.press_since >= HOLD_S:
          if left_blinker and not right_blinker:
            side = "left"
          elif right_blinker and not left_blinker:
            side = "right"
          else:
            side = "none"
          if side == "left":
            toward = -model_curvature
          elif side == "right":
            toward = model_curvature
          else:
            toward = -model_curvature if steering_angle_deg > 0 else model_curvature
          conf = 1.0 if toward < 0.01 else 0.6
          self.episode = {
            "t0": now, "ts": self.wall(), "mph": round(mph, 1), "side": side, "blinker": one_blinker,
            "angle0": round(steering_angle_deg, 1), "curv0": round(float(model_curvature), 4),
            "peak": angle, "press_s": 0.0, "live": False, "conf": conf,
          }
          self.quiet_since = None
          live = self._is_live(mph, conf)
          self.episode["live"] = live
          if live:
            self.released = True

      if self.episode is not None:
        ep = self.episode
        ep["peak"] = max(ep["peak"], angle)
        dt = 0.0 if self._last_now is None else max(0.0, now - self._last_now)
        if steering_pressed:
          ep["press_s"] += dt

        quiet = (not steering_pressed) and (not one_blinker) and angle < CENTER_DEG
        if quiet:
          if self.quiet_since is None:
            self.quiet_since = now
        else:
          self.quiet_since = None

        if (self.quiet_since is not None and now - self.quiet_since >= QUIET_S) or \
           now - ep["t0"] >= MAX_EPISODE_S or mph >= MAX_MPH + 6:
          self._finish(ep, now)
          if ep["live"] and not steering_pressed:
            # only a quiet hand-back is checked for a refight: a speed or time end while he
            # still holds the wheel is his turn running on, not the car fighting him
            self.reengaged_at = now
            self.reengage_angle = steering_angle_deg
            self.last_live_episode = ep
          self.episode = None
          self.released = False
          self.quiet_since = None

      self._last_now = now
      return self.released
    except Exception:
      self._last_now = None
      return False

  def _is_live(self, mph, conf):
    try:
      if self._off or self.gates is None:
        return False
      cond = self.gates.conditions(None, None, None, mph, None)
      if isinstance(cond, dict):
        cond["speed_band"] = cond.get("speed")
      return bool(self.gates.is_live(FEATURE, cond, conf))
    except Exception:
      return False

  def _band(self, mph):
    """Speed band for the log context, or None when the gates are unavailable."""
    try:
      if self.gates is None:
        return None
      cond = self.gates.conditions(None, None, None, mph, None)
      if isinstance(cond, dict):
        return cond.get("speed")
    except Exception:
      pass
    return None

  def _append(self, path, obj):
    try:
      d = os.path.dirname(path)
      if d:
        os.makedirs(d, exist_ok=True)
      with open(path, "a") as f:
        f.write(json.dumps(obj, separators=(",", ":")) + "\n")
    except OSError:
      pass

  def _finish(self, ep, now):
    try:
      dur = now - ep["t0"]
      turned = ep["peak"] >= TURN_DONE_DEG
      outcome = "clean" if turned and ep["press_s"] >= HOLD_S else "near"
      record = {k: v for k, v in ep.items() if k != "t0"}
      record.update({"event": "episode", "dur": round(dur, 1), "peak": round(ep["peak"], 1),
                     "press_s": round(ep["press_s"], 1), "outcome": outcome, "rule_version": 1})
      self._append(self.log_path, record)
    except Exception:
      pass
    try:
      try:
        if os.path.getsize(self.decisions_path) >= MAX_DECISIONS_BYTES:
          return
      except OSError:
        pass
      try:
        from openpilot.sunnypilot.canopy import leadkind
        drive = leadkind.route_name()
      except Exception:
        drive = "unknown"
      band = self._band(ep["mph"])
      ctx = "side=%s|speed_band=%s" % (ep["side"], band) if band is not None else "side=%s" % ep["side"]
      self._append(self.decisions_path, {
        "type": "decision", "feature": FEATURE, "action": "release", "conf": ep["conf"], "drive": drive,
        "ts": ep["ts"], "live": ep["live"], "ctx": ctx, "lat": None, "lon": None, "outcome": outcome,
        "scored_by": "canopy.turn_release",
      })
    except Exception:
      pass

  def _demote(self, ep, reason):
    if self.gates is None:
      return
    try:
      cond = self.gates.conditions(None, None, None, ep["mph"], None)
      if isinstance(cond, dict):
        cond["speed_band"] = cond.get("speed")
      self.gates.demote(FEATURE, cond, reason)
      self._append(self.log_path, {"ts": self.wall(), "event": "demote", "reason": reason})
    except Exception:
      pass
