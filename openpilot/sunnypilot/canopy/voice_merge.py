"""Voice merge: turn a Zia POST to the comma hudd into a lane change start."""

import json
import os
import time

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog

VOICE_FILE = os.environ.get("CANOPY_VOICE_FILE", "/dev/shm/canopy/voice_merge.json")
STATE_FILE = os.environ.get("CANOPY_VOICE_STATE", "/dev/shm/canopy/voice_merge_state.json")
LIVE_FLAG = os.environ.get("CANOPY_VOICE_LIVE", "/data/canopy/voice_merge_live")
LOG_FILE = os.environ.get("CANOPY_VOICE_LOG", "/data/canopy/voice_merge.jsonl")
LOG_MAX_BYTES = 1_000_000
EXPIRE_S = 3.0
SIGNAL_WINDOW_S = 1.5
SPEED_MIN = 25 * CV.MPH_TO_MS
SPEED_MAX = 50 * CV.MPH_TO_MS
MAX_ANGLE_DEG = 10.0
NO_BRAKE_S = 2.0
MIN_ACCEL = -1.0
READ_EVERY = 5
LIVE_EVERY = 20


class VoiceMerge:
  """Read voice_merge.json from the hudd and optionally start a lane change.

  Shadow mode by default: logs what it would do and starts nothing unless
  the LIVE_FLAG file exists. A 1.5 s window gives the driver time to signal
  if they want to; the blinker is not required."""

  def __init__(self, clock=time.time):
    self._clock = clock
    self._frame = 0
    self._seen_id = None
    self.pending = None
    self.live = False
    self._since_brake = NO_BRAKE_S

  def _read_cmd(self):
    """Read and json-parse VOICE_FILE; return None on error or non-dict."""
    try:
      with open(VOICE_FILE) as f:
        obj = json.load(f)
      if not isinstance(obj, dict):
        return None
      return obj
    except (OSError, ValueError):
      return None

  def _emit(self, phase, side, **kw):
    """Log the event and write state + log file."""
    rec = {
      "phase": phase,
      "side": side,
      "live": self.live,
      "t": round(self._clock(), 2),
      **kw,
    }
    cloudlog.event("canopy_voice_merge", **rec)
    tmp = STATE_FILE + ".tmp"
    try:
      with open(tmp, "w") as f:
        json.dump(rec, f)
      os.replace(tmp, STATE_FILE)
    except OSError:
      pass
    try:
      if os.path.getsize(LOG_FILE) > LOG_MAX_BYTES:
        return
      with open(LOG_FILE, "a") as f:
        f.write(json.dumps(rec) + "\n")
    except OSError:
      pass

  def step(
    self,
    cs,
    lateral_active: bool,
    idle: bool,
    left_edge: bool,
    right_edge: bool,
    right_bs_advisory: bool,
  ) -> str | None:
    """Evaluate the pending voice command each model frame (20 Hz).

    Returns "left" or "right" on the frame a LIVE voice merge should start,
    else None. cs is carState (vEgo, aEgo, brakePressed, steeringPressed,
    steeringTorque, steeringAngleDeg, leftBlinker, rightBlinker,
    leftBlindspot, rightBlindspot). idle is True when no lane change is
    under way (desire_helper is off or preLaneChange)."""
    self._frame += 1
    now = self._clock()
    self._since_brake = (
      0.0 if cs.brakePressed else min(self._since_brake + DT_MDL, NO_BRAKE_S)
    )

    if self._frame % LIVE_EVERY == 1:
      self.live = os.path.exists(LIVE_FLAG)

    if self._frame % READ_EVERY == 0:
      cmd = self._read_cmd()
      if (
        cmd
        and cmd.get("id") != self._seen_id
        and cmd.get("side") in ("left", "right")
        and isinstance(cmd.get("t"), (int, float))
      ):
        self._seen_id = cmd.get("id")
        age = now - cmd["t"]
        if 0 <= age < EXPIRE_S:
          self.pending = {
            "side": cmd["side"],
            "t_heard": cmd["t"],
            "id": cmd.get("id"),
            "signaled": False,
            "block": None,
            "opp_was": bool(
              cs.rightBlinker if cmd["side"] == "left" else cs.leftBlinker
            ),
          }
          self._emit(
            "heard", cmd["side"], mph=round(cs.vEgo * CV.MS_TO_MPH)
          )

    if self.pending is None:
      return None

    p = self.pending
    side = p["side"]
    left = side == "left"
    age = now - p["t_heard"]
    mph = round(cs.vEgo * CV.MS_TO_MPH)
    same_blinker = cs.leftBlinker if left else cs.rightBlinker
    opp_blinker = cs.rightBlinker if left else cs.leftBlinker

    if same_blinker:
      p["signaled"] = True

    if not lateral_active:
      self._emit(
        "cancel",
        side,
        reason="steering_off",
        signaled=p["signaled"],
        mph=mph,
        age=round(age, 2),
      )
      self.pending = None
      return None

    if cs.vEgo < SPEED_MIN or cs.vEgo >= SPEED_MAX:
      self._emit(
        "cancel",
        side,
        reason="speed",
        signaled=p["signaled"],
        mph=mph,
        age=round(age, 2),
      )
      self.pending = None
      return None

    if cs.brakePressed:
      self._emit(
        "cancel",
        side,
        reason="brake",
        signaled=p["signaled"],
        mph=mph,
        age=round(age, 2),
      )
      self.pending = None
      return None

    if (
      cs.steeringPressed
      and (cs.steeringTorque < 0 if left else cs.steeringTorque > 0)
    ):
      self._emit(
        "cancel",
        side,
        reason="override",
        signaled=p["signaled"],
        mph=mph,
        age=round(age, 2),
      )
      self.pending = None
      return None

    if opp_blinker and not p["opp_was"]:
      self._emit(
        "cancel",
        side,
        reason="opposite_blinker",
        signaled=p["signaled"],
        mph=mph,
        age=round(age, 2),
      )
      self.pending = None
      return None

    p["opp_was"] = bool(opp_blinker)

    if age < SIGNAL_WINDOW_S:
      return None

    if age >= EXPIRE_S:
      self._emit(
        "expired",
        side,
        reason=p["block"] or "busy",
        signaled=p["signaled"],
        mph=mph,
      )
      self.pending = None
      return None

    if not idle:
      p["block"] = "busy"
      return None

    if left and cs.leftBlindspot:
      p["block"] = "blind_spot"
      return None
    if not left and (cs.rightBlindspot and not right_bs_advisory):
      p["block"] = "blind_spot"
      return None

    if left and left_edge:
      p["block"] = "road_edge"
      return None
    if not left and right_edge:
      p["block"] = "road_edge"
      return None

    if abs(cs.steeringAngleDeg) >= MAX_ANGLE_DEG:
      p["block"] = "wheel_turned"
      return None

    if self._since_brake < NO_BRAKE_S:
      p["block"] = "recent_brake"
      return None

    if cs.aEgo <= MIN_ACCEL:
      p["block"] = "slowing"
      return None

    self._emit(
      "start" if self.live else "would_start",
      side,
      signaled=p["signaled"],
      mph=mph,
      age=round(age, 2),
      bs_left=bool(cs.leftBlindspot),
      bs_right=bool(cs.rightBlindspot),
    )
    self.pending = None
    return side if self.live else None
