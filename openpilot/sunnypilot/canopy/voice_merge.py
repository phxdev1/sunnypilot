"""Voice merge: turn a Zia POST to the comma hudd into a lane change start.

Shadow mode by default: logs what it would do and starts nothing unless
the LIVE_FLAG file exists. A 1.5 s window gives the driver time to signal
if they want to; the blinker is not required.

Speed range: 25 to 85 mph. 50+ mph is the freeway band, live only with a
separate freeway flag; the street band is under 50 mph."""
import json
import math
import os
import time

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog

VOICE_FILE = os.environ.get("CANOPY_VOICE_FILE", "/dev/shm/canopy/voice_merge.json")
STATE_FILE = os.environ.get("CANOPY_VOICE_STATE", "/dev/shm/canopy/voice_merge_state.json")
LIVE_FLAG = os.environ.get("CANOPY_VOICE_LIVE", "/data/canopy/voice_merge_live")
LIVE_FLAG_FREEWAY = os.environ.get("CANOPY_VOICE_LIVE_FREEWAY", "/data/canopy/voice_merge_live_freeway")
LOG_FILE = os.environ.get("CANOPY_VOICE_LOG", "/data/canopy/voice_merge.jsonl")
CONFIDENCE_FILE = os.environ.get("CANOPY_VOICE_CONF", "/data/canopy/voice_merge_confidence.json")
LOG_MAX_BYTES = 1_000_000
EXPIRE_S = 3.0
SIGNAL_WINDOW_S = 1.5
SPEED_MIN = 25 * CV.MPH_TO_MS
SPEED_MAX = 85 * CV.MPH_TO_MS
FREEWAY_MPH = 50
MAX_ANGLE_DEG = 10.0
NO_BRAKE_S = 2.0
MIN_ACCEL = -1.0
READ_EVERY = 5
LIVE_EVERY = 20

# Lane-change detector / outcome window
LC_JUMP_THRESH = 2.0          # m, near-left line y jump between consecutive steps
LC_PROB_THRESH = 0.3          # left_prob must exceed this on both sides of a completed LC
OUTCOME_WINDOW_S = 8.0        # how long an outcome stays open
Z = 1.96                      # Wilson lower-bound z


def _band(mph):
  """Which band a speed lands in for live-flag gating."""
  return "freeway" if mph >= FREEWAY_MPH else "street"


def _wilson_lb(s, f):
  """Wilson lower bound with z = 1.96. Returns 0 when n == 0."""
  n = s + f
  if n == 0:
    return 0.0
  p = s / n
  z2 = Z * Z
  num = p + z2 / (2 * n) - Z * math.sqrt(p * (1 - p) / n + z2 / (4 * n * n))
  den = 1 + z2 / n
  return num / den


def _load_confidence():
  """Load the persisted confidence dict; missing file => zeros."""
  try:
    with open(CONFIDENCE_FILE) as f:
      d = json.load(f)
    if not isinstance(d, dict):
      return {"street": _zero_band(), "freeway": _zero_band()}
    out = {}
    for b in ("street", "freeway"):
      bd = d.get(b, {})
      out[b] = {
        "s": float(bd.get("s", 0.0)),
        "f": float(bd.get("f", 0.0)),
        "n_events": int(bd.get("n_events", 0)),
        "lb": float(bd.get("lb", 0.0)),
        "live": bool(bd.get("live", False)),
        "pending_live": bool(bd.get("pending_live", False)),
      }
    return out
  except (OSError, ValueError):
    return {"street": _zero_band(), "freeway": _zero_band()}


def _zero_band():
  return {"s": 0.0, "f": 0.0, "n_events": 0, "lb": 0.0, "live": False, "pending_live": False}


def _atomic_write_confidence(conf):
  """Write confidence atomically (tmp + os.replace) only when it changes."""
  try:
    tmp = CONFIDENCE_FILE + ".tmp"
    with open(tmp, "w") as f:
      json.dump(conf, f)
    os.replace(tmp, CONFIDENCE_FILE)
  except OSError:
    pass


def _set_band_flag(band, live):
  """Atomically ensure (live) or remove (not live) the flag file for a band."""
  path = LIVE_FLAG if band == "street" else LIVE_FLAG_FREEWAY
  if live:
    try:
      open(path, "a").close()
    except OSError:
      pass
  else:
    try:
      os.remove(path)
    except OSError:
      pass


class VoiceMerge:
  """Read voice_merge.json from the hudd and optionally start a lane change.

  Shadow mode by default: logs what it would do and starts nothing unless
  the street LIVE_FLAG file exists (street band) or the freeway flag exists
  (freeway band, 50+ mph). A 1.5 s window gives the driver time to signal
  if they want to; the blinker is not required."""

  def __init__(self, clock=time.time):
    self._clock = clock
    self._frame = 0
    self._seen_id = None
    self.pending = None
    self.live = False
    self.live_freeway = False
    self._since_brake = NO_BRAKE_S
    self._prev_left_y = None
    self._prev_left_prob = None
    self.confidence = _load_confidence()
    self._outcome = None

    # Auto-live: ensure any band with a pending flag claim is applied BEFORE
    # the live flags are read from disk. A flip to live takes effect only here
    # (drive start, in Park) and is never applied mid-drive.
    for band in ("street", "freeway"):
      bd = self.confidence.get(band, {})
      if bd.get("pending_live"):
        lb = bd.get("lb", 0.0)
        _set_band_flag(band, True)
        bd["pending_live"] = False
        bd["live"] = True
        _atomic_write_confidence(self.confidence)
        self._emit_state("auto_live", None, band=band, lb=round(lb, 3))

    # Read live flags ONCE at drive start. The periodic re-read is removed;
    # LIVE_EVERY stays only because it is referenced by the frame loop below.
    self.live = os.path.exists(LIVE_FLAG)
    self.live_freeway = os.path.exists(LIVE_FLAG_FREEWAY)

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
      "live_freeway": self.live_freeway,
      "t": round(self._clock(), 2),
      "confidence": {
        "street": round(self.confidence.get("street", {}).get("lb", 0.0), 3),
        "freeway": round(self.confidence.get("freeway", {}).get("lb", 0.0), 3),
      },
      "pending": {
        "street": bool(self.confidence.get("street", {}).get("pending_live", False)),
        "freeway": bool(self.confidence.get("freeway", {}).get("pending_live", False)),
      },
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

  def _emit_state(self, phase, side, **kw):
    """Log an event that must reach the state file as the latest event (for hudd)."""
    rec = {
      "phase": phase,
      "side": side,
      "live": self.live,
      "live_freeway": self.live_freeway,
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

  def _close_outcome(self, clean, reason_close=None):
    """Close the open outcome, update confidence, log, and possibly queue a pending live flip."""
    if self._outcome is None:
      return
    o = self._outcome
    self._outcome = None
    side = o["side"]
    band = o["band"]
    live = o["live"]
    w = o["weight"]

    if clean:
      self.confidence[band]["s"] += w
    else:
      self.confidence[band]["f"] += 1.0

    bd = self.confidence[band]
    bd["n_events"] = int(bd["s"] + bd["f"])
    bd["lb"] = _wilson_lb(bd["s"], bd["f"])

    _atomic_write_confidence(self.confidence)

    # Log the closed outcome
    self._emit("outcome", side,
               band=band, live=live, clean=clean,
               weight=round(w, 2),
               s=round(self.confidence[band]["s"], 2),
               f=round(self.confidence[band]["f"], 2),
               lb=round(self.confidence[band]["lb"], 3))

    # Confidence-driven live flag transitions
    if not clean and live:
      # Demote: remove live flag, reset band to zeros, set in-memory live False
      bd["s"] = 0.0
      bd["f"] = 0.0
      bd["n_events"] = 0
      bd["lb"] = 0.0
      bd["live"] = False
      bd["pending_live"] = False
      _atomic_write_confidence(self.confidence)
      if band == "street":
        _set_band_flag("street", False)
        self.live = False
      else:
        _set_band_flag("freeway", False)
        self.live_freeway = False
      self._emit_state("demoted", side, band=band)
      return

    if clean and not live and o.get("was_practice"):
      # Practice outcome that crossed the line: queue a pending auto-live flip.
      # The flip is applied only in __init__ (drive start, in Park), never mid-drive.
      lb = self.confidence[band]["lb"]
      if lb >= 0.95 and not self.confidence[band]["live"]:
        bd["pending_live"] = True
        _atomic_write_confidence(self.confidence)
        self._emit("pending_live", side, band=band, lb=round(lb, 3))

  def step(
    self,
    cs,
    lateral_active: bool,
    idle: bool,
    left_edge: bool,
    right_edge: bool,
    right_bs_advisory: bool,
    lane_info=None,
  ) -> str | None:
    """Evaluate the pending voice command each model frame (20 Hz).

    Returns "left" or "right" on the frame a LIVE voice merge should start,
    else None. cs is carState (vEgo, aEgo, brakePressed, steeringPressed,
    steeringTorque, steeringAngleDeg, leftBlinker, rightBlinker,
    leftBlindspot, rightBlindspot). idle is True when no lane change is
    under way (desire_helper is off or preLaneChange).

    lane_info: tuple (left_y, right_y, left_prob, right_prob) in meters,
    openpilot frame (right is positive), or None."""
    self._frame += 1
    now = self._clock()
    self._since_brake = (
      0.0 if cs.brakePressed else min(self._since_brake + DT_MDL, NO_BRAKE_S)
    )

    if self._frame % LIVE_EVERY == 1:
      # LIVE_EVERY is retained because the frame loop references it.
      # The periodic re-read of self.live / self.live_freeway is removed:
      # live flags are read ONCE in __init__ (drive start, in Park).
      pass

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
            "heard", cmd["side"], mph=round(cs.vEgo * CV.MS_TO_MPH),
            band=_band(round(cs.vEgo * CV.MS_TO_MPH))
          )

    # --- Lane-change detector (runs every step while an outcome is open) ---
    if lane_info is not None:
      left_y, right_y, left_prob, right_prob = lane_info
    else:
      left_y = right_y = left_prob = right_prob = None

    if self._outcome is not None and left_y is not None and left_prob is not None:
      prev_y = self._prev_left_y
      prev_prob = self._prev_left_prob
      self._prev_left_y = left_y
      self._prev_left_prob = left_prob
      if prev_y is not None and prev_prob is not None:
        jump = left_y - prev_y
        if abs(jump) > LC_JUMP_THRESH and prev_prob > LC_PROB_THRESH and left_prob > LC_PROB_THRESH:
          # Completed lane change detected
          going_left = self._outcome["side"] == "left"
          lc_left = jump < 0   # negative jump => car moved left one lane
          if lc_left == going_left:
            self._close_outcome(clean=True)
          else:
            self._close_outcome(clean=False)

    # --- Check for brake press while an outcome is open ---
    if self._outcome is not None and cs.brakePressed:
      self._close_outcome(clean=False)

    # --- No lane change within the outcome window is NOT clean ---
    if self._outcome is not None and now - self._outcome["opened_at"] >= OUTCOME_WINDOW_S:
      self._close_outcome(clean=False)

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
        band=_band(mph),
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
        band=_band(mph),
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
        band=_band(mph),
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
        band=_band(mph),
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
        band=_band(mph),
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
        band=_band(mph),
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

    # Decide the band from the speed at the moment the merge would start
    # (the end of the 1.5 s window); only the matching band's flag makes it live.
    band = _band(mph)
    live_this_band = self.live_freeway if band == "freeway" else self.live

    # Open an outcome window for this command
    w = 0.5
    if lane_info is not None:
      left_prob_i, right_prob_i = lane_info[2], lane_info[3]
      if side == "left":
        w = max(0.0, min(1.0, left_prob_i))
      else:
        w = max(0.0, min(1.0, right_prob_i))

    # Close any existing open outcome: only NOT-clean it if it was live
    if self._outcome is not None:
      if self._outcome["live"]:
        self._close_outcome(clean=False)
      else:
        self._outcome = None

    self._outcome = {
      "side": side,
      "band": band,
      "live": live_this_band,
      "weight": w,
      "was_practice": not live_this_band,
      "opened_at": now,
    }
    self._prev_left_y = left_y
    self._prev_left_prob = left_prob

    self._emit(
      "start" if live_this_band else "would_start",
      side,
      signaled=p["signaled"],
      mph=mph,
      age=round(age, 2),
      band=band,
      bs_left=bool(cs.leftBlindspot),
      bs_right=bool(cs.rightBlindspot),
    )
    self.pending = None
    return side if live_this_band else None
