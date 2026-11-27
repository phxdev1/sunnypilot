"""Canopy turn path head: the wheel angle a slow left turn wants, scored in shadow.

Slow left turns under about 20 mph come out wide or tight. The model ignores the left
blinker and swings wide, and when the wheel cannot keep up with what it asks for it
overcorrects. The intended fix is a planned turn path that pre-winds seconds before the
corner. This is the car side of that plan: while a slow left turn is coming up it asks the
turn_path head (heads.HeadSet) how much pre-wind it would take, and writes one record per
turn in the shape the Forge replay writes, so the two can be compared line for line.

Shadow only. It reads and logs; it never returns a steer command and never touches
desired curvature, a lateral command or an actuator. `live` is always False here and the
head is scored only through heads.HeadSet's shadow slot. A takeover during a record is
ground truth against the would-be path: it demotes the feature and puts the last good
head back if one was live.

Right turns are not recorded: Mark's rule is that they do not count for this work.
No LLM runs in this path. Every speed here is MPH.
"""
import json
import os
import time

FEATURE = "turn_path"
LOG_PATH = "/data/canopy/turn_path.jsonl"
OFF_FLAG = "/data/canopy/turn_path.off"

MAX_MPH = 19.0          # a slow turn for this work; above this the model holds its own
MPH_TO_MS = 0.44704
APPROACH_DEG = 15.0     # hands near straight: still entering the turn, not in it
TURN_DEG = 60.0         # |steering angle| at or above this means the turn has started
CENTER_DEG = 30.0       # back near center with the blinker off ends the record
GAP_S = 2.0             # or this long with no blinker and no press
MAX_EPISODE_S = 60.0
HOLD_S = 0.5            # a press held this long is a takeover, not a nudge
SCORE_EVERY_S = 0.5     # refresh the head score this often inside a turn

# the would-be path, a rule baseline until a trained head carries its own shape
LEAD_MIN_S = 1.5
LEAD_MAX_S = 3.0
TARGET_MIN_DEG = 60.0
TARGET_MAX_DEG = 540.0
MAX_LOG_BYTES = 20 * 1024 * 1024


def would_be_path(mph):
  """(lead_s, target_deg) for a slow left at this speed: pre-wind earlier the slower
  the entry, and aim the wheel further round. Baseline only, logged next to the head's
  own confidence so replay can score the two against his takeovers."""
  try:
    mph = float(mph)
  except (TypeError, ValueError):
    mph = 0.0
  lead = min(LEAD_MAX_S, max(LEAD_MIN_S, 1.5 + 0.05 * (MAX_MPH - mph)))
  target = min(TARGET_MAX_DEG, max(TARGET_MIN_DEG, 30.0 + 28.0 * mph))
  return round(lead, 2), round(target, 1)


def record(ep, drive, rule_version=1):
  """The one shape both this module and the replay write. Pure: no I/O."""
  lead, target = would_be_path(ep.get("mph"))
  rec = {
    "type": "turn_path",
    "event": "turn",
    "drive": drive,
    "ts": ep.get("ts"),
    "side": ep.get("side", "left"),
    "mph": ep.get("mph"),
    "dur_s": ep.get("dur_s"),
    "peak_deg": ep.get("peak_deg"),
    "press_s": ep.get("press_s"),
    "blinker_s": ep.get("blinker_s"),
    "curv0": ep.get("curv0"),
    "lead_s": lead,
    "target_deg": target,
    "conf": ep.get("conf"),
    "head": ep.get("head"),
    "base": ep.get("base"),
    "shadow": ep.get("shadow") or {},
    "outcome": ep.get("outcome"),
    "live": False,
    "act": False,
    "rule_version": rule_version,
    "scored_by": "canopy.turn_path",
  }
  return rec


def gps_pose(sm):
  """(lat, lon, heading) from the SubMaster, or (None, None, None) if it has no GPS.

  controlsd does not subscribe to gpsLocationExternal, so this is normally None and
  the record keeps its position blank. Looked up here, in its own try, so a missing or
  late service can never stop turn_path.update from running.
  """
  try:
    if sm is None:
      return None, None, None
    if hasattr(sm, "all_checks") and not sm.all_checks(["gpsLocationExternal"]):
      return None, None, None
    gps = sm["gpsLocationExternal"]
    return gps.latitude, gps.longitude, gps.bearingDeg
  except Exception:
    return None, None, None


def _band(mph):
  try:
    mph = float(mph)
  except (TypeError, ValueError):
    return None
  if mph < 25.0:
    return "lt25"
  return "25to50" if mph < 50.0 else "50plus"


class TurnPath:
  """Watches for slow left turns, stamps each with the turn_path head, never steers."""

  def __init__(self, heads=None, gates=None, log_path=LOG_PATH, off_flag=OFF_FLAG,
               clock=time.monotonic, wall=time.time, head_root=None):
    self.log_path = log_path
    self.off_flag = off_flag
    self.clock = clock
    self.wall = wall
    self._off = os.path.exists(off_flag)
    self._heads = heads
    self._head_root = head_root
    self._heads_tried = heads is not None
    self._gates = gates
    self._gates_tried = gates is not None
    self.episode = None
    self.last = None
    self._last_score = None
    self._last_frame = None

  # ---- the pieces that live on the car ----

  def _head_set(self):
    if self._heads is None and not self._heads_tried:
      self._heads_tried = True
      try:
        from openpilot.sunnypilot.canopy import heads as canopy_heads
        self._heads = canopy_heads.HeadSet(root=self._head_root) if self._head_root else canopy_heads.HeadSet()
      except Exception:
        self._heads = None
    return self._heads

  def _gate_set(self):
    if self._gates is None and not self._gates_tried:
      self._gates_tried = True
      try:
        from openpilot.sunnypilot.canopy.gates import Gates
        self._gates = Gates()
      except Exception:
        self._gates = None
    return self._gates

  def _score(self, mph, lat, lon, heading):
    """The head's read on this turn. Always a dict; never raises."""
    out = {"head": None, "conf": None, "base": None, "shadow": {}}
    hs = self._head_set()
    if hs is None:
      return out
    try:
      state = {"mph": mph, "cr_on": True}
      scored = hs.score(FEATURE, state, {"speed": _band(mph)}, lat, lon, heading)
      out["base"] = scored.get("base")
      out["head"] = scored.get("head")
      out["conf"] = scored.get("conf")
      out["shadow"] = dict(scored.get("shadow") or {})
      if out["conf"] is None:
        vals = [v for v in out["shadow"].values() if isinstance(v, (int, float))]
        out["conf"] = vals[0] if vals else None
    except Exception:
      pass
    return out

  # ---- the loop ----

  def update(self, v_ego, left_blinker, right_blinker, steering_pressed, steering_angle_deg,
             lat_active, model_curvature=0.0, lat=None, lon=None, heading=None) -> None:
    """One control frame. Logs and returns nothing: there is no output to apply."""
    if self._off:
      return None
    try:
      now = self.clock()
      mph = float(v_ego) / MPH_TO_MS
      angle = abs(float(steering_angle_deg))
      left = bool(left_blinker) and not bool(right_blinker)
      pressed = bool(steering_pressed)

      ep = self.episode
      if ep is None:
        if left and lat_active and mph < MAX_MPH and angle < APPROACH_DEG:
          self._open(now, mph, angle, float(model_curvature or 0.0), lat, lon, heading)
        return None

      ep["dur_s"] = round(now - ep["t0"], 1)
      ep["peak_deg"] = round(max(ep["peak_deg"], angle), 1)
      if left:
        ep["blinker_s"] = round(now - ep["blinker_off_at"], 1) if ep.get("blinker_off_at") else round(now - ep["t0"], 1)
        ep["blinker_off_at"] = None
      elif ep.get("blinker_off_at") is None:
        ep["blinker_off_at"] = now
      if pressed:
        ep["press_s"] = round(ep.get("press_s", 0.0) + self._dt(now), 1)

      if self._last_score is None or now - self._last_score >= SCORE_EVERY_S:
        self._last_score = now
        scored = self._score(mph, lat, lon, heading)
        ep.update({k: scored[k] for k in ("head", "base", "shadow")})
        ep["conf"] = scored["conf"]

      if self._closing(now, mph, angle, left, pressed):
        self._close(now, mph)
      return None
    except Exception:
      return None

  def _dt(self, now):
    last = self._last_frame
    self._last_frame = now
    try:
      return min(0.5, max(0.0, now - last))
    except (TypeError, ValueError):
      return 0.0

  def _open(self, now, mph, angle, curv, lat, lon, heading):
    scored = self._score(mph, lat, lon, heading)
    self._last_score = now
    self._last_frame = now
    self.episode = {
      "t0": now, "ts": self.wall(), "side": "left", "mph": round(mph, 1), "dur_s": 0.0,
      "peak_deg": round(angle, 1), "press_s": 0.0, "blinker_s": 0.0, "blinker_off_at": None,
      "curv0": round(curv, 4), "conf": scored["conf"], "head": scored["head"],
      "base": scored["base"], "shadow": scored["shadow"], "outcome": None,
      "lat": lat, "lon": lon,
    }

  def _closing(self, now, mph, angle, left, pressed):
    ep = self.episode
    if ep is None:
      return False
    if now - ep["t0"] >= MAX_EPISODE_S:
      return True
    off = ep.get("blinker_off_at")
    if not left and off is not None and angle < CENTER_DEG and now - off >= GAP_S:
      return True
    if mph >= MAX_MPH + 4.0:
      return True
    return False

  def _close(self, now, mph):
    ep = self.episode
    if ep is None:
      return None
    self.episode = None
    self._last_score = None
    try:
      took = ep.get("press_s", 0.0) >= HOLD_S
      if took:
        # he put a hand on it: takeover is the label, and it is the would-be path that
        # gets scored against what his own hands then did
        outcome = "takeover" if ep["peak_deg"] >= TURN_DEG else "touch"
      else:
        outcome = "clean" if ep["peak_deg"] >= TURN_DEG else "near"
      ep["outcome"] = outcome
      rec = record(ep, self._drive())
      self._append(self.log_path, rec)
      self.last = rec
      self._decision(ep, rec)
      if outcome in ("takeover", "touch"):
        self._demote(ep, outcome)
    except Exception:
      pass
    return None

  def _drive(self):
    try:
      from openpilot.sunnypilot.canopy import leadkind
      return leadkind.route_name()
    except Exception:
      return "unknown"

  def _decision(self, ep, rec):
    """One gate row per turn, stamped with the head, so the gate can count clean turns."""
    path = rec.get("target_deg")
    try:
      from openpilot.sunnypilot.canopy.decision_log import log_decision
      log_decision(FEATURE, "prewind", 0.5 if ep.get("conf") is None else float(ep["conf"]),
                   ctx={"side": "left", "speed_band": _band(ep.get("mph")), "lead": rec.get("lead_s"),
                        "target": path, "outcome": ep.get("outcome")},
                   live=False, lat=ep.get("lat"), lon=ep.get("lon"),
                   head=ep.get("head"), base=ep.get("base"), shadow=ep.get("shadow"))
    except Exception:
      pass

  def _demote(self, ep, reason):
    try:
      gates = self._gate_set()
      if gates is not None:
        cond = gates.conditions(ep.get("lat"), ep.get("lon"), ep.get("ts"), ep.get("mph"), None)
        if isinstance(cond, dict):
          cond["speed_band"] = cond.get("speed")
        gates.demote(FEATURE, cond, reason)
      hs = self._head_set()
      if hs is not None:
        # the wheel is back near center and the blinker is off when a record closes
        hs.rollback(FEATURE, True)
    except Exception:
      pass

  @staticmethod
  def _append(path, obj):
    try:
      d = os.path.dirname(path)
      if d:
        os.makedirs(d, exist_ok=True)
      try:
        if os.path.getsize(path) >= MAX_LOG_BYTES:
          return
      except OSError:
        pass
      with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(obj, separators=(",", ":")) + "\n")
    except OSError:
      pass
