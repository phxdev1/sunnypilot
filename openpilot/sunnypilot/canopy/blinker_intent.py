"""Canopy blinker intent: lane change or turn, from the camera, at blinker-on.

Between 25 and 50 mph a blinker today always means "turn" and only a nudge starts a lane change.
Here, at the moment the blinker comes on, the road edges decide: if there is at least one lane
between the car and the road edge on the blinker side, the blinker means a lane change. The call is
logged every time (watch only). It only starts the change with no nudge (act) when the confidence
gate says the feature is live for the current speed band, so nothing changes until the gate is
opened. No LLM, no network, no blocking I/O in update: observe() runs every model frame and just
keeps a one second history, on_blinker() does the median and the call once.
"""
import json
import math
import os
import statistics
import threading
import time
from collections import deque

try:
  from openpilot.sunnypilot.canopy.decision_log import log_decision as _log_decision
except Exception:
  _log_decision = None

FEATURE = "blinker_intent"
LOG_PATH = "/data/canopy/blinker_intent.jsonl"
HINTS_PATH = "/data/canopy/blinker_hints.json"
OFF_FLAG = "/data/canopy/blinker_intent_off"
MIN_MPH = 25.0
MAX_MPH = 50.0
PARAMS = {
  "lane_w_m": 3.6, "right_shoulder_m": 1.8, "left_shoulder_m": 0.5, "left_edge_max_m": 12.0,
  "edge_std_good": 0.7, "edge_std_ok": 1.1, "edge_std_fair": 1.6,
  "conf_std_good": 0.9, "conf_std_ok": 0.7, "conf_std_fair": 0.45, "conf_std_poor": 0.25,
  "near_prob_min": 0.5, "near_prob_penalty": 0.8, "lane_conf_min": 0.5,
  "hint_radius_m": 200.0, "hint_heading_deg": 30.0, "multi_turn_penalty": 0.8,
}
HISTORY_FRAMES = 20   # 1 s at 20 Hz


def _dist_m(lat1, lon1, lat2, lon2):
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp, dl = p2 - p1, math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * 6371000.0 * math.asin(math.sqrt(a))


def _bearing_deg(lat1, lon1, lat2, lon2):
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dl = math.radians(lon2 - lon1)
  y = math.sin(dl) * math.cos(p2)
  x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
  return math.degrees(math.atan2(y, x)) % 360.0


def _angle_diff(a, b):
  return abs((a - b + 180.0) % 360.0 - 180.0)


def _median(vals):
  vals = [v for v in vals if v is not None]
  return statistics.median(vals) if vals else None


def _abs_or_none(v):
  try:
    return abs(v)
  except (TypeError, ValueError):
    return None


def lane_width(l1_y, l2_y, p1, p2):
  """Lane width (m) between two lane lines, or the default when the lines are not trusted."""
  try:
    w = abs(l2_y - l1_y)
  except (TypeError, ValueError):
    return PARAMS["lane_w_m"]
  if p1 is not None and p2 is not None and p1 > 0.5 and p2 > 0.5 and 2.6 <= w <= 4.5:
    return w
  return PARAMS["lane_w_m"]


def lanes_toward(side, edge_m, edge_std, lane_w, near_prob, params=PARAMS):
  """(lanes between the car and that road edge, confidence). n None when the edge is unknown."""
  if edge_m is None:
    return None, 0.0
  if side == "left" and edge_m > params["left_edge_max_m"]:
    return None, 0.0
  shoulder = params["right_shoulder_m"] if side == "right" else params["left_shoulder_m"]
  n = int(max(0, math.floor((edge_m - lane_w / 2.0 - shoulder) / lane_w + 1e-6)))
  if edge_std is None:
    conf = params["conf_std_poor"]
  elif edge_std < params["edge_std_good"]:
    conf = params["conf_std_good"]
  elif edge_std < params["edge_std_ok"]:
    conf = params["conf_std_ok"]
  elif edge_std < params["edge_std_fair"]:
    conf = params["conf_std_fair"]
  else:
    conf = params["conf_std_poor"]
  if near_prob is not None and near_prob < params["near_prob_min"]:
    conf *= params["near_prob_penalty"]
  return n, conf


def multi_turn_count(hints, lat, lon, heading_deg, side, params=PARAMS):
  """Largest learned count for a same-side, ahead, same-heading hint near the car, or None."""
  if hints is None or lat is None or lon is None or heading_deg is None:
    return None
  best = None
  for h in hints:
    try:
      if h.get("side") != side:
        continue
      hlat, hlon = float(h["lat"]), float(h["lon"])
      h_heading, count = float(h["heading"]), int(h["count"])
    except (AttributeError, KeyError, TypeError, ValueError):
      continue
    if _dist_m(lat, lon, hlat, hlon) > params["hint_radius_m"]:
      continue
    if _angle_diff(_bearing_deg(lat, lon, hlat, hlon), heading_deg) > params["hint_heading_deg"]:
      continue
    if _angle_diff(h_heading, heading_deg) > params["hint_heading_deg"]:
      continue
    if best is None or count > best:
      best = count
  return best


def decide(side, mph, n, conf, turn_count, params=PARAMS):
  """(call, confidence, reasons) from the speed band, the lanes to cross and the multi-turn hint."""
  if mph is None or mph < MIN_MPH or mph > MAX_MPH:
    return "none", 0.0, ["out_of_band"]
  if n is None or conf is None or conf < params["lane_conf_min"]:
    return "abstain", conf if conf is not None else 0.0, ["lane_unknown"]
  if n >= 1 and turn_count is not None and turn_count >= 2 and n < turn_count:
    return "turn", conf * params["multi_turn_penalty"], ["in_multi_turn_lane"]
  if n >= 1:
    return "lane_change", conf, ["lanes_to_cross=%d" % n]
  return "turn", conf, ["outer_lane"]


def _load_hints(path):
  try:
    with open(path) as f:
      data = json.load(f)
  except (OSError, ValueError):
    return []
  if isinstance(data, dict):
    data = data.get("hints", [])
  if not isinstance(data, list):
    return []
  return [h for h in data if isinstance(h, dict)]


class BlinkerIntent:
  def __init__(self, gates=None, log_path=LOG_PATH, hints_path=HINTS_PATH, off_flag=OFF_FLAG):
    self.log_path = log_path
    self.hints_path = hints_path
    self.off_flag = off_flag
    self.history = deque(maxlen=HISTORY_FRAMES)
    self.last = None
    self.hints = _load_hints(hints_path)
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

  def observe(self, edge_l, edge_r, std_l, std_r, l1_y, l2_y, p1, p2):
    """One camera sample. Called every model frame."""
    self.history.append({
      "edge_l": _abs_or_none(edge_l), "edge_r": _abs_or_none(edge_r),
      "std_l": std_l, "std_r": std_r,
      "l1_y": l1_y, "l2_y": l2_y, "p1": p1, "p2": p2,
    })

  def _append(self, obj):
    try:
      d = os.path.dirname(self.log_path)
      if d:
        os.makedirs(d, exist_ok=True)
      with open(self.log_path, "a") as f:
        f.write(json.dumps(obj, separators=(",", ":")) + "\n")
    except OSError:
      pass

  def on_blinker(self, side, mph, lat=None, lon=None, heading=None, now=None):
    """Called once at blinker-on. Returns the record it logged."""
    samples = list(self.history)
    if side == "left":
      edges, stds, probs = [s["edge_l"] for s in samples], [s["std_l"] for s in samples], [s["p1"] for s in samples]
    else:
      edges, stds, probs = [s["edge_r"] for s in samples], [s["std_r"] for s in samples], [s["p2"] for s in samples]

    have_edge = [e for e in edges if e is not None]
    edge_m = statistics.median(have_edge) if len(have_edge) >= 5 else None
    edge_std = _median(stds)
    lane_w = _median([lane_width(s["l1_y"], s["l2_y"], s["p1"], s["p2"]) for s in samples])
    if lane_w is None:
      lane_w = PARAMS["lane_w_m"]
    near_prob = _median(probs)

    n, conf = lanes_toward(side, edge_m, edge_std, lane_w, near_prob)
    turn_count = multi_turn_count(self.hints, lat, lon, heading, side)
    call, conf, why = decide(side, mph, n, conf, turn_count)

    live = False
    speed = None
    if call == "lane_change" and self.gates is not None and not self._off:
      try:
        cond = self.gates.conditions(None, None, None, mph, None)
        if isinstance(cond, dict):
          speed = cond.get("speed")
        live = bool(self.gates.is_live(FEATURE, cond, conf))
      except Exception:
        live = False

    record = {
      "ts": now or time.time(), "mph": round(mph, 1) if isinstance(mph, (int, float)) else mph,
      "side": side, "n": n, "conf": round(conf, 3), "call": call, "why": why,
      "turn_lanes": turn_count, "live": live, "act": live, "rule_version": 3,
    }
    self._append(record)
    if _log_decision is not None:
      try:
        _log_decision(FEATURE, call, conf,
                      ctx={"speed": speed, "side": side, "n": n, "why": why},
                      live=live, lat=lat, lon=lon)
      except Exception:
        pass
    self.last = record
    return record

  def on_cancel(self, reason, mph):
    """Called when an acted blinker change is cancelled: demote the feature for this cell and log it."""
    if not (self.last and self.last.get("act")):
      return
    if self.gates is not None:
      try:
        cond = self.gates.conditions(None, None, None, mph, None)
        self.gates.demote(FEATURE, cond, reason)
      except Exception:
        pass
    self._append({"ts": time.time(), "event": "cancel", "reason": reason})
    self.last["act"] = False
