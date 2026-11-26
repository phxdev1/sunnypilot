"""Tests for canopy/blinker_intent.py (pure logic and the stateful call)."""
import json
import math

from openpilot.sunnypilot.canopy import blinker_intent as bi
from openpilot.sunnypilot.canopy import gates


class FakeGates:
  def __init__(self, live=True):
    self.live = live
    self.demotes = []

  def conditions(self, lat, lon, unix, mph, speed_limit_mph):
    return {"light": None, "road": None, "weather": None, "speed": "35-45"}

  def is_live(self, fid, cond, conf=1.0):
    return self.live

  def demote(self, fid, cell, reason=""):
    self.demotes.append((fid, cell, reason))


def _fill(intent, frames=10):
  for _ in range(frames):
    intent.observe(-1.8, 7.2, 0.5, 0.5, -1.8, 1.8, 0.9, 0.9)


# ---- lanes_toward ----

def test_lanes_toward_one_lane_right():
  n, conf = bi.lanes_toward("right", 7.2, 0.5, 3.6, None)
  assert n == 1
  assert conf == 0.9


def test_lanes_toward_bike_lane_is_zero():
  n, _ = bi.lanes_toward("right", 4.8, 0.5, 3.6, None)
  assert n == 0


def test_lanes_toward_left_edge_too_far_unknown():
  assert bi.lanes_toward("left", 13.0, 0.5, 3.6, None) == (None, 0.0)


def test_lanes_toward_near_prob_penalty():
  n, conf = bi.lanes_toward("right", 7.2, 0.5, 3.6, 0.3)
  assert n == 1
  assert abs(conf - 0.9 * 0.8) < 1e-9


# ---- decide ----

def test_decide_out_of_band():
  assert bi.decide("right", 20.0, 1, 0.9, None) == ("none", 0.0, ["out_of_band"])
  assert bi.decide("right", 55.0, 1, 0.9, None) == ("none", 0.0, ["out_of_band"])


def test_decide_abstain_unknown_lane():
  call, _, why = bi.decide("right", 35.0, None, 0.0, None)
  assert call == "abstain"
  assert why == ["lane_unknown"]


def test_decide_outer_lane_is_turn():
  assert bi.decide("right", 35.0, 0, 0.9, None) == ("turn", 0.9, ["outer_lane"])


def test_decide_lane_change():
  assert bi.decide("right", 35.0, 1, 0.9, None) == ("lane_change", 0.9, ["lanes_to_cross=1"])


def test_decide_multi_turn_guard():
  call, conf, why = bi.decide("right", 35.0, 1, 0.9, 2)
  assert call == "turn"
  assert abs(conf - 0.9 * 0.8) < 1e-9
  assert why == ["in_multi_turn_lane"]


# ---- multi_turn_count ----

_LAT = 33.4
_M_PER_DEG_LON = 111320.0 * math.cos(math.radians(_LAT))
_DLON_100M = 100.0 / _M_PER_DEG_LON


def test_multi_turn_ahead_matches():
  hint = {"lat": _LAT, "lon": _DLON_100M, "heading": 90.0, "side": "right", "count": 3}
  assert bi.multi_turn_count([hint], _LAT, 0.0, 90.0, "right") == 3


def test_multi_turn_behind_no_match():
  hint = {"lat": _LAT, "lon": -_DLON_100M, "heading": 90.0, "side": "right", "count": 3}
  assert bi.multi_turn_count([hint], _LAT, 0.0, 90.0, "right") is None


def test_multi_turn_wrong_side_no_match():
  hint = {"lat": _LAT, "lon": _DLON_100M, "heading": 90.0, "side": "left", "count": 3}
  assert bi.multi_turn_count([hint], _LAT, 0.0, 90.0, "right") is None


def test_multi_turn_wrong_heading_no_match():
  hint = {"lat": _LAT, "lon": _DLON_100M, "heading": 180.0, "side": "right", "count": 3}
  assert bi.multi_turn_count([hint], _LAT, 0.0, 90.0, "right") is None


def test_multi_turn_none_inputs():
  assert bi.multi_turn_count([], None, 0.0, 90.0, "right") is None


# ---- BlinkerIntent ----

def test_acts_only_when_lane_change_and_live(tmp_path):
  log = tmp_path / "blinker.jsonl"
  hints = tmp_path / "hints.json"
  off = tmp_path / "off"
  gates = FakeGates(live=True)
  intent = bi.BlinkerIntent(gates=gates, log_path=str(log), hints_path=str(hints), off_flag=str(off))
  _fill(intent)
  rec = intent.on_blinker("right", 35.0)
  assert rec["call"] == "lane_change"
  assert rec["act"] is True
  assert intent.last is rec


def test_no_act_on_abstain_even_when_live(tmp_path):
  log = tmp_path / "blinker.jsonl"
  hints = tmp_path / "hints.json"
  off = tmp_path / "off"
  gates = FakeGates(live=True)
  intent = bi.BlinkerIntent(gates=gates, log_path=str(log), hints_path=str(hints), off_flag=str(off))
  # no history: edge unknown -> abstain
  rec = intent.on_blinker("right", 35.0)
  assert rec["call"] == "abstain"
  assert rec["act"] is False


def test_off_flag_blocks_act(tmp_path):
  log = tmp_path / "blinker.jsonl"
  hints = tmp_path / "hints.json"
  off = tmp_path / "off"
  off.write_text("")
  gates = FakeGates(live=True)
  intent = bi.BlinkerIntent(gates=gates, log_path=str(log), hints_path=str(hints), off_flag=str(off))
  _fill(intent)
  rec = intent.on_blinker("right", 35.0)
  assert rec["call"] == "lane_change"
  assert rec["act"] is False


def test_on_cancel_demotes_once(tmp_path):
  log = tmp_path / "blinker.jsonl"
  hints = tmp_path / "hints.json"
  off = tmp_path / "off"
  gates = FakeGates(live=True)
  intent = bi.BlinkerIntent(gates=gates, log_path=str(log), hints_path=str(hints), off_flag=str(off))
  _fill(intent)
  intent.on_blinker("right", 35.0)
  assert gates.demotes == []
  intent.on_cancel("torque_against", 35.0)
  assert len(gates.demotes) == 1
  assert gates.demotes[0][0] == bi.FEATURE
  intent.on_cancel("torque_against", 35.0)
  assert len(gates.demotes) == 1


def test_every_on_blinker_writes_one_line(tmp_path):
  log = tmp_path / "blinker.jsonl"
  hints = tmp_path / "hints.json"
  off = tmp_path / "off"
  gates = FakeGates(live=True)
  intent = bi.BlinkerIntent(gates=gates, log_path=str(log), hints_path=str(hints), off_flag=str(off))
  _fill(intent)
  for _ in range(3):
    intent.on_blinker("right", 35.0)
  lines = [ln for ln in log.read_text().splitlines() if ln.strip()]
  assert len(lines) == 3
  for ln in lines:
    assert json.loads(ln)["call"] in ("lane_change", "turn", "abstain", "none")


def test_calls_decision_logger_once_per_blinker(tmp_path, monkeypatch):
  calls = []

  def recorder(*args, **kwargs):
    calls.append((args, kwargs))

  monkeypatch.setattr(bi, "_log_decision", recorder)
  log = tmp_path / "blinker.jsonl"
  hints = tmp_path / "hints.json"
  off = tmp_path / "off"
  gates = FakeGates(live=True)
  intent = bi.BlinkerIntent(gates=gates, log_path=str(log), hints_path=str(hints), off_flag=str(off))
  _fill(intent)
  for _ in range(3):
    intent.on_blinker("right", 35.0)
  assert len(calls) == 3
  for args, _kwargs in calls:
    assert args[0] == "blinker_intent"


def test_real_gates_matches_exported_speed_band_cell(tmp_path, monkeypatch):
  """A real Gates loaded from a gates.json whose blinker_intent cell is keyed "speed_band=35-45"
  goes live at 40 mph and stays shadow at 30 mph."""
  gates_path = tmp_path / "gates.json"
  demoted_path = tmp_path / "gates_demoted.json"
  gates_path.write_text(json.dumps({
    "generated": 1000.0,
    "features": {
      "blinker_intent": {
        "speed_band=35-45": {"status": "live", "tier": "control", "cstar": 0.5},
        "_": {"status": "shadow", "tier": "control", "cstar": 1.0},
      },
    },
    "names": {"blinker_intent": "Blinker intent"},
    "combos": {},
  }))
  monkeypatch.setattr(gates, "GATES_PATH", str(gates_path))
  monkeypatch.setattr(gates, "DEMOTED_PATH", str(demoted_path))

  g = gates.Gates()
  g.snapshot()

  log = tmp_path / "blinker.jsonl"
  hints = tmp_path / "hints.json"
  off = tmp_path / "off"
  intent = bi.BlinkerIntent(gates=g, log_path=str(log), hints_path=str(hints), off_flag=str(off))
  _fill(intent)

  fast = intent.on_blinker("right", 40.0)
  assert fast["call"] == "lane_change"
  assert fast["conf"] == 0.9
  assert fast["act"] is True
  assert fast["live"] is True

  slow = intent.on_blinker("right", 30.0)
  assert slow["call"] == "lane_change"
  assert slow["act"] is False
  assert slow["live"] is False

