"""Tests for canopy/turn_path.py: the shadow record, the head stamp, the takeover demote,
and that nothing about steering comes out of it."""
import json

from openpilot.sunnypilot.canopy import decision_log, heads, turn_path
from openpilot.sunnypilot.canopy.turn_path import FEATURE, MPH_TO_MS, TurnPath, record, would_be_path


class FakeHeads:
  def __init__(self, conf=0.62, head=None, base="base0001", shadow=None):
    self.conf = conf
    self.live = head
    self.base_id = base
    self.shadow_out = shadow or {}
    self.scores = []
    self.rollbacks = []

  def score(self, feature, state, ctx, lat, lon, heading):
    self.scores.append((feature, state, ctx, lat, lon, heading))
    return {"head": self.live, "conf": self.conf, "base": self.base_id, "shadow": dict(self.shadow_out)}

  def rollback(self, feature, boundary_ok):
    self.rollbacks.append((feature, boundary_ok))
    return None


class FakeGates:
  def __init__(self):
    self.demotes = []

  def conditions(self, lat, lon, unix, mph, speed_limit_mph):
    return {"light": None, "road": None, "weather": None, "speed": "0-25"}

  def demote(self, fid, cell, reason=""):
    self.demotes.append((fid, cell, reason))


class BoomHeads:
  def score(self, *a, **k):
    raise RuntimeError("boom")

  def rollback(self, *a, **k):
    raise RuntimeError("boom")


def make_tp(tmp_path, heads_=None, gates=None, t=None):
  t = t if t is not None else [0.0]
  return TurnPath(heads=heads_, gates=gates,
                  log_path=str(tmp_path / "canopy" / "turn_path.jsonl"),
                  off_flag=str(tmp_path / "turn_path.off"),
                  clock=lambda: t[0], wall=lambda: 1000.0 + t[0]), t


def rows(path):
  try:
    with open(path) as f:
      return [json.loads(line) for line in f if line.strip()]
  except OSError:
    return []


def drive(tp, t, secs, mph=8.0, left=True, right=False, pressed=False, angle=0.0, lat_active=True):
  """Feed frames at 20 Hz for secs, advancing the fake clock."""
  for _ in range(int(secs * 20)):
    t[0] += 0.05
    tp.update(mph * MPH_TO_MS, left, right, pressed, angle, lat_active, -0.02)


def test_baseline_path_is_earlier_and_wider_the_slower_the_turn():
  slow_lead, slow_target = would_be_path(5.0)
  fast_lead, fast_target = would_be_path(18.0)
  assert slow_lead > fast_lead
  assert slow_target < fast_target
  assert turn_path.LEAD_MIN_S <= slow_lead <= turn_path.LEAD_MAX_S
  assert turn_path.TARGET_MIN_DEG <= fast_target <= turn_path.TARGET_MAX_DEG


def test_record_shape_carries_no_steer_command():
  rec = record({"ts": 1.0, "mph": 8.0, "side": "left", "peak_deg": 210.0, "press_s": 0.0,
                "outcome": "clean", "conf": 0.4}, "00000016--bff201bfb5")
  for key in ("type", "event", "drive", "side", "mph", "lead_s", "target_deg", "conf", "head", "base",
              "shadow", "outcome", "live", "act", "rule_version", "scored_by"):
    assert key in rec
  assert rec["live"] is False and rec["act"] is False
  assert not any("desired" in k or "actuator" in k for k in rec)


def test_slow_left_blinker_opens_and_closes_one_record(tmp_path):
  hs = FakeHeads()
  tp, t = make_tp(tmp_path, hs)
  drive(tp, t, 1.0, mph=8.0, left=True, angle=5.0)
  assert tp.episode is not None
  drive(tp, t, 1.0, mph=9.0, left=True, angle=180.0)
  drive(tp, t, 3.0, mph=9.0, left=False, angle=10.0)
  got = rows(tmp_path / "canopy" / "turn_path.jsonl")
  assert len(got) == 1
  assert got[0]["side"] == "left"
  assert got[0]["outcome"] == "clean"
  assert got[0]["peak_deg"] >= 180.0
  assert got[0]["live"] is False and got[0]["act"] is False


def test_right_turn_is_not_recorded(tmp_path):
  tp, t = make_tp(tmp_path, FakeHeads())
  drive(tp, t, 2.0, mph=8.0, left=False, right=True, angle=5.0)
  assert tp.episode is None
  assert rows(tmp_path / "canopy" / "turn_path.jsonl") == []


def test_no_record_above_the_slow_band(tmp_path):
  tp, t = make_tp(tmp_path, FakeHeads())
  drive(tp, t, 3.0, mph=26.0, left=True, angle=5.0)
  assert tp.episode is None
  assert rows(tmp_path / "canopy" / "turn_path.jsonl") == []


def test_head_stamp_goes_on_the_record_and_the_decision_row(tmp_path, monkeypatch):
  monkeypatch.setattr(decision_log, "PATH", str(tmp_path / "canopy" / "decisions.jsonl"))
  monkeypatch.setattr(decision_log, "_last", {})
  hs = FakeHeads(conf=0.71, head=None, base="base0001", shadow={"turn_path@abc12345": 0.71})
  tp, t = make_tp(tmp_path, hs)
  drive(tp, t, 1.0, mph=8.0, left=True, angle=5.0)
  drive(tp, t, 1.0, mph=8.0, left=True, angle=180.0)
  drive(tp, t, 3.0, mph=8.0, left=False, angle=5.0)
  got = rows(tmp_path / "canopy" / "turn_path.jsonl")[0]
  assert got["base"] == "base0001"
  assert got["shadow"] == {"turn_path@abc12345": 0.71}
  assert got["conf"] == 0.71
  assert hs.scores[0][0] == FEATURE
  dec = rows(tmp_path / "canopy" / "decisions.jsonl")
  assert dec and dec[0]["feature"] == FEATURE
  assert dec[0]["live"] is False
  assert dec[0]["base"] == "base0001"
  assert dec[0]["shadow"] == {"turn_path@abc12345": 0.71}


def test_takeover_is_the_label_and_demotes(tmp_path):
  gates = FakeGates()
  hs = FakeHeads(conf=0.9)
  tp, t = make_tp(tmp_path, hs, gates)
  drive(tp, t, 1.0, mph=7.0, left=True, angle=5.0)
  drive(tp, t, 1.0, mph=7.0, left=True, angle=180.0)
  drive(tp, t, 1.0, mph=7.0, left=True, pressed=True, angle=200.0)
  drive(tp, t, 3.0, mph=7.0, left=False, pressed=True, angle=5.0)
  got = rows(tmp_path / "canopy" / "turn_path.jsonl")[0]
  assert got["outcome"] == "takeover"
  assert got["press_s"] >= turn_path.HOLD_S
  assert gates.demotes and gates.demotes[0][0] == FEATURE
  assert hs.rollbacks == [(FEATURE, True)]


def test_off_flag_stops_everything(tmp_path):
  (tmp_path / "turn_path.off").write_text("1")
  tp, t = make_tp(tmp_path, FakeHeads())
  drive(tp, t, 2.0, mph=8.0, left=True, angle=5.0)
  assert tp.episode is None
  assert rows(tmp_path / "canopy" / "turn_path.jsonl") == []


def test_broken_heads_never_raise_and_still_log(tmp_path):
  tp, t = make_tp(tmp_path, BoomHeads(), FakeGates())
  drive(tp, t, 1.0, mph=8.0, left=True, angle=5.0)
  drive(tp, t, 1.0, mph=8.0, left=True, angle=180.0)
  drive(tp, t, 3.0, mph=8.0, left=False, angle=5.0)
  got = rows(tmp_path / "canopy" / "turn_path.jsonl")
  assert len(got) == 1
  assert got[0]["head"] is None


def test_update_returns_nothing(tmp_path):
  tp, t = make_tp(tmp_path, FakeHeads())
  t[0] += 0.05
  assert tp.update(8.0 * MPH_TO_MS, True, False, False, 5.0, True, -0.02) is None


def test_turn_path_is_control_tier():
  assert heads.HeadSet.__module__ and heads.N_MIN_CONTROL == 20
  assert "turn_path" in heads.TIER_CONTROL
  hs = heads.HeadSet(root="/nonexistent-canopy-heads")
  assert hs.floor(FEATURE) == heads.N_MIN_CONTROL


class FakeSubMaster:
  """A SubMaster with no gpsLocationExternal, like controlsd's real one."""

  def __init__(self, services=()):
    self.services = set(services)

  def all_checks(self, names):
    return all(n in self.services for n in names)

  def __getitem__(self, name):
    if name not in self.services:
      raise KeyError(name)
    raise AssertionError("no service data in this fake")


def test_gps_pose_is_none_when_the_service_is_not_subscribed():
  assert turn_path.gps_pose(FakeSubMaster()) == (None, None, None)
  assert turn_path.gps_pose(None) == (None, None, None)


def test_gps_pose_returns_position_when_it_is_there():
  class GPS:
    latitude, longitude, bearingDeg = 33.4, -111.8, 90.0

  class Sm(FakeSubMaster):
    def __getitem__(self, name):
      return GPS()

    def all_checks(self, names):
      return True

  assert turn_path.gps_pose(Sm()) == (33.4, -111.8, 90.0)


def test_wiring_calls_update_even_without_gps(tmp_path):
  """The controlsd block, as written: gps_pose may fail, update must still run."""
  seen = []
  hs = FakeHeads()

  class Spy(TurnPath):
    def update(self, *a, **k):
      seen.append((a, k))
      return super().update(*a, **k)

  tp = Spy(heads=hs, gates=FakeGates(), log_path=str(tmp_path / "turn_path.jsonl"),
           off_flag=str(tmp_path / "off"), clock=lambda: 0.0, wall=lambda: 0.0)
  sm = FakeSubMaster()
  lat = lon = heading = None
  try:
    lat, lon, heading = turn_path.gps_pose(sm)
  except Exception:
    lat = lon = heading = None
  tp.update(8.0 * MPH_TO_MS, True, False, False, 5.0, True, -0.02, lat, lon, heading)
  assert len(seen) == 1
  assert seen[0][0][7:] == (None, None, None)
  assert tp.episode is not None
