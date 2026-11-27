"""Tests for canopy/turn_release.py: the hold, the release, the outcome and the re-fight demote."""
import json

from openpilot.sunnypilot.canopy.turn_release import TurnRelease, FEATURE, MPH_TO_MS


class FakeGates:
  def __init__(self, live=False):
    self.live = live
    self.demotes = []

  def conditions(self, lat, lon, unix, mph, speed_limit_mph):
    return {"light": None, "road": None, "weather": None, "speed": "0-25"}

  def is_live(self, fid, cond, conf=1.0):
    return self.live

  def demote(self, fid, cell, reason=""):
    self.demotes.append((fid, cell, reason))


class BoomGates:
  def conditions(self, *a, **k):
    raise RuntimeError("boom")

  def is_live(self, *a, **k):
    raise RuntimeError("boom")

  def demote(self, *a, **k):
    raise RuntimeError("boom")


def make_tr(tmp_path, gates, t):
  return TurnRelease(gates=gates,
                     log_path=str(tmp_path / "canopy" / "turn_release.jsonl"),
                     decisions_path=str(tmp_path / "canopy" / "decisions.jsonl"),
                     off_flag=str(tmp_path / "turn_release.off"),
                     clock=lambda: t[0], wall=lambda: 1000.0 + t[0])


def drive(tr, t, seconds, step=0.05, **state):
  out = []
  for _ in range(int(round(seconds / step))):
    t[0] += step
    out.append(tr.update(state.get("v_ego", 0.0), state.get("left", False), state.get("right", False),
                         state.get("pressed", False), state.get("angle", 0.0),
                         state.get("curv", 0.0), state.get("lat", True)))
  return out


def rows(path):
  if not path.exists():
    return []
  return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def test_shadow_never_releases(tmp_path):
  t = [0.0]
  tr = make_tr(tmp_path, FakeGates(live=False), t)
  res = drive(tr, t, 2.0, v_ego=8 * MPH_TO_MS, left=True, pressed=True, angle=120.0, curv=0.0, lat=True)
  assert not any(res)
  res = drive(tr, t, 1.5, v_ego=8 * MPH_TO_MS, angle=5.0, curv=0.0, lat=True)
  assert not any(res)

  eps = rows(tmp_path / "canopy" / "turn_release.jsonl")
  assert len(eps) == 1
  assert eps[0]["event"] == "episode"
  assert eps[0]["outcome"] == "clean"
  assert eps[0]["live"] is False

  dec = rows(tmp_path / "canopy" / "decisions.jsonl")
  assert len(dec) == 1
  assert dec[0]["feature"] == "turn_release"
  assert dec[0]["outcome"] == "clean"
  assert dec[0]["live"] is False
  assert dec[0]["scored_by"] == "canopy.turn_release"


def test_live_releases_after_hold(tmp_path):
  t = [0.0]
  tr = make_tr(tmp_path, FakeGates(live=True), t)
  state = dict(v_ego=8 * MPH_TO_MS, left=True, pressed=True, angle=100.0, curv=0.0, lat=True)
  assert not any(drive(tr, t, 0.45, **state))
  assert drive(tr, t, 0.15, **state)[-1] is True

  quiet = dict(v_ego=8 * MPH_TO_MS, angle=5.0, curv=0.0, lat=True)
  assert drive(tr, t, 1.1, **quiet)[-1] is False


def test_brush_is_near(tmp_path):
  t = [0.0]
  tr = make_tr(tmp_path, FakeGates(live=False), t)
  drive(tr, t, 0.6, v_ego=8 * MPH_TO_MS, right=True, pressed=True, angle=70.0, curv=0.0, lat=True)
  drive(tr, t, 1.0, v_ego=8 * MPH_TO_MS, right=True, angle=70.0, curv=0.0, lat=True)
  drive(tr, t, 1.5, v_ego=8 * MPH_TO_MS, angle=5.0, curv=0.0, lat=True)

  eps = rows(tmp_path / "canopy" / "turn_release.jsonl")
  assert len(eps) == 1
  assert eps[0]["outcome"] == "near"


def test_fast_does_nothing(tmp_path):
  t = [0.0]
  tr = make_tr(tmp_path, FakeGates(live=True), t)
  assert not any(drive(tr, t, 2.0, v_ego=30 * MPH_TO_MS, left=True, pressed=True, angle=120.0, curv=0.0, lat=True))
  assert rows(tmp_path / "canopy" / "turn_release.jsonl") == []


def test_refight_demotes(tmp_path):
  t = [0.0]
  gates = FakeGates(live=True)
  tr = make_tr(tmp_path, gates, t)
  drive(tr, t, 1.0, v_ego=8 * MPH_TO_MS, left=True, pressed=True, angle=120.0, curv=0.0, lat=True)
  drive(tr, t, 1.5, v_ego=8 * MPH_TO_MS, angle=5.0, curv=0.0, lat=True)
  drive(tr, t, 1.0, v_ego=8 * MPH_TO_MS, pressed=True, angle=55.0, curv=0.0, lat=True)

  assert len(gates.demotes) == 1
  assert gates.demotes[0][0] == FEATURE
  assert gates.demotes[0][2] == "refight"


def test_next_turn_is_not_refight(tmp_path):
  t = [0.0]
  gates = FakeGates(live=True)
  tr = make_tr(tmp_path, gates, t)
  drive(tr, t, 1.0, v_ego=8 * MPH_TO_MS, left=True, pressed=True, angle=120.0, curv=0.0, lat=True)
  drive(tr, t, 1.5, v_ego=8 * MPH_TO_MS, angle=5.0, curv=0.0, lat=True)
  drive(tr, t, 1.0, v_ego=8 * MPH_TO_MS, left=True, pressed=True, angle=120.0, curv=0.0, lat=True)

  assert gates.demotes == []


def test_small_nudge_is_not_refight(tmp_path):
  t = [0.0]
  gates = FakeGates(live=True)
  tr = make_tr(tmp_path, gates, t)
  drive(tr, t, 1.0, v_ego=8 * MPH_TO_MS, left=True, pressed=True, angle=120.0, curv=0.0, lat=True)
  drive(tr, t, 1.5, v_ego=8 * MPH_TO_MS, angle=5.0, curv=0.0, lat=True)
  drive(tr, t, 1.0, v_ego=8 * MPH_TO_MS, pressed=True, angle=20.0, curv=0.0, lat=True)

  assert gates.demotes == []


def test_conf_model_disagrees(tmp_path):
  def run(tmp_path, curv):
    t = [0.0]
    tr = make_tr(tmp_path, FakeGates(live=False), t)
    drive(tr, t, 1.0, v_ego=8 * MPH_TO_MS, left=True, pressed=True, angle=120.0, curv=curv, lat=True)
    drive(tr, t, 1.5, v_ego=8 * MPH_TO_MS, angle=5.0, curv=curv, lat=True)
    dec = rows(tmp_path / "canopy" / "decisions.jsonl")
    assert len(dec) == 1
    return dec[0]["conf"]

  assert run(tmp_path / "a", 0.05) == 1.0   # model turning the other way: he is on his own
  assert run(tmp_path / "b", -0.05) == 0.6  # model agrees with the wheel


def test_update_never_raises(tmp_path):
  t = [0.0]
  tr = TurnRelease(gates=BoomGates(),
                   log_path=str(tmp_path / "canopy" / "turn_release.jsonl"),
                   decisions_path=str(tmp_path / "canopy" / "decisions.jsonl"),
                   off_flag=str(tmp_path / "turn_release.off"),
                   clock=lambda: t[0], wall=lambda: 1000.0 + t[0])
  res = drive(tr, t, 2.0, v_ego=8 * MPH_TO_MS, left=True, pressed=True, angle=120.0, curv=0.0, lat=True)
  assert all(isinstance(r, bool) for r in res)
  res = drive(tr, t, 1.5, v_ego=8 * MPH_TO_MS, angle=5.0, curv=0.0, lat=True)
  assert all(isinstance(r, bool) for r in res)
