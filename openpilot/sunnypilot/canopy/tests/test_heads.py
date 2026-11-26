"""Tests for canopy/heads.py: the copies, the sha rule, boundaries, promote, rollback,
staging, and the head stamp in the decision log."""
import hashlib
import io
import json
import math
import os

import pytest

from openpilot.sunnypilot.canopy import decision_log, heads, leadkind


# ---------------------------------------------------------------------------
# fixtures and small builders
# ---------------------------------------------------------------------------

@pytest.fixture
def root(tmp_path, monkeypatch):
  r = tmp_path / "heads"
  (r / "active").mkdir(parents=True)
  (r / "staged").mkdir()
  (r / "last_good").mkdir()
  monkeypatch.setattr(heads, "ROOT", str(r))
  return r


def _write(path, obj):
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(json.dumps(obj))


def _base():
  # in (14) -> 1 tanh -> 1 tanh: h1 = tanh(x[0]), h2 = tanh(2 h1)
  W1 = [[1.0 if i == 0 else 0.0] for i in range(len(heads.IN_KEYS))]
  weights = json.dumps({"W1": W1, "b1": [0.0], "W2": [[2.0]], "b2": [0.0]},
                       sort_keys=True, separators=(",", ":"))
  sha = hashlib.sha256(weights.encode("utf-8")).hexdigest()
  return {"base_id": "base_test", "W1": W1, "b1": [0.0], "W2": [[2.0]], "b2": [0.0],
          "in_keys": list(heads.IN_KEYS), "sha": sha, "version": sha[:8]}


def _head(base_sha, feature="corner_pass", w=None, b=-0.25, inputs="emb"):
  w = [1.5] if w is None else list(w)
  h = {"feature": feature, "base_id": "base_test", "base_sha": base_sha, "inputs": inputs,
       "w": w, "b": b, "n": 40, "pos": 5, "prior_only": False}
  h["version"] = heads.head_version(w, b, base_sha)
  h["head_id"] = f"{feature}@{h['version']}"
  return h


def _setup(root, head=None):
  """base in active/, plus optionally a live head for its feature."""
  b = _base()
  _write(root / "active" / "base.json", b)
  if head is not None:
    _write(root / "active" / f"{head['feature']}.json", head)
  return b


# ---------------------------------------------------------------------------
# the copies
# ---------------------------------------------------------------------------

def test_predict_py_parity_on_a_tiny_base(root):
  b = _setup(root)
  h = _head(b["sha"])
  state = {"mph": 40.0, "gas": True, "set_delta": 3.0}
  ctx = "light=night|road=freeway|speed=25-50"

  x = heads.state_vec(state, ctx)
  assert len(x) == 14
  assert x == [0.5, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0]

  e = heads.embed(b, x)
  assert len(e) == 1
  assert round(e[0], 12) == round(math.tanh(2.0 * math.tanh(0.5)), 12)

  z = 1.5 * math.tanh(2.0 * math.tanh(0.5)) - 0.25
  assert heads.predict_py(b, h, x, heads.kg_features([], None, None, None)) == pytest.approx(
    1.0 / (1.0 + math.exp(-z)), abs=1e-12)


def test_predict_py_parity_on_a_kg_head(root):
  b = _setup(root)
  w = [0.5] + [0.1 * i for i in range(len(heads.KG_KEYS) - 1)]
  h = _head(b["sha"], w=w, b=0.5, inputs="kg")
  kg = dict.fromkeys(heads.KG_KEYS, 0.0)
  kg["kg_takeovers"] = 1.0

  z = 0.5 * 1.0 + 0.5
  assert heads.predict_py(b, h, heads.state_vec({}, ""), kg) == pytest.approx(
    1.0 / (1.0 + math.exp(-z)), abs=1e-12)


def test_state_vec_and_kg_features_defaults():
  assert heads.state_vec(None, None) == [0.0] * 9 + [0.0, 0.0, 1.0, 0.0, 0.0]
  assert heads.kg_features([], None, None, None) == dict.fromkeys(heads.KG_KEYS, 0.0)


def test_kg_cache_recomputes_only_past_25_m_or_20_deg():
  item = {"k": "takeover", "lat": 33.0, "lon": -111.0, "h": 0.0, "drive": "d1", "x": {}}
  cache = heads.KgCache([item])
  first = cache.features(33.0, -111.0, 0.0)
  assert first["kg_takeovers"] > 0.0

  near = cache.features(33.0 + 0.0001, -111.0, 10.0)  # ~11 m, 10 deg: cached
  assert near is first
  turned = cache.features(33.0 + 0.0001, -111.0, 25.0)  # 25 deg: recomputed
  assert turned["kg_takeovers"] > 0.0
  far = cache.features(33.002, -111.0, 25.0)  # ~222 m: out of range
  assert far["kg_takeovers"] == 0.0


# ---------------------------------------------------------------------------
# the version rule
# ---------------------------------------------------------------------------

def test_head_version_fixed_vector():
  canon = "w=0.500000,-0.250000,0.000000|b=-1.000000|base=abc"
  w, b = [0.5, -0.25, 1e-7], -1.0
  assert heads.head_canon(w, b, "abc") == canon
  assert heads.head_version(w, b, "abc") == hashlib.sha256(canon.encode("utf-8")).hexdigest()[:8]


def test_head_version_folds_negative_zero():
  assert heads.head_canon([-0.0, -1e-9], -1e-9, "abc") == "w=0.000000,0.000000|b=0.000000|base=abc"


def test_staged_head_is_stored_at_six_decimals_and_rehashes(root):
  b = _setup(root)
  h = _head(b["sha"], w=[0.1234567891], b=-0.0000004)
  heads.stage_head(h, root=str(root))
  stored = json.loads((root / "staged" / "corner_pass.json").read_text())
  assert stored["w"] == [0.123457]
  assert stored["b"] == 0.0
  assert stored["version"] == heads.head_version(stored["w"], stored["b"], b["sha"])
  assert heads.head_error(stored, b["sha"]) is None


# ---------------------------------------------------------------------------
# load and the sha rule
# ---------------------------------------------------------------------------

def test_load_reads_live_and_shadow(root):
  b = _setup(root, _head(_base()["sha"]))
  shadow = _head(b["sha"], w=[0.5], b=0.1)
  _write(root / "staged" / "corner_pass.json", shadow)

  hs = heads.HeadSet(root=str(root))
  assert hs.errors == []
  assert hs.base_id == "base_test"
  assert hs.live["corner_pass"]["version"] == heads.head_version([1.5], -0.25, b["sha"])
  assert hs.shadow["corner_pass"]["version"] == shadow["version"]
  assert hs.live_ids() == {"corner_pass": f"corner_pass@{hs.live['corner_pass']['version']}"}
  assert hs.staged_ids() == [f"corner_pass@{shadow['version']}"]


def test_tampered_head_is_ignored_and_named(root):
  b = _setup(root)
  good = _head(b["sha"])
  tampered = dict(good)
  tampered["w"] = [9.0]  # version no longer matches w and b
  _write(root / "active" / "corner_pass.json", tampered)

  hs = heads.HeadSet(root=str(root))
  assert "corner_pass" not in hs.live
  assert any("version" in e for e in hs.errors)


def test_head_for_another_base_is_ignored(root):
  _setup(root)
  _write(root / "active" / "corner_pass.json", _head("0" * 64))
  hs = heads.HeadSet(root=str(root))
  assert hs.live == {}
  assert any("base_sha" in e for e in hs.errors)


def test_score_reports_live_and_shadow(root):
  b = _setup(root, _head(_base()["sha"]))
  shadow = _head(b["sha"], w=[0.5], b=0.1)
  _write(root / "staged" / "corner_pass.json", shadow)
  hs = heads.HeadSet(root=str(root))

  out = hs.score("corner_pass", {"mph": 40.0}, "light=day", 33.0, -111.0, 90.0)
  assert out["base"] == "base_test"
  assert out["head"] == f"corner_pass@{heads.head_version([1.5], -0.25, b['sha'])}"
  assert 0.0 < out["conf"] < 1.0
  assert list(out["shadow"]) == [f"corner_pass@{shadow['version']}"]
  assert 0.0 < out["shadow"][f"corner_pass@{shadow['version']}"] < 1.0

  missing = hs.score("nope", {}, "", None, None, None)
  assert missing == {"head": None, "conf": None, "base": "base_test", "shadow": {}}


# ---------------------------------------------------------------------------
# maneuver_boundary
# ---------------------------------------------------------------------------

def _boundary(**kw):
  args = dict(blinker_on=False, lane_change_active=False, lane_turn_active=False,
              steer_deg=0.0, secs_since_feature_decision=5.0, gear=2)
  args.update(kw)
  return heads.maneuver_boundary(**args)


def test_maneuver_boundary_park_is_always_one():
  assert _boundary(gear=1, blinker_on=True, steer_deg=40.0, secs_since_feature_decision=0.0)


@pytest.mark.parametrize("kw", [
  {"blinker_on": True},
  {"lane_change_active": True},
  {"lane_turn_active": True},
  {"steer_deg": 15.0},
  {"steer_deg": -15.0},
  {"secs_since_feature_decision": 2.9},
])
def test_maneuver_boundary_refuses_while_busy(kw):
  assert not _boundary(**kw)


def test_maneuver_boundary_clear_road():
  assert _boundary(steer_deg=-14.9, secs_since_feature_decision=3.0)
  assert not _boundary(steer_deg=None)


# ---------------------------------------------------------------------------
# promote
# ---------------------------------------------------------------------------

def _candidate(root, live_sha):
  live = _head(live_sha, w=[1.5], b=-0.25)
  cand = _head(live_sha, w=[0.4], b=0.2)
  _write(root / "active" / "corner_pass.json", live)
  _write(root / "staged" / "corner_pass.json", cand)
  return live, cand


def _stats(**kw):
  s = {"n": 40, "ll_new": 0.30, "ll_live": 0.35, "takeovers_on_disagree": 0}
  s.update(kw)
  return s


def test_promote_needs_a_boundary(root):
  b = _setup(root)
  live, cand = _candidate(root, b["sha"])
  hs = heads.HeadSet(root=str(root))
  assert hs.promote("corner_pass", False, _stats()) is None
  assert hs.live["corner_pass"]["version"] == live["version"]


def test_promote_floor_and_ll_rule(root):
  b = _setup(root)
  live, cand = _candidate(root, b["sha"])
  hs = heads.HeadSet(root=str(root))

  assert hs.promote("corner_pass", True, _stats(n=9)) is None
  assert hs.promote("corner_pass", True, _stats(ll_new=0.40)) is None
  assert hs.promote("corner_pass", True, _stats(takeovers_on_disagree=1)) is None
  assert hs.promote("corner_pass", True, {"n": 40}) is None  # missing stats
  assert hs.live["corner_pass"]["version"] == live["version"]

  assert hs.promote("corner_pass", True, _stats(n=10)) == f"corner_pass@{cand['version']}"
  assert hs.live["corner_pass"]["version"] == cand["version"]
  assert hs.last_good["corner_pass"]["version"] == live["version"]
  assert not (root / "staged" / "corner_pass.json").exists()
  assert json.loads((root / "active" / "corner_pass.json").read_text())["version"] == cand["version"]
  assert json.loads((root / "last_good" / "corner_pass.json").read_text())["version"] == live["version"]
  assert heads.HeadSet(root=str(root)).live["corner_pass"]["version"] == cand["version"]


def test_promote_control_tier_needs_twenty(root):
  b = _setup(root)
  live, cand = _candidate(root, b["sha"])
  hs = heads.HeadSet(root=str(root))
  ctl = _stats(n=19, tier="control")
  assert hs.promote("corner_pass", True, ctl) is None
  assert hs.promote("corner_pass", True, _stats(n=20, tier="control")) == f"corner_pass@{cand['version']}"


def test_control_tier_from_tiers_file(root):
  b = _setup(root)
  _candidate(root, b["sha"])
  _write(root / "tiers.json", {"corner_pass": "control"})
  hs = heads.HeadSet(root=str(root))
  assert hs.floor("corner_pass") == heads.N_MIN_CONTROL
  assert hs.floor("something_else") == heads.N_MIN


# ---------------------------------------------------------------------------
# rollback
# ---------------------------------------------------------------------------

def _with_previous(root):
  b = _setup(root)
  old = _head(b["sha"], w=[0.2], b=0.0)
  bad = _head(b["sha"], w=[3.0], b=1.0)
  _write(root / "active" / "corner_pass.json", bad)
  _write(root / "last_good" / "corner_pass.json", old)
  return old, bad


def test_rollback_is_queued_until_a_boundary(root):
  old, bad = _with_previous(root)
  hs = heads.HeadSet(root=str(root))
  assert hs.live["corner_pass"]["version"] == bad["version"]

  assert hs.rollback("corner_pass", False) is None
  assert hs.pending_rollback == {"corner_pass"}
  assert json.loads((root / "active" / "corner_pass.json").read_text())["version"] == bad["version"]
  assert heads.read_bad(str(root)) == {}

  assert hs.rollback("corner_pass", True) == f"corner_pass@{old['version']}"
  assert hs.pending_rollback == set()
  assert hs.live["corner_pass"]["version"] == old["version"]
  assert json.loads((root / "active" / "corner_pass.json").read_text())["version"] == old["version"]
  assert heads.read_bad(str(root)) == {"corner_pass": [bad["version"]]}


def test_pending_rollback_flushes_on_another_boundary_call(root):
  old, bad = _with_previous(root)
  hs = heads.HeadSet(root=str(root))
  hs.rollback("corner_pass", False)
  assert hs.rollback("some_other", True) is None
  assert hs.live["corner_pass"]["version"] == old["version"]
  assert heads.read_bad(str(root)) == {"corner_pass": [bad["version"]]}


def test_bad_json_blocks_restaging_and_reload(root):
  old, bad = _with_previous(root)
  hs = heads.HeadSet(root=str(root))
  hs.rollback("corner_pass", True)

  with pytest.raises(ValueError):
    heads.stage_head(bad, root=str(root))
  _write(root / "staged" / "corner_pass.json", bad)  # even a file dropped in by hand
  again = heads.HeadSet(root=str(root))
  assert "corner_pass" not in again.shadow
  assert any("bad" in e for e in again.errors)
  assert again.live["corner_pass"]["version"] == old["version"]


# ---------------------------------------------------------------------------
# staging
# ---------------------------------------------------------------------------

def test_stage_head_writes_only_staged(root):
  b = _setup(root)
  h = _head(b["sha"])
  hid = heads.stage_head(h, root=str(root))
  assert hid == f"corner_pass@{h['version']}"
  assert json.loads((root / "staged" / "corner_pass.json").read_text())["version"] == h["version"]
  assert not (root / "active" / "corner_pass.json").exists()


def test_stage_head_rejects_a_bad_feature_name(root):
  b = _setup(root)
  h = _head(b["sha"])
  h["feature"] = "../x"
  with pytest.raises(ValueError):
    heads.stage_head(h, root=str(root))
  assert not (root / "staged" / "../x.json").exists()


# ---------------------------------------------------------------------------
# decision_log stamps
# ---------------------------------------------------------------------------

@pytest.fixture
def log_path(tmp_path, monkeypatch):
  path = tmp_path / "canopy" / "decisions.jsonl"
  monkeypatch.setattr(decision_log, "PATH", str(path))
  monkeypatch.setattr(decision_log, "_last", {})
  monkeypatch.setattr(leadkind, "route_name", lambda: "drive-1")
  return path


def test_log_decision_unchanged_without_head_args(log_path):
  decision_log.log_decision("corner_pass", "take", 0.4, ctx={"a": 1}, live=True, lat=33.0, lon=-111.0)
  rec = json.loads(log_path.read_text().splitlines()[0])
  assert set(rec) == {"type", "feature", "action", "conf", "drive", "ts", "live", "ctx", "lat", "lon"}


def test_log_decision_stamped_with_head(log_path):
  decision_log.log_decision("corner_pass", "take", 0.4, head="corner_pass@abcd1234",
                            base="base_test", shadow={"corner_pass@eeee0000": 0.42})
  rec = json.loads(log_path.read_text().splitlines()[0])
  assert rec["head"] == "corner_pass@abcd1234"
  assert rec["base"] == "base_test"
  assert rec["shadow"] == {"corner_pass@eeee0000": 0.42}
  assert rec["conf"] == 0.4


# ---------------------------------------------------------------------------
# POST /heads/stage
# ---------------------------------------------------------------------------

class _Fake:
  """Just the pieces of hudd.H that do_POST touches."""

  def __init__(self, path, body, length=None):
    self.path = path
    self.headers = {"Content-Length": str(len(body) if length is None else length)}
    self.rfile = io.BytesIO(body)
    self.out = []

  def _json(self, code, obj):
    self.out.append((code, obj))


@pytest.fixture
def hudd(monkeypatch):
  """hudd as-is when openpilot's cereal bindings are built (on the comma); otherwise
  the same file, loaded with the few pieces it imports at load time stubbed out, so the
  handler itself is what these tests exercise."""
  try:
    from openpilot.sunnypilot.canopy import hudd as mod
    return mod
  except Exception:
    pass

  import importlib.util
  import sys
  import types

  class _Any:
    def __getattr__(self, _name):
      return 0

  cereal = types.ModuleType("openpilot.cereal")
  cereal.messaging = types.ModuleType("openpilot.cereal.messaging")
  cereal.custom = types.SimpleNamespace(SelfdriveStateSP=types.SimpleNamespace(AudibleAlert=_Any()))
  gps = types.ModuleType("openpilot.common.gps")
  gps.get_gps_location_service = lambda _p: "gpsLocation"
  params = types.ModuleType("openpilot.common.params")
  params.Params = lambda: types.SimpleNamespace(get_bool=lambda _k: False)
  monkeypatch.setitem(sys.modules, "openpilot.cereal", cereal)
  monkeypatch.setitem(sys.modules, "openpilot.cereal.messaging", cereal.messaging)
  monkeypatch.setitem(sys.modules, "openpilot.common.gps", gps)
  monkeypatch.setitem(sys.modules, "openpilot.common.params", params)

  path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "hudd.py")
  spec = importlib.util.spec_from_file_location("canopy_hudd_under_test", path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def test_post_heads_stage_size_check(root, hudd):
  f = _Fake("/heads/stage", b"", length=hudd.HEADS_MAX_BYTES + 1)
  hudd.H.do_POST(f)
  assert f.out[-1][0] == 413
  assert not (root / "staged").exists() or not any((root / "staged").iterdir())


def test_post_heads_stage_sha_check(root, hudd):
  b = _setup(root)
  h = _head(b["sha"])
  h["w"] = [7.0]  # version left behind
  f = _Fake("/heads/stage", json.dumps(h).encode())
  hudd.H.do_POST(f)
  assert f.out[-1][0] == 400
  assert "version" in f.out[-1][1]["error"]
  assert not (root / "staged" / "corner_pass.json").exists()


def test_post_heads_stage_writes_and_answers(root, hudd):
  b = _setup(root)
  h = _head(b["sha"])
  f = _Fake("/heads/stage", json.dumps(h).encode())
  hudd.H.do_POST(f)
  assert f.out[-1] == (200, {"ok": True, "head_id": f"corner_pass@{h['version']}"})
  assert json.loads((root / "staged" / "corner_pass.json").read_text())["version"] == h["version"]
  assert not (root / "active" / "corner_pass.json").exists()
