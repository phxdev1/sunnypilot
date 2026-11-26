"""Tests for canopy/hudd.py's local Look Ahead: one step at a time, watch only."""
import pytest

pytest.importorskip("openpilot.cereal")

from openpilot.sunnypilot.canopy import hudd  # noqa: E402


class FakeCarState:
  def __init__(self, v_ego=10.0):
    self.vEgo = v_ego


class FakeGPS:
  def __init__(self, fix=True):
    self.hasFix = fix
    self.latitude = 33.4152
    self.longitude = -111.7442
    self.bearingDeg = 92.0


class FakeSM:
  def __init__(self, cs=None, gps=None, frames=7, update_error=None):
    self._msgs = {"carState": cs if cs is not None else FakeCarState(),
                  hudd._GPS: gps if gps is not None else FakeGPS()}
    self.recv_frame = {hudd._GPS: frames}
    self.update_error = update_error
    self.updated = 0

  def update(self, _timeout):
    self.updated += 1
    if self.update_error is not None:
      raise self.update_error

  def __getitem__(self, service):
    return self._msgs[service]


class FakeLA:
  def __init__(self, boom=False):
    self.boom = boom
    self.views = []
    self.shms = []
    self.logs = []

  def view(self, lat, lon, heading, mph):
    if self.boom:
      raise RuntimeError("view blew up")
    self.views.append((lat, lon, heading, mph))
    return {"source": "heading", "items": [], "mph": mph}

  def write_shm(self, view):
    self.shms.append(view)

  def log(self, view, lat, lon, heading, mph):
    self.logs.append((view, lat, lon, heading, mph))


@pytest.fixture(autouse=True)
def clean_la(monkeypatch, tmp_path):
  monkeypatch.setattr(hudd, "LA_OFF", str(tmp_path / "lookahead_off"))
  hudd._la.update({"view": None, "t": 0.0, "err": None, "n": 0})
  yield


def test_onroad_with_fix_answers_once(monkeypatch):
  sm, la = FakeSM(), FakeLA()
  v = hudd.lookahead_step(sm, la, lambda: True)

  assert v is not None
  assert len(la.views) == 1
  lat, lon, heading, mph = la.views[0]
  assert round(mph, 2) == 22.37
  assert (lat, lon, heading) == (33.4152, -111.7442, 92.0)
  assert la.shms == [v]
  assert la.logs == [(v, 33.4152, -111.7442, 92.0, mph)]
  assert hudd._la["view"] is v
  assert hudd._la["n"] == 1
  assert hudd._la["err"] is None
  assert hudd._la["t"] > 0.0
  assert sm.updated == 1


def test_offroad_skips_and_clears():
  sm, la = FakeSM(), FakeLA()
  assert hudd.lookahead_step(sm, la, lambda: False) is None
  assert la.views == []
  assert la.shms == []
  assert la.logs == []
  assert hudd._la["view"] is None
  assert hudd._la["n"] == 0


def test_no_fix_skips():
  sm, la = FakeSM(gps=FakeGPS(fix=False)), FakeLA()
  assert hudd.lookahead_step(sm, la, lambda: True) is None
  assert la.views == []
  assert hudd._la["n"] == 0


def test_gps_never_received_skips():
  sm, la = FakeSM(frames=0), FakeLA()
  assert hudd.lookahead_step(sm, la, lambda: True) is None
  assert la.views == []
  assert hudd._la["n"] == 0


def test_off_flag_skips(monkeypatch, tmp_path):
  flag = tmp_path / "lookahead_off"
  flag.write_text("")
  monkeypatch.setattr(hudd, "LA_OFF", str(flag))
  sm, la = FakeSM(), FakeLA()
  assert hudd.lookahead_step(sm, la, lambda: True) is None
  assert la.views == []
  assert hudd._la["view"] is None
  assert hudd._la["n"] == 0


def test_view_raising_is_contained():
  sm, la = FakeSM(), FakeLA(boom=True)
  assert hudd.lookahead_step(sm, la, lambda: True) is None
  assert hudd._la["err"] == "view blew up"
  assert hudd._la["n"] == 0
  assert hudd._la["view"] is None


def test_step_errors_never_escape_the_loop():
  sleeps = []
  sm, la = FakeSM(update_error=ValueError("no stream")), FakeLA()
  calls = {"n": 0}

  def sleep(_s):
    calls["n"] += 1
    sleeps.append(_s)
    if calls["n"] >= 2:
      raise SystemExit  # two passes is enough

  with pytest.raises(SystemExit):
    hudd.lookahead_loop(sleep=sleep, sm=sm, la=la, onroad=lambda: True)

  assert sleeps == [hudd.LA_EVERY_S, hudd.LA_EVERY_S]
  assert hudd._la["err"] == "no stream"
  assert hudd._la["n"] == 0
