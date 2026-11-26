"""Tests for canopy/parking_release.py: the switch, the token, and one-shot taking."""
import json

import pytest

from openpilot.sunnypilot.canopy import parking_release


@pytest.fixture
def paths(tmp_path, monkeypatch):
  flag = tmp_path / "parking_release_on"
  req = tmp_path / "parking_release.json"
  monkeypatch.setattr(parking_release, "FLAG", str(flag))
  monkeypatch.setattr(parking_release, "REQ", str(req))
  return flag, req


def test_no_flag_requests_nothing(paths):
  flag, req = paths
  assert not parking_release.enabled()
  assert parking_release.request("garage", 1000.0) is False
  assert not req.exists()


def test_flag_writes_token(paths):
  flag, req = paths
  flag.write_text("")
  assert parking_release.enabled()
  assert parking_release.request("garage", 1000.0) is True
  d = json.loads(req.read_text())
  assert d == {"token": 1000.0, "t": 1000.0, "source": "garage"}


def poll_until_true(taker, wall, calls=20):
  for _ in range(calls):
    if taker.poll(wall):
      return True
  return False


def test_taker_takes_each_token_once(paths):
  flag, req = paths
  flag.write_text("")
  taker = parking_release.Taker()
  assert taker.poll(1000.0) is False  # first call never reads
  assert parking_release.request("garage", 1000.0)
  assert poll_until_true(taker, 1000.5)
  for _ in range(15):
    assert taker.poll(1000.5) is False


def test_taker_ignores_stale_token(paths):
  flag, req = paths
  flag.write_text("")
  assert parking_release.request("garage", 1000.0)
  taker = parking_release.Taker()
  assert not poll_until_true(taker, 1003.5)


def test_taker_ignores_missing_and_garbled(paths):
  flag, req = paths
  taker = parking_release.Taker()
  assert not poll_until_true(taker, 1000.0)
  req.write_text("{not json")
  assert not poll_until_true(taker, 1000.0)
  req.write_text(json.dumps({"token": 1000.0}))  # no t
  assert not poll_until_true(taker, 1000.0)
