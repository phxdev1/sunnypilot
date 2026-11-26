"""Tests for canopy/decision_log.py: one JSON line per decision, rate limited, never raises."""
import json

import pytest

from openpilot.sunnypilot.canopy import decision_log, leadkind


@pytest.fixture
def log_path(tmp_path, monkeypatch):
  path = tmp_path / "canopy" / "decisions.jsonl"
  monkeypatch.setattr(decision_log, "PATH", str(path))
  monkeypatch.setattr(decision_log, "_last", {})
  monkeypatch.setattr(leadkind, "route_name", lambda: "drive-1")
  return path


def _lines(path):
  return [json.loads(line) for line in path.read_text().splitlines()]


def test_writes_line(log_path):
  decision_log.log_decision("parking_release", "take", 0.12367, ctx={"b": 2, "a": 1, "c": None}, live=True, lat=33.4, lon=-111.8)
  rec = _lines(log_path)[0]
  assert set(rec) == {"type", "feature", "action", "conf", "drive", "ts", "live", "ctx", "lat", "lon"}
  assert rec["type"] == "decision"
  assert (rec["feature"], rec["action"], rec["conf"]) == ("parking_release", "take", 0.124)
  assert rec["drive"] == "drive-1"
  assert rec["live"] is True
  assert rec["ctx"] == "a=1|b=2"
  assert isinstance(rec["ts"], float)
  assert (rec["lat"], rec["lon"]) == (33.4, -111.8)


def test_rate_limited_per_feature_and_action(log_path):
  decision_log.log_decision("gate", "hold", 0.5)
  decision_log.log_decision("gate", "hold", 0.5)
  decision_log.log_decision("gate", "release", 0.5)
  lines = _lines(log_path)
  assert [rec["action"] for rec in lines] == ["hold", "release"]


def test_skips_when_file_too_big(log_path, monkeypatch):
  monkeypatch.setattr(decision_log, "MAX_BYTES", 64)
  log_path.parent.mkdir(parents=True)
  log_path.write_text("x" * 65)
  decision_log.log_decision("gate", "hold", 0.5)
  assert log_path.read_text() == "x" * 65


def test_never_raises_on_bad_path(tmp_path, monkeypatch):
  blocker = tmp_path / "blocker"
  blocker.write_text("")
  path = blocker / "decisions.jsonl"
  monkeypatch.setattr(decision_log, "PATH", str(path))
  monkeypatch.setattr(decision_log, "_last", {})
  assert decision_log.log_decision("gate", "hold", 0.5) is None
  assert not path.exists()


def test_creates_missing_parent(log_path):
  assert not log_path.parent.exists()
  decision_log.log_decision("gate", "hold", 0.5)
  assert log_path.exists()
