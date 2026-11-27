"""Tests for canopy/assist_torque.py: level file, low-speed gate, and the EPS fault latch."""
import json

import pytest

from openpilot.sunnypilot.canopy.assist_torque import (AssistTorque, CFG_PATH, LEVEL_MAX, LEVEL_MIN,
                                                      level_to_nm, read_level, write_level)


class DummyTarget:
  LKAS_MAX_TORQUE = 1.0


def make(tmp_path, body=None):
  cfg = tmp_path / "assist_level"
  if body is not None:
    cfg.write_text(body)
  log = tmp_path / "assist_torque.jsonl"
  target = DummyTarget()
  return AssistTorque(cfg_path=str(cfg), log_path=str(log), target=target), log, target


def test_cfg_path_is_the_level_file():
  assert CFG_PATH == "/data/canopy/assist_level"


@pytest.mark.parametrize("level,nm", [(1, 1.0), (2, 1.25), (3, 1.5), (4, 1.75), (5, 2.0)])
def test_level_to_nm(level, nm):
  assert level_to_nm(level) == nm


def test_missing_file_is_stock(tmp_path):
  at, _, _ = make(tmp_path)
  assert at.update(5.0, False, True) == 1.0
  assert at.level == 1


def test_level_three_is_half_again_below_20mph(tmp_path):
  at, _, _ = make(tmp_path, "3")
  assert at.update(5.0, False, True) == 1.5
  assert at.update(10.0, False, True) == 1.0


def test_clamp_high(tmp_path):
  at, _, _ = make(tmp_path, "9")
  assert at.update(5.0, False, True) == 2.0
  assert at.level == LEVEL_MAX


def test_clamp_low(tmp_path):
  at, _, _ = make(tmp_path, "0")
  assert at.update(5.0, False, True) == 1.0
  assert at.level == LEVEL_MIN


def test_garbage_is_stock(tmp_path):
  at, _, _ = make(tmp_path, "not a number")
  assert at.update(5.0, False, True) == 1.0


def test_empty_is_stock(tmp_path):
  at, _, _ = make(tmp_path, "")
  assert at.update(5.0, False, True) == 1.0


def test_fault_latches_back_to_stock(tmp_path):
  at, log, _ = make(tmp_path, "3")
  assert at.update(5.0, False, True) == 1.5
  assert at.update(5.0, True, True) == 1.0
  assert at.latched
  assert at.update(5.0, False, True) == 1.0
  assert at.update(5.0, False, True) == 1.0
  events = [json.loads(line)["event"] for line in log.read_text().splitlines()]
  assert "fault_latch" in events
  assert events.count("fault_latch") == 1


def test_logs_level_and_nm(tmp_path):
  at, log, _ = make(tmp_path, "4")
  at.update(5.0, False, True)
  applied = [json.loads(line) for line in log.read_text().splitlines() if json.loads(line)["event"] == "applied"]
  assert applied[-1]["level"] == 4
  assert applied[-1]["nm"] == 1.75


def test_target_attribute_set(tmp_path):
  at, _, target = make(tmp_path, "3")
  at.update(5.0, False, True)
  assert target.LKAS_MAX_TORQUE == 1.5
  at.update(10.0, False, True)
  assert target.LKAS_MAX_TORQUE == 1.0


def test_never_raises_on_bare_target(tmp_path):
  class Bare:
    pass

  cfg = tmp_path / "assist_level"
  cfg.write_text("3")
  at = AssistTorque(cfg_path=str(cfg), log_path=str(tmp_path / "log.jsonl"), target=Bare())
  assert at.update(5.0, False, True) == 1.5
  assert at.target.LKAS_MAX_TORQUE == 1.5


def test_never_raises_when_cfg_is_a_directory(tmp_path):
  cfg = tmp_path / "assist_level_dir"
  cfg.mkdir()
  at = AssistTorque(cfg_path=str(cfg), log_path=str(tmp_path / "log.jsonl"), target=DummyTarget())
  assert at.update(5.0, False, True) == 1.0


def test_read_level_missing_and_garbage(tmp_path):
  assert read_level(str(tmp_path / "nope")) == 1
  bad = tmp_path / "bad"
  bad.write_text("level four")
  assert read_level(str(bad)) == 1


def test_write_read_round_trip(tmp_path):
  cfg = tmp_path / "nested" / "assist_level"
  for level in range(LEVEL_MIN, LEVEL_MAX + 1):
    write_level(level, str(cfg))
    assert read_level(str(cfg)) == level


def test_write_clamps(tmp_path):
  cfg = tmp_path / "assist_level"
  write_level(9, str(cfg))
  assert read_level(str(cfg)) == LEVEL_MAX
  write_level(0, str(cfg))
  assert read_level(str(cfg)) == LEVEL_MIN


def test_write_never_raises_on_bad_path(tmp_path):
  write_level(3, str(tmp_path))  # a directory
