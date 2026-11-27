"""Tests for canopy/assist_torque.py: config read, low-speed gate, and the EPS fault latch."""
import json

from openpilot.sunnypilot.canopy.assist_torque import AssistTorque


class DummyTarget:
  LKAS_MAX_TORQUE = 1.0


def make(tmp_path, body=None):
  cfg = tmp_path / "assist_torque"
  if body is not None:
    cfg.write_text(body)
  log = tmp_path / "assist_torque.jsonl"
  target = DummyTarget()
  return AssistTorque(cfg_path=str(cfg), log_path=str(log), target=target), log, target


def test_missing_file_is_stock(tmp_path):
  at, _, _ = make(tmp_path)
  assert at.update(5.0, False, True) == 1.0


def test_higher_above_20mph_is_stock(tmp_path):
  at, _, _ = make(tmp_path, "1.5")
  assert at.update(5.0, False, True) == 1.5
  assert at.update(10.0, False, True) == 1.0


def test_clamp_high(tmp_path):
  at, _, _ = make(tmp_path, "9")
  assert at.update(5.0, False, True) == 2.0


def test_clamp_low_applies_at_speed(tmp_path):
  at, _, _ = make(tmp_path, "0.1")
  assert at.update(5.0, False, True) == 0.5
  assert at.update(10.0, False, True) == 0.5


def test_garbage_is_stock(tmp_path):
  at, _, _ = make(tmp_path, "not a number")
  assert at.update(5.0, False, True) == 1.0


def test_fault_latches_back_to_stock(tmp_path):
  at, log, _ = make(tmp_path, "1.5")
  assert at.update(5.0, False, True) == 1.5
  assert at.update(5.0, True, True) == 1.0
  assert at.latched
  assert at.update(5.0, False, True) == 1.0
  assert at.update(5.0, False, True) == 1.0
  events = [json.loads(line)["event"] for line in log.read_text().splitlines()]
  assert "fault_latch" in events
  assert events.count("fault_latch") == 1


def test_target_attribute_set(tmp_path):
  at, _, target = make(tmp_path, "1.5")
  at.update(5.0, False, True)
  assert target.LKAS_MAX_TORQUE == 1.5
  at.update(10.0, False, True)
  assert target.LKAS_MAX_TORQUE == 1.0


def test_never_raises_on_bare_target(tmp_path):
  class Bare:
    pass

  cfg = tmp_path / "assist_torque"
  cfg.write_text("1.5")
  at = AssistTorque(cfg_path=str(cfg), log_path=str(tmp_path / "log.jsonl"), target=Bare())
  assert at.update(5.0, False, True) == 1.5
  assert at.target.LKAS_MAX_TORQUE == 1.5


def test_never_raises_when_cfg_is_a_directory(tmp_path):
  cfg = tmp_path / "assist_torque_dir"
  cfg.mkdir()
  at = AssistTorque(cfg_path=str(cfg), log_path=str(tmp_path / "log.jsonl"), target=DummyTarget())
  assert at.update(5.0, False, True) == 1.0
