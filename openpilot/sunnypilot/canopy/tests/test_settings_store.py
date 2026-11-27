"""Tests for canopy/settings_store.py: flag files, gate/gate params views, garage row, assist level."""
import json

import pytest

from openpilot.sunnypilot.canopy import assist_torque, settings_store
from openpilot.sunnypilot.canopy.settings_store import (FLAGS, FlagSwitch, flag, gate_summary,
                                                       garage_summary, read_json, set_assist_level,
                                                       tuned_values)


def write_json(base, name, obj):
  (base / name).write_text(json.dumps(obj))


# ---- flag files ----

@pytest.mark.parametrize("key,on_when_present,default_on", [
  ("hud", False, True),
  ("look_ahead", False, True),
  ("lead_classifier", False, True),
  ("parking_release", True, False),
])
def test_flag_get_set_round_trip(tmp_path, key, on_when_present, default_on):
  sw = flag(key)
  assert sw.on_when_present is on_when_present
  assert sw.default_on is default_on
  assert sw.get(str(tmp_path)) is default_on
  for on in (True, False, True, False):
    sw.set(on, str(tmp_path))
    assert sw.get(str(tmp_path)) is on
    assert (tmp_path / sw.filename).exists() is (on == sw.on_when_present)


def test_flag_set_is_idempotent(tmp_path):
  sw = flag("hud")
  for _ in range(2):
    sw.set(True, str(tmp_path))
  assert sw.get(str(tmp_path)) is True
  for _ in range(2):
    sw.set(False, str(tmp_path))
  assert sw.get(str(tmp_path)) is False
  assert (tmp_path / "hud_off").exists()


def test_flag_remove_never_raises_when_absent(tmp_path):
  sw = flag("parking_release")
  sw.set(False, str(tmp_path))  # nothing to remove
  assert sw.get(str(tmp_path)) is False


def test_flag_creates_parent_dir(tmp_path):
  sw = flag("parking_release")
  base = tmp_path / "nested" / "canopy"
  sw.set(True, str(base))
  assert (base / "parking_release_on").exists()
  assert sw.get(str(base)) is True


@pytest.mark.parametrize("key,default_on", [("hud", True), ("parking_release", False)])
def test_default_on_read_error(tmp_path, key, default_on):
  base = tmp_path / "canopy"
  base.write_text("not a directory")
  assert flag(key).get(str(base)) is default_on


def test_flag_uses_env_at_call_time(tmp_path, monkeypatch):
  sw = flag("hud")
  monkeypatch.setenv("CANOPY_DIR", str(tmp_path))
  assert sw.get() is True
  sw.set(False)
  assert sw.get() is False
  assert (tmp_path / "hud_off").exists()


def test_unknown_key_raises():
  with pytest.raises(KeyError):
    flag("no_such_switch")


def test_flags_order_and_control():
  assert [f.key for f in FLAGS] == ["hud", "look_ahead", "lead_classifier", "parking_release"]
  assert [f.control for f in FLAGS] == [False, False, False, True]
  assert all(isinstance(f, FlagSwitch) for f in FLAGS)
  with pytest.raises(Exception):
    FLAGS[0].title = "changed"  # frozen


# ---- gates.json views ----

def test_gate_summary_most_advanced_and_sorted(tmp_path):
  write_json(tmp_path, "gates.json", {"features": {
    "zeta": {"_": {"status": "shadow"}, "light=day": {"status": "live"}},
    "alpha": {"_": {"status": "practice"}},
    "beta": {"_": {"status": "weird"}, "road=freeway": {"status": "shadow"}},
    "gamma": {"_": "not a dict"},
  }})
  rows = gate_summary(str(tmp_path))
  assert rows == [("alpha", "practice"), ("beta", "shadow"), ("gamma", "shadow"), ("zeta", "live")]


def test_gate_summary_missing_is_empty(tmp_path):
  assert gate_summary(str(tmp_path)) == []
  write_json(tmp_path, "gates.json", {"features": []})
  assert gate_summary(str(tmp_path)) == []


def test_tuned_values_skips_prev_and_scalars(tmp_path):
  write_json(tmp_path, "gates.json", {"params": {
    "lane_turn": {"max_mph": "19"},
    "voice_merge": {"delay_s": 1.5, "clean_window_s": "8"},
    "lane_turn:prev": {"max_mph": "15"},
    "lead_classifier": {"bar": "0.8", "bar:prev": "0.7"},
    "lane_change_auto_min_mph": "50.0",
    "lane_change_auto_min_mph:prev": "50.0",
  }})
  rows = tuned_values(str(tmp_path))
  assert rows == [
    ("lane_turn max_mph", "19"),
    ("lead_classifier bar", "0.8"),
    ("voice_merge clean_window_s", "8"),
    ("voice_merge delay_s", "1.5"),
  ]
  assert not any("prev" in name for name, _ in rows)
  assert not any("lane_change_auto_min_mph" == name for name, _ in rows)


def test_tuned_values_missing_is_empty(tmp_path):
  assert tuned_values(str(tmp_path)) == []


def test_read_json_errors_are_empty(tmp_path):
  assert read_json("gates.json", str(tmp_path)) == {}
  (tmp_path / "bad.json").write_text("{not json")
  assert read_json("bad.json", str(tmp_path)) == {}
  (tmp_path / "list.json").write_text("[1, 2]")
  assert read_json("list.json", str(tmp_path)) == {}


# ---- garage ----

def test_garage_summary_never_shows_lat_or_lon(tmp_path):
  write_json(tmp_path, "garage.json", {"lat": 33.4152, "lon": -111.8315,
                                       "heading_deg": 342.0, "zone_m": 40})
  s = garage_summary(str(tmp_path))
  assert s == "zone 40 m, heading 342"
  for secret in ("33.4152", "33", "111.8315", "111", "-111", "lat", "lon"):
    assert secret not in s


def test_garage_summary_missing_is_empty(tmp_path):
  assert garage_summary(str(tmp_path)) == ""
  write_json(tmp_path, "garage.json", {"lat": 1.0, "lon": 2.0})
  assert garage_summary(str(tmp_path)) == ""


# ---- assist strength ----

def test_assist_level_reads_assist_torque(tmp_path, monkeypatch):
  monkeypatch.setattr(assist_torque, "read_level", lambda: 3)
  assert settings_store.assist_level() == 3


def test_assist_level_falls_back_to_one(monkeypatch):
  def boom():
    raise RuntimeError("no card")
  monkeypatch.setattr(assist_torque, "read_level", boom)
  assert settings_store.assist_level() == 1


def test_set_assist_level_writes_and_validates(monkeypatch):
  seen = []
  monkeypatch.setattr(assist_torque, "write_level", lambda level: seen.append(level))
  set_assist_level(4)
  assert seen == [4]
  for bad in (0, 6, -1, "high"):
    with pytest.raises(ValueError):
      set_assist_level(bad)
  assert seen == [4]
