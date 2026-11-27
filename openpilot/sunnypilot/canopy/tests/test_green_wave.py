"""Tests for canopy/green_wave.py: pick the lowest speed that makes the lights, and ripple only upward."""
import json

from openpilot.sunnypilot.canopy import green_wave as gw

T60 = {"cycle_s": 60, "green_s": 30, "yellow_s": 4, "offset_s": 0}
T40 = {"cycle_s": 40, "green_s": 20, "yellow_s": 4, "offset_s": 0}


def sig(d, timing=T60):
  return {"d": float(d), "timing": timing}


def test_phase_wraps_and_calls_yellow():
    assert gw.phase(T60, 0.0) == "green"
    assert gw.phase(T60, 29.0) == "green"
    assert gw.phase(T60, 31.0) == "yellow"
    assert gw.phase(T60, 40.0) == "red"
    assert gw.phase(T60, 60.0) == "green"
    assert gw.phase(T60, 151.0) == "yellow"
    assert gw.phase({}, 0.0) == "unknown"


def test_the_limit_is_used_when_it_already_makes_the_lights():
    # 40 mph covers 300 m in 16.8 s and 700 m in 39 s: both inside the green at phase 0 with a 40 s cycle
    w = gw.solve([sig(300, T40), sig(700, T40)], 40.0, 0.0)
    assert w["on"] and w["base_mph"] == 40.0 and w["catch"] == 2


def test_it_speeds_up_only_as_far_as_it_has_to():
    # at 30 mph the first light is reached at 23.9 s, past the 20 s green of the 40 s cycle
    w = gw.solve([sig(300, T40), sig(700, T40)], 30.0, 0.0)
    assert w["on"] and w["catch"] == 2
    assert 30.0 < w["base_mph"] <= 30.0 + gw.CAP_MPH


def test_it_never_goes_past_the_cap():
    # nothing works until well above the cap: take the best the cap allows, or nothing at all
    w = gw.solve([sig(150, {"cycle_s": 200, "green_s": 3, "offset_s": 0}), sig(400, {"cycle_s": 200, "green_s": 3, "offset_s": 0})], 30.0, 0.0)
    assert w is None or w["base_mph"] <= 30.0 + gw.CAP_MPH


def test_a_freeway_or_a_parking_lot_has_no_wave():
    assert gw.solve([sig(300, T40), sig(700, T40)], 65.0, 0.0) is None
    assert gw.solve([sig(300, T40), sig(700, T40)], 15.0, 0.0) is None
    assert gw.solve([sig(300, T40), sig(700, T40)], 0.0, 0.0) is None


def test_one_light_is_not_a_wave_and_no_timing_is_not_either():
    assert gw.solve([sig(300, T40)], 30.0, 0.0) is None
    assert gw.solve([{"d": 300.0, "timing": None}, {"d": 700.0, "timing": None}], 30.0, 0.0) is None


def test_a_light_already_under_the_car_does_not_decide_anything():
    # the 10 m light is skipped, so the pair 300 and 700 is what counts
    w = gw.solve([sig(10, T40), sig(300, T40), sig(700, T40)], 40.0, 0.0)
    assert w["on"] and w["catch"] == 2


def test_the_ripple_only_ever_moves_up_from_the_base():
    w = {"on": True, "base_mph": 33.0, "amp_mph": 1.5, "period_s": 24.0, "phase_s": 100.0}
    vals = [gw.target_at(w, 100.0 + i * 0.5) for i in range(60)]
    assert min(vals) >= 33.0 - 1e-9
    assert max(vals) <= 34.5 + 1e-9
    assert len(set(vals)) > 10  # it really moves


def test_no_ripple_means_a_flat_target():
    w = {"on": True, "base_mph": 33.0, "amp_mph": 0.0, "period_s": 24.0, "phase_s": 0.0}
    assert gw.target_at(w, 1234.0) == 33.0
    assert gw.target_at(None, 1.0) is None


def test_the_file_the_resolver_reads_goes_stale(tmp_path):
    p = str(tmp_path / "green_wave.json")
    w = gw.solve([sig(300, T40), sig(700, T40)], 40.0, 0.0)
    assert gw.write(w, path=p)
    assert gw.read(path=p, now=1.0)["base_mph"] == 40.0
    assert gw.read(path=p, now=1.0 + gw.FRESH_S + 1) is None
    (tmp_path / "bad.json").write_text("{not json")
    assert gw.read(path=str(tmp_path / "bad.json")) is None
    assert gw.write(None, path=p) is False


def test_signals_come_out_of_the_look_ahead_view_with_their_timing():
    view = {"items": [
        {"k": "signal", "along_m": 300.0, "lat": 33.4, "lon": -111.8, "x": {"timing": T40}},
        {"k": "tricky", "along_m": 200.0, "x": {}},
        {"k": "signal", "along_m": 700.0, "x": {"timing": None}},
        {"k": "signal", "along_m": 900.0, "x": {"timing": T60}},
    ]}
    got = gw.signals_from_view(view, 40.0, 0.0)
    assert [s["d"] for s in got] == [300.0, 900.0]
    assert got[0]["timing"]["cycle_s"] == 40
