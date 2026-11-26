"""Tests for canopy/trouble_spot.py path mode: the heads-up follows the comma's Look Ahead view.

A spot off to the side never passes the straight-ahead bearing check. If the local Look Ahead view
runs past it, the spot may still fire (via "path") when the gate turns look_ahead_comma on, and logs
one shadow row per approach when it does not.
"""
import builtins
import json
import math
import os
import time
import types
from unittest import mock

from openpilot.sunnypilot.canopy import gates as gates_mod
from openpilot.sunnypilot.canopy import trouble_spot as ts

LAT, LON = 33.4, -111.8
M_PER_DEG_LAT = 111320.0
M_PER_DEG_LON = M_PER_DEG_LAT * math.cos(math.radians(LAT))
CAR = types.SimpleNamespace(brakePressed=False, steeringPressed=False)
OFFSET = {"id": "s1", "k": "tricky", "h": 0}


class FakeGates:
  def __init__(self, live=False):
    self.live = live
    self.calls = []
    self.demoted = []

  def conditions(self, lat, lon, unix, mph, speed_limit_mph):
    return {"speed_band": "x"}

  def is_live(self, fid, cond, conf=1.0, acting=()):
    self.calls.append((fid, dict(cond), conf))
    return self.live

  def demote(self, fid, cell, reason=""):
    self.demoted.append((fid, dict(cell), reason))


def offset_point(north_m, east_m):
  return LAT + north_m / M_PER_DEG_LAT, LON + east_m / M_PER_DEG_LON


def gps_at(lat=LAT, lon=LON, bearing=0.0):
  return types.SimpleNamespace(latitude=lat, longitude=lon, bearingDeg=bearing, hasFix=True)


def side_spot():
  """200 m due east of the car, which is heading north: 90 deg off, so the line check fails."""
  lat, lon = offset_point(0.0, 200.0)
  return {"id": "s1", "lat": lat, "lon": lon, "heading": 0.0, "count": 3, "label": "tricky"}


def ahead_spot():
  lat, lon = offset_point(150.0, 0.0)
  return {"id": "s2", "lat": lat, "lon": lon, "heading": 0.0, "count": 2, "label": "tricky"}


def build(tmp_path, gates, spots):
  spots_path = tmp_path / "trouble_spots.json"
  spots_path.write_text(json.dumps({"version": 1, "spots": spots}))
  la_shadow = tmp_path / "lookahead_cue_shadow.jsonl"
  view_path = tmp_path / "canopy_lookahead.json"
  off = tmp_path / "lookahead_off"
  calls = []
  obj = ts.TroubleSpots(path=str(spots_path), garage_path=str(tmp_path / "garage.json"),
                        shadow_path=str(tmp_path / "trouble_shadow.jsonl"),
                        post=lambda *a: calls.append(a),
                        lookahead_path=str(view_path), off_flag=str(off),
                        la_shadow_path=str(la_shadow), gates=gates)
  return obj, calls, la_shadow, view_path, off


def write_view(path, ts_, spot, along_m=150.0, source="history", conf=0.9):
  item = dict(OFFSET, along_m=along_m, lat=spot["lat"], lon=spot["lon"], id="v1")
  path.write_text(json.dumps({"ts": ts_, "source": source, "path_conf": conf, "items": [item]}))


def run(obj, now, wall, engaged=True, lat_active=True):
  return obj.update(gps_at(), CAR, 20.0, engaged, lat_active, 25.0, None, now=now, wall=wall)


def test_gate_not_live_logs_one_shadow_row_and_posts_nothing(tmp_path):
  g = FakeGates(live=False)
  spot = side_spot()
  obj, calls, la_shadow, view_path, _ = build(tmp_path, g, [spot])
  write_view(view_path, 1000.0, spot)

  assert run(obj, 50.0, 1000.0) is False
  assert calls == []
  assert obj.last_fire is None
  rows = [json.loads(line) for line in la_shadow.read_text().splitlines()]
  assert len(rows) == 1
  assert rows[0]["live"] is False
  assert rows[0]["spot"] == "s1"
  assert rows[0]["along_m"] == 150
  assert rows[0]["source"] == "history"
  assert rows[0]["spot_lat"] == round(spot["lat"], 6)
  assert rows[0]["spot_lon"] == round(spot["lon"], 6)

  assert run(obj, 51.0, 1001.0) is False
  assert len(la_shadow.read_text().splitlines()) == 1


def test_gate_live_fires_on_the_path(tmp_path):
  g = FakeGates(live=True)
  spot = side_spot()
  obj, calls, la_shadow, view_path, _ = build(tmp_path, g, [spot])
  write_view(view_path, 1000.0, spot)

  assert run(obj, 50.0, 1000.0) is True
  assert obj.last_fire["via"] == "path"
  assert obj.last_fire["dist_m"] == 150
  fid, cond, conf = g.calls[-1]
  assert fid == ts.LA_FEATURE
  assert cond["path_source"] == "history"
  assert conf == 0.9
  assert len(calls) == 1 and calls[0][0] == "trouble"
  assert not la_shadow.exists()


def test_stale_view_fires_nothing(tmp_path):
  g = FakeGates(live=True)
  spot = side_spot()
  obj, calls, la_shadow, view_path, _ = build(tmp_path, g, [spot])
  write_view(view_path, 1000.0 - 3.5, spot)

  assert run(obj, 50.0, 1000.0) is False
  assert calls == []
  assert g.calls == []
  assert not la_shadow.exists()


def test_off_flag_fires_nothing(tmp_path):
  g = FakeGates(live=True)
  spot = side_spot()
  obj, calls, la_shadow, view_path, off = build(tmp_path, g, [spot])
  write_view(view_path, 1000.0, spot)
  off.write_text("")

  assert run(obj, 50.0, 1000.0) is False
  assert calls == []
  assert g.calls == []
  assert not la_shadow.exists()


def test_takeover_demotes_the_path_cell(tmp_path):
  g = FakeGates(live=True)
  spot = side_spot()
  obj, calls, _, view_path, _ = build(tmp_path, g, [spot])
  write_view(view_path, 1000.0, spot)
  assert run(obj, 100.0, 1000.0) is True
  fired_cond = obj._la_cond

  # He takes the wheel back, then the approach times out.
  assert run(obj, 100.0 + ts.ZONE_TIMEOUT_S + 1.0, 1000.0 + ts.ZONE_TIMEOUT_S + 1.0, engaged=False) is False
  assert obj.last_row["via"] == "path"
  assert obj.last_row["took_over"] is True
  assert len(g.demoted) == 1
  fid, cell, reason = g.demoted[0]
  assert fid == ts.LA_FEATURE
  assert cell == fired_cond
  assert reason == "takeover"
  assert obj._la_cond is None

  # Nothing more to demote on later frames.
  assert run(obj, 200.0, 1100.0, engaged=False) is False
  assert len(g.demoted) == 1


def test_straight_ahead_still_fires_on_the_line(tmp_path):
  g = FakeGates(live=False)  # its is_live must never be needed
  obj, calls, la_shadow, _, _ = build(tmp_path, g, [ahead_spot()])

  assert run(obj, 50.0, 1000.0) is True
  assert obj.last_fire["via"] == "line"
  assert len(calls) == 1 and calls[0][0] == "trouble"
  assert g.calls == []
  assert not la_shadow.exists()


def test_shadow_row_does_not_block_the_later_line_cue(tmp_path):
  """A shadow path hit must leave the live cue's rearm alone: the same spot, once straight
  ahead on the right heading, still gets its line heads-up."""
  g = FakeGates(live=False)
  spot = side_spot()
  obj, calls, la_shadow, view_path, _ = build(tmp_path, g, [spot])
  write_view(view_path, 1000.0, spot)

  # 90 deg off to the east: only the shadow path sees it.
  assert run(obj, 50.0, 1000.0) is False
  assert len(la_shadow.read_text().splitlines()) == 1
  assert "s1" not in obj._last_fire
  assert obj.last_fire is None
  assert calls == []

  # Move south of the spot, still heading north: it is now 150 m straight ahead and in reach.
  lat, lon = offset_point(-150.0, 200.0)
  fired = obj.update(gps_at(lat, lon, 0.0), CAR, 20.0, True, True, 25.0, None,
                     now=51.0, wall=1001.0)
  assert fired is True
  assert obj.last_fire["via"] == "line"
  assert obj.last_fire["spot"] == "s1"
  assert len(calls) == 1 and calls[0][0] == "trouble"


def test_view_file_is_read_at_most_twice_a_second(tmp_path):
  g = FakeGates(live=False)
  spot = side_spot()
  obj, _, _, view_path, _ = build(tmp_path, g, [spot])
  write_view(view_path, 1000.0, spot)

  reads, real_open = [], builtins.open

  def counting_open(file, *a, **k):
    if str(file) == str(view_path):
      reads.append(file)
    return real_open(file, *a, **k)

  with mock.patch("builtins.open", counting_open):
    # Two frames 0.1 s apart: the file is stat'ed and parsed once.
    assert run(obj, 50.0, 1000.0) is False
    assert run(obj, 50.1, 1000.1) is False
    assert len(reads) == 1

    # hudd rewrites the view; past the recheck window the new one is picked up.
    write_view(view_path, 1000.5, spot, source="map")
    os.utime(view_path, (1000.5, 1000.5))  # the write clock here is too coarse to move the mtime
    assert run(obj, 50.6, 1000.6) is False
    assert len(reads) == 2
    assert obj._path_view(50.6, 1000.6)["source"] == "map"
    assert len(reads) == 2


def test_own_gates_loads_its_snapshot_so_path_cue_can_fire(tmp_path, monkeypatch):
  """A TroubleSpots that builds its own Gates must snapshot it. Otherwise is_live stays False on
  empty data forever and the path cue can never switch on."""
  gates_path = tmp_path / "gates.json"
  gates_path.write_text(json.dumps({
    "generated": 1000.0,
    "features": {
      ts.LA_FEATURE: {
        "path_source=history": {"status": "live", "tier": "cue", "cstar": 0.0},
      },
    },
    "names": {ts.LA_FEATURE: "Look Ahead"},
    "combos": {},
  }))
  monkeypatch.setattr(gates_mod, "GATES_PATH", str(gates_path))
  monkeypatch.setattr(gates_mod, "DEMOTED_PATH", str(tmp_path / "gates_demoted.json"))

  spot = side_spot()
  obj, calls, la_shadow, view_path, _ = build(tmp_path, None, [spot])
  assert obj.gates is not None

  # The class starts the snapshot in a daemon thread; give it up to 2 s to land.
  deadline = time.monotonic() + 2.0
  while obj.gates._data is None and time.monotonic() < deadline:
    time.sleep(0.01)
  assert obj.gates._data is not None

  # 90 deg off to the east: only the path can see it, and the loaded gate says it is live.
  write_view(view_path, 1000.0, spot)
  assert run(obj, 50.0, 1000.0) is True
  assert obj.last_fire["via"] == "path"
  assert obj.last_fire["spot"] == "s1"
  assert len(calls) == 1 and calls[0][0] == "trouble"
  assert not la_shadow.exists()
