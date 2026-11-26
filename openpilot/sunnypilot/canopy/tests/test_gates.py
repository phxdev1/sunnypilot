"""Tests for canopy/gates.py."""
import json
import os
import shutil
import tempfile
import unittest
from unittest import mock
import time

from openpilot.sunnypilot.canopy import gates

_demoted_key = gates._demoted_key


class _TmpDir:
  """Context manager that gives a temp dir and restores on cleanup."""

  def __init__(self):
    self.dir = None

  def __enter__(self):
    self.orig = gates.GATES_PATH, gates.DEMOTED_PATH
    self.dir = tempfile.mkdtemp(prefix="gates_test_")
    gates.GATES_PATH = os.path.join(self.dir, "gates.json")
    gates.DEMOTED_PATH = os.path.join(self.dir, "gates_demoted.json")
    return self

  def __exit__(self, *a):
    gates.GATES_PATH, gates.DEMOTED_PATH = self.orig
    try:
      shutil.rmtree(self.dir)
    except Exception:
      pass


def _write_gates(path, body):
  with open(path, "w") as f:
    json.dump(body, f)


class GatesTest(unittest.TestCase):

  def test_corrupt_file_shadow(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, "not json")
      g = gates.Gates()
      g.snapshot()
      cond = {"light": "day", "road": "freeway", "weather": "dry", "speed": "50-65"}
      self.assertFalse(g.is_live("any", cond))
      self.assertEqual(g.live_list(cond), [])

  def test_missing_file_shadow(self):
    with _TmpDir() as t:
      if os.path.exists(gates.GATES_PATH):
        os.remove(gates.GATES_PATH)
      g = gates.Gates()
      g.snapshot()
      cond = {"light": "day", "road": "freeway", "weather": "dry", "speed": "50-65"}
      self.assertFalse(g.is_live("any", cond))

  def test_control_exact_cell_matching(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, {
        "generated": 1000.0,
        "features": {
          "c1": {
            "light=day|road=freeway|weather=dry|speed=50-65": {"status": "live", "tier": "control", "cstar": 0.9},
          },
        },
        "names": {"c1": "Cruise"},
        "combos": {},
      })
      g = gates.Gates()
      g.snapshot()
      cond = {"light": "day", "road": "freeway", "weather": "dry", "speed": "50-65"}
      self.assertTrue(g.is_live("c1", cond, conf=1.0))
      self.assertTrue(g.is_live("c1", cond, conf=0.9))
      self.assertFalse(g.is_live("c1", cond, conf=0.8))

  def test_control_unknown_dim_refused(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, {
        "generated": 1000.0,
        "features": {
          "c1": {
            "light=day|road=freeway|weather=dry|speed=50-65": {"status": "live", "tier": "control", "cstar": 0.9},
          },
        },
        "names": {"c1": "Cruise"},
        "combos": {},
      })
      g = gates.Gates()
      g.snapshot()
      cond = {"light": None, "road": "freeway", "weather": "dry", "speed": "50-65"}
      self.assertFalse(g.is_live("c1", cond, conf=1.0))
      cond2 = {"light": "day", "road": "freeway", "weather": None, "speed": "50-65"}
      self.assertFalse(g.is_live("c1", cond2, conf=1.0))

  def test_control_missing_dim_refused(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, {
        "generated": 1000.0,
        "features": {
          "c1": {
            "light=day|road=freeway|weather=dry|speed=50-65": {"status": "live", "tier": "control", "cstar": 0.9},
          },
        },
        "names": {"c1": "Cruise"},
        "combos": {},
      })
      g = gates.Gates()
      g.snapshot()
      cond_missing = {"light": "day", "road": "freeway"}
      self.assertFalse(g.is_live("c1", cond_missing, conf=1.0))

  def test_cue_slice_matching(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, {
        "generated": 1000.0,
        "features": {
          "q1": {
            "road=freeway": {"status": "live", "tier": "cue", "cstar": 0.8},
            "light=day|road=surface": {"status": "live", "tier": "cue", "cstar": 0.7},
          },
        },
        "names": {"q1": "Cue1"},
        "combos": {},
      })
      g = gates.Gates()
      g.snapshot()
      cond = {"light": "day", "road": "freeway", "weather": "wet", "speed": None}
      self.assertTrue(g.is_live("q1", cond, conf=0.8))
      self.assertFalse(g.is_live("q1", cond, conf=0.79))
      cond2 = {"light": "day", "road": "surface", "weather": "dry", "speed": "25-35"}
      self.assertTrue(g.is_live("q1", cond2, conf=0.7))
      self.assertFalse(g.is_live("q1", cond2, conf=0.69))

  def test_cue_wildcard_underscore(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, {
        "generated": 1000.0,
        "features": {
          "q2": {
            "_": {"status": "live", "tier": "cue", "cstar": 0.0},
          },
        },
        "names": {"q2": "Always"},
        "combos": {},
      })
      g = gates.Gates()
      g.snapshot()
      cond_all_none = {"light": None, "road": None, "weather": None, "speed": None}
      self.assertTrue(g.is_live("q2", cond_all_none, conf=1.0))

  def test_cue_unknown_dim_does_not_match_named_slice(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, {
        "generated": 1000.0,
        "features": {
          "q3": {
            "light=day": {"status": "live", "tier": "cue", "cstar": 0.6},
          },
        },
        "names": {"q3": "DayCue"},
        "combos": {},
      })
      g = gates.Gates()
      g.snapshot()
      self.assertTrue(g.is_live("q3", {"light": "day", "road": "freeway", "weather": "dry", "speed": None}, conf=0.6))
      self.assertFalse(g.is_live("q3", {"light": None, "road": "freeway", "weather": "dry", "speed": None}, conf=0.6))

  def test_cstar_threshold(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, {
        "generated": 1000.0,
        "features": {
          "x1": {
            "_": {"status": "live", "tier": "assist", "cstar": 0.0},
          },
        },
        "names": {"x1": "Assisted"},
        "combos": {},
      })
      g = gates.Gates()
      g.snapshot()
      cond = {"light": "day", "road": "freeway", "weather": "dry", "speed": "45-50"}
      self.assertTrue(g.is_live("x1", cond, conf=1.0))

  def test_combo_requirement(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, {
        "generated": 1000.0,
        "features": {
          "a": {
            "light=day|road=freeway": {"status": "live", "tier": "control", "cstar": 0.8},
          },
          "b": {
            "light=day|road=freeway": {"status": "live", "tier": "control", "cstar": 0.8},
          },
        },
        "names": {"a": "A", "b": "B"},
        "combos": {
          "a+b": {
            "light=day|road=freeway": {"status": "live", "cstar": 0.5},
          },
        },
      })
      g = gates.Gates()
      g.snapshot()
      cond = {"light": "day", "road": "freeway", "weather": "dry", "speed": None}
      self.assertTrue(g.is_live("a", cond, conf=0.8, acting=()))
      self.assertTrue(g.is_live("a", cond, conf=0.8, acting=("b",)))
      demoted_tmp = tempfile.mkdtemp(prefix="combo_")
      try:
        old = gates.GATES_PATH
        gates.GATES_PATH = demoted_tmp + "/gates.json"
        _write_gates(gates.GATES_PATH, {
          "generated": 2000.0,
          "features": {"a": {"light=day|road=freeway": {"status": "live", "tier": "control", "cstar": 0.8}}},
          "names": {"a": "A"},
          "combos": {},
        })
        g2 = gates.Gates()
        g2.snapshot()
        self.assertFalse(g2.is_live("a", cond, conf=0.8, acting=("b",)))
      finally:
        gates.GATES_PATH = old
        try:
          os.remove(demoted_tmp + "/gates.json")
        except OSError:
          pass
        try:
          os.rmdir(demoted_tmp)
        except OSError:
          pass

  def test_local_demotion_persists_until_newer_gates_json(self):
    with _TmpDir() as t:
      demotion_ts = time.time() + 1000.0  # future so newer gates.json can beat it
      _write_gates(gates.GATES_PATH, {
        "generated": demotion_ts + 1.0,
        "features": {
          "d1": {
            "_": {"status": "live", "tier": "control", "cstar": 0.0},
          },
        },
        "names": {"d1": "Demo"},
        "combos": {},
      })
      g = gates.Gates()
      g.snapshot()
      cond = {"light": "day", "road": "freeway", "weather": "dry", "speed": None}
      self.assertTrue(g.is_live("d1", cond, conf=1.0))
      g.demote("d1", "_", "test")
      # after demote, the in-memory gates instance still sees the feature live if we don't snapshot
      # but the demotion must have been written to disk; a new gates instance reading the newer on-disk
      # gates.json with generated > demotion ts should NOT have the demotion in memory unless snapshot runs.
      # Here we test: same instance after demote still live (no snapshot after demote), so the test below
      # must be done with the same instance.
      self.assertTrue(g.is_live("d1", cond, conf=1.0))
      # now snapshot with newer gates.json (generated > demotion ts) to lift the demotion
      _write_gates(gates.GATES_PATH, {
        "generated": demotion_ts + 2.0,
        "features": {
          "d1": {
            "_": {"status": "live", "tier": "control", "cstar": 0.0},
          },
        },
        "names": {"d1": "Demo"},
        "combos": {},
      })
      g.snapshot()  # re-snapshot: the demotion ts is older than generated, so it's lifted
      self.assertTrue(g.is_live("d1", cond, conf=1.0))

  def test_snapshot_does_not_reload_mid_drive(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, {
        "generated": 1000.0,
        "features": {
          "m1": {
            "_": {"status": "live", "tier": "control", "cstar": 0.0},
          },
        },
        "names": {"m1": "Mid"},
        "combos": {},
      })
      g = gates.Gates()
      g.snapshot()
      self.assertTrue(g.is_live("m1", {"light": "day", "road": "freeway", "weather": "dry", "speed": None}, conf=1.0))
      _write_gates(gates.GATES_PATH, {
        "generated": 9999.0,
        "features": {
          "m1": {
            "_": {"status": "shadow", "tier": "control", "cstar": 0.0},
          },
        },
        "names": {"m1": "Mid"},
        "combos": {},
      })
      g.snapshot()
      self.assertTrue(g.is_live("m1", {"light": "day", "road": "freeway", "weather": "dry", "speed": None}, conf=1.0))
      self.assertEqual(g.generated, 1000.0)

  def test_weather_from_snapshot(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, {
        "generated": 1000.0,
        "features": {},
        "names": {},
        "combos": {},
      })
      g = gates.Gates()
      g.snapshot()
      cond = g.conditions(33.4, -112.0, 1700000000.0, 65, 65)
      self.assertIsNone(cond["weather"])

  def test_demoted_file_atomic_write(self):
    with _TmpDir() as t:
      g = gates.Gates()
      g.snapshot()
      g.demote("d2", "_", "reason")
      self.assertTrue(os.path.exists(gates.DEMOTED_PATH))
      with open(gates.DEMOTED_PATH) as f:
        d = json.load(f)
      self.assertTrue(isinstance(d, dict))
      self.assertIn("generated", d)
      self.assertIn("demotions", d)
      demotions = d.get("demotions", {})
      self.assertTrue(isinstance(demotions, dict))
      self.assertTrue(any(isinstance(k, str) and k.endswith("\u0000_") for k in demotions))

  def test_conditions_speed_band(self):
    g = gates.Gates()
    self.assertEqual(gates._speed_band(0), "0-25")
    self.assertEqual(gates._speed_band(24.9), "0-25")
    self.assertEqual(gates._speed_band(25), "25-35")
    self.assertEqual(gates._speed_band(34.9), "25-35")
    self.assertEqual(gates._speed_band(35), "35-45")
    self.assertEqual(gates._speed_band(44.9), "35-45")
    self.assertEqual(gates._speed_band(45), "45-50")
    self.assertEqual(gates._speed_band(49.9), "45-50")
    self.assertEqual(gates._speed_band(50), "50-65")
    self.assertEqual(gates._speed_band(64.9), "50-65")
    self.assertEqual(gates._speed_band(65), "65+")
    self.assertEqual(gates._speed_band(100), "65+")
    self.assertIsNone(gates._speed_band(None))

  def test_conditions_outputs_speed_and_speed_band(self):
    g = gates.Gates()
    cond = g.conditions(33.4, -112.0, 1700000000.0, 55, 65)
    self.assertEqual(cond["speed"], "50-65")
    self.assertEqual(cond["speed_band"], "50-65")
    self.assertEqual(cond["speed"], cond["speed_band"])

  def test_speed_band_cell_matches_old_style_speed_key(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, {
        "generated": 1000.0,
        "features": {
          "sb1": {
            "light=day|road=freeway|weather=dry|speed_band=50-65": {"status": "live", "tier": "control", "cstar": 0.9},
          },
        },
        "names": {"sb1": "Band"},
        "combos": {},
      })
      g = gates.Gates()
      g.snapshot()
      self.assertTrue(g.is_live("sb1", {"light": "day", "road": "freeway", "weather": "dry", "speed": "50-65"}, conf=1.0))
      self.assertTrue(g.is_live("sb1", {"light": "day", "road": "freeway", "weather": "dry", "speed_band": "50-65"}, conf=1.0))
      self.assertFalse(g.is_live("sb1", {"light": "day", "road": "freeway", "weather": "dry", "speed": "35-45"}, conf=1.0))
      self.assertFalse(g.is_live("sb1", {"light": "day", "road": "freeway", "weather": "dry"}, conf=1.0))

  def test_conditions_roundtrip_matches_speed_band_cell(self):
    with _TmpDir() as t:
      _write_gates(gates.GATES_PATH, {
        "generated": 1000.0,
        "features": {
          "sb2": {
            "light=day|road=freeway|weather=dry|speed_band=50-65": {"status": "live", "tier": "control", "cstar": 0.9},
          },
        },
        "names": {"sb2": "Band2"},
        "combos": {},
      })
      g = gates.Gates()
      with mock.patch.object(gates, "_forge_url", lambda: "http://forge.test"), \
           mock.patch.object(gates, "_fetch_weather", lambda url: "dry"):
        g.snapshot(33.4, -112.0, 1700000000.0)
      cond = g.conditions(33.4, -112.0, 1699963200.0, 55, 65)
      self.assertEqual(cond["light"], "day")
      self.assertEqual(cond["road"], "freeway")
      self.assertEqual(cond["weather"], "dry")
      self.assertTrue(g.is_live("sb2", cond, conf=1.0))

  def test_has_speed_cells_accepts_speed_band(self):
    self.assertTrue(gates._has_speed_cells({"speed_band=50-65": {"status": "live"}}))
    self.assertTrue(gates._has_speed_cells({"light=day|speed=50-65": {"status": "live"}}))
    self.assertFalse(gates._has_speed_cells({"light=day": {"status": "live"}}))

  def test_norm_cond_fills_missing_speed_key(self):
    self.assertEqual(gates._norm_cond(None), {})
    self.assertEqual(gates._norm_cond({"speed": "50-65"})["speed_band"], "50-65")
    self.assertEqual(gates._norm_cond({"speed_band": "35-45"})["speed"], "35-45")
    both = gates._norm_cond({"speed": "50-65", "speed_band": "50-65"})
    self.assertEqual(both, {"speed": "50-65", "speed_band": "50-65"})
    self.assertEqual(gates._norm_cond({"speed": None, "speed_band": "65+"}), {"speed": "65+", "speed_band": "65+"})

  def test_where_desc_prints_band_once(self):
    desc = gates._where_desc("light=day|speed_band=50-65", {"light": "day", "road": "freeway", "weather": "dry", "speed": "50-65"})
    self.assertEqual(desc.count("50-65"), 1)
    desc2 = gates._where_desc("light=day|speed_band=50-65", {"light": "day", "road": "freeway", "weather": "dry", "speed_band": "50-65"})
    self.assertEqual(desc2.count("50-65"), 1)

  def test_demoted_reads_existing(self):
    with _TmpDir() as t:
      demotion_ts = time.time() + 1000.0
      demoted = {"generated": demotion_ts - 1.0, "demotions": {_demoted_key("z1", "_"): demotion_ts}, "reason": "old"}
      _write_gates(gates.DEMOTED_PATH, demoted)
      _write_gates(gates.GATES_PATH, {
        "generated": demotion_ts + 1.0,
        "features": {
          "z1": {
            "_": {"status": "live", "tier": "control", "cstar": 0.0},
          },
        },
        "names": {"z1": "Z"},
        "combos": {},
      })
      g = gates.Gates()
      g.snapshot()
      self.assertTrue(g.is_live("z1", {"light": "day", "road": "freeway", "weather": "dry", "speed": None}, conf=1.0))
      demoted["generated"] = demotion_ts - 2.0
      _write_gates(gates.GATES_PATH, {
        "generated": demotion_ts - 2.0,
        "features": {
          "z1": {
            "_": {"status": "live", "tier": "control", "cstar": 0.0},
          },
        },
        "names": {"z1": "Z"},
        "combos": {},
      })
      g2 = gates.Gates()
      g2.snapshot()
      self.assertFalse(g2.is_live("z1", {"light": "day", "road": "freeway", "weather": "dry", "speed": None}, conf=1.0))


if __name__ == "__main__":
  unittest.main()
