"""Learned parameters read once at drive start from /data/canopy/params.json.

env CANOPY_PARAMS overrides the path; CANOPY_PARAMS_IN_USE overrides the
in-use snapshot path (defaults below). Unknown names in the file are ignored.
"""
import json
import os
import time

PARAMS_PATH = os.environ.get("CANOPY_PARAMS", "/data/canopy/params.json")
IN_USE_PATH = os.environ.get("CANOPY_PARAMS_IN_USE", "/data/canopy/params_in_use.json")

# (default, lo, hi). None default = keep the existing param/behavior.
BOUNDS = {
    "lane_change_speed_min_mph": (25.0, 15.0, 35.0),
    "lane_change_auto_min_mph": (50.0, 40.0, 65.0),
    "lane_turn_max_mph": (None, 10.0, 25.0),
    "voice_merge_delay_s": (1.5, 1.0, 3.0),
    "voice_merge_window_s": (8.0, 5.0, 12.0),
}

_cache = None


def load() -> dict:
  """Read PARAMS_PATH once per process; later calls return the cached dict."""
  global _cache
  if _cache is not None:
    return _cache

  result = {}
  source = {}
  file_version = "default"

  try:
    with open(PARAMS_PATH) as f:
      obj = json.load(f)
    if not isinstance(obj, dict):
      obj = {}
    file_version = obj.get("version", "default")
    raw_params = obj.get("params")
    if not isinstance(raw_params, dict):
      raw_params = {}
  except Exception:
    raw_params = {}

  for name, (default, lo, hi) in BOUNDS.items():
    v = raw_params.get(name)
    if isinstance(v, (int, float)) and not isinstance(v, bool):
      if lo is not None:
        v = max(lo, min(hi, float(v)))
      else:
        v = float(v)
    else:
      v = default
    result[name] = v
    source[name] = "file" if name in raw_params else "default"

  # sanity: auto_min must be at least speed_min + 10
  speed_min = result["lane_change_speed_min_mph"]
  auto_min = result["lane_change_auto_min_mph"]
  if auto_min < speed_min + 10:
    result["lane_change_auto_min_mph"] = speed_min + 10
    source["lane_change_auto_min_mph"] = "derived"

  _cache = result

  # write in-use snapshot atomically; ignore write errors
  try:
    snap = {
      "t": time.time(),
      "version": file_version,
      "params": dict(result),
      "source": dict(source),
    }
    tmp = IN_USE_PATH + ".tmp"
    with open(tmp, "w") as f:
      json.dump(snap, f)
    os.replace(tmp, IN_USE_PATH)
  except Exception:
    pass

  return result
