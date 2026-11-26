"""Append-only decision log for the canopy gate: one JSON line per decision."""
import json
import os
import time

PATH = "/data/canopy/decisions.jsonl"
MAX_BYTES = 20 * 1024 * 1024
MIN_GAP_S = 0.5

_last = {}


def log_decision(feature: str, action: str, conf: float, ctx: dict | None = None,
                 live: bool = False, lat: float | None = None, lon: float | None = None,
                 head: str | None = None, base: str | None = None,
                 shadow: dict | None = None) -> None:
  """Log one gate decision; rate-limited per (feature, action), never raises.

  head/base/shadow are the decision head's stamp (heads.py): the live head_id, the
  base_id and the shadow head_ids with their confidences. Only when one is given do
  the record gain those fields, so older records stay as they are.
  """
  try:
    now = time.monotonic()
    key = (feature, action)
    if now - _last.get(key, -MIN_GAP_S - 1.0) < MIN_GAP_S:
      return None
    _last[key] = now
    try:
      from openpilot.sunnypilot.canopy import leadkind
      drive = leadkind.route_name()
    except Exception:
      drive = "unknown"
    rec = {
      "type": "decision",
      "feature": feature,
      "action": action,
      "conf": round(float(conf), 3),
      "drive": drive,
      "ts": round(time.time(), 3),
      "live": bool(live),
      "ctx": "|".join(sorted(f"{k}={v}" for k, v in (ctx or {}).items() if v is not None)),
      "lat": lat,
      "lon": lon,
    }
    if head is not None or base is not None or shadow is not None:
      rec["head"] = head
      rec["base"] = base
      rec["shadow"] = shadow
    try:
      if os.path.getsize(PATH) >= MAX_BYTES:
        return None
    except OSError:
      pass
    os.makedirs(os.path.dirname(PATH) or ".", exist_ok=True)
    with open(PATH, "a", encoding="utf-8") as f:
      f.write(json.dumps(rec, separators=(",", ":")) + "\n")
  except Exception:
    pass
  return None
