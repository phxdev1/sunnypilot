"""Canopy: learned speed offsets over the posted limit, per posted limit in mph.

Forge learns how Mark drives relative to the posted limit after every drive and writes
/data/canopy/speed_offsets.json on the comma:
  {"by_limit_mph": {"25": 0.0, "45": 2.0, "65": 5.0}, "allow_above": false, "updated": "..."}
allow_above stays false until Mark okays going over a posted limit; until then any positive
offset is clamped to 0, so the sync never sets above the limit.
No file, an unreadable file or no entry within 5 mph of the limit means no learned offset.
"""
import json
import os

PATH = "/data/canopy/speed_offsets.json"
MAX_ABS_MPH = 10.0
NEAREST_MPH = 5


class LearnedSpeedOffsets:
  def __init__(self, path: str = PATH):
    self.path = path
    self._mtime: float | None = None
    self.table: dict[int, float] = {}
    self.allow_above = False

  def refresh(self) -> None:
    try:
      mtime = os.stat(self.path).st_mtime
    except OSError:
      self._mtime, self.table, self.allow_above = None, {}, False
      return
    if mtime == self._mtime:
      return
    self._mtime = mtime
    try:
      with open(self.path) as f:
        d = json.load(f)
      self.table = {int(k): float(v) for k, v in d.get("by_limit_mph", {}).items()}
      self.allow_above = d.get("allow_above") is True
    except (OSError, ValueError, TypeError, AttributeError):
      self.table, self.allow_above = {}, False

  def has(self, limit_mph: float) -> bool:
    return self._nearest(limit_mph) is not None

  def _nearest(self, limit_mph: float) -> int | None:
    if not self.table or limit_mph <= 0:
      return None
    k = min(self.table, key=lambda x: abs(x - limit_mph))
    return k if abs(k - limit_mph) <= NEAREST_MPH else None

  def offset_mph(self, limit_mph: float) -> float:
    k = self._nearest(limit_mph)
    if k is None:
      return 0.0
    off = max(-MAX_ABS_MPH, min(MAX_ABS_MPH, self.table[k]))
    if not self.allow_above:
      off = min(off, 0.0)
    return off
