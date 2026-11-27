"""Canopy: how hard the Leaf pulls the wheel, and only down at parking-lot speed.

opendbc's Nissan carcontroller reads CarControllerParams.LKAS_MAX_TORQUE every frame when it
builds the LKAS message (stock is 1.0 Nm, the CAN field takes 0..2.55). card.py runs in the
same process as that controller, so setting the class attribute here changes the very next
frame.

The file /data/canopy/assist_level holds one integer level, 1..5, which maps to 1.0..2.0 Nm of
LKAS torque; no file means level 1 (stock). Levels above stock are only applied below MAX_MPH,
since more torque at road speed is a hand-fight. If the EPS reports a temperature fault while
lateral is active and we were pulling harder than stock, we latch back to stock for the rest of
the drive: a hot motor is not a place to keep leaning on.
"""
import json
import math
import os
import time

FEATURE = "assist_torque"
CFG_PATH = "/data/canopy/assist_level"
LOG_PATH = "/data/canopy/assist_torque.jsonl"
STOCK_NM = 1.0
MAX_NM = 2.0
LEVEL_MIN = 1
LEVEL_MAX = 5
MAX_MPH = 20.0
MPH_TO_MS = 0.44704
READ_EVERY = 100  # frames between cfg reads, ~1 s at 100 Hz


def level_to_nm(level: int) -> float:
  """Level 1 is stock; each step adds a quarter Nm up to 2.0 at level 5."""
  try:
    level = int(round(level))
  except Exception:
    level = LEVEL_MIN
  level = max(LEVEL_MIN, min(LEVEL_MAX, level))
  return round(STOCK_NM + 0.25 * (level - LEVEL_MIN), 4)


def read_level(path: str = CFG_PATH) -> int:
  """The configured level, 1..5. Missing, empty or unparsable means level 1."""
  try:
    with open(path, encoding="utf-8") as f:
      level = float(f.read().strip())
    if not math.isfinite(level):
      return LEVEL_MIN
  except Exception:
    return LEVEL_MIN
  return max(LEVEL_MIN, min(LEVEL_MAX, int(round(level))))


def write_level(level: int, path: str = CFG_PATH) -> None:
  """Persist a level, clamped to 1..5, atomically. Never raises."""
  try:
    level = max(LEVEL_MIN, min(LEVEL_MAX, int(round(level))))
    parent = os.path.dirname(path)
    if parent:
      os.makedirs(parent, exist_ok=True)
    tmp = f"{path}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
      f.write(f"{level}\n")
    os.replace(tmp, path)
  except Exception:
    pass


class AssistTorque:
  def __init__(self, cfg_path=CFG_PATH, log_path=LOG_PATH, target=None):
    self.cfg_path = cfg_path
    self.log_path = log_path
    self.n = 0
    self.level = LEVEL_MIN
    self.configured = STOCK_NM
    self.applied = None
    self.latched = False
    if target is None:
      try:
        from opendbc.car.nissan.values import CarControllerParams
        target = CarControllerParams
      except Exception:
        target = None
    self.target = target

  def _read_cfg(self) -> tuple[int, float]:
    level = read_level(self.cfg_path)
    nm = min(MAX_NM, max(STOCK_NM, level_to_nm(level)))
    return level, nm

  def _log(self, rec: dict) -> None:
    try:
      parent = os.path.dirname(self.log_path)
      if parent:
        os.makedirs(parent, exist_ok=True)
      with open(self.log_path, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec, separators=(",", ":")) + "\n")
    except Exception:
      pass

  def update(self, v_ego: float, steer_fault_temp: bool, lat_active: bool) -> float:
    try:
      if self.n % READ_EVERY == 0:
        self.level, self.configured = self._read_cfg()
      self.n += 1

      if self.latched:
        applied = STOCK_NM
      else:
        # anything above stock only counts below MAX_MPH
        applied = self.configured if v_ego < MAX_MPH * MPH_TO_MS else STOCK_NM
        if steer_fault_temp and lat_active and applied > STOCK_NM:
          self._log({"t": time.time(), "event": "fault_latch", "level": self.level,
                     "nm": applied, "v": v_ego})
          self.latched = True
          applied = STOCK_NM

      if applied != self.applied:
        self._log({"t": time.time(), "event": "applied", "level": self.level, "nm": applied,
                   "configured": self.configured, "v": v_ego})
        self.applied = applied

      if self.target is not None:
        self.target.LKAS_MAX_TORQUE = applied
      return applied
    except Exception:
      try:
        if self.target is not None:
          self.target.LKAS_MAX_TORQUE = STOCK_NM
      except Exception:
        pass
      return STOCK_NM
