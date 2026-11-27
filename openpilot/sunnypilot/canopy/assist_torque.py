"""Canopy: how hard the Leaf pulls the wheel, and only down at parking-lot speed.

opendbc's Nissan carcontroller reads CarControllerParams.LKAS_MAX_TORQUE every frame when it
builds the LKAS message (stock is 1.0 Nm, the CAN field takes 0..2.55). card.py runs in the
same process as that controller, so setting the class attribute here changes the very next
frame.

The file /data/canopy/assist_torque holds one float in Nm; no file means stock. Anything above
stock is only applied below MAX_MPH, since more torque at road speed is a hand-fight. Asking for
less than stock is always allowed, at any speed. If the EPS reports a temperature fault while
lateral is active and we were pulling harder than stock, we latch back to stock for the rest of
the drive: a hot motor is not a place to keep leaning on.
"""
import json
import math
import os
import time

FEATURE = "assist_torque"
CFG_PATH = "/data/canopy/assist_torque"
LOG_PATH = "/data/canopy/assist_torque.jsonl"
STOCK_NM = 1.0
MIN_NM = 0.5
MAX_NM = 2.0
MAX_MPH = 20.0
MPH_TO_MS = 0.44704
READ_EVERY = 100  # frames between cfg reads, ~1 s at 100 Hz


class AssistTorque:
  def __init__(self, cfg_path=CFG_PATH, log_path=LOG_PATH, target=None):
    self.cfg_path = cfg_path
    self.log_path = log_path
    self.n = 0
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

  def _read_cfg(self) -> float:
    try:
      with open(self.cfg_path, encoding="utf-8") as f:
        nm = float(f.read().strip())
      if not math.isfinite(nm):
        return STOCK_NM
    except Exception:
      return STOCK_NM
    return max(MIN_NM, min(MAX_NM, nm))

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
        self.configured = self._read_cfg()
      self.n += 1

      if self.latched:
        applied = STOCK_NM
      else:
        # stronger than stock only counts below MAX_MPH; weaker is fine anywhere
        applied = self.configured if (v_ego < MAX_MPH * MPH_TO_MS or self.configured < STOCK_NM) else STOCK_NM
        if steer_fault_temp and lat_active and applied > STOCK_NM:
          self._log({"t": time.time(), "event": "fault_latch", "nm": applied, "v": v_ego})
          self.latched = True
          applied = STOCK_NM

      if applied != self.applied:
        self._log({"t": time.time(), "event": "applied", "nm": applied,
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
