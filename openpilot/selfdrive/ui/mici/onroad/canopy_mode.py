"""Canopy parking modes on the comma screen: which one is on, and how its path looks.

Garage mode draws the driving path as amber dashes; lot mode (scouting a lot for a space) as purple
dots. Either way the path no longer looks like normal driving, so he can tell at a glance that a
parking mode is on. The state comes from the live files garage.py (and lot mode) write on tmpfs,
checked twice a second and read only when they change. A file older than STALE_S counts as off.
"""
import json
import os
import time

import numpy as np
import pyray as rl

from openpilot.sunnypilot.canopy import banner
from openpilot.system.ui.lib.shader_polygon import draw_polygon

LIVE = {"garage": os.path.join(banner.BANNER_DIR, "garage_live.json"),
        "lot": os.path.join(banner.BANNER_DIR, "lot_live.json")}
CHECK_S = 0.5
STALE_S = 5.0

AMBER = rl.Color(255, 176, 46, 200)
PURPLE = rl.Color(176, 120, 255, 200)
# (color, on px, off px) measured along the path's centre line on screen
LOOK = {"garage": (AMBER, 44, 26), "lot": (PURPLE, 12, 16)}


class CanopyMode:
  def __init__(self):
    self._next = 0.0
    self._mt = {k: None for k in LIVE}
    self._on = {k: False for k in LIVE}
    self._t = {k: 0.0 for k in LIVE}

  def current(self) -> str | None:
    now = time.monotonic()
    if now >= self._next:
      self._next = now + CHECK_S
      for k, p in LIVE.items():
        try:
          m = os.path.getmtime(p)
        except OSError:
          self._mt[k], self._on[k] = None, False
          continue
        if m != self._mt[k]:
          self._mt[k] = m
          try:
            with open(p) as f:
              self._on[k] = bool(json.load(f).get("active"))
          except (OSError, ValueError):
            self._on[k] = False
        self._t[k] = time.time() - m
    for k in ("garage", "lot"):
      if self._on[k] and self._t[k] < STALE_S:
        return k
    return None


def dash_runs(pts: np.ndarray, on_px: float, off_px: float) -> list[np.ndarray]:
  """Split a ribbon polygon [L0..Lk-1, Rk-1..R0] into dash polygons along its centre line."""
  n = len(pts) // 2
  if n < 2:
    return []
  left, right = pts[:n], pts[n:][::-1]
  centre = (left + right) / 2
  d = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(centre, axis=0), axis=1))])
  on = (d % (on_px + off_px)) < on_px
  runs, start = [], None
  for i in range(n):
    if on[i] and start is None:
      start = i
    if (not on[i] or i == n - 1) and start is not None:
      end = i + 1  # the first point past the dash closes it
      if end - start >= 2:
        runs.append(np.concatenate([left[start:end], right[start:end][::-1]]))
      start = None
  return runs


def draw_mode_path(rect: rl.Rectangle, pts: np.ndarray, mode: str) -> None:
  color, on_px, off_px = LOOK[mode]
  for poly in dash_runs(pts, on_px, off_px):
    draw_polygon(rect, poly, color)
