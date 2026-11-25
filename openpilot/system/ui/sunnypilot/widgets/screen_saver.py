"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Canopy screen saver: a flock of little leaf-winged Leafs drifting across the night,
with loose leaves tumbling past. Original art in the Canopy look.
"""
import math
import os
import random
import time

import pyray as rl

from openpilot.common.hardware import HARDWARE
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

SAVER_DIR = "../../sunnypilot/selfdrive/assets/images/canopy/saver"
FLAP_FRAMES = 4
LEAF_TINTS = 3
FLAP_FPS = 7.0


class _Sprite:
  __slots__ = ("x", "y", "depth", "phase", "spin", "angle", "kind", "sway")

  def __init__(self, x, y, depth, phase, spin=0.0, angle=0.0, kind=0, sway=0.0):
    self.x, self.y, self.depth, self.phase = x, y, depth, phase
    self.spin, self.angle, self.kind, self.sway = spin, angle, kind, sway


class ScreenSaverSP(Widget):
  def __init__(self, params: Params | None = None):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    self._params = params or Params()
    self._is_mici = HARDWARE.get_device_type() == 'mici' or (HARDWARE.get_device_type() == "pc" and os.getenv("BIG") != "1")

    self.text = "Canopy"
    self._unit = 1.0 if self._is_mici else 2.6   # mici is 536x240; bigger screens scale up
    self._flyer_w = 78 * self._unit                # flyer drawn at this width at depth 1.0
    self._leaf_w = 24 * self._unit
    self._speed = 46.0 * self._unit                # px/s at depth 1.0

    self._start_time = None
    self._dismiss = False
    self._screensaver_timeout = 300
    self._rng = random.Random()
    self._flyers: list[_Sprite] = []
    self._leaves: list[_Sprite] = []
    self._tex_flyer: list = []
    self._tex_leaf: list = []

  @property
  def is_active(self) -> bool:
    return self._start_time is not None and not self._dismiss

  @property
  def was_dismissed(self) -> bool:
    return self._dismiss

  def initialize(self):
    self._screensaver_timeout = self._params.get("ScreenSaverTimeout", return_default=True)
    if self._start_time is None:
      self._start_time = time.monotonic()
      self._seed()
    self._dismiss = False

  def hide_event(self):
    super().hide_event()
    self._dismiss = False
    self._start_time = None

  def _handle_mouse_release(self, mouse_pos):
    self._dismiss = True
    self._start_time = None
    gui_app.pop_widget()
    return super()._handle_mouse_release(mouse_pos)

  # flock

  def _load(self):
    if self._tex_flyer:
      return
    self._tex_flyer = [gui_app.texture(f"{SAVER_DIR}/flyer_{i}.png") for i in range(FLAP_FRAMES)]
    self._tex_leaf = [gui_app.texture(f"{SAVER_DIR}/leaf_{i}.png") for i in range(LEAF_TINTS)]

  def _seed(self):
    w, h, r = self.rect.width, self.rect.height, self._rng
    # three depths: far ones are small, slow and dim, near ones big and quick
    depths = [0.55, 0.55, 0.75, 0.75, 1.0] if self._is_mici else [0.55, 0.55, 0.7, 0.7, 0.85, 1.0, 1.0]
    self._flyers = [_Sprite(r.uniform(0, w), r.uniform(0, h), d, r.uniform(0, FLAP_FRAMES)) for d in depths]
    self._leaves = [_Sprite(r.uniform(0, w), r.uniform(0, h), r.uniform(0.6, 1.1), r.uniform(0, 6.28),
                            spin=r.uniform(-90, 90), angle=r.uniform(0, 360), kind=r.randrange(LEAF_TINTS),
                            sway=r.uniform(0.6, 1.4))
                    for _ in range(7 if self._is_mici else 12)]

  def _respawn(self, s, size):
    # re-enter from the left or bottom edge, so the flock keeps flowing up and to the right
    w, h, r = self.rect.width, self.rect.height, self._rng
    if r.random() < w / (w + h):
      s.x, s.y = r.uniform(-size, w * 0.6), h + r.uniform(0, size)
    else:
      s.x, s.y = -size - r.uniform(0, size), r.uniform(h * 0.3, h + size)

  def _update_state(self):
    super()._update_state()

    if self._start_time and time.monotonic() - self._start_time > self._screensaver_timeout:
      self._dismiss = True
      self._start_time = None

    if not self._flyers:
      self._seed()

    dt = min(rl.get_frame_time(), 0.1)
    w = self.rect.width

    for f in self._flyers:
      v = self._speed * f.depth
      f.x += v * dt
      f.y -= v * 0.5 * dt
      f.phase = (f.phase + FLAP_FPS * dt * (0.8 + 0.4 * f.depth)) % FLAP_FRAMES
      size = self._flyer_w * f.depth
      if f.x > w + size or f.y < -size:
        self._respawn(f, size)

    for lf in self._leaves:
      v = self._speed * 0.75 * lf.depth
      lf.phase += dt * 1.6 * lf.sway
      lf.x += (v + 10 * self._unit * math.sin(lf.phase)) * dt
      lf.y -= v * 0.35 * dt
      lf.angle = (lf.angle + lf.spin * dt) % 360
      size = self._leaf_w * lf.depth
      if lf.x > w + size or lf.y < -size:
        self._respawn(lf, size)

  def _render(self, rect: rl.Rectangle):
    self.set_rect(rect)
    rl.clear_background(rl.BLACK)
    self._load()

    # far to near, leaves mixed in by depth
    items = [(f.depth, 0, f) for f in self._flyers] + [(lf.depth, 1, lf) for lf in self._leaves]
    for depth, kind, s in sorted(items, key=lambda it: it[0]):
      shade = int(110 + 145 * min(1.0, (depth - 0.5) / 0.5))
      tint = rl.Color(shade, shade, shade, 255)
      if kind == 0:
        tex = self._tex_flyer[int(s.phase) % FLAP_FRAMES]
        dw = self._flyer_w * depth
        dh = dw * tex.height / tex.width
        rl.draw_texture_pro(tex, rl.Rectangle(0, 0, tex.width, tex.height),
                            rl.Rectangle(rect.x + s.x, rect.y + s.y, dw, dh), rl.Vector2(dw / 2, dh / 2), 0, tint)
      else:
        tex = self._tex_leaf[s.kind]
        dw = self._leaf_w * depth
        rl.draw_texture_pro(tex, rl.Rectangle(0, 0, tex.width, tex.height),
                            rl.Rectangle(rect.x + s.x, rect.y + s.y, dw, dw), rl.Vector2(dw / 2, dw / 2), s.angle, tint)
    return -1
