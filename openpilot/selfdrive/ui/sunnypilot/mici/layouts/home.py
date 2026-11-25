"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import pyray as rl

from openpilot.selfdrive.ui.mici.layouts.home import MiciHomeLayout
from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
from openpilot.system.ui.lib.application import gui_app

CANOPY_WORDMARK = "../../sunnypilot/selfdrive/assets/images/canopy/canopy_wordmark.png"
WORDMARK_ASPECT = 699 / 177


class CanopyWordmark:
  """Draws the Canopy wordmark where the home screen drew its brand label."""
  def __init__(self, height: int = 72, slot: int = 88):
    self.font_size = slot  # the parent lays out the version line under this
    self._top = (slot - height) // 2 + 8
    self._tex = gui_app.texture(CANOPY_WORDMARK, round(height * WORDMARK_ASPECT), height)
    self._x = self._y = 0.0

  def set_position(self, x: float, y: float):
    self._x, self._y = x, y

  def render(self, *_):
    rl.draw_texture_v(self._tex, rl.Vector2(self._x + 4, self._y + self._top), rl.WHITE)


class MiciHomeLayoutSP(MiciHomeLayout):
  def __init__(self):
    super().__init__()
    self._openpilot_label = CanopyWordmark()

  def _set_chestnut_visibility(self):
    usb_connected = ui_state.usb_connected
    usb_unknown = ui_state.usb_unknown
    chestnut_state = ui_state.chestnut_state
    loading = chestnut_state == ChestnutState.LOADING

    self._usb_icon.set_visible(usb_connected and usb_unknown)
    self._chestnut_loading_icon.set_opacity(0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0)))
    self._chestnut_loading_icon.set_visible(not usb_unknown and loading)
    self._chestnut_icon.set_visible(not usb_unknown and not loading and
                                    chestnut_state in (ChestnutState.READY, ChestnutState.ACTIVE))
    self._chestnut_failed_icon.set_visible(not usb_unknown and chestnut_state in (ChestnutState.UNCOMPILED, ChestnutState.FAILED))
