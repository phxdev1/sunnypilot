"""Canopy status banner on the comma screen (state written by sunnypilot/canopy/banner.py).

A small Lantern-style pill at the bottom of the camera view: dark panel, a cyan ring for
info, an amber ring for a heads-up, an amber dashed ring for garage mode or a purple dotted ring
for lot mode, one line of Wick text. It never covers a real openpilot
alert: while one is showing (or animating out) the banner fades away. It never makes a sound.
The state file is checked twice a second, read only when it changed.
"""
from pathlib import Path

import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.sunnypilot.canopy import banner
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

WICK = Path(__file__).resolve().parents[4] / "sunnypilot/selfdrive/assets/images/canopy/Wick-Regular.ttf"

FONT_SIZE = 30
PAD_X = 22
DOT_R = 9
DOT_GAP = 14
HEIGHT = 58
BOTTOM_MARGIN = 18
CHECK_S = 0.5

PANEL = rl.Color(17, 20, 24, 235)        # Lantern surface
PANEL_EDGE = rl.Color(255, 255, 255, 28)
TEXT = rl.Color(240, 242, 245, 255)
RING = {
  "info": rl.Color(108, 211, 240, 255),     # Lantern cyan
  "heads_up": rl.Color(255, 176, 46, 255),  # amber
  "garage": rl.Color(255, 176, 46, 255),    # amber, dashed ring (garage mode)
  "lot": rl.Color(176, 120, 255, 255),      # purple, dotted ring (lot mode)
}
RING_DASH = {"garage": (8, 0.6), "lot": (14, 0.35)}  # (pieces, share of each piece drawn)


class CanopyBanner(Widget):
  def __init__(self):
    super().__init__()
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    if WICK.exists():
      font = rl.load_font_ex(WICK.as_posix(), FONT_SIZE * 2, rl.ffi.NULL, 0)
      if font.texture.id != 0:
        rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
        self._font = font
    self._alpha = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)
    self._next_check = 0.0
    self._mtime = -1.0
    self._data: dict | None = None
    self._shown: dict | None = None
    self._blocked = False
    self._want = False

  def set_blocked(self, blocked: bool):
    """True while an openpilot alert is on screen: the banner steps aside."""
    self._blocked = blocked

  def _update_state(self):
    now = rl.get_time()
    if now >= self._next_check:
      self._next_check = now + CHECK_S
      try:
        m = banner.mtime()
        if m != self._mtime:
          self._mtime = m
          self._data = banner._read() if m else None
        cur = banner.current(d={"entries": dict(self._data["entries"])}) if self._data else None
      except Exception:
        cur = None
      if cur is not None:
        self._shown = cur
      self._want = cur is not None
    want = self._want and not self._blocked
    self._alpha.update(1.0 if want else 0.0)

  def _render(self, rect: rl.Rectangle):
    if self._alpha.x < 0.02 or self._shown is None:
      return
    a = self._alpha.x
    text = self._shown["text"]
    ring = RING.get(self._shown["style"], RING["info"])
    room = rect.width - 24 - (PAD_X * 2 + DOT_R * 2 + DOT_GAP)
    size = FONT_SIZE
    tw = measure_text_cached(self._font, text, size).x
    if tw > room:  # long text: smaller type rather than clipped words
      size = max(18, int(size * room / tw))
      tw = measure_text_cached(self._font, text, size).x
    w = min(rect.width - 24, PAD_X * 2 + DOT_R * 2 + DOT_GAP + tw)
    x = rect.x + (rect.width - w) / 2
    y = rect.y + rect.height - HEIGHT - BOTTOM_MARGIN + (1.0 - a) * 12  # small rise as it fades in
    box = rl.Rectangle(x, y, w, HEIGHT)

    def fade(c: rl.Color) -> rl.Color:
      return rl.Color(c.r, c.g, c.b, int(c.a * a))

    rl.draw_rectangle_rounded(box, 1.0, 16, fade(PANEL))
    rl.draw_rectangle_rounded_lines_ex(box, 1.0, 16, 2, fade(PANEL_EDGE))
    cx, cy = x + PAD_X + DOT_R, y + HEIGHT / 2
    dash = RING_DASH.get(self._shown["style"])
    if dash:  # parking modes: a broken ring that matches their broken path
      pieces, share = dash
      step = 360.0 / pieces
      for i in range(pieces):
        a0 = i * step
        rl.draw_ring(rl.Vector2(cx, cy), DOT_R - 2, DOT_R + 0.5, a0, a0 + step * share, 6, fade(ring))
    else:
      rl.draw_circle_lines(int(cx), int(cy), DOT_R, fade(ring))
      rl.draw_circle_lines(int(cx), int(cy), DOT_R - 1, fade(ring))
    rl.draw_circle(int(cx), int(cy), DOT_R - 4, fade(rl.Color(ring.r, ring.g, ring.b, 90)))
    tx = cx + DOT_R + DOT_GAP
    rl.draw_text_ex(self._font, text, rl.Vector2(tx, cy - size / 2), size, 0, fade(TEXT))
