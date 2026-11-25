#!/usr/bin/env python3
import pyray as rl
import select
import sys
from pathlib import Path

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

# Constants
if gui_app.big_ui():
  PROGRESS_BAR_WIDTH = 1000
  PROGRESS_BAR_HEIGHT = 20
  TEXTURE_SIZE = 360
  WRAPPED_SPACING = 50
  CENTERED_SPACING = 150
else:
  PROGRESS_BAR_WIDTH = 268
  PROGRESS_BAR_HEIGHT = 10
  TEXTURE_SIZE = 140
  WRAPPED_SPACING = 10
  CENTERED_SPACING = 20
DEGREES_PER_SECOND = 360.0  # one full rotation per second
MARGIN_H = 100
FONT_SIZE = 96
LINE_HEIGHT = 104
DARKGRAY = (55, 55, 55, 255)

# Canopy boot lockup: animated leaf on the left, "Canopy" and a tagline that changes every start on the right
CANOPY_FRAMES = 24  # lane dashes flow down the leaf's road
CANOPY_FPS = 24.0
CANOPY_ASSETS = "../../sunnypilot/selfdrive/assets/images/canopy"
CANOPY_DIR = Path(__file__).resolve().parents[2] / "sunnypilot/selfdrive/assets/images/canopy"
TAGLINE_STATE = Path("/data/canopy_tagline_index")
TAGLINE_COLOR = rl.Color(163, 170, 180, 255)  # Lantern muted text
K = TEXTURE_SIZE / 140.0  # everything below is laid out for the 536x240 screen and scaled up for big screens
GAP = -12 * K             # leaf to text; the leaf image has ~35 px of clear space on its right
WORDMARK_H = 50 * K       # "Canopy" image height
WORDMARK_ASPECT = 699 / 177  # canopy_wordmark.png width / height
TAGLINE_SIZE = 18 * K     # before the UI's font scale
TAGLINE_LINE = 24 * K
TAGLINE_TOP = 10 * K      # wordmark to tagline
EDGE = 16 * K
STATUS_ROOM = 40 * K      # strip under the lockup for the progress bar or a status line
STATUS_BAR_H = 6 * K


def clamp(value, min_value, max_value):
  return max(min(value, max_value), min_value)


def next_tagline() -> str:
  """One line from taglines.txt, a different one each start, cycling through the whole list."""
  try:
    lines = [t.strip() for t in (CANOPY_DIR / "taglines.txt").read_text().splitlines() if t.strip() and not t.startswith("#")]
    if not lines:
      return ""
    try:
      idx = int(TAGLINE_STATE.read_text().strip())
    except Exception:
      idx = 0
    try:
      TAGLINE_STATE.write_text(str((idx + 1) % len(lines)))
    except Exception:
      pass
    return lines[idx % len(lines)]
  except Exception:
    return ""


class Spinner(Widget):
  def __init__(self):
    super().__init__()
    self._frames = [gui_app.texture(f"{CANOPY_ASSETS}/spinner_canopy_{i:02d}.png", TEXTURE_SIZE, TEXTURE_SIZE)
                    for i in range(CANOPY_FRAMES)]
    self._wordmark = gui_app.texture(f"{CANOPY_ASSETS}/canopy_wordmark.png", WORDMARK_H * WORDMARK_ASPECT, WORDMARK_H)
    self._anim_t = 0.0
    self._progress: int | None = None
    self._wrapped_lines: list[str] = []

    self._tag_font = gui_app.font()
    wick = CANOPY_DIR / "Wick-Regular.ttf"
    if wick.exists():
      font = rl.load_font_ex(wick.as_posix(), int(TAGLINE_SIZE * 3), rl.ffi.NULL, 0)
      if font.texture.id != 0:
        rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
        self._tag_font = font
    max_w = gui_app.width - 2 * EDGE - TEXTURE_SIZE - GAP
    self._tag_lines = self._wrap(next_tagline(), max_w, 2)

  def _wrap(self, text: str, max_w: float, max_lines: int) -> list[str]:
    fits = lambda t: measure_text_cached(self._tag_font, t, TAGLINE_SIZE).x <= max_w
    if fits(text):
      return [text]
    # two-beat taglines break between sentences, so no line ends on a lone word
    cuts = [i + 1 for i, c in enumerate(text[:-1]) if c in ".,!?" and text[i + 1] == " "]
    for i in sorted(cuts, key=lambda i: abs(i - len(text) / 2)):
      a, b = text[:i].strip(), text[i:].strip()
      if fits(a) and fits(b):
        return [a, b]
    lines: list[str] = []
    for word in text.split():
      if lines and measure_text_cached(self._tag_font, f"{lines[-1]} {word}", TAGLINE_SIZE).x <= max_w:
        lines[-1] = f"{lines[-1]} {word}"
      else:
        lines.append(word)
    return lines[:max_lines]

  def set_text(self, text: str) -> None:
    if text.isdigit():
      self._progress = clamp(int(text), 0, 100)
      self._wrapped_lines = []
    else:
      self._progress = None
      self._wrapped_lines = [text.strip()] if text.strip() else []

  def _leaf(self):
    self._anim_t = (self._anim_t + rl.get_frame_time()) % (CANOPY_FRAMES / CANOPY_FPS)
    return self._frames[int(self._anim_t * CANOPY_FPS) % CANOPY_FRAMES]

  def _render(self, rect: rl.Rectangle):
    # the lockup (leaf, wordmark, tagline) always shows; progress and status sit under it
    status = self._progress is not None or bool(self._wrapped_lines)
    lift = STATUS_ROOM / 2.0 if status else 0.0
    self._render_lockup(rl.Rectangle(rect.x, rect.y - lift, rect.width, rect.height))
    base = rect.y + rect.height - STATUS_ROOM

    if self._progress is not None:
      w = min(PROGRESS_BAR_WIDTH, rect.width * 0.6)
      bar = rl.Rectangle(rect.x + (rect.width - w) / 2.0, base + (STATUS_ROOM - STATUS_BAR_H) / 2.0, w, STATUS_BAR_H)
      rl.draw_rectangle_rounded(bar, 1, 10, DARKGRAY)
      bar.width *= self._progress / 100.0
      rl.draw_rectangle_rounded(bar, 1, 10, TAGLINE_COLOR)
    elif self._wrapped_lines:
      line = self._wrapped_lines[0]
      size = measure_text_cached(self._tag_font, line, TAGLINE_SIZE)
      rl.draw_text_ex(self._tag_font, line, rl.Vector2(rect.x + (rect.width - size.x) / 2.0, base + (STATUS_ROOM - TAGLINE_SIZE) / 2.0),
                      TAGLINE_SIZE, 0.0, rl.WHITE)

  def _render_lockup(self, rect: rl.Rectangle):
    wm_w, wm_h = self._wordmark.width, self._wordmark.height
    tag_w = max((measure_text_cached(self._tag_font, t, TAGLINE_SIZE).x for t in self._tag_lines), default=0)
    text_w = max(wm_w, tag_w)
    text_h = wm_h + (TAGLINE_TOP + len(self._tag_lines) * TAGLINE_LINE if self._tag_lines else 0)

    x = rect.x + (rect.width - (TEXTURE_SIZE + GAP + text_w)) / 2.0
    cy = rect.y + rect.height / 2.0
    rl.draw_texture_v(self._leaf(), rl.Vector2(x, cy - TEXTURE_SIZE / 2.0), rl.WHITE)

    tx = x + TEXTURE_SIZE + GAP
    ty = cy - text_h / 2.0
    rl.draw_texture_v(self._wordmark, rl.Vector2(tx, ty), rl.WHITE)
    ty += wm_h + TAGLINE_TOP
    for i, line in enumerate(self._tag_lines):
      rl.draw_text_ex(self._tag_font, line, rl.Vector2(tx, ty + i * TAGLINE_LINE), TAGLINE_SIZE, 0.0, TAGLINE_COLOR)


def _read_stdin():
  """Non-blocking read of available lines from stdin."""
  lines = []
  while True:
    rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not rlist:
      break
    line = sys.stdin.readline().strip()
    if line == "":
      break
    lines.append(line)
  return lines


def main():
  gui_app.init_window("Spinner")
  spinner = Spinner()
  for _ in gui_app.render():
    text_list = _read_stdin()
    if text_list:
      spinner.set_text(text_list[-1])

    spinner.render(rl.Rectangle(0, 0, gui_app.width, gui_app.height))


if __name__ == "__main__":
  main()
