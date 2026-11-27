"""Canopy settings page for the comma four's small screen (mici).

One page for every switch canopy owns: assist strength first, then the file-backed on/off flags,
then the values the gates and the garage have written. Everything reads and writes through
openpilot.sunnypilot.canopy.settings_store, and every call is wrapped so a bad file can never take
the settings screen down.
"""
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigMultiToggle, BigToggle, GreyBigButton
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.sunnypilot.canopy import settings_store

ASSIST_OPTIONS = ("1 stock", "2", "3", "4", "5")
ASSIST_HELP = ("How hard steering assist pushes below 20 mph. 1 is stock. "
               "A steering fault drops it back to 1 for the drive.")


class CanopyLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._assist = BigMultiToggle("assist strength", list(ASSIST_OPTIONS), select_callback=self._on_assist,
                                  description=ASSIST_HELP)

    self._flags = []
    for switch in settings_store.FLAGS:
      row = BigToggle(switch.title, initial_state=self._flag_state(switch),
                      toggle_callback=lambda checked, s=switch: self._on_flag(s, checked),
                      description=switch.description)
      self._flags.append((switch, row))

    self._gates = GreyBigButton("gates", "")
    self._tuned = GreyBigButton("tuned values", "")
    self._garage = GreyBigButton("garage", "")

    self._driving = BigButton("driving", "lane turn, blinker and steer delay")
    self._driving.set_click_callback(self._open_driving)

    self._scroller.add_widgets([self._assist] + [row for _, row in self._flags] +
                               [self._driving, self._gates, self._tuned, self._garage])

    # nothing changes while Canopy is engaged
    for row in [self._assist] + [row for _, row in self._flags]:
      row.set_enabled(lambda: not ui_state.engaged)

    self._refresh()

  def show_event(self):
    super().show_event()
    self._refresh()

  # ---- reading ----

  @staticmethod
  def _flag_state(switch):
    try:
      return switch.get()
    except Exception:
      return switch.default_on

  def _refresh(self):
    self._refresh_assist()
    for switch, row in self._flags:
      self._refresh_flag(switch, row)
    self._refresh_gates()
    self._refresh_tuned()
    self._refresh_garage()

  def _refresh_assist(self):
    level = 1
    try:
      level = int(settings_store.assist_level())
    except Exception:
      level = 1
    level = max(1, min(len(ASSIST_OPTIONS), level))
    self._assist.set_value(ASSIST_OPTIONS[level - 1])

  def _refresh_flag(self, switch, row):
    row.set_checked(self._flag_state(switch))

  def _refresh_gates(self):
    text = ""
    try:
      rows = settings_store.gate_summary()
      if rows:
        counts = {"shadow": 0, "practice": 0, "live": 0}
        for _, status in rows:
          counts[status] = counts.get(status, 0) + 1
        text = f"{counts['shadow']} shadow, {counts['practice']} practice, {counts['live']} live"
    except Exception:
      text = ""
    self._set_value(self._gates, text)

  def _refresh_tuned(self):
    text = ""
    try:
      rows = settings_store.tuned_values()[:4]
      text = "\n".join(f"{name} {value}" for name, value in rows)
    except Exception:
      text = ""
    self._set_value(self._tuned, text)

  def _refresh_garage(self):
    text = ""
    try:
      text = settings_store.garage_summary()
    except Exception:
      text = ""
    self._set_value(self._garage, text)

  @staticmethod
  def _set_value(row, text):
    row.set_value(text)
    row.set_visible(bool(text))

  # ---- writing ----

  def _on_assist(self, value):
    try:
      settings_store.set_assist_level(ASSIST_OPTIONS.index(value) + 1)
    except Exception:
      pass

  def _on_flag(self, switch, checked):
    try:
      switch.set(checked)
    except Exception:
      pass

  def _open_driving(self):
    # imported on click so a broken driving page never takes the settings screen down
    try:
      from openpilot.selfdrive.ui.sunnypilot.mici.layouts.canopy_driving import CanopyDrivingLayoutMici
      gui_app.push_widget(CanopyDrivingLayoutMici())
    except Exception:
      pass
