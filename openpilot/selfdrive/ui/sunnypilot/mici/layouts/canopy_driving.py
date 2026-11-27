"""Canopy driving page for the comma four's small screen: the values behind the behavior.

Quiet mode, lane turn, auto lane change, blinker pause and the steer delay / camera offset only
appear on sunnypilot's big-screen pages, which the mici UI never loads. This page puts them on the
comma, bound to the same params, with the same values the big UI would write.

Plain switches use BigParamControl (it owns the param); stepped values use ParamChoiceToggle, which
maps a label back to the stored integer. Every call is wrapped so a bad param can never take the
page down, and every row locks while Canopy is engaged.
"""
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigMultiToggle, BigParamControl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.canopy import settings_store


class ParamChoiceToggle(BigMultiToggle):
  """A stepped openpilot param: shows setting.labels, stores setting.values[index]."""

  def __init__(self, setting):
    super().__init__(setting.title, list(setting.labels), select_callback=self._on_select,
                     description=setting.description)
    self._setting = setting
    self.refresh()

  def refresh(self):
    value = settings_store.read_param(self._setting)
    index = settings_store.choice_index(self._setting, value)
    self.set_value(self._setting.labels[index])

  def _on_select(self, label):
    try:
      index = self._setting.labels.index(label)
      settings_store.write_param(self._setting, self._setting.values[index])
    except Exception:
      pass


class CanopyDrivingLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._rows = []
    for setting in settings_store.PARAMS:
      if setting.is_choice:
        row = ParamChoiceToggle(setting)
      else:
        row = BigParamControl(setting.title, setting.name, description=setting.description)
      self._rows.append(row)

    self._scroller.add_widgets(self._rows)

    # nothing changes while Canopy is engaged
    for row in self._rows:
      row.set_enabled(lambda: not ui_state.engaged)

    self._refresh()

  def show_event(self):
    super().show_event()
    self._refresh()

  def _refresh(self):
    try:
      ui_state.update_params()
    except Exception:
      pass
    for row in self._rows:
      try:
        row.refresh()
      except Exception:
        pass
