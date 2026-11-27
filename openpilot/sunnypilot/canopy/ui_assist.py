"""Canopy: the Assist Strength row on Settings > Steering.

The sunnypilot option stepper, except the value lives in /data/canopy/assist_level (read/written
through canopy/assist_torque.py) instead of Params, so the parameter key is never touched. That
keeps the level out of params_keys.h and readable by card.py without a rebuild.
"""
from openpilot.sunnypilot.canopy.assist_torque import LEVEL_MAX, LEVEL_MIN, read_level, write_level
from openpilot.system.ui.sunnypilot.widgets.list_view import ListItemSP
from openpilot.system.ui.sunnypilot.widgets.option_control import OptionControlSP

TITLE = "Assist Strength"
DESCRIPTION = ("How hard the steering assist pushes below 20 mph. 1 is stock. "
               "If the steering faults, it drops back to 1 for the rest of the drive.")


class AssistStrengthControlSP(OptionControlSP):
  """A 1-5 stepper backed by the canopy level file. param_key is unused and never read."""

  def __init__(self):
    super().__init__("", LEVEL_MIN, LEVEL_MAX)

  def _load_value(self) -> int:
    return read_level()

  def _store_value(self, value: int):
    write_level(value)

  def refresh(self):
    self.current_value = read_level()

  def get_displayed_value(self) -> str:
    return "1 (stock)" if self.current_value == LEVEL_MIN else str(self.current_value)


def assist_strength_item_sp() -> ListItemSP:
  return ListItemSP(title=TITLE, description=DESCRIPTION, action_item=AssistStrengthControlSP(), inline=False)
