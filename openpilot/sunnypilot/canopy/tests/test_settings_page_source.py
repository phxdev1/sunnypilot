"""Source-level checks for the Canopy settings page (pyray is not installed off device).

Parses both UI files with ast so nothing gets imported, and compiles them with py_compile to a temp
path so no __pycache__ lands in the repo.
"""
import ast
import py_compile
from pathlib import Path

import pytest

from openpilot.sunnypilot.canopy import settings_store

REPO = Path(__file__).resolve().parents[4]
CANOPY = REPO / "openpilot/selfdrive/ui/sunnypilot/mici/layouts/canopy.py"
CANOPY_DRIVING = REPO / "openpilot/selfdrive/ui/sunnypilot/mici/layouts/canopy_driving.py"
SETTINGS = REPO / "openpilot/selfdrive/ui/sunnypilot/mici/layouts/settings.py"

BUTTON_CALLS = {"SettingsBigButton", "SunnylinkBigButton", "BigButton", "BigToggle", "BigMultiToggle",
                "GreyBigButton", "BigCircleButton", "BigParamControl", "BigMultiParamToggle", "tr"}


def tree(path):
  return ast.parse(Path(path).read_text(encoding="utf-8"), filename=str(path))


def call_name(node):
  if isinstance(node.func, ast.Name):
    return node.func.id
  if isinstance(node.func, ast.Attribute):
    return node.func.attr
  return None


def visible_strings(node):
  """String literals a driver can read, not icon paths buried in nested gui_app.texture calls."""
  out = []

  def add(arg):
    if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
      out.append(arg.value)
    elif isinstance(arg, ast.Call) and call_name(arg) == "tr":
      for sub in arg.args:
        if isinstance(sub, ast.Constant) and isinstance(sub.value, str):
          out.append(sub.value)

  for arg in list(node.args) + [k.value for k in node.keywords]:
    add(arg)
  return out


def button_strings(path):
  out = []
  for node in ast.walk(tree(path)):
    if isinstance(node, ast.Call) and call_name(node) in BUTTON_CALLS:
      out.extend(visible_strings(node))
  return out


def test_canopy_page_defines_the_layout_class():
  classes = [n.name for n in ast.walk(tree(CANOPY)) if isinstance(n, ast.ClassDef)]
  assert "CanopyLayoutMici" in classes
  src = CANOPY.read_text(encoding="utf-8")
  assert "NavScroller" in src
  assert "add_widgets" in src


def test_canopy_page_iterates_flags_instead_of_hardcoding_titles():
  t = tree(CANOPY)
  attrs = {n.attr for n in ast.walk(t) if isinstance(n, ast.Attribute)}
  assert "FLAGS" in attrs, "canopy.py must iterate settings_store.FLAGS"

  constants = {n.value for n in ast.walk(t) if isinstance(n, ast.Constant) and isinstance(n.value, str)}
  for switch in settings_store.FLAGS:
    assert switch.title not in constants, f"{switch.title!r} is hard-coded in canopy.py"
    assert switch.filename not in constants, f"{switch.filename!r} is hard-coded in canopy.py"


def test_canopy_page_locks_controls_while_engaged():
  calls = [n for n in ast.walk(tree(CANOPY))
           if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "set_enabled"]
  assert calls, "control rows must call set_enabled"
  src = CANOPY.read_text(encoding="utf-8")
  assert "ui_state.engaged" in src


def test_canopy_page_uses_the_store():
  src = CANOPY.read_text(encoding="utf-8")
  for call in ("assist_level()", "set_assist_level(", "gate_summary()", "tuned_values()", "garage_summary()"):
    assert call in src
  assert "show_event" in src


def test_settings_imports_canopy_inside_try():
  t = tree(SETTINGS)
  parents = {}
  for node in ast.walk(t):
    for child in ast.iter_child_nodes(node):
      parents[child] = node

  guarded = False
  found = False
  for node in ast.walk(t):
    if isinstance(node, ast.ImportFrom):
      names = {a.name for a in node.names}
      module = node.module or ""
    elif isinstance(node, ast.Import):
      names = {a.name for a in node.names}
      module = ""
    else:
      continue
    if "CanopyLayoutMici" not in names and "CanopyLayoutMici" not in module:
      continue
    found = True
    cur = node
    while cur in parents:
      cur = parents[cur]
      if isinstance(cur, ast.Try):
        guarded = True
        break
  assert found, "settings.py must import CanopyLayoutMici"
  assert guarded, "the canopy import must sit in a try block"


def test_settings_adds_a_canopy_button():
  src = SETTINGS.read_text(encoding="utf-8")
  assert 'tr("canopy")' in src
  assert "items.insert" in src and "canopy_btn" in src
  assert "gui_app.push_widget" in src


@pytest.mark.parametrize("path", [CANOPY, CANOPY_DRIVING, SETTINGS])
def test_no_user_visible_sunnypilot(path):
  for text in button_strings(path):
    assert "sunnypilot" not in text.lower(), f"driver-visible string says sunnypilot: {text!r}"


# ---- driving page (openpilot params) ----

def test_driving_page_defines_the_layout_class():
  classes = [n.name for n in ast.walk(tree(CANOPY_DRIVING)) if isinstance(n, ast.ClassDef)]
  assert "CanopyDrivingLayoutMici" in classes
  assert "NavScroller" in CANOPY_DRIVING.read_text(encoding="utf-8")


def test_driving_page_iterates_params_instead_of_hardcoding_titles():
  t = tree(CANOPY_DRIVING)
  attrs = {n.attr for n in ast.walk(t) if isinstance(n, ast.Attribute)}
  assert "PARAMS" in attrs, "canopy_driving.py must iterate settings_store.PARAMS"

  constants = {n.value for n in ast.walk(t) if isinstance(n, ast.Constant) and isinstance(n.value, str)}
  for spec in settings_store.PARAMS:
    assert spec.title not in constants, f"{spec.title!r} is hard-coded in canopy_driving.py"
    assert spec.name not in constants, f"{spec.name!r} is hard-coded in canopy_driving.py"


def test_driving_page_locks_controls_and_uses_the_store():
  calls = [n for n in ast.walk(tree(CANOPY_DRIVING))
           if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "set_enabled"]
  assert calls, "param rows must call set_enabled"
  src = CANOPY_DRIVING.read_text(encoding="utf-8")
  assert "ui_state.engaged" in src
  assert "read_param(" in src and "write_param(" in src
  assert "show_event" in src


def test_canopy_page_pushes_driving_page_lazily():
  src = CANOPY.read_text(encoding="utf-8")
  assert "canopy_driving import CanopyDrivingLayoutMici" in src
  assert "gui_app.push_widget(CanopyDrivingLayoutMici())" in src
  t = tree(CANOPY)
  parents = {}
  for node in ast.walk(t):
    for child in ast.iter_child_nodes(node):
      parents[child] = node
  guarded = False
  for node in ast.walk(t):
    if isinstance(node, ast.ImportFrom) and "CanopyDrivingLayoutMici" in {a.name for a in node.names}:
      cur = node
      while cur in parents:
        cur = parents[cur]
        if isinstance(cur, ast.Try):
          guarded = True
          break
  assert guarded, "the driving page import must sit in a try block"


@pytest.mark.parametrize("path", [CANOPY, CANOPY_DRIVING, SETTINGS])
def test_py_compile(path, tmp_path):
  py_compile.compile(str(path), cfile=str(tmp_path / (Path(path).stem + ".pyc")), doraise=True)
