"""Canopy: one registry for every switch the settings page shows.

Each on/off switch is a file in /data/canopy. Some files turn a feature off when present
(hud_off, lookahead_off, leadkind_off), one turns it on when present (parking_release_on), and no
file at all means the stock default. Reads that fail for any reason other than "not there" fall
back to the stock default, so a bad card or a half-read directory never silently disables
something the driver relies on.

Writes are atomic (tmp + os.replace) so a power cut mid-change leaves either the old state or the
new one, never an empty half-written file. Everything is pure python and safe to import anywhere:
no pyray, no cereal.
"""
import json
import math
import os
from dataclasses import dataclass
from typing import Optional

CANOPY_DIR = os.environ.get("CANOPY_DIR", "/data/canopy")

ASSIST_MIN = 1
ASSIST_MAX = 5

GATES_FILE = "gates.json"
GARAGE_FILE = "garage.json"

# gates.json statuses, weakest to strongest
_GATE_ORDER = ("shadow", "practice", "live")


def base_path() -> str:
  """The canopy data directory right now. Read at call time so tests can move it."""
  return os.environ.get("CANOPY_DIR", CANOPY_DIR)


def _dir(base_dir: Optional[str] = None) -> str:
  return base_dir if base_dir is not None else base_path()


@dataclass(frozen=True)
class FlagSwitch:
  """One on/off flag backed by the presence or absence of a file."""
  key: str
  title: str
  description: str
  filename: str
  on_when_present: bool
  default_on: bool
  control: bool

  def path(self, base_dir: Optional[str] = None) -> str:
    return os.path.join(_dir(base_dir), self.filename)

  def get(self, base_dir: Optional[str] = None) -> bool:
    """The switch as the car will see it. Present -> on_when_present; unreadable -> default."""
    try:
      os.stat(self.path(base_dir))
      present = True
    except FileNotFoundError:
      present = False
    except OSError:
      return self.default_on
    return self.on_when_present if present else not self.on_when_present

  def set(self, on: bool, base_dir: Optional[str] = None) -> None:
    """Make the file match ``on``. Creating a flag is atomic; removing one never raises."""
    path = self.path(base_dir)
    want_present = bool(on) == self.on_when_present
    if not want_present:
      try:
        os.remove(path)
      except OSError:
        pass
      return

    parent = os.path.dirname(path)
    try:
      if parent:
        os.makedirs(parent, exist_ok=True)
      tmp = f"{path}.tmp"
      with open(tmp, "w", encoding="utf-8"):
        pass
      os.replace(tmp, path)
    except OSError:
      pass


FLAGS = (
  FlagSwitch(
    key="hud",
    title="car screen feed",
    description="Shows the Canopy feed on the car's own center screen.",
    filename="hud_off",
    on_when_present=False,
    default_on=True,
    control=False,
  ),
  FlagSwitch(
    key="look_ahead",
    title="look ahead",
    description="Draws the Look Ahead road preview on the comma screen.",
    filename="lookahead_off",
    on_when_present=False,
    default_on=True,
    control=False,
  ),
  FlagSwitch(
    key="lead_classifier",
    title="lead classifier",
    description="Labels the car ahead as the lead and draws its bar on the HUD.",
    filename="leadkind_off",
    on_when_present=False,
    default_on=True,
    control=False,
  ),
  FlagSwitch(
    key="parking_release",
    title="parking steering release",
    description="Frees the wheel for parking so it can be turned by hand in the garage.",
    filename="parking_release_on",
    on_when_present=True,
    default_on=False,
    control=True,
  ),
)

_BY_KEY = {f.key: f for f in FLAGS}


def flag(key: str) -> FlagSwitch:
  """The switch for ``key``. Raises KeyError for anything not in FLAGS."""
  return _BY_KEY[key]


# ---- assist strength (owned by the left turn thread; we only call read/write) ----

def assist_level() -> int:
  """Assist strength 1..5, or 1 (stock) whenever the level file cannot be read."""
  try:
    from openpilot.sunnypilot.canopy.assist_torque import read_level
    return int(read_level())
  except Exception:
    return 1


def set_assist_level(level: int) -> None:
  """Persist assist strength. Raises ValueError outside 1..5."""
  try:
    lvl = int(round(float(level)))
  except (TypeError, ValueError) as e:
    raise ValueError(f"assist level must be an integer 1..5, got {level!r}") from e
  if not ASSIST_MIN <= lvl <= ASSIST_MAX:
    raise ValueError(f"assist level must be {ASSIST_MIN}..{ASSIST_MAX}, got {level!r}")
  from openpilot.sunnypilot.canopy.assist_torque import write_level
  write_level(lvl)


# ---- read-only views of what Forge and the car wrote ----

def read_json(name: str, base_dir: Optional[str] = None) -> dict:
  """A json object from the canopy dir. Anything unreadable is an empty dict."""
  try:
    with open(os.path.join(_dir(base_dir), name), encoding="utf-8") as f:
      obj = json.load(f)
  except Exception:
    return {}
  return obj if isinstance(obj, dict) else {}


def gate_summary(base_dir: Optional[str] = None) -> list[tuple[str, str]]:
  """(feature, status) for every gate, at its most advanced cell. Sorted by feature."""
  feats = read_json(GATES_FILE, base_dir).get("features")
  if not isinstance(feats, dict):
    return []
  rows = []
  for fid, cells in feats.items():
    best = 0
    if isinstance(cells, dict):
      for cell in cells.values():
        status = cell.get("status") if isinstance(cell, dict) else None
        rank = _GATE_ORDER.index(status) if status in _GATE_ORDER else 0
        best = max(best, rank)
    rows.append((str(fid), _GATE_ORDER[best]))
  return sorted(rows)


def tuned_values(base_dir: Optional[str] = None) -> list[tuple[str, str]]:
  """Human rows for the tuned parameters in gates.json, e.g. ("lane_turn max_mph", "19")."""
  params = read_json(GATES_FILE, base_dir).get("params")
  if not isinstance(params, dict):
    return []
  rows = []
  for group, group_vals in params.items():
    if ":prev" in str(group) or not isinstance(group_vals, dict):
      continue
    for name, value in group_vals.items():
      if ":prev" in str(name):
        continue
      rows.append((f"{group} {name}", str(value)))
  return sorted(rows)


def _num(value) -> Optional[str]:
  try:
    f = float(value)
  except (TypeError, ValueError):
    return None
  if not math.isfinite(f):
    return None
  r = round(f)
  if abs(f - r) < 1e-6:
    return str(int(r))
  return f"{f:.1f}"


def garage_summary(base_dir: Optional[str] = None) -> str:
  """"zone 40 m, heading 342" from garage.json. Never exposes lat or lon."""
  cfg = read_json(GARAGE_FILE, base_dir)
  zone = _num(cfg.get("zone_m"))
  heading = _num(cfg.get("heading_deg"))
  parts = []
  if zone is not None:
    parts.append(f"zone {zone} m")
  if heading is not None:
    parts.append(f"heading {heading}")
  return ", ".join(parts)
