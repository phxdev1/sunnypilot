"""Parking release: when garage parking starts while he is learning, hand lateral back to him.

garage.py drops a one-shot token when parking starts; mads.py polls it and sends the same event
a real LKAS button press sends, so openpilot lateral turns OFF exactly as if he had pressed the
button, and comes back only when he presses it again. Gas and brake stay his either way.

The switch is a plain file (/data/canopy/parking_release_on): no file, nothing happens.
The token lives next to the banner on tmpfs.
"""
import json
import os

from openpilot.sunnypilot.canopy.banner import BANNER_DIR

FLAG = os.environ.get("CANOPY_PARKING_RELEASE_FLAG", "/data/canopy/parking_release_on")
REQ = os.path.join(BANNER_DIR, "parking_release.json")
WINDOW_S = 3.0        # a token older than this is a stale request, not this frame's
POLL_EVERY = 10       # read the file every 10th poll only


def enabled() -> bool:
  return os.path.exists(FLAG)


def request(source: str, wall: float) -> bool:
  """Ask for lateral to be released. Returns True when the switch is on and the token is down."""
  if not enabled():
    return False
  try:
    tmp = f"{REQ}.{os.getpid()}.tmp"
    with open(tmp, "w") as f:
      json.dump({"token": wall, "t": wall, "source": source}, f, separators=(",", ":"))
    os.replace(tmp, REQ)
    return True
  except OSError:
    return False


class Taker:
  """Takes each request token once, and only while it is fresh."""

  def __init__(self):
    self.token = None
    self._n = 0

  def poll(self, wall: float) -> bool:
    self._n += 1
    if self._n % POLL_EVERY:
      return False
    try:
      with open(REQ) as f:
        d = json.load(f)
      token = d["token"]
      t = float(d["t"])
    except (OSError, ValueError, KeyError, TypeError):
      return False
    if token == self.token:
      return False
    self.token = token
    return abs(wall - t) <= WINDOW_S
