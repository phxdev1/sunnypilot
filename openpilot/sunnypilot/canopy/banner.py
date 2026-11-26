"""Canopy status banner: one small line of text on the comma screen.

A generic channel any part of Canopy can use (plannerd logic, hudd for Forge and Zia over
the tailnet). It is a display only: no chime, no blocking, and the UI hides it whenever a
real openpilot alert is showing. States that need a sound use an onroad event instead.

The state lives in a small JSON file on tmpfs, so it costs nothing to write, is gone after a
reboot, and needs no new param or cereal service:

  /dev/shm/canopy/banner.json
  {"entries": {"<source>": {"text": "...", "style": "info"|"heads_up"|"garage"|"lot", "until": <unix s>, "t": <unix s>}}}

The UI shows the newest entry that has not expired. hudd also streams changes on /events as
{"type": "banner", ...} so the car app can follow the same state later.
"""
import json
import os
import re
import time

BANNER_DIR = os.environ.get("CANOPY_BANNER_DIR", "/dev/shm/canopy")
BANNER_PATH = os.path.join(BANNER_DIR, "banner.json")

STYLES = ("info", "heads_up", "garage", "lot")  # garage: amber dashed ring; lot: purple dotted ring
MAX_TEXT = 48
MAX_TIMEOUT_S = 600.0
SOURCE_RE = re.compile(r"^[a-z0-9_]{1,24}$")


def _read() -> dict:
  try:
    with open(BANNER_PATH) as f:
      d = json.load(f)
    if isinstance(d, dict) and isinstance(d.get("entries"), dict):
      return d
  except (OSError, ValueError):
    pass
  return {"entries": {}}


def _write(d: dict) -> None:
  os.makedirs(BANNER_DIR, exist_ok=True)
  tmp = f"{BANNER_PATH}.{os.getpid()}.tmp"
  with open(tmp, "w") as f:
    json.dump(d, f, separators=(",", ":"))
  os.replace(tmp, BANNER_PATH)


def _prune(d: dict, now: float) -> dict:
  d["entries"] = {k: v for k, v in d["entries"].items() if isinstance(v, dict) and v.get("until", 0) > now}
  return d


def validate(source: str, text: str, style: str = "info", timeout_s: float = 10.0) -> str | None:
  """Returns an error string, or None when the banner is fine to show."""
  if not isinstance(source, str) or not SOURCE_RE.match(source):
    return "source must be 1 to 24 of a-z, 0-9, _"
  if not isinstance(text, str) or not text.strip() or len(text) > MAX_TEXT or "\n" in text:
    return f"text must be one line of 1 to {MAX_TEXT} characters"
  if style not in STYLES:
    return f"style must be one of {', '.join(STYLES)}"
  try:
    t = float(timeout_s)
  except (TypeError, ValueError):
    return "timeout_s must be a number"
  if not 1.0 <= t <= MAX_TIMEOUT_S:
    return f"timeout_s must be 1 to {int(MAX_TIMEOUT_S)}"
  return None


def post(source: str, text: str, style: str = "info", timeout_s: float = 10.0, now: float | None = None) -> str | None:
  """Show (or refresh) this source's banner. Returns an error string, or None."""
  err = validate(source, text, style, timeout_s)
  if err:
    return err
  now = time.time() if now is None else now
  d = _prune(_read(), now)
  d["entries"][source] = {"text": text.strip(), "style": style, "until": round(now + float(timeout_s), 2), "t": round(now, 2)}
  _write(d)
  return None


def clear(source: str, now: float | None = None) -> bool:
  """Take this source's banner down. Returns True when one was showing."""
  now = time.time() if now is None else now
  d = _read()
  had = source in d["entries"]
  if had:
    del d["entries"][source]
    _write(_prune(d, now))
  return had


def current(now: float | None = None, d: dict | None = None) -> dict | None:
  """The banner to show now: the newest entry that has not expired, with its source."""
  now = time.time() if now is None else now
  d = _prune(_read() if d is None else d, now)
  best = None
  for src, e in d["entries"].items():
    if best is None or e.get("t", 0) > best["t"]:
      best = {"source": src, "text": e.get("text", ""), "style": e.get("style", "info"),
              "until": e.get("until", 0), "t": e.get("t", 0)}
  return best


def mtime() -> float:
  try:
    return os.stat(BANNER_PATH).st_mtime
  except OSError:
    return 0.0
