"""Canopy green wave: choose a set speed that rides the lights, never below the posted limit.

The head unit already names "lights timed at N" on a coordinated corridor. This is the same timing
read used to decide a speed: for each candidate speed above the limit (up to CAP_MPH over), walk the
next signals and count how many the car reaches on green. The lowest speed that makes the most wins,
because going faster only buys the next light at the cost of the one after it.

Then the target ripples. A flat target arrives at the first green the instant it opens and waits at
the second; a target that eases a little up and down stays inside each window instead. The ripple
only ever moves *up* from the chosen base, so it can never fall below a speed that makes the lights,
and it is capped at the same CAP_MPH.

Nothing here acts: hudd computes it, writes /data/canopy/green_wave.json and publishes it on the bus,
and the speed limit resolver applies it only when the gate says live and the learned offsets allow
going over a posted limit. Timings are the prerender guess until a signal has been learned.
"""
import json
import math
import os
import time

MPH = 2.23694
HORIZON_M = 900.0        # only the lights this far out matter for a speed choice
MAX_SIGNALS = 3          # the same three the head unit's band looks at
MIN_LIMIT_MPH = 25.0     # a parking lot, a freeway: no wave
MAX_LIMIT_MPH = 55.0
CAP_MPH = 7.0            # never more than this over the posted limit
AMP_MPH = 1.5            # the ripple, half peak to peak
PERIOD_S = 24.0          # one gentle wave
MIN_LEAD_M = 20.0        # a light this close decides nothing about speed
PATH = "/data/canopy/green_wave.json"
FRESH_S = 4.0
STEP_MPH = 0.5


def phase(timing, t_s):
  """Where in its cycle the signal is at t_s: green, yellow or red (the head unit's predictState)."""
  cyc = float(timing.get("cycle_s") or 0.0)
  if cyc <= 0.0:
    return "unknown"
  ph = (t_s + float(timing.get("offset_s") or 0.0)) % cyc
  if ph < float(timing.get("green_s") or 0.0):
    return "green"
  if ph < float(timing.get("green_s") or 0.0) + float(timing.get("yellow_s") or 4.0):
    return "yellow"
  return "red"


def lead_run(signals, v_mph, now, horizon=HORIZON_M, margin_s=1.5):
  """How many of the next signals, nearest first, the car reaches on green at this constant speed."""
  v_ms = max(v_mph, 1.0) * 0.44704
  n = 0
  for s in signals[:MAX_SIGNALS]:
    d = float(s.get("d") or 0.0)
    if d > horizon or d < MIN_LEAD_M:
      continue
    if d == 0.0 or not s.get("timing"):
      break
    if phase(s["timing"], now + d / v_ms + margin_s) != "green":
      break
    n += 1
  return n


def solve(signals, limit_mph, now):
  """The wave for this moment: None when nothing is worth changing, else the target and its ripple.

  signals: [{"d": metres ahead, "timing": {cycle_s, green_s, yellow_s, offset_s}}, ...] nearest first.
  """
  if not limit_mph or not (MIN_LIMIT_MPH <= limit_mph <= MAX_LIMIT_MPH):
    return None
  usable = [s for s in signals if s.get("timing") and (s.get("d") or 0.0) <= HORIZON_M]
  if len(usable) < 2:  # one light is not a wave, it is a coin toss
    return None
  cap = limit_mph + CAP_MPH
  best = (0, 0.0)  # (lights made, how far over the limit)
  v = limit_mph
  while v <= cap + 1e-9:
    run = lead_run(usable, v, now)
    if run > best[0] or (run == best[0] and run > 0 and v - limit_mph < best[1]):
      best = (run, v - limit_mph)
    v += STEP_MPH
  made, over = best
  if made <= 0:
    return None
  base = round(limit_mph + over, 1)
  amp = 0.0
  if made >= 2:  # ripple only where it cannot cost a light
    for a in (AMP_MPH, AMP_MPH * 0.6, AMP_MPH * 0.3):
      if base + a <= cap and lead_run(usable, base + a, now) >= made:
        amp = round(a, 1)
        break
  return {"on": True, "limit_mph": round(limit_mph, 1), "base_mph": base,
          "target_mph": round(base + amp * (0.5 + 0.5 * math.sin(0.0)), 1),
          "amp_mph": amp, "period_s": PERIOD_S, "phase_s": round(now, 1),
          "catch": made, "of": min(len(usable), MAX_SIGNALS), "ts": round(now, 1)}


def target_at(wave, t_s):
  """The speed the wave asks for at time t_s (mph). The ripple rides between base and base + amp."""
  if not wave or not wave.get("on"):
    return None
  base = float(wave.get("base_mph") or 0.0)
  amp = float(wave.get("amp_mph") or 0.0)
  period = max(float(wave.get("period_s") or PERIOD_S), 1.0)
  if amp <= 0.0:
    return base
  ph = 2.0 * math.pi * ((t_s - float(wave.get("phase_s") or 0.0)) % period) / period
  return round(base + amp * (0.5 + 0.5 * math.sin(ph)), 2)


def signals_from_view(view, limit_mph, now):
  """The signals the Look Ahead view already resolved on this path, nearest first, with their timing."""
  out = []
  for item in (view or {}).get("items", []):
    if item.get("k") != "signal":
      continue
    x = item.get("x") or {}
    timing = x.get("timing")
    if not isinstance(timing, dict) or not timing.get("cycle_s"):
      continue
    out.append({"d": float(item.get("along_m") or 0.0), "timing": timing,
                "lat": item.get("lat"), "lon": item.get("lon")})
  return out


def write(wave, path=PATH):
  """Atomic write for the resolver: it reads this file instead of computing a wave in the controls loop."""
  if wave is None:
    return False
  tmp = path + ".tmp"
  try:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(tmp, "w") as f:
      json.dump(wave, f, separators=(",", ":"))
    os.replace(tmp, path)
    return True
  except OSError:
    return False


def read(path=PATH, now=None, fresh_s=FRESH_S):
  """The wave the car should be riding, or None when there is none or it has gone stale."""
  now = time.time() if now is None else now
  try:
    with open(path) as f:
      w = json.load(f)
  except (OSError, ValueError):
    return None
  if not isinstance(w, dict) or not w.get("on"):
    return None
  try:
    if now - float(w.get("ts") or 0.0) > fresh_s:
      return None
  except (TypeError, ValueError):
    return None
  return w
