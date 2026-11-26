"""Canopy auto-resume from a stop, SHADOW ONLY: logs where the comma would have pressed RES, and when
Mark actually moved off, so the two can be compared before anything goes live. Nothing here sends a
button. (Live, the press would be ICBM's "increase", which on the Leaf is the RES/+ button.)

Would press RES only when all hold:
  - stopped (standstill) for at least MIN_STOP_S, cruise on and openpilot engaged the whole stop
  - a car was close ahead when he stopped (LEAD_CLOSE_M): never with no lead, never on a light read
  - that car has pulled away: LEAD_GAIN_M farther than its closest, moving off at LEAD_MOVE_MS,
    for LEAD_HOLD_S, and still tracked
  - no brake, no gas at that moment
Live it would chime and say "Resuming" just before the press, and the brake cancels. Not built yet.

Green light reads are logged at every stop too, car ahead or not (Mark, Nov 26: the goal is RES on a
green read eventually). A read is the stock green light condition: stopped 2 s+, and the model's path
opens past GREEN_X_M for GREEN_HOLD_S. A read he didn't move off within GREEN_OK_S counts as a false
green. Nothing acts on a green read yet.

Each stop of MIN_STOP_S or more logs one row to SHADOW_PATH (no position, no plate, nothing about the
other car but distance), and the planner logs it as canopy_resume_shadow.
"""
import json
import os
import time

SHADOW_PATH = "/data/canopy/resume_shadow.jsonl"
SHADOW_MAX_BYTES = 512 * 1024
STOP_V = 0.1
GO_V = 0.5
MIN_STOP_S = 2.0
LEAD_CLOSE_M = 8.0
LEAD_GAIN_M = 1.5
LEAD_MOVE_MS = 0.5
LEAD_HOLD_S = 0.3
# ProPILOT holds a stop for about 3 minutes, then switches off and sets the parking brake (Leaf owner's
# manual). RES does nothing after that, so never press past this.
MAX_HOLD_S = 150.
GREEN_X_M = 30.
GREEN_HOLD_S = 0.3
GREEN_OK_S = 5.0


def _btn_res(car_state) -> bool:
  try:
    return any(b.pressed and str(b.type) == "accelCruise" for b in car_state.buttonEvents)
  except Exception:
    return False


class AutoResumeShadow:
  def __init__(self, path: str | None = SHADOW_PATH):
    self.path = path
    self.stop = None
    self.last_row = None

  def _reset(self):
    self.stop = None

  def update(self, car_state, lead, engaged: bool, path_end_x: float | None = None, now: float | None = None,
             wall: float | None = None) -> bool:
    """True on the frame a logged stop ends (see last_row). path_end_x: modelV2.position.x[-1]."""
    now = time.monotonic() if now is None else now
    wall = time.time() if wall is None else wall
    self.last_row = None
    v = car_state.vEgo
    cruise = bool(car_state.cruiseState.enabled)
    lead_ok = lead is not None and getattr(lead, "present", False)

    if self.stop is None:
      if v < STOP_V and car_state.standstill:
        d0 = lead.dRel if lead_ok else None
        self.stop = {"t0": now, "lead_d0": None if d0 is None else round(d0, 1), "min_d": d0, "engaged": engaged,
                     "cruise": cruise, "would_t": None, "skip": None, "gain_since": None, "res": False,
                     "open_since": None, "green_t": None, "greens": 0, "false_greens": 0, "blinker": None}
      return False

    s = self.stop
    s["engaged"] &= engaged
    s["cruise"] &= cruise
    s["res"] |= _btn_res(car_state)
    stopped_s = now - s["t0"]
    if car_state.leftBlinker or car_state.rightBlinker:
      s["blinker"] = "left" if car_state.leftBlinker else "right"

    # green light read, shadow only
    if stopped_s >= MIN_STOP_S and v < GO_V and path_end_x is not None:
      if path_end_x > GREEN_X_M:
        s["open_since"] = s["open_since"] or now
        if s["green_t"] is None and now - s["open_since"] >= GREEN_HOLD_S:
          s["green_t"] = now
          s["greens"] += 1
      else:
        s["open_since"] = None
    if s["green_t"] is not None and now - s["green_t"] > GREEN_OK_S and v < GO_V:
      s["false_greens"] += 1  # read green, he stayed put: a false green
      s["green_t"] = None

    if s["would_t"] is None and s["skip"] is None and stopped_s >= MIN_STOP_S and v < GO_V:
      if s["lead_d0"] is None or s["lead_d0"] > LEAD_CLOSE_M:
        s["skip"] = "no_close_lead"
      elif lead_ok:
        s["min_d"] = lead.dRel if s["min_d"] is None else min(s["min_d"], lead.dRel)
        leaving = lead.dRel - s["min_d"] > LEAD_GAIN_M and lead.vLead > LEAD_MOVE_MS
        if leaving:
          s["gain_since"] = s["gain_since"] or now
          if now - s["gain_since"] >= LEAD_HOLD_S:
            if stopped_s > MAX_HOLD_S:
              s["skip"] = "held_too_long"
            elif not s["engaged"]:
              s["skip"] = "not_engaged"
            elif not s["cruise"]:
              s["skip"] = "cruise_off"
            elif car_state.brakePressed:
              s["skip"] = "brake"
            elif car_state.gasPressed:
              s["skip"] = "gas"
            else:
              s["would_t"] = now
        else:
          s["gain_since"] = None

    went = v > GO_V or car_state.gasPressed or s["res"]
    if not went:
      return False
    how = "res" if s["res"] else ("gas" if car_state.gasPressed else "rolled")
    row = {"wall": round(wall, 1), "stopped_s": round(stopped_s, 1), "lead_d0": s["lead_d0"], "how": how,
           "would": s["would_t"] is not None, "skip": s["skip"] if s["would_t"] is None else None,
           "would_s": None if s["would_t"] is None else round(s["would_t"] - s["t0"], 1),
           "lag_s": None if s["would_t"] is None else round(now - s["would_t"], 1),
           "blinker": s["blinker"], "greens": s["greens"], "false_greens": s["false_greens"],
           "green_lead_s": None if s["green_t"] is None else round(now - s["green_t"], 1)}
    if row["skip"] is None and not row["would"]:
      row["skip"] = "driver_first"  # he moved before the car ahead had clearly left
    self._reset()
    if stopped_s < MIN_STOP_S:
      return False
    self.last_row = row
    self._append(row)
    return True

  def _append(self, row):
    if self.path is None:
      return
    try:
      if os.path.exists(self.path) and os.path.getsize(self.path) > SHADOW_MAX_BYTES:
        os.replace(self.path, self.path + ".1")
      with open(self.path, "a") as f:
        f.write(json.dumps(row, separators=(",", ":")) + "\n")
    except OSError:
      pass
