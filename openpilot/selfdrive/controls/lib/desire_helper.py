from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController
from openpilot.sunnypilot.canopy.voice_merge import VoiceMerge
from openpilot.sunnypilot.canopy.blinker_intent import BlinkerIntent
from openpilot.sunnypilot.canopy import params_file

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.ModelDataV2SP.TurnDirection

# Canopy lane change speeds (Mark, Nov 26: "Let's limit it to 50"):
#  - 50 mph and up: the blinker alone starts a lane change, as on the freeway.
#  - 25 to 50 mph (surface streets): the blinker alone is a turn. A light nudge on the wheel toward the
#    blinker starts the change, only with the wheel near straight, no brake in the last 2 s and not
#    slowing hard. (Nov 25 lap, 27.4 min: a right turn signalled at 31 mph was taken as a lane change
#    and steered toward the curb, so the blinker alone never starts one here.)
#  - under 25 mph: always a turn.
LANE_CHANGE_SPEED_MIN = 25 * CV.MPH_TO_MS
LANE_CHANGE_AUTO_MIN = 50 * CV.MPH_TO_MS
NUDGE_MAX_ANGLE_DEG = 10.0
NUDGE_NO_BRAKE_FRAMES = int(2.0 / DT_MDL)
NUDGE_MIN_ACCEL = -1.0
# A change under way is dropped if the car slows below this or the wheel goes past TURN_ANGLE_DEG
# under LANE_CHANGE_AUTO_MIN: that is a turn, not a merge.
LANE_CHANGE_ABORT_SPEED = 20 * CV.MPH_TO_MS
TURN_ANGLE_DEG = 30.0
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5
# Blind spot can cancel a started lane change only this early, while the car is still well inside
# its own lane. Typical openpilot lane changes take ~4-6 s and cross the line around the midpoint.
LANE_CHANGE_BSM_ABORT_TIME = 1.5
# Canopy: a live intent change (no nudge) waits 1 s after blinker-on, giving him a beat to cancel a
# flick of the stalk before the car commits to the change.
INTENT_DELAY_FRAMES = int(1.5 / DT_MDL)

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

BLINKER_HOLD_FRAMES = int(0.5 / DT_MDL)  # Canopy: hold each side 0.5 s past its last lit frame

# Canopy hold-to-merge: plannerd (canopy/lane_hold.py) writes "right" here while the car is before
# one of the route's exits and not yet in the rightmost lane. Only then does a held right blinker get
# another lane change after the first; everywhere else a hold is one change. Each extra change still
# waits HOLD_SETTLE_TIME, the blind spot gate and the road edge check, and a hold stops at
# MAX_HOLD_CHANGES.
HOLD_FILE = "/dev/shm/canopy_hold"
HOLD_READ_FRAMES = 5
HOLD_SETTLE_TIME = 1.5
# Canopy: Mark checks his right before he signals (Nov 25), so the right blind spot read warns but
# never blocks or cancels a right change. The read is logged on every right change so the right side
# of 0x3F0/0x3F1 keeps being learned. Left still blocks and cancels on the blind spot.
RIGHT_BS_ADVISORY = True
MAX_HOLD_CHANGES = 4

# Canopy voice merge (Mark, Nov 26: "Let's not tie it to the stalk"): "Zia, merge left/right" at 25 to 50 mph starts a lane change
# by itself after a 1.5 s window, blinker or not (canopy/voice_merge.py; shadow only until /data/canopy/voice_merge_live exists).
# Once started it runs like a nudged change, except a blinker already on the other side does not cancel it (only a new one does)
# and it is always a single change.
# The right blind spot read blocks and cancels a voice merge (the advisory rule is for his own signalled changes).


class _HeldBlinkers:
  """carState with the turn signals held through the dark phase of each flash, so a quick
  3-blink stalk tap reads as one continuous signal. Every other field passes through."""
  __slots__ = ("_cs", "leftBlinker", "rightBlinker")

  def __init__(self, cs, left, right):
    self._cs, self.leftBlinker, self.rightBlinker = cs, left, right

  def __getattr__(self, name):
    return getattr(self._cs, name)


class DesireHelper:
  def __init__(self):
    p = params_file.load()
    self.lane_change_speed_min = p["lane_change_speed_min_mph"] * CV.MPH_TO_MS
    self.lane_change_auto_min = p["lane_change_auto_min_mph"] * CV.MPH_TO_MS
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self._blinker_hold = [0, 0]  # frames left on the hold, left and right
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)
    self.lane_turn_controller = LaneTurnController(self)
    self.lane_turn_direction = TurnDirection.none
    self.enable_bsm = False  # set from has_bsm(CP) by modeld
    self.hold_side = ""      # Canopy: "right" while a held right blinker may keep going
    self._hold_read = 0
    self.hold_changes = 0    # lane changes made on this hold
    self._right_bs_seen = False  # Canopy: right blind spot read occupied at any point in this right change
    self._since_brake = NUDGE_NO_BRAKE_FRAMES  # Canopy: model frames since the brake was last pressed
    self.voice = VoiceMerge()
    self.voice_change = False      # Canopy: the change under way was started by voice
    self._prev_held = (False, False)  # Canopy: held blinkers on the previous frame, for a new opposite signal
    try:
      self.blinker_intent = BlinkerIntent()
    except Exception:
      self.blinker_intent = None
    self._intent_go = False        # Canopy: blinker_intent said lane change, gate live: no nudge needed
    self._intent_frames = 0        # Canopy: model frames in preLaneChange under this blinker, for the intent delay

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def _read_hold_side(self):
    self._hold_read += 1
    if self._hold_read % HOLD_READ_FRAMES:
      return
    try:
      with open(HOLD_FILE) as f:
        self.hold_side = f.read(8).strip()
    except OSError:
      self.hold_side = ""

  def _hold_allows(self, direction) -> bool:
    return self.hold_side == "right" and direction == LaneChangeDirection.right and self.hold_changes < MAX_HOLD_CHANGES

  def _hold_blinkers(self, left, right):
    for i, lit in enumerate((left, right)):
      self._blinker_hold[i] = BLINKER_HOLD_FRAMES if lit else max(0, self._blinker_hold[i] - 1)
    return self._blinker_hold[0] > 0, self._blinker_hold[1] > 0

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False,
            lane_info=None, edges=None, gps=None):
    carstate = _HeldBlinkers(carstate, *self._hold_blinkers(carstate.leftBlinker, carstate.rightBlinker))
    if self.blinker_intent and edges and lane_info:
      try:
        self.blinker_intent.observe(edges[0], edges[1], edges[2], edges[3],
                                    lane_info[0], lane_info[1], lane_info[2], lane_info[3])
      except Exception:
        pass
    self._read_hold_side()
    self.alc.update_params()
    self.alc.bsm_ok = self.enable_bsm and carstate.canValid
    self.lane_turn_controller.update_params()
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < self.lane_change_speed_min
    nudge_band = v_ego < self.lane_change_auto_min
    self._since_brake = 0 if carstate.brakePressed else min(self._since_brake + 1, NUDGE_NO_BRAKE_FRAMES)
    voice_side = self.voice.step(carstate, lateral_active, self.lane_change_state != LaneChangeState.laneChangeStarting,
                                 left_edge_detected, right_edge_detected, False,
                                 lane_info=lane_info)

    # Lane turn controller update
    self.lane_turn_controller.update_lane_turn(blindspot_left=carstate.leftBlindspot, blindspot_right=carstate.rightBlindspot,
                                               left_blinker=carstate.leftBlinker, right_blinker=carstate.rightBlinker, v_ego=v_ego)
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self.voice_change = False
      self._intent_go = False
    else:
      if self.lane_change_state == LaneChangeState.off:
        self.hold_changes = 0
      if voice_side and self.lane_change_state != LaneChangeState.laneChangeStarting:
        self.lane_change_state = LaneChangeState.laneChangeStarting
        self.lane_change_direction = LaneChangeDirection.left if voice_side == "left" else LaneChangeDirection.right
        self.lane_change_timer = 0.0
        self.voice_change = True
        if voice_side == "right":
          self._right_bs_seen = bool(carstate.rightBlindspot)
      elif self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)
        self._intent_go = False
        self._intent_frames = 0
        if self.blinker_intent is not None and nudge_band:
          try:
            side = "left" if carstate.leftBlinker else "right"
            record = self.blinker_intent.on_blinker(side, v_ego * CV.MS_TO_MPH, *(gps if gps else ()))
            self._intent_go = bool(record["act"])
          except Exception:
            self._intent_go = False

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        # Canopy: a hold that was allowed another change loses it the moment the route target is met
        if self.hold_changes and not self._hold_allows(self.lane_change_direction):
          self.alc.prev_lane_change = True

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right
                               and not RIGHT_BS_ADVISORY))

        edge_detected = ((left_edge_detected and self.lane_change_direction == LaneChangeDirection.left) or
                         (right_edge_detected and self.lane_change_direction == LaneChangeDirection.right))

        self.alc.update_lane_change(blindspot_detected, edge_detected, carstate.brakePressed, torque_applied)


        nudge_ok = torque_applied and abs(carstate.steeringAngleDeg) < NUDGE_MAX_ANGLE_DEG and \
                   self._since_brake >= NUDGE_NO_BRAKE_FRAMES and carstate.aEgo > NUDGE_MIN_ACCEL
        # Canopy: an intent change has no nudge behind it, so the blind spot on the change side blocks it
        # the way it blocks a voice change (the advisory rule is for his own signalled, nudged changes).
        self._intent_frames += 1
        intent_bs = carstate.leftBlindspot if self.lane_change_direction == LaneChangeDirection.left else carstate.rightBlindspot
        intent_ok = self._intent_go and not intent_bs and \
                    self._intent_frames >= INTENT_DELAY_FRAMES and \
                    self._since_brake >= NUDGE_NO_BRAKE_FRAMES and carstate.aEgo > NUDGE_MIN_ACCEL
        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
          self._intent_go = False
        elif nudge_band and not (nudge_ok or intent_ok):
          pass  # surface street: waits for his nudge; the blinker alone stays a turn
        elif (torque_applied or intent_ok or self.alc.auto_lane_change_allowed) and not self.alc.lane_change_blocked:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0
          if self.lane_change_direction == LaneChangeDirection.right:
            self._right_bs_seen = bool(carstate.rightBlindspot)
            cloudlog.event("canopy_right_change", phase="start", bs_right=bool(carstate.rightBlindspot),
                           bs_left=bool(carstate.leftBlindspot), mph=round(v_ego * CV.MS_TO_MPH), nudge=bool(torque_applied),
                           hold=self.hold_changes, surface=bool(nudge_band))

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        # Latched: once started, the signal going off does not cancel it, so a tap carries a full merge.
        # Cancel on driver torque against the change or the opposite signal (or hazards) at any point.
        # A blind spot on that side cancels only in the first LANE_CHANGE_BSM_ABORT_TIME, before the car
        # nears the line: after that the sensor is watching the lane beyond the target, so a car there
        # must not pull us back across. lateral_active loss and LANE_CHANGE_TIME_MAX still cancel at the top.
        going_left = self.lane_change_direction == LaneChangeDirection.left
        opposite_blinker = carstate.rightBlinker if going_left else carstate.leftBlinker
        if self.voice_change:
          opposite_blinker = opposite_blinker and not (self._prev_held[1] if going_left else self._prev_held[0])  # only a new one cancels
        torque_against = carstate.steeringPressed and \
                         ((carstate.steeringTorque < 0) if going_left else (carstate.steeringTorque > 0))
        blindspot_ahead = (carstate.leftBlindspot if going_left
                           else (carstate.rightBlindspot and (self.voice_change or self._intent_go or not RIGHT_BS_ADVISORY))) and \
                          self.lane_change_timer < LANE_CHANGE_BSM_ABORT_TIME
        if not going_left and carstate.rightBlindspot:
          self._right_bs_seen = True

        slowed_to_turn = v_ego < LANE_CHANGE_ABORT_SPEED or (nudge_band and abs(carstate.steeringAngleDeg) > TURN_ANGLE_DEG)
        if opposite_blinker or torque_against or blindspot_ahead or slowed_to_turn:
          if self._intent_go and self.blinker_intent is not None:
            reason = "opposite_blinker" if opposite_blinker else \
                     "torque_against" if torque_against else "blindspot" if blindspot_ahead else "slowed_to_turn"
            try:
              self.blinker_intent.on_cancel(reason, v_ego * CV.MS_TO_MPH)
            except Exception:
              pass
            self._intent_go = False
          if not going_left:
            cloudlog.event("canopy_right_change", phase="cancel", bs_seen=self._right_bs_seen, opposite_blinker=bool(opposite_blinker),
                           torque_against=bool(torque_against), slowed=bool(slowed_to_turn), t=round(self.lane_change_timer, 2))
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
          self.voice_change = False

        elif lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self._intent_go = False
          if not going_left:
            cloudlog.event("canopy_right_change", phase="done", bs_seen=self._right_bs_seen, t=round(self.lane_change_timer, 2))
          self.lane_change_timer = 0.0
          self.hold_changes += 1
          if one_blinker and not self.voice_change:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
            if self._hold_allows(self.lane_change_direction):
              # Canopy: still short of the route's exit lane, blinker held: allow one more, after a settle
              self.alc.prev_lane_change = False
              self.alc.lane_change_wait_timer = -HOLD_SETTLE_TIME
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
            self.voice_change = False

    self.prev_one_blinker = one_blinker and lateral_active
    self._prev_held = (bool(carstate.leftBlinker), bool(carstate.rightBlinker))

    if self.lane_turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.lane_turn_direction]
    else:
      self.desire = log.Desire.none
      if self.lane_change_state == LaneChangeState.laneChangeStarting:
        if self.lane_change_direction == LaneChangeDirection.left:
          self.desire = log.Desire.laneChangeLeft
        elif self.lane_change_direction == LaneChangeDirection.right:
          self.desire = log.Desire.laneChangeRight

    self.alc.update_state()
