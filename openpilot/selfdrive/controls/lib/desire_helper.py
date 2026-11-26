from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.ModelDataV2SP.TurnDirection

# Canopy (Nov 26): under 40 mph the blinker means a turn, never a comma lane change. The 27.4 min
# event on the Nov 26 lap was a right turn signalled at 31 mph that the comma took as a lane change
# and began steering toward the curb. Every lane change it finished that night was 59 mph or faster.
LANE_CHANGE_SPEED_MIN = 40 * CV.MPH_TO_MS
# A change already under way is dropped if the car slows below this: that is a turn, not a merge.
LANE_CHANGE_ABORT_SPEED = 30 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5
# Blind spot can cancel a started lane change only this early, while the car is still well inside
# its own lane. Typical openpilot lane changes take ~4-6 s and cross the line around the midpoint.
LANE_CHANGE_BSM_ABORT_TIME = 1.5

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

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False):
    carstate = _HeldBlinkers(carstate, *self._hold_blinkers(carstate.leftBlinker, carstate.rightBlinker))
    self._read_hold_side()
    self.alc.update_params()
    self.alc.bsm_ok = self.enable_bsm and carstate.canValid
    self.lane_turn_controller.update_params()
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # Lane turn controller update
    self.lane_turn_controller.update_lane_turn(blindspot_left=carstate.leftBlindspot, blindspot_right=carstate.rightBlindspot,
                                               left_blinker=carstate.leftBlinker, right_blinker=carstate.rightBlinker, v_ego=v_ego)
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off:
        self.hold_changes = 0
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

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


        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        elif (torque_applied or self.alc.auto_lane_change_allowed) and not self.alc.lane_change_blocked:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0
          if self.lane_change_direction == LaneChangeDirection.right:
            self._right_bs_seen = bool(carstate.rightBlindspot)
            cloudlog.event("canopy_right_change", phase="start", bs_right=bool(carstate.rightBlindspot),
                           bs_left=bool(carstate.leftBlindspot), mph=round(v_ego * CV.MS_TO_MPH), nudge=bool(torque_applied),
                           hold=self.hold_changes)

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        # Latched: once started, the signal going off does not cancel it, so a tap carries a full merge.
        # Cancel on driver torque against the change or the opposite signal (or hazards) at any point.
        # A blind spot on that side cancels only in the first LANE_CHANGE_BSM_ABORT_TIME, before the car
        # nears the line: after that the sensor is watching the lane beyond the target, so a car there
        # must not pull us back across. lateral_active loss and LANE_CHANGE_TIME_MAX still cancel at the top.
        going_left = self.lane_change_direction == LaneChangeDirection.left
        opposite_blinker = carstate.rightBlinker if going_left else carstate.leftBlinker
        torque_against = carstate.steeringPressed and \
                         ((carstate.steeringTorque < 0) if going_left else (carstate.steeringTorque > 0))
        blindspot_ahead = (carstate.leftBlindspot if going_left else (carstate.rightBlindspot and not RIGHT_BS_ADVISORY)) and \
                          self.lane_change_timer < LANE_CHANGE_BSM_ABORT_TIME
        if not going_left and carstate.rightBlindspot:
          self._right_bs_seen = True

        slowed_to_turn = v_ego < LANE_CHANGE_ABORT_SPEED
        if opposite_blinker or torque_against or blindspot_ahead or slowed_to_turn:
          if not going_left:
            cloudlog.event("canopy_right_change", phase="cancel", bs_seen=self._right_bs_seen, opposite_blinker=bool(opposite_blinker),
                           torque_against=bool(torque_against), slowed=bool(slowed_to_turn), t=round(self.lane_change_timer, 2))
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0

        elif lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          if not going_left:
            cloudlog.event("canopy_right_change", phase="done", bs_seen=self._right_bs_seen, t=round(self.lane_change_timer, 2))
          self.lane_change_timer = 0.0
          self.hold_changes += 1
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
            if self._hold_allows(self.lane_change_direction):
              # Canopy: still short of the route's exit lane, blinker held: allow one more, after a settle
              self.alc.prev_lane_change = False
              self.alc.lane_change_wait_timer = -HOLD_SETTLE_TIME
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

    self.prev_one_blinker = one_blinker and lateral_active

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
