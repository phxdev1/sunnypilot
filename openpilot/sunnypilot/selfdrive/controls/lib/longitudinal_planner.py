"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.sunnypilot.selfdrive.controls.lib.accel_boost import AccelBoostSP
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle
from openpilot.sunnypilot.canopy.move_over import MoveOver
from openpilot.sunnypilot.canopy.lane_drop import LaneDrop
from openpilot.sunnypilot.canopy.red_light import RedLight
from openpilot.sunnypilot.canopy.garage import GaragePark
from openpilot.sunnypilot.canopy.auto_resume import AutoResumeShadow
from openpilot.sunnypilot.canopy.lane_hold import LaneHold
from openpilot.sunnypilot.canopy.trouble_spot import TroubleSpots
from openpilot.common.swaglog import cloudlog

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()
    self.move_over = MoveOver()
    self.lane_drop = LaneDrop()
    self.lane_hold = LaneHold()
    self.red_light = RedLight()
    self.garage = GaragePark()
    self.auto_resume = AutoResumeShadow()
    self.trouble = TroubleSpots()
    self.accel_boost_sp = AccelBoostSP()
    self.openpilot_longitudinal = CP.openpilotLongitudinalControl

    self.output_v_target = 0.
    self.output_a_target = 0.

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    # Canopy: one chime as a ramp meter's light comes up (ProPILOT won't stop for it on its own)
    if self.resolver.ramp_meter.chime:
      self.events_sp.add(custom.OnroadEventSP.EventName.canopyRampMeter)

    # Canopy: right lane at an on-ramp merge, left clear: one chime to move over (his blinker tap does it)
    try:
      map_data = sm['liveMapDataSP']
      gps = sm[self.resolver._gps_location_service]
      map_limit = map_data.speedLimit if map_data.speedLimitValid else 0.
      if self.move_over.update(gps, sm['modelV2'], CS, v_ego, map_limit):
        self.events_sp.add(custom.OnroadEventSP.EventName.canopyMoveOver)
      # Canopy: right lane ends or turns into an exit ahead: one chime ~20 s out
      if self.lane_drop.update(gps, sm['modelV2'], CS, v_ego, map_limit):
        self.events_sp.add(custom.OnroadEventSP.EventName.canopyLaneExit if self.lane_drop.kind == LaneDrop.EXITS
                           else custom.OnroadEventSP.EventName.canopyLaneDrop)
    except Exception:  # a prompt must never take the planner down
      cloudlog.exception("canopy lane prompts failed")

    # Canopy: hold-to-merge toward the route's exit lane, and a chime when a lane change is done but
    # the blinker is still on
    try:
      if self.lane_hold.update(sm[self.resolver._gps_location_service], sm['modelV2'], CS, v_ego):
        self.events_sp.add(custom.OnroadEventSP.EventName.canopyBlinkerOff)
        cloudlog.event("canopy_blinker_off", in_zone=self.lane_hold.in_zone, at_edge=self.lane_hold.at_edge)
      if self.lane_hold.merge_cue == "left":
        self.events_sp.add(custom.OnroadEventSP.EventName.canopyMergeLeft)
      elif self.lane_hold.merge_cue == "right":
        if CS.rightBlindspot:
          self.events_sp.add(custom.OnroadEventSP.EventName.canopyRightBlindspot)
        else:
          self.events_sp.add(custom.OnroadEventSP.EventName.canopyMergeRight)
    except Exception:
      cloudlog.exception("canopy lane hold failed")

    # Canopy: red light ahead with cruise on and nobody to stop behind: one chime so he brakes
    try:
      if self.red_light.update(sm[self.resolver._gps_location_service], sm['modelV2'], CS, sm['radarState'].leadOne, v_ego):
        self.events_sp.add(custom.OnroadEventSP.EventName.canopyRedLight)
        cloudlog.event("canopy_red_light", signal=self.red_light.signal, distance=round(self.red_light.distance, 1),
                       v_ego=round(v_ego, 2))
    except Exception:
      cloudlog.exception("canopy red light warning failed")

    # Canopy: pulling into the garage at home: amber banner and path on the comma screen, a cue as it
    # starts and ends (Reverse ends it at once). Shadow only: nothing steers.
    try:
      if self.garage.update(sm[self.resolver._gps_location_service], CS, CS.vEgo) and self.garage.last_event:
        kind, info = self.garage.last_event
        cloudlog.event(f"canopy_garage_{kind}", **info)
        self.events_sp.add(custom.OnroadEventSP.EventName.canopyGarageStart if kind == "start"
                           else custom.OnroadEventSP.EventName.canopyGarageEnd)
    except Exception:
      cloudlog.exception("canopy garage banner failed")

    # Canopy: auto-resume from a stop, SHADOW ONLY. Logs where it would press RES (car ahead pulled
    # away, engaged, no pedals) and each green light read, next to when he actually moved off.
    try:
      px = sm['modelV2'].position.x
      if self.auto_resume.update(CS, sm['radarState'].leadOne, bool(sm['selfdriveState'].enabled),
                                 px[len(px) - 1] if len(px) else None) and self.auto_resume.last_row:
        cloudlog.event("canopy_resume_shadow", **self.auto_resume.last_row)
    except Exception:
      cloudlog.exception("canopy auto resume shadow failed")

    # Canopy: trouble spot ahead (a place he keeps taking the wheel back): chime + Zia heads-up live;
    # the set-speed trim going in is SHADOW ONLY (logged, never added to the targets below).
    try:
      lm = sm['liveMapDataSP']
      limit = lm.speedLimit if lm.speedLimitValid and lm.speedLimit > 0 else None
      if self.trouble.update(sm[self.resolver._gps_location_service], CS, v_ego, bool(sm['selfdriveState'].enabled),
                             bool(sm['carControl'].latActive), v_cruise_cluster, limit):
        self.events_sp.add(custom.OnroadEventSP.EventName.canopyTroubleSpot)
        cloudlog.event("canopy_trouble_spot", **self.trouble.last_fire)
      if self.trouble.last_row:
        cloudlog.event("canopy_trouble_shadow", **self.trouble.last_row)
    except Exception:
      cloudlog.exception("canopy trouble spot failed")

    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    self.sla.note_plan_target(self.output_v_target)
    return self.output_v_target, self.output_a_target

  def update(self, sm: messaging.SubMaster) -> None:
    if self.accel_boost_sp.update_params():
      self.accel_boost = self.accel_boost_sp if self.accel_boost_sp.enabled and self.openpilot_longitudinal else AccelBoost()

    self.events_sp.clear()
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)
