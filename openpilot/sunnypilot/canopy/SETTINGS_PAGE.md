# Canopy settings page: handoff

Anyone (OpenCode, Claude, Mark) can pick this up from here. Keep this file current.

## Goal (Mark, Nov 27)
"We have a lot of settings we should probably port over." One "canopy" page on the comma four's
Settings screen that shows, and where it is safe changes, every Canopy switch we have added, built on
one file-backed control pattern. Assist strength (1-5) first.

## Where things are
- Worktree / branch: /opt/leaf-sp/canopy-settings-wt, branch `canopy-settings`, based on 2ee6d33
  (the left turn thread's assist strength commit, branch in /opt/leaf-sp/assist-torque-wt).
- OpenCode session: title "Leaf: Canopy settings page" (id in the thread and below once created).
- Ship: `/opt/leaf-can/ci/bin/leafci propose <sha> --by canopy-settings --note "..."` (Park only,
  auto rollback). Never push by hand.

## Key fact
The comma four is device type "mici" (small screen). gui_app.big_ui() is false, so the UI loads
selfdrive/ui/sunnypilot/mici/layouts/settings.py (SettingsLayoutSP). Anything added to the big
layouts under selfdrive/ui/sunnypilot/layouts/settings/ (e.g. Settings > Steering) does NOT show on
Mark's comma.

## Ownership
- Assist strength logic (assist_torque.py, card.py hook, /data/canopy/assist_level, read_level and
  write_level) belongs to the left turn thread. This page only calls read_level/write_level.
- Gates flip automatically ("Fully automatic, but visible"): the page shows gate status read-only.

## Switch inventory (Nov 27, comma at 3f36f05)
On/off flag files in /data/canopy (no file = default):
- hud_off: car screen feed (hudd). Default on.
- lookahead_off: Look Ahead on the comma. Default on.
- leadkind_off: lead classifier. Default on.
- parking_release_on: parking steering release (garage). Default off; currently ON on the comma.
Values:
- assist_level: assist strength 1-5 (1 = stock 1.0 Nm, 5 = 2.0 Nm, under 20 mph). Default 1.
- gates.json "params": lane change auto 50 mph / nudge 25 / lane turn max 19, voice merge delay 1.5 s
  and clean window 8 s, trouble spot heads-up 9 s / 50 m / 2 passes, lead bar 0.8, Zia quiet 5 s.
  Written by Forge (gate thread); read-only on the page.
- params_in_use.json: what the comma actually used last drive. Read-only.
- garage.json: garage spot (zone_m, heading). Never show lat/lon.
- speed_offsets.json: learned speed offsets. Read-only.
Gates: gates.json "features" (25, all shadow on Nov 27). Read-only status.
Openpilot params already on the device: LaneTurnDesire, LaneTurnValue, AutoLaneChangeTimer,
BlinkerPauseLateralControl, BlinkerMinLateralControlSpeed, BlinkerLateralReengageDelay, QuietMode.
Forge-side (not on the comma): /opt/leaf-can/ci/LIVE_ENABLED, /opt/leaf-can/lookahead-comma/PUSH_ENABLED.
Map/data files (not settings, not on the page): traffic_signals, ramp_meters, lane_drops,
onramp_merges, trouble_spots, handback_corners, hold_targets, blinker_hints.

## Plan
1. canopy/settings_store.py: one registry of switches with get/set, atomic writes, stock default on
   any read error; pure python, tested. [step 1]
2. selfdrive/ui/sunnypilot/mici/layouts/canopy.py: CanopyLayoutMici, plus a "canopy" button on
   SettingsLayoutSP. Assist strength first, then the flags, then read-only rows. Import-guarded so a
   failure leaves stock settings untouched. Controls locked while engaged. [step 2]
3. Propose through leafci in Park.

## State / next
- Nov 27 17:40Z: 0a0bb3e settings_store, d05818a mici page committed; import + data smoke on the comma OK
  (flags all read, assist 1, 25 gates, garage "zone 40 m, heading 342"). Rendering untested (no pyray on
  Forge). Proposed d05818a to leafci (replaces turn thread's 2ee6d33 candidate, contains it).
- Next: after it ships, look at the page on the comma.
- Step 2b (Nov 27): loaded the rest in. FLAGS now 9 (added blinker_intent_off, forge_route_off,
  turn_release.off, voice_merge_live, voice_merge_live_freeway; the voice pair is present=on and Forge
  may flip it). New canopy_driving.py page behind a "driving" row on the canopy page holds the 10 PARAMS
  the big-UI Models/Steering pages hide from mici (QuietMode, lane turn, auto lane change, blinker pause,
  Lagd/steer delay, CameraOffset). Left for its own pass: MADS / EnforceTorqueControl / NNLC.

