#!/usr/bin/env python3
"""Canopy head unit stream.

Serves what openpilot sees, in near real time, to the car screen (phone car app) and Zia.
Stdlib only, nothing to install. Binds 127.0.0.1 only: the comma's Tailscale runs in
userspace mode and hands tailnet connections to localhost, so the stream is reachable from
the tailnet and nowhere else (not from the LTE side, not from home Wi-Fi).

  GET /hud     Server-Sent Events while a client is connected: discrete changes at once, geometry at most
               5 frames/s and only past 0.1 m or 0.5 mph, and at least 2 frames/s (see hud_due).
  GET /events  Server-Sent Events, only when something happens (engage, lead, stop, intent).
  GET /health  one JSON line.
  GET /lookahead  the local Look Ahead answer from the last second (watch only, nothing in the planner
               reads it): view, age_s, n, err, off.
  /hud frame "v": 2 adds "lead.mph" (lead car speed over the ground) and "nav_lanes": the nav lane strip
               (lanes, current, target, confident) passed through from /dev/shm/canopy/nav_lanes.json while
               that file is under 2 s old, else null. The nav side writes it; hudd never guesses lanes.
  /hud frame "signal": the comma's own read of the mapped signal it is closing on (canopy/signal_read.py):
               {m, state: red|green|unknown, conf, lat, lon}, or null when none is ahead. A plan read,
               never a colour read. It also goes to Forge on the bus (topic "signal", key "ahead",
               on every state change and at most every 3 s) so a consumer off the car network can
               follow it; the head unit draws it over the top of its own prediction.
  /events also carries {"type": "signal", "state", "m", "conf"} when the read changes signal or state.
  GET /banner, POST /banner {"source","text","style":"info"|"heads_up","timeout_s"},
  DELETE /banner?source=NAME   the status banner on the comma screen (canopy/banner.py).
               Changes also go out on /events as {"type": "banner", "banner": {...} | null},
               so the car app can follow the same state.
  POST /voice_merge {"side": "left"|"right", "id"?}  Zia heard "merge left/right". Answers {"ok", "verdict", "mph", "live", "band"} at once:
               verdict "ok" (written to /dev/shm/canopy/voice_merge.json for the controls to act on), "too_slow" (under 25 mph),
               "too_fast" (85 mph and up: the blinker does it), "steering_off", or "no_frame" (nothing to check against: still written).
               "live" is the freeway flag's existence when the frame's mph >= 50, otherwise the street flag's (unchanged when no frame).
               "band" is "freeway" (mph >= 50), "street" (mph < 50) or None (no frame).
  /events also carries {"type": "voice_merge", "phase", "side", "reason"?, "signaled"?, "live", ...} each time the controls
               report a voice merge step (heard, would_start, start, cancel, expired), from /dev/shm/canopy/voice_merge_state.json.
   POST /heads/stage  one head json (8 KB at most) from canopy/heads.py: checks feature, base_sha, version, w and b, that the
                version matches its sha rule and that it is not in bad.json, then writes it to /data/canopy/heads/staged/.
                Answers {"ok", "head_id"}. Staging only: it never touches active/, so nothing live changes.
   GET /heads   the decision heads on disk: live, shadow, staged, last_good and bad ids (canopy/heads.py).
  GET /garage  live garage parking state (also on /events as {"type": "garage", ...}, 2/s
               at most while parking, one final {"active": false, "reason"} at the end).
  /events also carries {"type": "cue", "slot", "sound": "custom"|"stock"|"silent", "played", "mph",
               "text", "lead_m"} each time a Canopy prompt fires, so Zia can speak only when the
               comma stayed silent and keep quiet right after a cue that already played.
  /hud "lead" also carries "kind" (sedan, hatchback, suv, pickup, van, semi, motorcycle, bus, unknown),
               "kind_conf", "make", "model", "color" (null under the 0.8 bar) and "plate" (null unless read),
               from canopy/leadkind.py: one look-up per new lead track, only while a client is connected.
               The plate is for the live screen only: it is never on /events, never logged, and dropped when
               the lead changes. Draw a generic car when kind is "unknown" or missing.
  /events also carries {"type": "drive", ...} for the car app's drive view, thinned like /hud: path, lanes,
               edges, lead and the nav strip with y positive to the RIGHT, rounded to 0.1 m (see drive_event).
               It never carries the plate. Only for clients that want it (see wants_drive): ?drive=1 or ?drive=0,
               and python or curl clients like Zia don't get it by default.

Frame coordinates are the car frame in meters: x forward, y left-positive (model y is
flipped here so path, lanes, edges and the radar lead all agree).
Speeds in mph (Mark: the assistant always runs in MPH).

The comma's own LTE carries this on the road, so no phone hotspot step is needed.
Turn it off without a rebuild: touch /data/canopy/hud_off (checked every second).
The local Look Ahead thread stops on its own flag too, /data/canopy/lookahead_off, and offroad.
"""
import json
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from openpilot.cereal import messaging, custom
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params

from openpilot.sunnypilot.canopy import banner
from openpilot.sunnypilot.canopy import bus as canopy_bus
from openpilot.sunnypilot.canopy import heads as canopy_heads
from openpilot.sunnypilot.canopy import leadkind
from openpilot.sunnypilot.canopy import lookahead_local
from openpilot.sunnypilot.canopy import signal_read
from openpilot.sunnypilot.canopy import sounds as canopy_sounds
from openpilot.sunnypilot.canopy import params_file

PORT = int(os.environ.get("CANOPY_HUD_PORT", "8870"))
OFF_FLAG = "/data/canopy/hud_off"
VOICE_FILE = os.path.join(banner.BANNER_DIR, "voice_merge.json")
VOICE_STATE = os.path.join(banner.BANNER_DIR, "voice_merge_state.json")
VOICE_LIVE = "/data/canopy/voice_merge_live"
VOICE_LIVE_FREEWAY = "/data/canopy/voice_merge_live_freeway"
HZ = 10
MS_TO_MPH = 2.23694
LA_OFF = "/data/canopy/lookahead_off"
LA_EVERY_S = 1.0
HEADS_MAX_BYTES = 8 * 1024  # one head json, anything bigger is not a head
_ASP = custom.SelfdriveStateSP.AudibleAlert
CUE_SLOTS = {getattr(_ASP, field): slot for slot, (field, _) in canopy_sounds.SLOTS.items()}
_params = Params()
_GPS = get_gps_location_service(_params)
_lk = {"obj": None}
_la = {"view": None, "t": 0.0, "err": None, "n": 0}
_sig = {"obj": None, "state": None, "t": 0.0, "key": None}
SIGNAL_BUS_S = 3.0  # the read goes to the bus on every state change and at most this often

_params_cache = {"mtime": None, "data": None}


def params_snapshot():
  """The in-use params snapshot, read on each /events request and cached by mtime; {} if missing."""
  try:
    m = os.path.getmtime(params_file.IN_USE_PATH)
  except OSError:
    return {}
  if m == _params_cache["mtime"]:
    return _params_cache["data"]
  _params_cache["mtime"] = m
  try:
    with open(params_file.IN_USE_PATH) as f:
      d = json.load(f)
    _params_cache["data"] = d if isinstance(d, dict) else {}
  except (OSError, ValueError):
    _params_cache["data"] = {}
  return _params_cache["data"]


def lead_kind():
  if _lk["obj"] is None:
    _lk["obj"] = leadkind.LeadKind()
  return _lk["obj"]


def signal_reader():
  if _sig["obj"] is None:
    _sig["obj"] = signal_read.SignalRead()
  return _sig["obj"]


def publish_signal(sig, now):
  """The read on the bus: on a state change, else at most every SIGNAL_BUS_S. The bus drops an unchanged payload."""
  if sig is None:
    if _sig["state"] not in (None, "none"):
      _sig["state"], _sig["t"] = "none", now
      canopy_bus.bus().publish("signal", {"m": None, "state": "none", "conf": 0.0, "lat": None, "lon": None}, key="ahead")
    return
  if sig["state"] == _sig["state"] and now - _sig["t"] < SIGNAL_BUS_S:
    return
  _sig["state"], _sig["t"] = sig["state"], now
  canopy_bus.bus().publish("signal", sig, key="ahead")


def cue_sound(slot: str) -> str:
  """What the comma plays for this cue: custom, stock or silent (QuietMode silences prompts)."""
  try:
    if _params.get_bool("QuietMode"):
      return "silent"
  except Exception:
    pass
  if canopy_sounds.read_wav(os.path.join(canopy_sounds.SOUND_DIR, f"{slot}.wav")) is not None:
    return "custom"
  return "stock" if canopy_sounds.SLOTS[slot][1] else "silent"
# model plan points to send (index into the 33 point model plan, roughly 0 to 190 m)
IDX = [0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 23, 26]

_clients_hud: list[queue.Queue] = []
_clients_ev: list[queue.Queue] = []
_lock = threading.Lock()
_last = {"frame": None, "t": 0.0}
_lead = {"state": False, "since": None}
_banner = {"mtime": None, "last": None}
_garage = {"mtime": None}
_voice = {"mtime": None}
GARAGE_LIVE = os.path.join(banner.BANNER_DIR, "garage_live.json")


def garage_state():
  try:
    with open(GARAGE_LIVE) as f:
      return json.load(f)
  except (OSError, ValueError):
    return None


def garage_event():
  """One "garage" event each time the live parking state changes (at most 2 a second)."""
  try:
    m = os.path.getmtime(GARAGE_LIVE)
  except OSError:
    return None
  if m == _garage["mtime"]:
    return None
  _garage["mtime"] = m
  st = garage_state()
  if st is None:
    return None
  return json.dumps(dict(st, type="garage", params=params_snapshot()), separators=(",", ":"))


def voice_event():
  """One "voice_merge" event each time the controls write a new voice merge step."""
  try:
    m = os.path.getmtime(VOICE_STATE)
  except OSError:
    return None
  if m == _voice["mtime"]:
    return None
  _voice["mtime"] = m
  try:
    with open(VOICE_STATE) as f:
      st = json.load(f)
  except (OSError, ValueError):
    return None
  if not isinstance(st, dict):
    return None
  return json.dumps(dict(st, type="voice_merge", params=params_snapshot()), separators=(",", ":"))


def voice_verdict(frame, age_s) -> str:
  if frame is None or age_s > 1.0:
    return "no_frame"
  if frame.get("steering") is not True:
    return "steering_off"
  mph = frame.get("mph") or 0
  if mph < 25:
    return "too_slow"
  if mph >= 85:
    return "too_fast"
  return "ok"


def banner_event():
  """One banner event when the banner file changed and what shows changed."""
  m = banner.mtime()
  if m == _banner["mtime"]:
    return None
  _banner["mtime"] = m
  cur = banner.current()
  key = None if cur is None else (cur["source"], cur["text"], cur["style"])
  if key == _banner["last"]:
    return None
  _banner["last"] = key
  b = None if cur is None else {k: cur[k] for k in ("source", "text", "style", "until")}
  return json.dumps({"type": "banner", "banner": b, "t": _r(time.time(), 2), "params": params_snapshot()}, separators=(",", ":"))


NAV_LANES = os.path.join(banner.BANNER_DIR, "nav_lanes.json")
NAV_MAX_AGE_S = 2.0
_nav = {"mtime": None, "data": None}


def nav_lanes():
  """The nav lane strip as the nav side wrote it, only while fresh. Checked by mtime, so a 10 Hz
  stream costs a stat() call, not a JSON parse, unless the file changed."""
  try:
    st = os.stat(NAV_LANES)
  except OSError:
    return None
  if time.time() - st.st_mtime > NAV_MAX_AGE_S:
    return None
  key = (st.st_mtime_ns, st.st_size, st.st_ino)
  if key != _nav["mtime"]:
    _nav["mtime"] = key
    try:
      with open(NAV_LANES) as f:
        d = json.load(f)
      lanes = d.get("lanes")
      _nav["data"] = {"lanes": int(lanes) if lanes is not None else None,
                      "current": d.get("current"), "target": d.get("target"),
                      "confident": bool(d.get("confident")),
                      "reason": str(d.get("reason") or "")[:48] or None} if isinstance(d, dict) else None
    except (OSError, ValueError, TypeError):
      _nav["data"] = None
  return _nav["data"]


def _r(v, n=2):
  return round(float(v), n)


def _pick(xyz, idx):
  x, y = list(xyz.x), list(xyz.y)
  if len(x) < 2:
    return []
  return [[_r(x[i], 1), _r(-y[i])] for i in idx if i < len(x)]  # modelV2 y is right-positive


def _ys(xyz, idx):
  y = list(xyz.y)
  return [_r(-y[i]) for i in idx if i < len(y)]  # flip to left-positive


def _xs(xyz, idx):
  x = list(xyz.x)
  return [_r(x[i], 1) for i in idx if i < len(x)]


def build_frame(sm) -> dict:
  md = sm["modelV2"]
  cs = sm["carState"]
  ss = sm["selfdriveState"]
  rs = sm["radarState"]
  lead = rs.leadOne
  f = {
    "v": 2,
    "t": _r(time.time(), 2),
    "mph": _r(cs.vEgo * MS_TO_MPH, 1),
    "engaged": bool(ss.enabled),
    "steering": bool(sm["carControl"].latActive),
    "override": bool(cs.steeringPressed),
    "blinker": "left" if cs.leftBlinker else ("right" if cs.rightBlinker else None),
    "path": _pick(md.position, IDX),
    # lane lines and road edges share one x list: lanes far left, left, right, far right
    "lane_x": _xs(md.laneLines[0], IDX) if len(md.laneLines) else [],
    "lanes": [{"p": _r(p), "y": _ys(l, IDX)} for l, p in zip(md.laneLines, md.laneLineProbs)],  # far left first
    "edges": [_ys(e, IDX) for e in md.roadEdges],
    "lead": None,
    "should_stop": bool(md.action.shouldStop),
    "want_accel": _r(md.action.desiredAcceleration),
    "curvature": _r(md.action.desiredCurvature, 5),
    "lane_change": str(md.meta.laneChangeState),
    "lane_change_dir": str(md.meta.laneChangeDirection),
    "brake_soon": _r(max(md.meta.disengagePredictions.brake3MetersPerSecondSquaredProbs or [0])),
    "alert": ss.alertText1 or None,
    "sound": int(ss.alertSound.raw),
    "nav_lanes": nav_lanes(),
  }
  lk = None
  try:
    g = sm[_GPS] if sm.recv_frame[_GPS] > 0 and sm[_GPS].hasFix else None
  except Exception:
    g = None
  try:
    lk = lead_kind()
    rpy = list(sm["liveCalibration"].rpyCalib) or [0.0, 0.0, 0.0]
    lk.observe(lead, rpy, (g.latitude, g.longitude) if g is not None else None)
  except Exception:
    lk = None  # display only
  if lead.present:
    f["lead"] = {"m": _r(lead.dRel, 1), "y": _r(lead.yRel, 2), "rel_mph": _r(lead.vRel * MS_TO_MPH, 1),
                 "mph": _r(max(lead.vLead, 0.0) * MS_TO_MPH, 1),
                 "from_radar": bool(lead.radar),
                 "kind": "unknown", "kind_conf": None, "make": None, "model": None, "color": None, "plate": None}
    info = lk.current() if lk else None
    if info:
      f["lead"].update({k: info.get(k) for k in ("kind", "kind_conf", "make", "model", "color", "plate")})
      f["lead"]["kind"] = f["lead"]["kind"] or "unknown"
  f["signal"] = None
  try:
    f["signal"] = signal_reader().update(
      g.latitude if g is not None else None, g.longitude if g is not None else None,
      g.bearingDeg if g is not None else None, cs.vEgo,
      list(md.position.x), list(md.velocity.x), lead.dRel if lead.present else None)
  except Exception:
    f["signal"] = None  # display only
  return f


def _rt(pts):
  """[[x, y_left]] to the car app's [[x, y_right]], 0.1 m."""
  return [[round(x, 1), round(-y, 1)] for x, y in pts]


def drive_event(cur, lead_p) -> dict:
  """The car app's calm drive view (car app 0.15.11 contract): y positive to the RIGHT, 0.1 m, ~10 Hz on /events.
  No plate here: /events may be logged by other listeners."""
  xs = cur["lane_x"]
  lead = cur["lead"]
  nav = cur["nav_lanes"]
  strip = None
  if nav and isinstance(nav.get("lanes"), int) and nav["lanes"] > 0:
    n = nav["lanes"]

    def idx(v):
      if isinstance(v, int):
        return v
      return {"rightmost": n - 1, "leftmost": 0}.get(v)
    strip = {"n": n, "cur": idx(nav.get("current")), "target": idx(nav.get("target")), "confident": nav["confident"]}
  return {
    "type": "drive", "t": cur["t"], "engaged": cur["steering"] or cur["engaged"], "mph": cur["mph"],
    "path": _rt(cur["path"]),
    "lanes": [{"pts": _rt(zip(xs, l["y"])), "p": l["p"]} for l in cur["lanes"]],
    "edges": [_rt(zip(xs, e)) for e in cur["edges"]],
    "lead": None if lead is None else {
      "d_m": round(lead["m"], 1), "y_m": round(-lead["y"], 1), "v_rel_mph": lead["rel_mph"], "p": lead_p,
      "cls": "other" if lead["kind"] in (None, "unknown") else lead["kind"],
      "cls_p": lead["kind_conf"] if lead["kind"] not in (None, "unknown") else 0.0,
      "make": lead["make"], "model": lead["model"], "color": lead["color"]},
    "strip": strip,
    "params": params_snapshot(),
  }


def no_plate(lead):
  """The plate goes to the live /hud screen only, never into /events (Zia and others may log those)."""
  return None if lead is None else {k: v for k, v in lead.items() if k != "plate"}


def events_from(prev, cur) -> list[dict]:
  ev = []
  if prev is None:
    return ev
  if prev["engaged"] != cur["engaged"]:
    ev.append({"type": "engaged" if cur["engaged"] else "disengaged"})
  if prev["should_stop"] != cur["should_stop"] and cur["mph"] > 5:
    ev.append({"type": "stop_ahead" if cur["should_stop"] else "stop_clear"})
  has = cur["lead"] is not None
  if has != _lead["state"]:
    if _lead["since"] is None:
      _lead["since"] = cur["t"]
    elif cur["t"] - _lead["since"] >= 1.0:  # 1 s stable, no flicker on the screen
      _lead["state"], _lead["since"] = has, None
      ev.append({"type": "lead_new" if has else "lead_gone", "lead": no_plate(cur["lead"])})
  else:
    _lead["since"] = None
  if prev["lane_change"] != cur["lane_change"]:
    ev.append({"type": "lane_change", "state": cur["lane_change"], "dir": cur["lane_change_dir"]})
  sig = cur.get("signal")
  ident = None if sig is None else (sig.get("state"), sig.get("lat"), sig.get("lon"))
  if ident != _sig["key"] and (ident is not None or _sig["key"] is not None):
    _sig["key"] = ident
    ev.append({"type": "signal", "state": (sig or {}).get("state", "unknown"), "m": (sig or {}).get("m"),
               "conf": (sig or {}).get("conf", 0.0)})
  if prev["alert"] != cur["alert"] and cur["alert"]:
    ev.append({"type": "alert", "text": cur["alert"]})
  if prev["sound"] != cur["sound"] and cur["sound"] in CUE_SLOTS:
    # a Canopy prompt fired: say which, and whether the comma made a sound, so Zia can speak only
    # into silence (garage, QuietMode) and stay quiet right after a cue that already played
    slot = CUE_SLOTS[cur["sound"]]
    snd = cue_sound(slot)
    ev.append({"type": "cue", "slot": slot, "sound": snd, "played": snd != "silent", "mph": cur["mph"],
               "text": cur["alert"], "lead_m": (cur["lead"] or {}).get("m")})
  for e in ev:
    e["t"] = cur["t"]
    e["params"] = params_snapshot()
  return ev


def publish(qs, item):
  with _lock:
    for q in list(qs):
      try:
        q.put_nowait(item)
      except queue.Full:
        pass  # slow client: drop frames rather than lag behind


def wants_drive(path: str, ua: str) -> bool:
  """The 10 Hz "drive" event is for car app 0.15.11 only (0.15.12 reads /hud). Clients ask with
  /events?drive=1 or turn it off with ?drive=0; with neither, scripts (python-httpx, curl: Zia on Forge)
  don't get it, so it doesn't spend the comma's LTE on clients that never draw it."""
  q = path.partition("?")[2]
  if "drive=1" in q:
    return True
  if "drive=0" in q:
    return False
  return not ua.lower().startswith(("python", "curl"))


HUD_MIN_S = 0.2    # /hud at most 5 frames/s for geometry changes (car app LTE, Nov 26)
HUD_KEEP_S = 0.5   # and at least 2/s, under the car app's 1 s stale mark
HUD_MOVE_M = 0.1   # path, lanes, edges or lead must move this much to count as changed
_hud_sent = {"frame": None, "t": 0.0}
_drive_sent = {"frame": None, "t": 0.0}  # the drive event on /events is thinned the same way
_HUD_SKIP = {"t", "path", "lanes", "edges", "lead", "mph"}


def _moved(a, b, tol) -> bool:
  if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
    return abs(a - b) > tol
  if isinstance(a, list) and isinstance(b, list):
    return len(a) != len(b) or any(_moved(x, y, tol) for x, y in zip(a, b))
  if isinstance(a, dict) and isinstance(b, dict):
    return a.keys() != b.keys() or any(_moved(a[k], b[k], tol) for k in a)
  return a != b


def hud_due(sent: dict, cur: dict, now: float) -> bool:
  """Whether a /hud frame goes out. Discrete changes (engage, override, blinker, lead kind, nav strip,
  a lead appearing or going) go out at once; geometry and speed changes at most every 0.2 s and only past
  0.1 m or 0.5 mph; and a frame at least every 0.5 s so the screen never reads stale. About half the data
  of a flat 10 Hz on a steady road."""
  prev = sent["frame"]
  if prev is None or now - sent["t"] >= HUD_KEEP_S:
    return True
  if any(prev.get(k) != cur.get(k) for k in cur.keys() | prev.keys() if k not in _HUD_SKIP):
    return True
  pl, cl = prev.get("lead"), cur.get("lead")
  if (pl is None) != (cl is None):
    return True
  if pl and cl and any(pl.get(k) != cl.get(k) for k in ("kind", "make", "model", "color", "plate")):
    return True
  if now - sent["t"] < HUD_MIN_S:
    return False
  return (_moved(prev.get("mph"), cur.get("mph"), 0.5) or
          any(_moved(prev.get(k), cur.get(k), HUD_MOVE_M) for k in ("path", "lanes", "edges", "lead")))


def publish_lossy(qs, item, keep_free=16):
  """10 Hz drive events: only to clients that want them, and skipped for a client that is falling behind,
  so cues and alerts always fit."""
  with _lock:
    for q in list(qs):
      if getattr(q, "drive", False) and q.maxsize - q.qsize() > keep_free:
        try:
          q.put_nowait(item)
        except queue.Full:
          pass


def onroad_now() -> bool:
  return _params.get_bool("IsOnroad")


def lookahead_step(sm, la, onroad):
  """One second of the local Look Ahead, watch only: nothing in the planner reads it.
  Returns the view it wrote, or None when it skipped or something went wrong."""
  try:
    sm.update(0)
    if os.path.exists(LA_OFF) or not onroad():
      _la["view"] = None
      return None
    g = sm[_GPS]
    if sm.recv_frame[_GPS] == 0 or not getattr(g, "hasFix", True):
      return None
    mph = sm["carState"].vEgo * MS_TO_MPH
    v = la.view(g.latitude, g.longitude, g.bearingDeg, mph)
    la.write_shm(v)
    la.log(v, g.latitude, g.longitude, g.bearingDeg, mph)
    _la["view"], _la["t"] = v, time.monotonic()
    _la["n"] += 1
    _la["err"] = None
    return v
  except Exception as e:  # watch only: never take anything down over it
    _la["err"] = str(e)[:200]
    return None


def lookahead_loop(sleep=time.sleep, sm=None, la=None, onroad=None):
  """Answers Look Ahead locally once a second for the whole drive. Its own thread, its own
  SubMaster and reader, and it runs at the process's nice level (main already lowered it)."""
  if sm is None:
    sm = messaging.SubMaster(["carState", _GPS])
  if la is None:
    la = lookahead_local.LookAhead()
  if onroad is None:
    onroad = onroad_now
  while True:
    try:
      lookahead_step(sm, la, onroad)
    except Exception as e:
      _la["err"] = str(e)[:200]
    sleep(LA_EVERY_S)


def pump():
  sm = messaging.SubMaster(["modelV2", "carState", "selfdriveState", "radarState", "carControl",
                              "liveCalibration", _GPS])
  prev = None
  next_t = 0.0
  next_banner = 0.0
  while True:
    sm.update(100)
    if time.monotonic() >= next_banner:
      next_banner = time.monotonic() + 0.5
      if _clients_ev and not os.path.exists(OFF_FLAG):
        try:
          for ev in (banner_event(), garage_event(), voice_event()):
            if ev:
              publish(_clients_ev, ev)
        except Exception:
          pass
    if not sm.updated["modelV2"]:
      continue
    now = time.monotonic()
    if now < next_t:
      continue
    next_t = now + 1.0 / HZ
    if os.path.exists(OFF_FLAG):
      prev = None
      continue
    try:
      cur = build_frame(sm)
    except Exception as e:  # never take anything down over a display stream
      cur = None
      publish(_clients_ev, json.dumps({"type": "hud_error", "msg": str(e)[:200], "params": params_snapshot()}))
    if cur is None:
      continue
    _last["frame"], _last["t"] = cur, now
    try:
      publish_signal(cur.get("signal"), now)  # the read goes to Forge whether or not anyone is watching locally
    except Exception:
      pass
    if _clients_hud and hud_due(_hud_sent, cur, now):
      _hud_sent["frame"], _hud_sent["t"] = cur, now
      publish(_clients_hud, json.dumps(cur, separators=(",", ":")))
    if any(getattr(q, "drive", False) for q in _clients_ev) and hud_due(_drive_sent, cur, now):
      _drive_sent["frame"], _drive_sent["t"] = cur, now
      try:
        lp = _r(sm["radarState"].leadOne.modelProb) if cur["lead"] else None
        publish_lossy(_clients_ev, json.dumps(drive_event(cur, lp), separators=(",", ":")))
      except Exception:
        pass
    for e in events_from(prev, cur):
      publish(_clients_ev, json.dumps(e, separators=(",", ":")))
    prev = cur


class H(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"

  def log_message(self, *a):
    pass

  def _sse(self, qs, drive=False):
    q: queue.Queue = queue.Queue(maxsize=5 if qs is _clients_hud else 20)
    q.drive = drive  # type: ignore[attr-defined]
    with _lock:
      qs.append(q)
    try:
      self.send_response(200)
      self.send_header("Content-Type", "text/event-stream")
      self.send_header("Cache-Control", "no-cache")
      self.send_header("Connection", "keep-alive")
      self.end_headers()
      while True:
        try:
          data = q.get(timeout=5)
          self.wfile.write(f"data: {data}\n\n".encode())
        except queue.Empty:
          self.wfile.write(b": keepalive\n\n")
        self.wfile.flush()
    except (BrokenPipeError, ConnectionResetError, OSError):
      pass
    finally:
      with _lock:
        if q in qs:
          qs.remove(q)

  def _json(self, code, obj):
    body = json.dumps(obj).encode()
    self.send_response(code)
    self.send_header("Content-Type", "application/json")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)

  def do_POST(self):
    path = self.path.split("?")[0]
    if path == "/voice_merge":
      try:
        n = int(self.headers.get("Content-Length") or 0)
        if n > 2048:
          return self._json(413, {"ok": False, "error": "too big"})
        d = json.loads(self.rfile.read(n) or b"{}")
      except (ValueError, OSError) as e:
        return self._json(400, {"ok": False, "error": str(e)[:120]})
      side = d.get("side") if isinstance(d, dict) else None
      if side not in ("left", "right"):
        return self._json(400, {"ok": False, "error": "side"})
      frame = _last["frame"]
      age = time.monotonic() - _last["t"] if frame else 1e9
      verdict = voice_verdict(frame, age)
      if verdict in ("ok", "no_frame"):
        body = {"side": side, "t": time.time(), "id": str(d.get("id") or time.time())[:40]}
        try:
          with open(VOICE_FILE + ".tmp", "w") as f:
            json.dump(body, f, separators=(",", ":"))
          os.replace(VOICE_FILE + ".tmp", VOICE_FILE)
        except OSError:
          return self._json(500, {"ok": False, "error": "write"})
      mph = frame.get("mph") if frame else None
      band = "freeway" if (frame is not None and mph is not None and mph >= 50) else "street" if (frame is not None and mph is not None) else None
      live = os.path.exists(VOICE_LIVE_FREEWAY) if band == "freeway" else (os.path.exists(VOICE_LIVE) if frame is not None else os.path.exists(VOICE_LIVE))
      return self._json(200, {"ok": True, "verdict": verdict, "mph": mph,
                              "live": live, "band": band})
    if path == "/heads/stage":
      try:
        n = int(self.headers.get("Content-Length") or 0)
        if n > HEADS_MAX_BYTES:
          return self._json(413, {"ok": False, "error": "too big"})
        d = json.loads(self.rfile.read(n) or b"{}")
      except (ValueError, OSError) as e:
        return self._json(400, {"ok": False, "error": str(e)[:120]})
      try:
        hid = canopy_heads.stage_head(d)
      except (ValueError, OSError) as e:
        return self._json(400, {"ok": False, "error": str(e)[:120]})
      return self._json(200, {"ok": True, "head_id": hid})
    if path != "/banner":
      return self._json(404, {"ok": False})
    try:
      n = int(self.headers.get("Content-Length") or 0)
      if n > 2048:
        return self._json(413, {"ok": False, "error": "too big"})
      d = json.loads(self.rfile.read(n) or b"{}")
      err = banner.post(d.get("source"), d.get("text"), d.get("style", "info"), d.get("timeout_s", 10))
    except (ValueError, OSError, AttributeError) as e:
      err = str(e)[:120]
    if err:
      return self._json(400, {"ok": False, "error": err})
    return self._json(200, {"ok": True, "banner": banner.current()})

  def do_DELETE(self):
    path, _, q = self.path.partition("?")
    if path != "/banner":
      return self._json(404, {"ok": False})
    src = dict(p.split("=", 1) for p in q.split("&") if "=" in p).get("source", "")
    return self._json(200, {"ok": True, "cleared": banner.clear(src)})

  def do_GET(self):
    path = self.path.split("?")[0]
    if path == "/banner":
      return self._json(200, {"banner": banner.current()})
    if path == "/garage":
      return self._json(200, {"garage": garage_state()})
    if path == "/lookahead":
      return self._json(200, {"lookahead": _la["view"],
                              "age_s": round(time.monotonic() - _la["t"], 1) if _la["view"] else None,
                              "n": _la["n"], "err": _la["err"], "off": os.path.exists(LA_OFF)})
    if path == "/heads":
      hs = canopy_heads.HeadSet()
      return self._json(200, {"ok": True, "base": hs.base_id,
                              "live": hs.live_ids(), "shadow": hs.shadow_ids(),
                              "staged": hs.staged_ids(), "last_good": hs.last_good_ids(),
                              "bad": hs.bad, "errors": hs.errors})
    if path == "/hud":
      return self._sse(_clients_hud)
    if path == "/events":
      return self._sse(_clients_ev, drive=wants_drive(self.path, self.headers.get("User-Agent") or ""))
    if path == "/health":
      body = json.dumps({"ok": True, "hud_clients": len(_clients_hud), "event_clients": len(_clients_ev),
                         "frame_age_s": _r(time.monotonic() - _last["t"], 1) if _last["frame"] else None,
                         "lookahead_n": _la["n"],
                         "off": os.path.exists(OFF_FLAG)}).encode()
      self.send_response(200)
      self.send_header("Content-Type", "application/json")
      self.send_header("Content-Length", str(len(body)))
      self.end_headers()
      self.wfile.write(body)
      return
    self.send_response(404)
    self.send_header("Content-Length", "0")
    self.end_headers()


def main():
  try:
    os.nice(10)  # display only: always behind openpilot's own processes
  except OSError:
    pass
  threading.Thread(target=pump, daemon=True).start()
  threading.Thread(target=lookahead_loop, daemon=True).start()
  srv = ThreadingHTTPServer(("127.0.0.1", PORT), H)
  srv.daemon_threads = True
  srv.serve_forever()


if __name__ == "__main__":
  main()
