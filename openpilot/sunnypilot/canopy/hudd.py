#!/usr/bin/env python3
"""Canopy head unit stream.

Serves what openpilot sees, in near real time, to the car screen (phone car app) and Zia.
Stdlib only, nothing to install. Binds 127.0.0.1 only: the comma's Tailscale runs in
userspace mode and hands tailnet connections to localhost, so the stream is reachable from
the tailnet and nowhere else (not from the LTE side, not from home Wi-Fi).

  GET /hud     Server-Sent Events, about 10 frames/s while a client is connected.
  GET /events  Server-Sent Events, only when something happens (engage, lead, stop, intent).
  GET /health  one JSON line.
  GET /banner, POST /banner {"source","text","style":"info"|"heads_up","timeout_s"},
  DELETE /banner?source=NAME   the status banner on the comma screen (canopy/banner.py).
               Changes also go out on /events as {"type": "banner", "banner": {...} | null},
               so the car app can follow the same state.
  GET /garage  live garage parking state (also on /events as {"type": "garage", ...}, 2/s
               at most while parking, one final {"active": false, "reason"} at the end).

Frame coordinates are the car frame in meters: x forward, y left-positive (model y is
flipped here so path, lanes, edges and the radar lead all agree).
Speeds in mph (Mark: the assistant always runs in MPH).

The comma's own LTE carries this on the road, so no phone hotspot step is needed.
Turn it off without a rebuild: touch /data/canopy/hud_off (checked every second).
"""
import json
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cereal.messaging as messaging

from openpilot.sunnypilot.canopy import banner

PORT = int(os.environ.get("CANOPY_HUD_PORT", "8870"))
OFF_FLAG = "/data/canopy/hud_off"
HZ = 10
MS_TO_MPH = 2.23694
# model plan points to send (index into the 33 point model plan, roughly 0 to 190 m)
IDX = [0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 23, 26]

_clients_hud: list[queue.Queue] = []
_clients_ev: list[queue.Queue] = []
_lock = threading.Lock()
_last = {"frame": None, "t": 0.0}
_lead = {"state": False, "since": None}
_banner = {"mtime": None, "last": None}
_garage = {"mtime": None}
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
  return json.dumps(dict(st, type="garage"), separators=(",", ":"))


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
  return json.dumps({"type": "banner", "banner": b, "t": _r(time.time(), 2)}, separators=(",", ":"))


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
  }
  if lead.present:
    f["lead"] = {"m": _r(lead.dRel, 1), "y": _r(lead.yRel, 2), "rel_mph": _r(lead.vRel * MS_TO_MPH, 1),
                 "from_radar": bool(lead.radar)}
  return f


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
      ev.append({"type": "lead_new" if has else "lead_gone", "lead": cur["lead"]})
  else:
    _lead["since"] = None
  if prev["lane_change"] != cur["lane_change"]:
    ev.append({"type": "lane_change", "state": cur["lane_change"], "dir": cur["lane_change_dir"]})
  if prev["alert"] != cur["alert"] and cur["alert"]:
    ev.append({"type": "alert", "text": cur["alert"]})
  for e in ev:
    e["t"] = cur["t"]
  return ev


def publish(qs, item):
  with _lock:
    for q in list(qs):
      try:
        q.put_nowait(item)
      except queue.Full:
        pass  # slow client: drop frames rather than lag behind


def pump():
  sm = messaging.SubMaster(["modelV2", "carState", "selfdriveState", "radarState", "carControl"])
  prev = None
  next_t = 0.0
  next_banner = 0.0
  while True:
    sm.update(100)
    if time.monotonic() >= next_banner:
      next_banner = time.monotonic() + 0.5
      if _clients_ev and not os.path.exists(OFF_FLAG):
        try:
          for ev in (banner_event(), garage_event()):
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
    if os.path.exists(OFF_FLAG) or not (_clients_hud or _clients_ev):
      prev = None
      continue
    try:
      cur = build_frame(sm)
    except Exception as e:  # never take anything down over a display stream
      cur = None
      publish(_clients_ev, json.dumps({"type": "hud_error", "msg": str(e)[:200]}))
    if cur is None:
      continue
    _last["frame"], _last["t"] = cur, now
    publish(_clients_hud, json.dumps(cur, separators=(",", ":")))
    for e in events_from(prev, cur):
      publish(_clients_ev, json.dumps(e, separators=(",", ":")))
    prev = cur


class H(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"

  def log_message(self, *a):
    pass

  def _sse(self, qs):
    q: queue.Queue = queue.Queue(maxsize=5)
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
    if self.path.split("?")[0] != "/banner":
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
    if path == "/hud":
      return self._sse(_clients_hud)
    if path == "/events":
      return self._sse(_clients_ev)
    if path == "/health":
      body = json.dumps({"ok": True, "hud_clients": len(_clients_hud), "event_clients": len(_clients_ev),
                         "frame_age_s": _r(time.monotonic() - _last["t"], 1) if _last["frame"] else None,
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
  srv = ThreadingHTTPServer(("127.0.0.1", PORT), H)
  srv.daemon_threads = True
  srv.serve_forever()


if __name__ == "__main__":
  main()
