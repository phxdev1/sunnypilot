"""Canopy lead car look-up for the head unit drive view. Display only: nothing here feeds control.

Once per new lead track (never per frame), and only while a head unit client is connected, grab one
road camera frame, crop the lead car using radar distance and the camera calibration, and send the
small JPEG to Forge's leadclass service at https://forge.mpvt.io/canopy/lead (the comma's userspace
tailscale can't reach Forge's tailnet IP), with the token in /data/canopy/forge_token. Forge asks Gemini (the OpenRouter key stays
on Forge) and answers body type, make, model and color, each "unknown" under the 0.8 bar.

The plate is read in a second, separate call only when the lead is close enough for the plate to
be legible (most often stopped at a light), once per track. It is held in memory for the current
lead only, shown on the head unit, and dropped the moment the lead changes. It is never written to
disk, logs or any message on the comma, and Forge never stores it either.

Everything runs on one worker thread at nice 10 behind hudd, so a slow network never delays a frame.
"""
import io
import json
import os
import queue
import threading
import time
import urllib.request

import numpy as np

FORGE = os.environ.get("CANOPY_LEADCLASS_URL", "https://forge.mpvt.io/canopy/lead")
# Cloudflare answers the default urllib User-Agent with 1010 before the request reaches Forge.
USER_AGENT = "canopy-lead/1"
TOKEN_PATH = "/data/canopy/forge_token"
OFF_FLAG = "/data/canopy/leadkind_off"
REALDATA = "/data/media/0/realdata"
TIMEOUT_S = 6.0
MIN_CROP_PX = 56  # narrower than this the car is too far to tell anything
PLATE_MAX_M = 6.0  # plate read only this close (about 55 px of plate on the 1344 camera)
PLATE_MIN_CROP_PX = 220
CAM_HEIGHT_M = 1.25
FOCAL = {1928: 2648.0, 1344: 1522.0 * 3 / 4}  # narrow road camera, by frame width (ar/ox vs os04c10)

# calib frame: x forward, y right, z down (relative to the camera); view frame: x right, y down, z forward
VIEW_FROM_DEVICE = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])


def rot_from_euler(rpy):
  r, p, y = rpy
  cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
  rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
  ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
  rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
  return rz @ ry @ rx


def lead_box(d_m, y_left_m, rpy, w, h):
  """Pixel box (x0, y0, x1, y1) around a car d_m ahead, y_left_m to the left, or None if off frame."""
  f = FOCAL.get(w)
  if f is None or d_m <= 1.0:
    return None
  k = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.]])
  m = k @ VIEW_FROM_DEVICE @ rot_from_euler(rpy)
  yc = -y_left_m  # calib y is right-positive
  # a box 2.6 m wide, from the ground to 2.4 m up (covers pickups and vans), at the radar's distance
  pts = np.array([[d_m, yc + sy, z] for sy in (-1.3, 1.3) for z in (CAM_HEIGHT_M, CAM_HEIGHT_M - 2.4)]).T
  uv = m @ pts
  if (uv[2] <= 0).any():
    return None
  u, v = uv[0] / uv[2], uv[1] / uv[2]
  x0, x1, y0, y1 = int(u.min()), int(u.max()), int(v.min()), int(v.max())
  x0, y0, x1, y1 = max(x0, 0) & ~1, max(y0, 0) & ~1, min(x1, w) & ~1, min(y1, h) & ~1
  if x1 - x0 < MIN_CROP_PX or y1 - y0 < MIN_CROP_PX // 2:
    return None
  return x0, y0, x1, y1


def nv12_crop_rgb(buf, stride, uv_off, box):
  """RGB uint8 crop out of an NV12 frame (BT.601 limited range), only the crop is converted."""
  x0, y0, x1, y1 = box
  raw = np.frombuffer(buf, dtype=np.uint8)
  yp = raw[:uv_off].reshape(-1, stride)[y0:y1, x0:x1].astype(np.float32)
  uvp = raw[uv_off:].reshape(-1, stride)[y0 // 2:y1 // 2, x0:x1]
  u = np.repeat(np.repeat(uvp[:, 0::2], 2, 0), 2, 1)[:y1 - y0, :x1 - x0].astype(np.float32) - 128
  v = np.repeat(np.repeat(uvp[:, 1::2], 2, 0), 2, 1)[:y1 - y0, :x1 - x0].astype(np.float32) - 128
  yy = (yp - 16) * 1.164
  rgb = np.stack([yy + 1.596 * v, yy - 0.392 * u - 0.813 * v, yy + 2.017 * u], -1)
  return np.clip(rgb, 0, 255).astype(np.uint8)


def jpeg(rgb, max_w):
  from PIL import Image
  im = Image.fromarray(rgb)
  if im.width > max_w:
    im = im.resize((max_w, max(1, round(im.height * max_w / im.width))))
  b = io.BytesIO()
  im.save(b, "JPEG", quality=85)
  return b.getvalue()


def route_name():
  """This drive's route name (newest log folder), so Forge can count sightings per drive."""
  try:
    ds = [e for e in os.scandir(REALDATA) if e.is_dir() and "--" in e.name]
    return max(ds, key=lambda e: e.stat().st_mtime).name.rsplit("--", 1)[0][:40]
  except (OSError, ValueError):
    return "unknown"


def _token() -> str:
  try:
    with open(TOKEN_PATH) as f:
      return f.read().strip()
  except OSError:
    return ""


def _post(path, body, headers):
  req = urllib.request.Request(FORGE + path, data=body, method="POST",
                               headers=dict({"Content-Type": "image/jpeg", "X-Canopy-Token": _token(),
                                             "User-Agent": USER_AGENT}, **headers))
  with urllib.request.urlopen(req, timeout=TIMEOUT_S) as r:
    return json.load(r)


class LeadKind:
  """hudd calls observe() each frame it builds and reads current() into the /hud lead object."""

  def __init__(self):
    self.q: queue.Queue = queue.Queue(maxsize=2)
    self.track = None  # (track key, since)
    self.info: dict = {}
    self.plate = None  # memory only, current lead only
    self.asked_plate = False
    self.route = None
    self.vipc = None
    self.lock = threading.Lock()
    threading.Thread(target=self._work, daemon=True).start()

  def _key(self, lead):
    """A lead track: the radar track id when there is one, else a new key on a gap or a jump in distance."""
    tid = int(getattr(lead, "radarTrackId", -1))
    if tid >= 0:
      return ("r", tid)
    prev = self.track
    if prev and prev[0][0] == "v" and abs(lead.dRel - prev[2]) < 6.0:
      return prev[0]
    return ("v", time.monotonic())

  def observe(self, lead, rpy, gps):
    """lead: radarState.leadOne; rpy: liveCalibration.rpyCalib; gps: (lat, lon) or None."""
    if os.path.exists(OFF_FLAG):
      return
    if not lead.present:
      if self.track is not None:
        self._drop()
      return
    key = self._key(lead)
    if self.track is None or key != self.track[0]:
      self._drop()
      self.track = (key, time.monotonic(), float(lead.dRel))
      self._ask("classify", key, lead, rpy, gps)
    else:
      self.track = (key, self.track[1], float(lead.dRel))
      if not self.asked_plate and lead.dRel < PLATE_MAX_M:
        self.asked_plate = self._ask("plate", key, lead, rpy, None)

  def _drop(self):
    with self.lock:
      self.track, self.info, self.plate, self.asked_plate = None, {}, None, False

  def _ask(self, kind, key, lead, rpy, gps):
    try:
      self.q.put_nowait((kind, key, float(lead.dRel), float(lead.yRel), list(rpy), gps))
      return True
    except queue.Full:
      return False

  def current(self):
    with self.lock:
      if not self.info and not self.plate:
        return None
      return dict(self.info, plate=self.plate)

  def _frame(self):
    from openpilot.cereal.visionipc import VisionStreamType
    from msgq.visionipc import VisionIpcClient
    if self.vipc is None:
      self.vipc = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
    c = self.vipc
    if not c.is_connected() and not c.connect(False):
      return None
    buf = c.recv(200)
    if buf is None:
      buf = c.recv(200)  # a fresh connection can miss the first frame
    if buf is None:
      return None
    return bytes(buf.data), buf.width, buf.height, buf.stride, buf.uv_offset

  def _work(self):
    try:
      os.nice(10)
    except OSError:
      pass
    while True:
      kind, key, d, y, rpy, gps = self.q.get()
      try:
        fr = self._frame()
        if fr is None:
          continue
        data, w, h, stride, uv_off = fr
        box = lead_box(d, y, rpy, w, h)
        if box is None:
          continue
        if kind == "plate":
          x0, y0, x1, y1 = box
          box = (x0, (y0 + y1) // 2 & ~1, x1, y1)  # lower half of the car, where the plate is
          if x1 - x0 < PLATE_MIN_CROP_PX:
            continue
          img = jpeg(nv12_crop_rgb(data, stride, uv_off, box), 640)
          del data
          ans = _post("/plate", img, {})
          with self.lock:
            if self.track and self.track[0] == key:  # still the same car ahead
              self.plate = ans.get("plate") or None
        else:
          img = jpeg(nv12_crop_rgb(data, stride, uv_off, box), 384)
          del data
          if self.route is None:
            self.route = route_name()
          hdr = {"X-Drive": self.route}
          if gps:
            hdr["X-Where"] = f"{gps[0]:.5f},{gps[1]:.5f}"  # used on Forge only to name the road, never stored
          ans = _post("/classify", img, hdr)
          with self.lock:
            if self.track and self.track[0] == key:
              self.info = {k: ans.get(k) for k in ("kind", "kind_conf", "make", "model", "color")}
      except Exception:
        pass  # display only: on any error the app draws a generic car
