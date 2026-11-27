"""Canopy bus: our own events, from the comma to Forge, over whatever uplink is up.

One queue and one worker thread per process. Events are small JSON objects with a topic, POSTed in
batches to Forge (https://forge.mpvt.io/canopy/bus/publish) with the shared X-Canopy-Token header,
the same token and the same route leadkind already uses. forge_route pins Forge traffic to LTE off
the home Wi-Fi, so the bus rides the cellular link without knowing about it.

Publishing never blocks and never raises: publish() appends and returns. If Forge is unreachable
the events wait in the queue (the oldest are dropped past queue_max), the worker backs off, and
the drive goes on. Events carry an id ("<src>:<seq>"), so Forge can drop a duplicate after a
reconnect without caring about ordering.

Two kinds of traffic:
  state   publish(topic, data, key=...) is edge-triggered: an unchanged payload is not re-sent,
          but the worker re-sends the latest payload for each key every heartbeat_s, so a consumer
          that reconnects (or Forge that restarts) is never left drawing a stale read.
  events  publish(topic, data) with no key is sent once, in order, and dropped only when the queue
          is full or Forge stays unreachable long enough to fill it.

Nothing here is logged with coordinates: callers decide what goes on the wire.
"""
import json
import os
import queue
import threading
import time
import urllib.request

BUS_URL = os.environ.get("CANOPY_BUS_URL", "https://forge.mpvt.io/canopy/bus/publish")
# The comma's tailscale is userspace with no proxy listener, so it cannot dial a 100.x address by
# itself. When the device grows --outbound-http-proxy-listen (or a socks5 server), point these at it
# and the bus goes over the tailnet with no public surface at all; the public route stays as the
# fallback for a device that does not have the listener (a comma update can take it away).
TAILNET_URL = os.environ.get("CANOPY_BUS_TAILNET_URL", "")
PROXY = os.environ.get("CANOPY_BUS_PROXY", "127.0.0.1:1055")
TOKEN_PATH = "/data/canopy/forge_token"
QUEUE_MAX = 256
BATCH_MAX = 20
HEARTBEAT_S = 10.0
BACKOFF_S = (2.0, 5.0, 15.0, 60.0)
TIMEOUT_S = 8.0


def _post(url, token, events, proxy=None):
  body = json.dumps({"v": 1, "events": events}, separators=(",", ":")).encode()
  req = urllib.request.Request(url, data=body, method="POST",
                               headers={"Content-Type": "application/json", "X-Canopy-Token": token})
  handler = urllib.request.ProxyHandler({"http": "http://" + proxy}) if proxy and url.startswith("http://") \
    else urllib.request.ProxyHandler({})
  with urllib.request.build_opener(handler).open(req, timeout=TIMEOUT_S) as r:
    answer = json.load(r)
  if not isinstance(answer, dict) or not answer.get("ok"):
    raise RuntimeError("bus refused the batch")


def bus_targets():
  """Where to POST, in order: the tailnet proxy when it is configured, then the public route."""
  out = []
  if TAILNET_URL:
    out.append((TAILNET_URL, PROXY))
  out.append((BUS_URL, None))
  return out


class Bus:
  def __init__(self, src="comma", url=BUS_URL, token_path=TOKEN_PATH, post=None, clock=time.time,
               queue_max=QUEUE_MAX, heartbeat_s=HEARTBEAT_S, worker=True, targets=None):
    self.src = src
    self.url = url
    self._targets = targets if targets is not None else bus_targets()
    self.token_path = token_path
    self._post = post or _post
    self._clock = clock
    self.heartbeat_s = heartbeat_s
    self._worker = worker
    self._q: queue.Queue = queue.Queue(maxsize=queue_max)
    self._lock = threading.Lock()
    self._seq = 0
    self._latest = {}          # (topic, key) -> payload, the state a reconnect gets
    self._hb_at = 0.0
    self._started = False
    self.sent = 0
    self.dropped = 0
    self.errors = 0

  def token(self):
    try:
      with open(self.token_path) as f:
        return f.read().strip()
    except OSError:
      return ""

  def _envelope(self, topic, data, key=None):
    with self._lock:
      self._seq += 1
      ev = {"id": "%s:%d" % (self.src, self._seq), "src": self.src, "topic": topic,
            "t": round(self._clock(), 3), "data": data}
      if key is not None:
        ev["key"] = str(key)[:40]
      return ev

  def publish(self, topic, data, key=None):
    """Queue one event. With a key it is also the state a reconnect gets; unchanged payload, no send."""
    if key is not None:
      with self._lock:
        if self._latest.get((topic, key)) == data:
          return False
        self._latest[(topic, key)] = data
    self._push(self._envelope(topic, data, key))
    return True

  def _push(self, ev):
    try:
      self._q.put_nowait(ev)
    except queue.Full:
      try:
        self._q.get_nowait()
        self.dropped += 1
      except queue.Empty:
        pass
      try:
        self._q.put_nowait(ev)
      except queue.Full:
        self.dropped += 1
    self._start()

  def _take(self, n=BATCH_MAX):
    out = []
    while len(out) < n:
      try:
        out.append(self._q.get_nowait())
      except queue.Empty:
        break
    return out

  def _put_back(self, events):
    for ev in events:
      try:
        self._q.put_nowait(ev)
      except queue.Full:
        self.dropped += 1

  def flush(self, n=BATCH_MAX):
    """Send what is queued right now. Returns how many events went out (0 on any failure)."""
    events = self._take(n)
    if not events:
      return 0
    token = self.token()
    for url, proxy in self._targets:
      try:
        self._post(url, token, events, proxy)
      except Exception:
        continue
      self.sent += len(events)
      return len(events)
    self.errors += 1
    self._put_back(events)
    return 0

  def heartbeat(self, now=None):
    """Re-send every keyed state so a consumer that reconnects has the current read, not an old one."""
    now = self._clock() if now is None else now
    with self._lock:
      if now < self._hb_at:
        return 0
      self._hb_at = now + self.heartbeat_s
      latest = dict(self._latest)
    for (topic, _key), data in latest.items():
      self._push(self._envelope(topic, data))
    return len(latest)

  def _start(self):
    if self._started or not self._worker:
      return
    self._started = True
    threading.Thread(target=self._run, daemon=True, name="canopy-bus").start()

  def _run(self):
    step = 0
    while True:
      self.heartbeat()
      n = self.flush()
      if n:
        step = 0
        time.sleep(0.5)
      else:
        time.sleep(BACKOFF_S[min(step, len(BACKOFF_S) - 1)])
        step += 1


_bus = {"obj": None}


def bus(src="comma"):
  """The process's one bus, created on first use."""
  if _bus["obj"] is None:
    _bus["obj"] = Bus(src=src)
  return _bus["obj"]
