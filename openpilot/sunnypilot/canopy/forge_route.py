"""Canopy Forge route: send traffic to Forge over the comma's own LTE, home Wi-Fi first.

The comma reaches Forge through tailscale in userspace mode, which talks straight to Forge's
public IPv4, so a host route for that address over ppp0 puts all Forge traffic on LTE and
leaves everything else on whatever the default route is (the phone hotspot LAN, for instance).

Home Wi-Fi wins: while the comma sits on HOME_SSIDS the route is cleared and Forge traffic goes
over the home link. Off the home network, with ppp0 up, the route is installed. A probe runs
every PROBE_EVERY_S; PROBE_FAILS_TO_BACKOFF straight failures clear the route and hold off for
BACKOFF_S, and Forge traffic falls back to the default route until then.

Touch OFF_FLAG (/data/canopy/forge_route_off) to hold it off entirely. State lands in
STATE_PATH for whoever wants to look.
"""
import json
import os
import subprocess
import time

FORGE_IPS = ["173.249.26.106"]
HOME_SSIDS = {"Daly WiFi Lompoc"}
LTE_DEV = "ppp0"
INTERVAL_S = 10
PROBE_EVERY_S = 30
PROBE_FAILS_TO_BACKOFF = 3
BACKOFF_S = 300
OFF_FLAG = "/data/canopy/forge_route_off"
STATE_PATH = "/data/canopy/forge_route_state.json"

TIMEOUT_S = 5


def _run(cmd):
  return subprocess.run(cmd, capture_output=True, text=True, timeout=TIMEOUT_S)


def wifi_ssid():
  try:
    out = _run(["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"]).stdout
  except (OSError, subprocess.SubprocessError, ValueError):
    return None
  for line in (out or "").splitlines():
    if line.startswith("yes:"):
      return line[len("yes:"):]
  return None


def lte_up():
  try:
    out = _run(["ip", "-4", "-o", "addr", "show", "dev", LTE_DEV]).stdout
  except (OSError, subprocess.SubprocessError, ValueError):
    return False
  return " inet " in (out or "")


def route_present(ip):
  try:
    out = _run(["ip", "route", "show", f"{ip}/32"]).stdout
  except (OSError, subprocess.SubprocessError, ValueError):
    return False
  return f"dev {LTE_DEV}" in (out or "")


def set_route(ip):
  try:
    _run(["sudo", "-n", "ip", "route", "replace", f"{ip}/32", "dev", LTE_DEV])
    return True
  except (OSError, subprocess.SubprocessError, ValueError):
    return False


def clear_route(ip):
  if not route_present(ip):
    return True
  try:
    _run(["sudo", "-n", "ip", "route", "del", f"{ip}/32", "dev", LTE_DEV])
    return True
  except (OSError, subprocess.SubprocessError, ValueError):
    return False


def probe(ip):
  try:
    return _run(["ping", "-c", "1", "-W", "3", "-I", LTE_DEV, ip]).returncode == 0
  except (OSError, subprocess.SubprocessError, ValueError):
    return False


def decide(ssid, lte_up, off, backoff_active):
  if off or backoff_active or not lte_up:
    return "none"
  if ssid in HOME_SSIDS:
    return "none"
  return "lte"


class ForgeRoute:
  def __init__(self, wifi_ssid=wifi_ssid, lte_up=lte_up, route_present=route_present,
               set_route=set_route, clear_route=clear_route, probe=probe,
               state_path=STATE_PATH, off_flag=OFF_FLAG):
    self._wifi_ssid = wifi_ssid
    self._lte_up = lte_up
    self._route_present = route_present
    self._set_route = set_route
    self._clear_route = clear_route
    self._probe = probe
    self.state_path = state_path
    self.off_flag = off_flag
    self.mode = "none"
    self.probe_fails = 0
    self.backoff_until = 0.0
    self._last_probe = -1e9

  def _safe(self, fn, default, *args):
    try:
      return fn(*args)
    except Exception:
      return default

  def _write_state(self, ssid, lte, mode, backoff_until):
    state = {
      "mode": mode,
      "ssid": ssid,
      "lte_up": bool(lte),
      "backoff_until": max(0.0, float(backoff_until)),
      "probe_fails": self.probe_fails,
      "ts": round(time.time(), 2),
    }
    try:
      os.makedirs(os.path.dirname(self.state_path), exist_ok=True)
      tmp = self.state_path + ".tmp"
      with open(tmp, "w") as f:
        json.dump(state, f, separators=(",", ":"))
      os.replace(tmp, self.state_path)
    except OSError:
      pass

  def step(self, now=None):
    now = time.monotonic() if now is None else now
    ssid = self._safe(self._wifi_ssid, None)
    lte = self._safe(self._lte_up, False)
    off = False
    try:
      off = os.path.exists(self.off_flag)
    except OSError:
      off = False
    backoff_active = now < self.backoff_until
    mode = decide(ssid, lte, off, backoff_active)

    if mode == "lte" and now - self._last_probe >= PROBE_EVERY_S:
      self._last_probe = now
      results = [self._safe(self._probe, False, ip) for ip in FORGE_IPS]
      if results and all(results):
        self.probe_fails = 0
      else:
        self.probe_fails += 1
        if self.probe_fails >= PROBE_FAILS_TO_BACKOFF:
          self.probe_fails = 0
          self.backoff_until = now + BACKOFF_S
          mode = "none"

    for ip in FORGE_IPS:
      if mode == "lte":
        if not self._safe(self._route_present, False, ip):
          self._safe(self._set_route, False, ip)
      else:
        self._safe(self._clear_route, False, ip)

    self.mode = mode
    self._write_state(ssid, lte, mode, self.backoff_until)
    return mode


def main():
  route = ForgeRoute()
  while True:
    try:
      route.step(time.monotonic())
    except Exception:
      pass
    time.sleep(INTERVAL_S)


if __name__ == "__main__":
  main()
