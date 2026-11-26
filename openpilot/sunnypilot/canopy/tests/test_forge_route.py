"""Tests for canopy/forge_route.py. No network, no subprocess."""
import os

from openpilot.sunnypilot.canopy import forge_route
from openpilot.sunnypilot.canopy.forge_route import ForgeRoute, decide

HOTSPOT = "Marks S24 Ultra"


class FakeNet:
  """Stands in for the nmcli/ip/ping helpers, all in memory."""

  def __init__(self):
    self.ssid = HOTSPOT
    self.lte = True
    self.routes = set()
    self.set_calls = []
    self.clear_calls = []
    self.probe_ok = True
    self.probe_calls = 0

  def wifi_ssid(self):
    return self.ssid

  def lte_up(self):
    return self.lte

  def route_present(self, ip):
    return ip in self.routes

  def set_route(self, ip):
    self.set_calls.append(ip)
    self.routes.add(ip)

  def clear_route(self, ip):
    if ip in self.routes:
      self.clear_calls.append(ip)
      self.routes.discard(ip)

  def probe(self, ip):
    self.probe_calls += 1
    return self.probe_ok


def _make(tmp_path, net):
  return ForgeRoute(wifi_ssid=net.wifi_ssid, lte_up=net.lte_up, route_present=net.route_present,
                    set_route=net.set_route, clear_route=net.clear_route, probe=net.probe,
                    state_path=str(tmp_path / "state.json"), off_flag=str(tmp_path / "off"))


def test_decide_table():
  assert decide("Daly WiFi Lompoc", True, False, False) == "none"
  assert decide(HOTSPOT, True, False, False) == "lte"
  assert decide(None, True, False, False) == "lte"
  assert decide(HOTSPOT, False, False, False) == "none"
  assert decide(HOTSPOT, True, True, False) == "none"
  assert decide(HOTSPOT, True, False, True) == "none"


def test_sets_route_once_keyed_on_presence(tmp_path):
  net = FakeNet()
  route = _make(tmp_path, net)

  assert route.step(now=0.0) == "lte"
  assert net.set_calls == forge_route.FORGE_IPS
  assert net.routes == set(forge_route.FORGE_IPS)

  assert route.step(now=10.0) == "lte"
  assert net.set_calls == forge_route.FORGE_IPS  # no second set


def test_clears_on_home_ssid(tmp_path):
  net = FakeNet()
  route = _make(tmp_path, net)

  route.step(now=0.0)
  assert net.routes

  net.ssid = "Daly WiFi Lompoc"
  assert route.step(now=10.0) == "none"
  assert net.clear_calls == forge_route.FORGE_IPS
  assert net.routes == set()


def test_three_failed_probes_backoff_then_recover(tmp_path):
  net = FakeNet()
  net.probe_ok = False
  route = _make(tmp_path, net)

  route.step(now=0.0)
  assert route.probe_fails == 1
  assert net.routes == set(forge_route.FORGE_IPS)

  route.step(now=30.0)
  assert route.probe_fails == 2

  route.step(now=60.0)
  assert route.probe_fails == 0
  assert route.backoff_until >= 60.0 + forge_route.BACKOFF_S
  assert net.routes == set()
  assert net.clear_calls == forge_route.FORGE_IPS

  # inside the backoff window it stays off
  net.probe_ok = True
  assert route.step(now=90.0) == "none"
  assert net.routes == set()

  # after backoff it comes back
  assert route.step(now=60.0 + forge_route.BACKOFF_S + 1.0) == "lte"
  assert net.routes == set(forge_route.FORGE_IPS)


def test_raising_helpers_never_break_step(tmp_path):
  def boom(*a, **k):
    raise RuntimeError("boom")

  route = ForgeRoute(wifi_ssid=boom, lte_up=boom, route_present=boom, set_route=boom,
                     clear_route=boom, probe=boom,
                     state_path=str(tmp_path / "state.json"), off_flag=str(tmp_path / "off"))
  assert route.step(now=0.0) == "none"


def test_off_flag_clears_route(tmp_path):
  net = FakeNet()
  route = _make(tmp_path, net)

  route.step(now=0.0)
  assert net.routes

  with open(route.off_flag, "w") as f:
    f.write("off\n")
  assert route.step(now=10.0) == "none"
  assert net.routes == set()


def test_state_file_written(tmp_path):
  import json
  net = FakeNet()
  route = _make(tmp_path, net)
  route.step(now=0.0)
  with open(route.state_path) as f:
    state = json.load(f)
  assert state["mode"] == "lte"
  assert state["ssid"] == HOTSPOT
  assert state["lte_up"] is True
  assert os.path.exists(route.state_path)
