"""Tests for canopy/bus.py: edge-triggered state, batching, and never losing the drive to a dead link."""
from openpilot.sunnypilot.canopy import bus as canopy_bus
from openpilot.sunnypilot.canopy.bus import Bus


class Post:
  """Stands in for the HTTPS POST: records batches, optionally fails."""

  def __init__(self, fail=0):
    self.batches = []
    self.tokens = []
    self.urls = []
    self.fail = fail
    self.calls = 0

  def __call__(self, url, token, events, proxy=None):
    self.calls += 1
    self.tokens.append(token)
    self.urls.append((url, proxy))
    if self.fail and self.calls <= self.fail:
      raise OSError("no route to host")
    self.batches.append(events)


def make(tmp_path, post):
  (tmp_path / "tok").write_text("secret\n")
  return Bus(url="https://example/publish", token_path=str(tmp_path / "tok"), post=post, heartbeat_s=1e9, worker=False)


def test_an_envelope_carries_the_source_sequence_and_key(tmp_path):
  p = Post()
  b = make(tmp_path, p)
  b.publish("signal", {"state": "red"}, key="ahead")
  assert b.flush() == 1
  ev, = p.batches[0]
  assert ev["id"] == "comma:1" and ev["src"] == "comma" and ev["topic"] == "signal" and ev["key"] == "ahead"
  assert ev["data"] == {"state": "red"}
  assert p.tokens == ["secret"]


def test_an_unchanged_state_is_not_sent_again(tmp_path):
  p = Post()
  b = make(tmp_path, p)
  assert b.publish("signal", {"state": "red"}, key="ahead") is True
  assert b.publish("signal", {"state": "red"}, key="ahead") is False
  assert b.publish("signal", {"state": "green"}, key="ahead") is True
  assert b.flush() == 2


def test_a_batch_caps_at_batch_max(tmp_path):
  p = Post()
  b = make(tmp_path, p)
  for i in range(30):
    b.publish("cue", {"i": i})
  assert b.flush() == canopy_bus.BATCH_MAX
  assert b.flush() == 30 - canopy_bus.BATCH_MAX


def test_a_failed_post_keeps_the_events_for_the_next_try(tmp_path):
  p = Post(fail=1)
  b = make(tmp_path, p)
  b.publish("cue", {"i": 1})
  assert b.flush() == 0 and b.errors == 1 and len(p.batches) == 0
  assert b.flush() == 1 and p.batches[0][0]["data"] == {"i": 1}


def test_a_full_queue_drops_the_oldest_not_the_newest(tmp_path):
  p = Post()
  b = Bus(url="u", token_path=str(tmp_path / "tok"), post=p, queue_max=3, heartbeat_s=1e9, worker=False)
  for i in range(6):
    b.publish("cue", {"i": i})
  assert b.dropped == 3
  b.flush(10)
  assert [e["data"]["i"] for e in p.batches[0]] == [3, 4, 5]


def test_the_heartbeat_re_sends_the_latest_state_with_a_new_id(tmp_path):
  p = Post()
  b = make(tmp_path, p)
  b.publish("signal", {"state": "red"}, key="ahead")
  b.flush()
  assert b.heartbeat(now=1.0) == 1
  b.flush()
  first, second = p.batches[0][0], p.batches[1][0]
  assert second["data"] == first["data"] and second["id"] != first["id"]


def test_publishing_never_raises_when_the_token_file_is_missing(tmp_path):
  b = Bus(url="u", token_path=str(tmp_path / "nope"), post=Post(), heartbeat_s=1e9, worker=False)
  assert b.publish("cue", {"i": 1}) is True
  assert b.token() == ""


def test_the_tailnet_comes_first_and_the_public_route_catches_the_fall(tmp_path):
  p = Post(fail=1)
  b = Bus(token_path=str(tmp_path / "tok"), post=p, worker=False, heartbeat_s=1e9,
          targets=[("http://100.80.204.53:18865/publish", "127.0.0.1:1055"), ("https://example/publish", None)])
  b.publish("cue", {"i": 1})
  assert b.flush() == 1
  assert p.urls == [("http://100.80.204.53:18865/publish", "127.0.0.1:1055"), ("https://example/publish", None)]
  assert b.errors == 0 and b.sent == 1


def test_both_routes_down_keeps_the_batch(tmp_path):
  p = Post(fail=9)
  b = Bus(token_path=str(tmp_path / "tok"), post=p, worker=False, heartbeat_s=1e9,
          targets=[("http://a/publish", "127.0.0.1:1055"), ("https://b/publish", None)])
  b.publish("cue", {"i": 1})
  assert b.flush() == 0 and b.errors == 1 and b.sent == 0
