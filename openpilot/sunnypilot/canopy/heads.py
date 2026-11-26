"""Small per-feature decision heads on the comma: active, staged, last_good.

copy of /opt/leaf-can/heads, keep in step

Pure python, stdlib only, lists (no numpy): kg_features, state_vec, embed and
predict_py are the comma-side copies of kg.py and model.py, so a head trained on
Forge scores the same here.

Layout under ROOT (/data/canopy/heads):

  active/base.json      the frozen encoder (W1, b1, W2, b2, sha)
  active/<feature>.json the live head for a feature
  staged/<feature>.json a candidate, trained off-car and pushed by Zia
  last_good/<feature>.json the head that was live before the current one
  bad.json              {feature: [version, ...]} never promoted again

A head carries feature, base_sha, version (first 8 hex of the sha of w, b and
base_sha) and head_id "<feature>@<version>". Anything that does not check out
against the loaded base is ignored and named in HeadSet.errors.
"""

import hashlib
import json
import math
import os
import re

ROOT = "/data/canopy/heads"
ACTIVE = "active"
STAGED = "staged"
LAST_GOOD = "last_good"
BASE_NAME = "base.json"
BAD_NAME = "bad.json"
TIERS_NAME = "tiers.json"

RADIUS_M = 150.0
HEADING_TOL_DEG = 60.0
EARTH_R_M = 6371000.0
H2 = 8

# floors for promotion: a control-tier head can move the wheel or the pedals,
# so it needs more drives behind it than a cue or an assist head.
N_MIN = 10
N_MIN_CONTROL = 20

# fallback tier for features the gates file does not name (ROOT/tiers.json and
# stats["tier"] both win over this set).
TIER_CONTROL = {
  "auto_resume_green",
  "auto_resume_lead",
  "az511_move_over",
  "blinker_intent",
  "corner_handback",
  "lane_change_nudge",
  "voice_merge",
}

KG_KEYS = [
  "kg_takeovers",
  "kg_near",
  "kg_slow",
  "kg_hazard",
  "kg_drives_seen",
  "kg_takeover_spot",
  "kg_slow_spot",
  "kg_signal",
  "kg_lanes_in",
  "kg_left_lanes",
  "kg_right_lanes",
  "kg_speed_limit",
]

STATE_KEYS = [
  "mph",
  "cr_on",
  "set_delta",
  "gas",
  "brake",
  "lead",
  "limit_delta",
  "dmph",
  "dhead",
]

CTX_KEYS = [
  "ctx_light_night",
  "ctx_road_freeway",
  "ctx_band_lt25",
  "ctx_band_25to50",
  "ctx_band_50plus",
]

IN_KEYS = STATE_KEYS + CTX_KEYS

_FEATURE_RE = re.compile(r"^[A-Za-z0-9_.-]{1,64}$")


# ---------------------------------------------------------------------------
# kg.py, copied
# ---------------------------------------------------------------------------

def haversine_m(lat1, lon1, lat2, lon2):
  p1 = math.radians(lat1)
  p2 = math.radians(lat2)
  dp = p2 - p1
  dl = math.radians(lon2 - lon1)
  a = math.sin(dp * 0.5) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl * 0.5) ** 2
  return 2.0 * EARTH_R_M * math.asin(min(1.0, math.sqrt(a)))


def heading_diff_deg(a, b):
  d = (float(a) - float(b)) % 360.0
  return d if d <= 180.0 else 360.0 - d


def _num(v):
  return 0.0 if v is None else float(v)


def kg_features(items, lat, lon, heading, exclude_drive=None):
  """Binned proximity features within RADIUS_M of (lat, lon).

  An item with a heading only counts when its heading is within
  HEADING_TOL_DEG of the query heading; the check is skipped when either
  side has no heading.
  """
  out = dict.fromkeys(KG_KEYS, 0.0)
  if lat is None or lon is None:
    return out

  c_take = c_near = c_slow = c_hazard = 0
  drives = set()
  spot_take = 0.0
  spot_slow = 0.0
  signal = 0
  best_d = None
  best_ap = None

  for it in items:
    d = it.get("drive")
    if exclude_drive is not None:
      if d == exclude_drive:
        continue
      lesson_drives = it.get("drives")
      if lesson_drives and exclude_drive in lesson_drives:
        continue
    ilat = it.get("lat")
    ilon = it.get("lon")
    if ilat is None or ilon is None:
      continue
    dist = haversine_m(lat, lon, ilat, ilon)
    if dist > RADIUS_M:
      continue
    ih = it.get("h")
    if ih is not None and heading is not None and heading_diff_deg(ih, heading) > HEADING_TOL_DEG:
      continue

    k = it.get("k")
    if k == "takeover" or k == "pedal_takeover_gas" or k == "pedal_takeover_brake":
      c_take += 1
    elif k == "near_steer" or k == "near_saturated":
      c_near += 1
    elif k == "firm_slow" or k == "coast_stop" or k == "brake_lights":
      c_slow += 1
    elif k == "lane_drop" or k == "work_zone":
      c_hazard += 1
    elif k == "lesson_takeover_spot":
      v = _num(it["x"].get("confidence"))
      if v > spot_take:
        spot_take = v
    elif k == "lesson_slow_spot":
      v = _num(it["x"].get("confidence"))
      if v > spot_slow:
        spot_slow = v
    elif k == "approach":
      if it["x"].get("signal"):
        signal = 1
      if best_d is None or dist < best_d:
        best_d = dist
        best_ap = it["x"]

    if d is not None:
      drives.add(d)

  out["kg_takeovers"] = math.log1p(c_take)
  out["kg_near"] = math.log1p(c_near)
  out["kg_slow"] = math.log1p(c_slow)
  out["kg_hazard"] = math.log1p(c_hazard)
  out["kg_drives_seen"] = math.log1p(len(drives))
  out["kg_takeover_spot"] = spot_take
  out["kg_slow_spot"] = spot_slow
  out["kg_signal"] = float(signal)
  if best_ap is not None:
    out["kg_lanes_in"] = _num(best_ap.get("lanes_in"))
    out["kg_left_lanes"] = _num(best_ap.get("left_turn_lanes"))
    out["kg_right_lanes"] = _num(best_ap.get("right_turn_lanes"))
    out["kg_speed_limit"] = _num(best_ap.get("speed_limit_mph"))
  return out


class KgCache:
  """kg_features for the last position, recomputed only after the car moved 25 m
  or turned 20 deg. 150 m of look-around does not change for less than that, and
  walking the graph at 10 Hz for the same meters is wasted."""
  MOVE_M = 25.0
  TURN_DEG = 20.0

  def __init__(self, items):
    self.items = list(items or [])
    self.last = None   # (lat, lon, heading, exclude_drive)
    self._out = None

  def features(self, lat, lon, heading, exclude_drive=None):
    """Cached kg_features; the returned dict is shared, treat it as read only."""
    if lat is None or lon is None:
      return dict.fromkeys(KG_KEYS, 0.0)
    if self.last is not None and self.last[3] == exclude_drive:
      plat, plon, ph, _ = self.last
      if haversine_m(lat, lon, plat, plon) < self.MOVE_M:
        if heading is None or ph is None or heading_diff_deg(heading, ph) < self.TURN_DEG:
          return self._out
    out = kg_features(self.items, lat, lon, heading, exclude_drive)
    self.last = (lat, lon, heading, exclude_drive)
    self._out = out
    return out


# ---------------------------------------------------------------------------
# model.py, copied
# ---------------------------------------------------------------------------

def parse_ctx(ctx):
  """Split "k=v|k=v" or "k:v" into a dict; unknown pieces are skipped."""
  out = {}
  if not ctx:
    return out
  for part in str(ctx).split("|"):
    part = part.strip()
    if not part:
      continue
    if "=" in part:
      k, v = part.split("=", 1)
    elif ":" in part:
      k, v = part.split(":", 1)
    else:
      continue
    out[k.strip()] = v.strip()
  return out


def _band(v):
  """Map a speed descriptor (or numeric mph) to lt25 / 25to50 / 50plus."""
  if v is None:
    return None
  s = str(v).strip().lower()
  if s in ("lt25", "<25", "under25"):
    return "lt25"
  if s in ("50plus", "50+", ">50", "gt50"):
    return "50plus"
  if s in ("25to50", "25-50", "mid"):
    return "25to50"
  if s in ("25-35", "35-45"):
    return "25to50"
  try:
    x = float(s)
  except (TypeError, ValueError):
    if "-" in s:
      try:
        a, b = s.split("-", 1)
        x = (float(a) + float(b)) / 2.0
      except (TypeError, ValueError):
        return None
    else:
      return None
  if x < 25.0:
    return "lt25"
  if x < 50.0:
    return "25to50"
  return "50plus"


def state_vec(state, ctx):
  """Fixed-scaling numeric state plus parsed ctx one-hots (14 floats)."""
  state = state or {}
  mph = _num(state.get("mph"))
  vec = [
    mph / 80.0,
    1.0 if state.get("cr_on") else 0.0,
    _num(state.get("set_delta")) / 30.0,
    1.0 if state.get("gas") else 0.0,
    1.0 if state.get("brake") else 0.0,
    1.0 if state.get("lead") else 0.0,
    _num(state.get("limit_delta")) / 30.0,
    _num(state.get("dmph")) / 30.0,
    _num(state.get("dhead")) / 90.0,
  ]
  c = parse_ctx(ctx)
  light_night = 1.0 if str(c.get("light", "")).strip().lower() == "night" else 0.0
  road_freeway = 1.0 if str(c.get("road", "")).strip().lower() == "freeway" else 0.0
  band = _band(c.get("speed"))
  if band is None:
    band = "lt25" if mph < 25.0 else ("25to50" if mph < 50.0 else "50plus")
  vec.extend([
    light_night,
    road_freeway,
    1.0 if band == "lt25" else 0.0,
    1.0 if band == "25to50" else 0.0,
    1.0 if band == "50plus" else 0.0,
  ])
  return vec


def _sigmoid(z):
  if z >= 0.0:
    return 1.0 / (1.0 + math.exp(-z))
  e = math.exp(z)
  return e / (1.0 + e)


def embed(base, x):
  """The frozen 8-d embedding for one state vector (lists only)."""
  W1 = base["W1"]
  b1 = base["b1"]
  W2 = base["W2"]
  b2 = base["b2"]
  h1 = [math.tanh(sum(x[i] * W1[i][j] for i in range(len(x))) + b1[j]) for j in range(len(b1))]
  h2 = [math.tanh(sum(h1[i] * W2[i][j] for i in range(len(h1))) + b2[j]) for j in range(len(b2))]
  return h2


def _kg_list(kg_vec):
  if isinstance(kg_vec, dict):
    return [float(kg_vec.get(k, 0.0)) for k in KG_KEYS]
  return [float(v) for v in (kg_vec or [])]


def _head_kind(head):
  kind = head.get("inputs")
  if kind in ("emb", "kg", "emb+kg"):
    return kind
  L = len(head.get("w", []))
  if L == H2:
    return "emb"
  if L == len(KG_KEYS):
    return "kg"
  return "emb+kg"


def predict_py(base, head, state_vec, kg_vec):
  """Pure-python mirror of model.predict (lists only, no numpy)."""
  kind = _head_kind(head)
  if kind == "kg":
    x = _kg_list(kg_vec)
  else:
    x = embed(base, state_vec)
    if kind == "emb+kg":
      x = x + _kg_list(kg_vec)
  z = sum(x[i] * head["w"][i] for i in range(len(x))) + head["b"]
  return _sigmoid(z)


# ---------------------------------------------------------------------------
# head ids
# ---------------------------------------------------------------------------

def head_version(w, b, base_sha):
  """The version a head's w and b must agree with, first 8 hex of the sha."""
  blob = json.dumps({"w": w, "b": b, "base_sha": base_sha}, sort_keys=True, separators=(",", ":"))
  return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:8]


def head_id(head):
  """<feature>@<version>, or None for something that is not a head."""
  if not isinstance(head, dict):
    return None
  f, v = head.get("feature"), head.get("version")
  if not isinstance(f, str) or not isinstance(v, str):
    return None
  return f"{f}@{v}"


def _numeric(v):
  return isinstance(v, (int, float)) and not isinstance(v, bool)


def head_error(head, base_sha, bad_versions=()):
  """Why this head cannot be used, or None when it checks out."""
  if not isinstance(head, dict):
    return "not an object"
  f = head.get("feature")
  if not isinstance(f, str) or not _FEATURE_RE.match(f):
    return "feature"
  w = head.get("w")
  if not isinstance(w, list) or not w or not all(_numeric(v) for v in w):
    return "w"
  b = head.get("b")
  if not _numeric(b):
    return "b"
  if not isinstance(head.get("base_sha"), str) or not head["base_sha"]:
    return "base_sha"
  if base_sha is None:
    return "no base"
  if head["base_sha"] != base_sha:
    return "base_sha"
  v = head.get("version")
  if not isinstance(v, str) or v != head_version(w, b, head["base_sha"]):
    return "version"
  if v in bad_versions:
    return "bad"
  return None


# ---------------------------------------------------------------------------
# files
# ---------------------------------------------------------------------------

def _paths(root):
  return (os.path.join(root, ACTIVE), os.path.join(root, STAGED), os.path.join(root, LAST_GOOD))


def _write_json(path, obj):
  """Write through a temp file and rename, so a reader never sees half a head."""
  os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
  tmp = path + ".tmp"
  with open(tmp, "w", encoding="utf-8") as f:
    json.dump(obj, f, sort_keys=True, separators=(",", ":"))
  os.replace(tmp, path)


def _read_json(path):
  with open(path, encoding="utf-8") as f:
    return json.load(f)


def read_bad(root=None):
  """{feature: [version, ...]} from ROOT/bad.json; {} when missing or broken."""
  try:
    d = _read_json(os.path.join(root or ROOT, BAD_NAME))
  except (OSError, ValueError):
    return {}
  if not isinstance(d, dict):
    return {}
  out = {}
  for f, vs in d.items():
    if isinstance(f, str) and isinstance(vs, list):
      out[f] = [v for v in vs if isinstance(v, str)]
  return out


def read_tiers(root=None):
  """{feature: tier} from ROOT/tiers.json; {} when missing or broken."""
  try:
    d = _read_json(os.path.join(root or ROOT, TIERS_NAME))
  except (OSError, ValueError):
    return {}
  if not isinstance(d, dict):
    return {}
  return {f: v for f, v in d.items() if isinstance(f, str) and isinstance(v, str)}


def stage_head(head, root=None):
  """Check one head and put it in staged/. Returns its head_id, raises ValueError.

  Staging only: nothing here touches active/.
  """
  root = root or ROOT
  if not isinstance(head, dict):
    raise ValueError("not an object")
  f = head.get("feature")
  if not isinstance(f, str) or not _FEATURE_RE.match(f):
    raise ValueError("feature")
  for k in ("base_sha", "version", "w", "b"):
    if head.get(k) is None:
      raise ValueError(k)
  if not isinstance(head["w"], list) or not head["w"] or not all(_numeric(v) for v in head["w"]):
    raise ValueError("w")
  if not _numeric(head["b"]):
    raise ValueError("b")
  if not isinstance(head["base_sha"], str) or not isinstance(head["version"], str):
    raise ValueError("base_sha")
  if head["version"] != head_version(head["w"], head["b"], head["base_sha"]):
    raise ValueError("version")
  if head["version"] in read_bad(root).get(f, []):
    raise ValueError("bad")
  _write_json(os.path.join(root, STAGED, f + ".json"), head)
  return head_id(head)


# ---------------------------------------------------------------------------
# boundary
# ---------------------------------------------------------------------------

def maneuver_boundary(blinker_on, lane_change_active, lane_turn_active, steer_deg,
                      secs_since_feature_decision, gear) -> bool:
  """Parked is always a boundary. Otherwise: nothing the driver or the planner is
  doing right now, hands near straight, and 3 s since this feature last acted."""
  if gear == 1:
    return True
  if blinker_on or lane_change_active or lane_turn_active:
    return False
  try:
    if abs(float(steer_deg)) >= 15.0:
      return False
    return float(secs_since_feature_decision) >= 3.0
  except (TypeError, ValueError):
    return False


# ---------------------------------------------------------------------------
# the set
# ---------------------------------------------------------------------------

class HeadSet:
  """The heads on disk, what is live and what is shadowing it."""

  def __init__(self, root=None, items=None):
    self.root = root or ROOT
    self.base = None
    self.base_id = None
    self.live = {}
    self.shadow = {}
    self.last_good = {}
    self.bad = {}
    self.tiers = {}
    self.errors = []
    self.pending_rollback = set()
    self.kg = KgCache(items or [])
    self.load()

  # ---- load ----

  def load(self):
    """Read base, active/, staged/ and last_good/; bad files are named in errors."""
    self.errors = []
    self.base = self.base_id = None
    self.bad = read_bad(self.root)
    self.tiers = read_tiers(self.root)
    active, staged, last_good = _paths(self.root)

    try:
      base = _read_json(os.path.join(active, BASE_NAME))
      if isinstance(base, dict) and isinstance(base.get("sha"), str) and base.get("W1"):
        self.base, self.base_id = base, base.get("base_id") or base["sha"][:8]
      else:
        self.errors.append(f"{ACTIVE}/{BASE_NAME}: base")
    except (OSError, ValueError) as e:
      self.errors.append(f"{ACTIVE}/{BASE_NAME}: {str(e)[:60]}")

    base_sha = self.base["sha"] if self.base is not None else None
    self.live = self._scan(active, base_sha, skip_base=True)
    self.shadow = self._scan(staged, base_sha)
    self.last_good = self._scan(last_good, base_sha)
    return self

  def refresh(self):
    """Re-read the tree; a staged head becomes the shadow of its feature."""
    return self.load()

  def _scan(self, d, base_sha, skip_base=False):
    out = {}
    try:
      names = sorted(os.listdir(d))
    except OSError:
      return out
    for name in names:
      if not name.endswith(".json") or (skip_base and name == BASE_NAME):
        continue
      path = os.path.join(d, name)
      try:
        head = _read_json(path)
      except (OSError, ValueError) as e:
        self.errors.append(f"{os.path.basename(d)}/{name}: {str(e)[:60]}")
        continue
      err = head_error(head, base_sha, self.bad.get(head.get("feature"), ()) if isinstance(head, dict) else ())
      if err is not None:
        self.errors.append(f"{os.path.basename(d)}/{name}: {err}")
        continue
      out[head["feature"]] = head
    return out

  # ---- ids ----

  def live_ids(self):
    return {f: head_id(h) for f, h in self.live.items()}

  def shadow_ids(self):
    return {f: head_id(h) for f, h in self.shadow.items()}

  def last_good_ids(self):
    return {f: head_id(h) for f, h in self.last_good.items()}

  def staged_ids(self):
    try:
      names = sorted(os.listdir(os.path.join(self.root, STAGED)))
    except OSError:
      return []
    out = []
    for name in names:
      if name.endswith(".json"):
        try:
          hid = head_id(_read_json(os.path.join(self.root, STAGED, name)))
        except (OSError, ValueError):
          continue
        if hid is not None:
          out.append(hid)
    return out

  # ---- score ----

  def score(self, feature, state, ctx, lat, lon, heading):
    """The live head's confidence for this state, plus every shadow head's.

    {"head": live head_id or None, "conf": float or None, "base": base_id,
     "shadow": {shadow head_id: conf}}
    """
    out = {"head": None, "conf": None, "base": self.base_id, "shadow": {}}
    if self.base is None:
      return out
    try:
      x = state_vec(state, ctx)
      kg = self.kg.features(lat, lon, heading)
    except (TypeError, ValueError, KeyError):
      return out
    live = self.live.get(feature)
    if live is not None:
      try:
        out["head"], out["conf"] = head_id(live), predict_py(self.base, live, x, kg)
      except (KeyError, IndexError, TypeError, ValueError):
        pass
    shadow = self.shadow.get(feature)
    if shadow is not None:
      try:
        out["shadow"][head_id(shadow)] = predict_py(self.base, shadow, x, kg)
      except (KeyError, IndexError, TypeError, ValueError):
        pass
    return out

  # ---- promote ----

  def floor(self, feature, stats=None):
    """Drives a candidate needs: 20 for a control-tier feature, 10 otherwise."""
    tier = (stats or {}).get("tier") or self.tiers.get(feature)
    if tier is None and feature in TIER_CONTROL:
      tier = "control"
    return N_MIN_CONTROL if tier == "control" else N_MIN

  def _evidence(self, feature, stats):
    if not isinstance(stats, dict):
      return False
    try:
      n = int(stats["n"])
      ll_new = float(stats["ll_new"])
      ll_live = float(stats["ll_live"])
      takeovers = int(stats["takeovers_on_disagree"])
    except (KeyError, TypeError, ValueError):
      return False
    return n >= self.floor(feature, stats) and ll_new <= ll_live and takeovers == 0

  def promote(self, feature, boundary_ok, stats):
    """Make the shadow head live when it has earned it. Returns the new head_id or None."""
    if not boundary_ok:
      return None
    sh = self.shadow.get(feature)
    if sh is None or not self._evidence(feature, stats):
      return None
    active, staged, last_good = _paths(self.root)
    try:
      live = self.live.get(feature)
      if live is not None:
        _write_json(os.path.join(last_good, feature + ".json"), live)
      _write_json(os.path.join(active, feature + ".json"), sh)
      p = os.path.join(staged, feature + ".json")
      if os.path.exists(p):
        os.remove(p)
    except OSError:
      return None
    if self.live.get(feature) is not None:
      self.last_good[feature] = self.live[feature]
    self.live[feature] = sh
    self.shadow.pop(feature, None)
    return head_id(sh)

  # ---- rollback ----

  def rollback(self, feature, boundary_ok):
    """After a takeover on a decision stamped with a live head: put the previous
    head back and never promote the bad one again. Returns the restored id or None.

    Off a boundary the rollback is queued (pending_rollback) and done on the next
    boundary call.
    """
    if not boundary_ok:
      self.pending_rollback.add(feature)
      return None
    self.pending_rollback.discard(feature)
    restored = self._restore(feature)
    for f in sorted(self.pending_rollback):
      self._restore(f)
    self.pending_rollback.clear()
    return restored

  def _restore(self, feature):
    lg = self.last_good.get(feature)
    if lg is None:
      return None
    active, _staged, _lg = _paths(self.root)
    try:
      _write_json(os.path.join(active, feature + ".json"), lg)
    except OSError:
      return None
    bad_live = self.live.get(feature)
    if bad_live is not None and head_id(bad_live) != head_id(lg):
      self._mark_bad(feature, bad_live)
    self.live[feature] = lg
    if self.shadow.get(feature) is not None and self.shadow[feature].get("version") in self.bad.get(feature, []):
      self.shadow.pop(feature, None)
    return head_id(lg)

  def _mark_bad(self, feature, head):
    """Record the bad version in bad.json so it is never re-staged or promoted."""
    v = head.get("version")
    if not isinstance(v, str):
      return
    vers = self.bad.setdefault(feature, [])
    if v in vers:
      return
    vers.append(v)
    try:
      _write_json(os.path.join(self.root, BAD_NAME), self.bad)
    except OSError:
      pass
