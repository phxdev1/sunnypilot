"""Generic confidence-gate reader on the comma.

One snapshot at drive start (onroad): load gates.json + gates_demoted.json, fetch current
weather from Forge once in a thread.  Flips only take effect at the next onroad start.

Fail safe: any parse error or missing file -> everything shadow.
"""
import json
import os
import threading
import time
import urllib.request
import math

GATES_PATH = "/data/canopy/gates.json"
DEMOTED_PATH = "/data/canopy/gates_demoted.json"

_FORGE_URL_PATH = "/data/canopy/forge_url"
_FORGE_TOKEN_PATH = "/data/canopy/forge_token"


def _read_token():
  try:
    with open(_FORGE_TOKEN_PATH) as f:
      return f.read().strip()
  except OSError:
    return ""


def _forge_url():
  try:
    with open(_FORGE_URL_PATH) as f:
      u = f.read().strip()
  except OSError:
    u = ""
  return u or ""


def _sun_elevation(lat, lon, unix):
  """Rough local sun elevation (degrees).  Good enough for day / low / night.
  > 6 day, -6..6 low, < -6 night."""
  try:
    td = time.gmtime(unix)
    lat_f = float(lat)
    hour = td.tm_hour + td.tm_min / 60.0 + td.tm_sec / 3600.0
  except (TypeError, ValueError):
    return None
  td_jd = td.tm_yday + 1
  decl = 23.44 * math.sin(math.radians(360.0 / 365.0 * (td_jd - 81)))
  ha_deg = 15.0 * (hour - 12.0)
  lat_r = math.radians(lat_f)
  decl_r = math.radians(decl)
  ha_r = math.radians(ha_deg)
  try:
    el = math.asin(math.sin(lat_r) * math.sin(decl_r) +
                   math.cos(lat_r) * math.cos(decl_r) * math.cos(ha_r))
  except ValueError:
    return None
  return math.degrees(el)


def _speed_band(mph):
  if mph is None:
    return None
  try:
    v = float(mph)
  except (TypeError, ValueError):
    return None
  if v < 25:
    return "0-25"
  if v < 35:
    return "25-35"
  if v < 45:
    return "35-45"
  if v < 50:
    return "45-50"
  if v < 65:
    return "50-65"
  return "65+"


def _fetch_weather(url):
  if not url:
    return None
  try:
    req = urllib.request.Request(url + "/api/conditions", method="GET")
    with urllib.request.urlopen(req, timeout=2.0) as r:
      data = json.load(r)
  except Exception:
    return None
  if not isinstance(data, dict):
    return None
  w = data.get("weather")
  if w is None:
    return None
  s = str(w).strip().lower()
  if s in ("dry", "wet"):
    return s
  return None


class _GatesData:
  """Parsed snapshot held in memory for one drive."""

  def __init__(self, generated, features, combos, names, weather):
    self.generated = generated
    self.features = features or {}
    self.combos = combos or {}
    self.names = names or {}
    self.weather = weather  # "dry" | "wet" | None (unknown)


def _parse_cell_keys(ctx_or_blank):
  if ctx_or_blank == "_":
    return set()
  out = set()
  for piece in ctx_or_blank.split("|"):
    if "=" in piece:
      k, _, _ = piece.partition("=")
      out.add(k)
  return out


def _norm_cond(cond):
  """Copy cond with "speed" and "speed_band" made interchangeable.

  gates.json (built on Forge) names speed cells "speed_band=35-45" while older
  callers pass the band under "speed".  Fill in whichever key is missing so a
  single comparison treats them as one dimension."""
  if not isinstance(cond, dict):
    cond = {}
  out = dict(cond)
  for k, other in (("speed", "speed_band"), ("speed_band", "speed")):
    v = out.get(k)
    if v is not None and out.get(other) is None:
      out[other] = v
  return out


def _cell_matches(cell_ctx, cond):
  cond = _norm_cond(cond)
  if cell_ctx == "_":
    return True
  for piece in cell_ctx.split("|"):
    if "=" not in piece:
      continue
    k, _, v = piece.partition("=")
    v = v.strip()
    if k not in cond or cond[k] is None:
      return False
    if cond[k] != v:
      return False
  return True


def _has_speed_cells(features_for_fid):
  for ctx in features_for_fid:
    if not isinstance(ctx, str):
      continue
    if ctx == "_":
      continue
    for piece in ctx.split("|"):
      if "=" in piece:
        k, _, _ = piece.partition("=")
        if k in ("speed", "speed_band"):
          return True
  return False


def _demoted_key(fid, cell):
  return f"{fid}\0{cell}"


def _demoted_to_pairs(demoted):
  out = {}
  if not isinstance(demoted, dict):
    return out
  for k, v in demoted.items():
    if isinstance(k, str) and isinstance(v, (int, float)):
      try:
        fk, ck = k.split("\0", 1)
      except ValueError:
        continue
      out[(fk, ck)] = float(v)
  return out


def _pairs_to_demoted(pairs):
  out = {}
  for (fid, cell), ts in pairs.items():
    out[_demoted_key(fid, cell)] = ts
  return out


class Gates:
  def __init__(self):
    self._lock = threading.Lock()
    self._data = None
    self._demoted = {}  # (fid, ctx) -> demotion ts
    self._ran_snapshot = False

  # ---- snapshot (once per drive) ----

  def snapshot(self, lat=None, lon=None, unix=None):
    if self._ran_snapshot:
      return
    with self._lock:
      self._ran_snapshot = True
    gen = 0.0
    features = {}
    combos = {}
    names = {}
    weather = None
    try:
      with open(GATES_PATH) as f:
        raw = json.load(f)
      if not isinstance(raw, dict):
        raw = {}
      gen = float(raw.get("generated") or 0.0)
      feats = raw.get("features")
      if isinstance(feats, dict):
        features = feats
      combos_raw = raw.get("combos")
      if isinstance(combos_raw, dict):
        combos = combos_raw
      nm = raw.get("names")
      if isinstance(nm, dict):
        names = nm
    except (OSError, ValueError, TypeError):
      pass
    demoted = {}
    demoted_gen = 0.0
    try:
      with open(DEMOTED_PATH) as f:
        d_raw = json.load(f)
      if isinstance(d_raw, dict):
        demoted_raw = d_raw.get("demotions")
        if isinstance(demoted_raw, dict):
          demoted = _demoted_to_pairs(demoted_raw)
        demoted_gen = float(d_raw.get("generated") or 0.0)
    except (OSError, ValueError, TypeError):
      pass
    furl = _forge_url()
    if furl and lat is not None and lon is not None and unix is not None:
      def _th():
        nonlocal weather
        weather = _fetch_weather(furl)
      t = threading.Thread(target=_th, daemon=True)
      t.start()
      try:
        t.join(timeout=2.5)
      except Exception:
        pass
    with self._lock:
      self._data = _GatesData(gen, features, combos, names, weather)
      self._demoted = demoted

  # ---- conditions ----

  def conditions(self, lat, lon, unix, mph, speed_limit_mph):
    light = None
    if lat is not None and lon is not None and unix is not None:
      el = _sun_elevation(lat, lon, unix)
      if el is not None:
        if el > 6.0:
          light = "day"
        elif el >= -6.0:
          light = "low"
        else:
          light = "night"
    road = None
    if speed_limit_mph is not None:
      try:
        sl = float(speed_limit_mph)
      except (TypeError, ValueError):
        sl = None
      if sl is not None:
        road = "freeway" if sl >= 55 else "surface"
    weather = None
    with self._lock:
      if self._data is not None:
        weather = self._data.weather
    speed = _speed_band(mph)
    return {"light": light, "road": road, "weather": weather, "speed": speed, "speed_band": speed}

  # ---- is_live ----

  def is_live(self, fid, cond, conf=1.0, acting=()):
    cond = _norm_cond(cond)
    if self._data is None:
      return False
    with self._lock:
      data = self._data
    feats = data.features
    feat = feats.get(fid)
    if not isinstance(feat, dict):
      return False
    names = data.names

    def _entry_cstar(ctx):
      if ctx == "_":
        return 1.0
      e = feat.get(ctx)
      if not isinstance(e, dict):
        return 1.0
      c = e.get("cstar")
      try:
        return float(c) if c is not None else 1.0
      except (TypeError, ValueError):
        return 1.0

    # local demotion check
    for ctx in feat:
      key = (fid, ctx)
      dem = self._demoted.get(key)
      if dem is not None:
        if data.generated > dem:
          continue
        return False

    tier = None
    for ctx, info in feat.items():
      if not isinstance(info, dict):
        continue
      st = info.get("status")
      if st == "live":
        tier = info.get("tier")
        break
    if tier is None:
      return False

    if tier == "control":
      has_speed = _has_speed_cells(feat)
      for ctx in feat:
        info = feat[ctx]
        if not isinstance(info, dict):
          continue
        if info.get("status") != "live":
          continue
        cell_keys = _parse_cell_keys(ctx)
        missing = False
        for k in cell_keys:
          if k not in cond or cond[k] is None:
            missing = True
            break
        if missing:
          continue
        ok = True
        for piece in ctx.split("|"):
          if "=" not in piece:
            continue
          k, _, v = piece.partition("=")
          v = v.strip()
          if k not in cond or cond[k] != v:
            ok = False
            break
        if not ok:
          continue
        cstar = _entry_cstar(ctx)
        if conf < cstar:
          continue
        if acting:
          pair_ok = True
          for other in acting:
            if other == fid:
              continue
            pair_key = "+".join(sorted([fid, other]))
            combo = data.combos.get(pair_key)
            if not isinstance(combo, dict):
              pair_ok = False
              break
            cinfo = combo.get(ctx)
            if not isinstance(cinfo, dict) or cinfo.get("status") != "live":
              pair_ok = False
              break
            if not _cell_matches(ctx, cond):
              pair_ok = False
              break
            ccstar = cinfo.get("cstar")
            try:
              cc = float(ccstar) if ccstar is not None else 1.0
            except (TypeError, ValueError):
              cc = 1.0
            if conf < cc:
              pair_ok = False
              break
          if not pair_ok:
            continue
        return True
      return False
    else:
      for ctx in feat:
        info = feat[ctx]
        if not isinstance(info, dict):
          continue
        if info.get("status") != "live":
          continue
        if not _cell_matches(ctx, cond):
          continue
        cstar = _entry_cstar(ctx)
        if conf < cstar:
          continue
        if acting:
          pair_ok = True
          for other in acting:
            if other == fid:
              continue
            pair_key = "+".join(sorted([fid, other]))
            combo = data.combos.get(pair_key)
            if not isinstance(combo, dict):
              pair_ok = False
              break
            cinfo = combo.get(ctx)
            if not isinstance(cinfo, dict) or cinfo.get("status") != "live":
              pair_ok = False
              break
            if not _cell_matches(ctx, cond):
              pair_ok = False
              break
            ccstar = cinfo.get("cstar")
            try:
              cc = float(ccstar) if ccstar is not None else 1.0
            except (TypeError, ValueError):
              cc = 1.0
            if conf < cc:
              pair_ok = False
              break
          if not pair_ok:
            continue
        return True
      return False

  # ---- demote ----

  def demote(self, fid, cell, reason=""):
    now = time.time()
    with self._lock:
      demoted = dict(self._demoted)
      demoted[(fid, cell)] = now
      body = {"generated": now, "demotions": _pairs_to_demoted(demoted), "reason": reason}
    tmp = DEMOTED_PATH + ".tmp"
    try:
      with open(tmp, "w") as f:
        json.dump(body, f, separators=(",", ":"))
      os.replace(tmp, DEMOTED_PATH)
    except OSError:
      pass
    with self._lock:
      self._demoted = demoted

  # ---- live_list ----

  def live_list(self, cond):
    out = []
    if self._data is None:
      return out
    with self._lock:
      data = self._data
    if data is None:
      return out
    feats = data.features
    if not feats:
      return out
    for fid, feat in feats.items():
      if not isinstance(feat, dict):
        continue
      names = data.names
      nm = names.get(fid) if isinstance(names, dict) else None
      if nm is None:
        continue
      for ctx, info in feat.items():
        if not isinstance(info, dict):
          continue
        if info.get("status") != "live":
          continue
        if not _cell_matches(ctx, cond):
          continue
        cstar = _entry_cstar(ctx)
        if cstar > 1.0:
          continue
        where = _where_desc(ctx, cond)
        out.append((nm, where))
    return out

  @property
  def generated(self):
    with self._lock:
      if self._data is None:
        return 0.0
      return self._data.generated


def _where_desc(ctx, cond):
  cond = _norm_cond(cond)
  parts = []
  if cond.get("light"):
    parts.append(cond["light"])
  if cond.get("road"):
    parts.append(cond["road"])
  if cond.get("weather"):
    parts.append(cond["weather"])
  if cond.get("speed"):
    parts.append(cond["speed"])
  if ctx != "_":
    for piece in ctx.split("|"):
      if "=" in piece:
        k, _, v = piece.partition("=")
        v = v.strip()
        if k == "light":
          parts.append(v)
  return ", ".join(parts) if parts else "all"
