# Copy of /opt/leaf-can/lookahead-comma/la_local.py (comma-side Look Ahead reader, stdlib only). Keep in step.
"""Local Look Ahead reader for the car computer.

Reads two files pushed by a server (pack.json / pack.json.gz and live.json)
and answers "what is on the road ahead" without any network access.

Stdlib only. Public methods never raise.
"""

import gzip
import json
import math
import os
import time

PACK_MAX_AGE_S = 7 * 86400
ROUTE_MAX_AGE_S = 12 * 3600
EVENT_MAX_AGE_S = 2700
MATCH_M = 30
MATCH_DEG = 45
ROUTE_ON_M = 60
ROUTE_ON_DEG = 60
ITEM_LAT_M = 35
JUNCTION_LAT_M = 45
EVENT_LAT_M = 60
ITEM_DEG = 60
HEADING_FALLBACK_M = 300
LOG_EVERY_S = 2.0
LOG_MAX_BYTES = 20_000_000
CELL_DEG = 0.002

M_PER_DEG = 111320.0
MTIME_CHECK_S = 5.0


def _num(value):
    """Return value as a float when it is a real number, else None."""
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _ang_diff(a, b):
    """Smallest signed difference between two bearings, in degrees."""
    return ((a - b + 180.0) % 360.0) - 180.0


def _dist(lat1, lon1, lat2, lon2):
    """Flat-earth distance in metres (fine at road scale)."""
    cosl = math.cos(math.radians((lat1 + lat2) * 0.5))
    dy = (lat2 - lat1) * M_PER_DEG
    dx = (lon2 - lon1) * M_PER_DEG * cosl
    return math.hypot(dx, dy)


def _bearing(lat1, lon1, lat2, lon2):
    """Bearing in degrees clockwise from north."""
    cosl = math.cos(math.radians((lat1 + lat2) * 0.5))
    dx = (lon2 - lon1) * M_PER_DEG * cosl
    dy = (lat2 - lat1) * M_PER_DEG
    return math.degrees(math.atan2(dx, dy)) % 360.0


def _parse_item(obj):
    """Normalise one pack item / live event. Returns None when unusable."""
    if not isinstance(obj, dict):
        return None
    lat = _num(obj.get("lat"))
    lon = _num(obj.get("lon"))
    if lat is None or lon is None:
        return None
    x = obj.get("x")
    if not isinstance(x, dict):
        x = {}
    return {
        "id": obj.get("id"),
        "k": obj.get("k"),
        "lat": lat,
        "lon": lon,
        "h": _num(obj.get("h")),
        "asof": _num(obj.get("asof")),
        "exp": _num(obj.get("exp")),
        "x": x,
    }


def _continuation(trace, idx, horizon):
    """Points of a trace starting at idx, out to horizon metres."""
    pts = trace["p"]
    cum = trace["cum"]
    base = cum[idx]
    out = []
    j = idx
    n = len(pts)
    while j < n and (cum[j] - base) <= horizon:
        out.append((pts[j][0], pts[j][1]))
        j += 1
    if len(out) < 2 and j < n:
        out.append((pts[j][0], pts[j][1]))
    return out


def _junction_key(item):
    """Group key for a junction candidate: intersection id, else id before '@'."""
    x = item["x"]
    if isinstance(x, dict):
        jid = x.get("intersection_id")
        if jid is not None:
            return "ix:%s" % (jid,)
    iid = item["id"]
    if isinstance(iid, str) and iid:
        return "ix:%s" % (iid.split("@", 1)[0],)
    return None


class LookAhead:
    def __init__(self, base="/data/canopy/lookahead",
                 shadow_path="/data/canopy/lookahead_shadow.jsonl"):
        self.base = base
        self.shadow_path = shadow_path
        self._last_check = None
        self._pack_sig = None
        self._live_sig = None
        self._pack = self._empty_pack()
        self._live = self._empty_live()
        self._loads = 0
        self._last_log_ts = None
        self._last_ids = None

    # ------------------------------------------------------------------ load
    @staticmethod
    def _empty_pack():
        return {"built_at": None, "traces": [], "items": [], "grid": {}}

    @staticmethod
    def _empty_live():
        return {"asof": None, "route": None, "events": []}

    @staticmethod
    def _stat_sig(path):
        try:
            return os.stat(path).st_mtime_ns
        except OSError:
            return None

    def _maybe_reload(self, now):
        if self._last_check is not None:
            try:
                if (now - self._last_check) < MTIME_CHECK_S:
                    return
            except TypeError:
                return
        self._last_check = now
        p1 = os.path.join(self.base, "pack.json")
        p2 = p1 + ".gz"
        sig = (self._stat_sig(p1), self._stat_sig(p2))
        if sig != self._pack_sig:
            self._pack_sig = sig
            self._pack = self._load_pack(p1, p2)
            self._loads += 1
        lp = os.path.join(self.base, "live.json")
        lsig = self._stat_sig(lp)
        if lsig != self._live_sig:
            self._live_sig = lsig
            self._live = self._load_live(lp)
            self._loads += 1

    def _load_pack(self, p1, p2):
        pack = self._empty_pack()
        raw = None
        try:
            with open(p1, "rb") as fh:
                raw = fh.read()
        except OSError:
            raw = None
        if raw is None:
            try:
                with open(p2, "rb") as fh:
                    raw = gzip.decompress(fh.read())
            except Exception:
                raw = None
        if not raw:
            return pack
        try:
            data = json.loads(raw.decode("utf-8"))
        except Exception:
            return pack
        if not isinstance(data, dict) or data.get("v") != 1:
            return pack
        built = _num(data.get("built_at"))
        if built is None:
            return pack
        pack["built_at"] = built

        traces = []
        for tr in data.get("traces") or []:
            if not isinstance(tr, dict):
                continue
            pts = []
            for p in tr.get("p") or []:
                if isinstance(p, (list, tuple)) and len(p) >= 3:
                    la = _num(p[0])
                    lo = _num(p[1])
                    h = _num(p[2])
                    if la is not None and lo is not None and h is not None:
                        pts.append((la, lo, h))
            if not pts:
                continue
            cum = [0.0]
            for i in range(1, len(pts)):
                cum.append(cum[-1] + _dist(pts[i - 1][0], pts[i - 1][1],
                                           pts[i][0], pts[i][1]))
            traces.append({"p": pts, "cum": cum})
        pack["traces"] = traces

        items = []
        for it in data.get("items") or []:
            parsed = _parse_item(it)
            if parsed is not None:
                items.append(parsed)
        pack["items"] = items

        grid = {}
        for ti, tr in enumerate(traces):
            for pi, (la, lo, _h) in enumerate(tr["p"]):
                key = (int(math.floor(la / CELL_DEG)),
                       int(math.floor(lo / CELL_DEG)))
                bucket = grid.get(key)
                if bucket is None:
                    grid[key] = [(la, lo, tr["p"][pi][2], ti, pi)]
                else:
                    bucket.append((la, lo, tr["p"][pi][2], ti, pi))
        pack["grid"] = grid
        return pack

    def _load_live(self, path):
        live = self._empty_live()
        try:
            with open(path, "rb") as fh:
                raw = fh.read()
        except OSError:
            return live
        if not raw:
            return live
        try:
            data = json.loads(raw.decode("utf-8"))
        except Exception:
            return live
        if not isinstance(data, dict) or data.get("v") != 1:
            return live
        asof = _num(data.get("asof"))
        if asof is None:
            return live
        live["asof"] = asof

        route = data.get("route")
        if isinstance(route, dict):
            rasof = _num(route.get("asof"))
            pts = []
            for p in route.get("p") or []:
                if isinstance(p, (list, tuple)) and len(p) >= 2:
                    la = _num(p[0])
                    lo = _num(p[1])
                    if la is not None and lo is not None:
                        pts.append((la, lo))
            if rasof is not None and len(pts) >= 2:
                live["route"] = {"id": route.get("id"), "asof": rasof, "p": pts}

        events = []
        for ev in data.get("events") or []:
            parsed = _parse_item(ev)
            if parsed is not None:
                events.append(parsed)
        live["events"] = events
        return live

    # ------------------------------------------------------------------ paths
    def _route_path(self, pts, lat, lon, heading, horizon):
        n = len(pts)
        if n < 2:
            return None
        bearings = [
            _bearing(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
            for i in range(n - 1)
        ]
        best = None
        best_d = None
        for i in range(n):
            d = _dist(lat, lon, pts[i][0], pts[i][1])
            if d > ROUTE_ON_M:
                continue
            b = bearings[i] if i < n - 1 else bearings[-1]
            if abs(_ang_diff(b, heading)) > ROUTE_ON_DEG:
                continue
            if best_d is None or d < best_d:
                best_d = d
                best = i
        if best is None:
            return None
        out = [(pts[best][0], pts[best][1])]
        acc = 0.0
        for i in range(best, n - 1):
            acc += _dist(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
            if acc > horizon:
                break
            out.append((pts[i + 1][0], pts[i + 1][1]))
        if len(out) < 2:
            return None
        return out

    def _history_path(self, lat, lon, heading, horizon):
        pack = self._pack
        ci = int(math.floor(lat / CELL_DEG))
        cj = int(math.floor(lon / CELL_DEG))
        best = {}
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                for (pla, plo, ph, ti, pi) in pack["grid"].get((ci + di, cj + dj), ()):
                    if abs(_ang_diff(ph, heading)) > MATCH_DEG:
                        continue
                    d = _dist(lat, lon, pla, plo)
                    if d > MATCH_M:
                        continue
                    prev = best.get(ti)
                    if prev is None or d < prev[0]:
                        best[ti] = (d, pi)
        if not best:
            return None

        cands = []
        for ti, (d, pi) in sorted(best.items(), key=lambda kv: kv[1][0]):
            tr = pack["traces"][ti]
            cont = _continuation(tr, pi, horizon)
            if not cont:
                continue
            end_idx = min(len(tr["p"]) - 1, pi + len(cont) - 1)
            cands.append({
                "cont": cont,
                "end": cont[-1],
                "len": tr["cum"][end_idx] - tr["cum"][pi],
            })
        if not cands:
            return None

        used = [False] * len(cands)
        groups = []
        for i, c in enumerate(cands):
            if used[i]:
                continue
            group = [c]
            used[i] = True
            for j in range(i + 1, len(cands)):
                if used[j]:
                    continue
                other = cands[j]
                if _dist(c["end"][0], c["end"][1],
                         other["end"][0], other["end"][1]) <= 60.0:
                    group.append(other)
                    used[j] = True
            groups.append(group)

        group = max(groups, key=lambda g: (len(g), max(m["len"] for m in g)))
        member = max(group, key=lambda m: m["len"])
        return member["cont"], len(group) / float(len(cands))

    @staticmethod
    def _heading_path(lat, lon, heading):
        rad = math.radians(heading)
        cosl = max(math.cos(math.radians(lat)), 1e-9)
        pts = []
        d = 0.0
        while d <= HEADING_FALLBACK_M + 1e-9:
            pts.append((lat + math.cos(rad) * d / M_PER_DEG,
                        lon + math.sin(rad) * d / (M_PER_DEG * cosl)))
            d += 25.0
        return pts

    # ------------------------------------------------------------------ items
    def _project_items(self, raw_items, path, lat, lon):
        n = len(path)
        if n < 2:
            return [], 0.0

        cos_lat = max(math.cos(math.radians(lat)), 1e-6)
        px = [((p[1] - lon) * M_PER_DEG * cos_lat,
               (p[0] - lat) * M_PER_DEG) for p in path]
        cum = [0.0] * n
        for i in range(1, n):
            cum[i] = cum[i - 1] + math.hypot(px[i][0] - px[i - 1][0],
                                             px[i][1] - px[i - 1][1])
        path_len = cum[-1]

        if not raw_items:
            return [], path_len

        pad_m = max(ITEM_LAT_M, JUNCTION_LAT_M, EVENT_LAT_M)
        pad_lat = pad_m / M_PER_DEG
        pad_lon = pad_m / (M_PER_DEG * cos_lat)
        lats = [p[0] for p in path]
        lons = [p[1] for p in path]
        lat_min = min(lats) - pad_lat
        lat_max = max(lats) + pad_lat
        lon_min = min(lons) - pad_lon
        lon_max = max(lons) + pad_lon

        cands = []
        for it in raw_items:
            la = it["lat"]
            lo = it["lon"]
            if la < lat_min or la > lat_max or lo < lon_min or lo > lon_max:
                continue
            mx = (lo - lon) * M_PER_DEG * cos_lat
            my = (la - lat) * M_PER_DEG
            best = None
            for i in range(n - 1):
                ax, ay = px[i]
                bx, by = px[i + 1]
                vx = bx - ax
                vy = by - ay
                l2 = vx * vx + vy * vy
                if l2 <= 0.0:
                    continue
                t = ((mx - ax) * vx + (my - ay) * vy) / l2
                if t < 0.0:
                    t = 0.0
                elif t > 1.0:
                    t = 1.0
                cx = ax + t * vx
                cy = ay + t * vy
                lat_m = math.hypot(mx - cx, my - cy)
                if best is None or lat_m < best[0]:
                    along = cum[i] + t * math.sqrt(l2)
                    bearing = math.degrees(math.atan2(vx, vy)) % 360.0
                    best = (lat_m, along, bearing)
            if best is None:
                continue
            lat_m, along, bearing = best
            if it.get("k") == "az511":
                limit = EVENT_LAT_M
            elif it.get("k") in ("intersection", "signal"):
                limit = JUNCTION_LAT_M
            else:
                limit = ITEM_LAT_M
            if along < 0.0 or along > path_len + 1e-9:
                continue
            if lat_m > limit:
                continue
            h = it.get("h")
            kind = it.get("k")
            junction = kind in ("intersection", "signal")
            if (not junction and h is not None
                    and abs(_ang_diff(h, bearing)) > ITEM_DEG):
                continue
            cands.append({
                "id": it.get("id"),
                "k": kind,
                "along_m": along,
                "bearing": bearing,
                "h": h,
                "x": it.get("x", {}),
                "junction": junction,
            })

        out = []
        groups = {}
        for c in cands:
            if not c["junction"]:
                out.append({
                    "id": c["id"],
                    "k": c["k"],
                    "along_m": round(c["along_m"], 1),
                    "x": c["x"],
                })
                continue
            key = _junction_key(c)
            if key is None:
                out.append({
                    "id": c["id"],
                    "k": c["k"],
                    "along_m": round(c["along_m"], 1),
                    "x": c["x"],
                })
                continue
            groups.setdefault(key, []).append(c)

        for group in groups.values():
            def pick_key(c):
                h = c["h"]
                dev = 361.0 if h is None else abs(_ang_diff(h, c["bearing"]))
                return (dev, c["along_m"], str(c["id"]))

            pick = min(group, key=pick_key)
            out.append({
                "id": pick["id"],
                "k": "signal" if any(c["k"] == "signal" for c in group)
                else pick["k"],
                "along_m": round(min(c["along_m"] for c in group), 1),
                "x": pick["x"],
            })

        out.sort(key=lambda r: r["along_m"])
        return out, path_len

    # ------------------------------------------------------------------ public
    def view(self, lat, lon, heading, mph, now=None):
        if now is None:
            now = time.time()
        t0 = time.perf_counter()
        try:
            return self._view(lat, lon, heading, mph, now, t0)
        except Exception as exc:
            return {
                "ts": now,
                "source": "none",
                "items": [],
                "error": str(exc)[:200],
                "stale": [],
                "age_s": {},
            }

    def _view(self, lat, lon, heading, mph, now, t0):
        now = float(now)
        try:
            self._maybe_reload(now)
        except Exception:
            pass

        stale = []
        pack = self._pack
        built_at = pack.get("built_at")
        if built_at is None:
            stale.append("no_pack")
            pack_fresh = False
        else:
            pack_fresh = (now - built_at) <= PACK_MAX_AGE_S
            if not pack_fresh:
                stale.append("pack_old")

        live = self._live
        route = live.get("route")
        route_fresh = False
        if route is not None:
            if (now - route.get("asof", 0.0)) > ROUTE_MAX_AGE_S:
                stale.append("route_old")
            else:
                route_fresh = True

        dropped = 0
        events = []
        for ev in live.get("events", []):
            asof = ev.get("asof")
            exp = ev.get("exp")
            if exp and now > exp:
                dropped += 1
                continue
            if asof is None or (now - asof) > EVENT_MAX_AGE_S:
                dropped += 1
                continue
            events.append(ev)

        try:
            speed = float(mph) * 0.44704
        except (TypeError, ValueError):
            speed = 0.0
        horizon_m = min(5000.0, max(1500.0, speed * 90.0))

        if route_fresh:
            path = self._route_path(route["p"], lat, lon, heading, horizon_m)
            if path:
                source, path_conf = "route", 1.0
            else:
                path = None
        else:
            path = None

        if path is None and pack_fresh:
            hp = self._history_path(lat, lon, heading, horizon_m)
            if hp is not None:
                path, path_conf = hp
                source = "history"

        if path is None:
            path = self._heading_path(lat, lon, heading)
            source, path_conf = "heading", 0.2

        raw_items = []
        if pack_fresh:
            raw_items.extend(pack["items"])
        raw_items.extend(events)
        items, path_m = self._project_items(raw_items, path, lat, lon)

        live_asof = live.get("asof")
        route_asof = route.get("asof") if route is not None else None
        age_s = {
            "pack": (now - built_at) if built_at is not None else None,
            "live": (now - live_asof) if live_asof is not None else None,
            "route": (now - route_asof) if route_asof is not None else None,
        }

        return {
            "ts": now,
            "source": source,
            "path_conf": float(path_conf),
            "horizon_m": horizon_m,
            "path_m": path_m,
            "items": items,
            "age_s": age_s,
            "stale": stale,
            "dropped_events": dropped,
            "ms": (time.perf_counter() - t0) * 1000.0,
        }

    def log(self, view, lat, lon, heading, mph):
        try:
            ts = view.get("ts")
            ids = [it.get("id") for it in view.get("items", [])]
            changed = self._last_ids is None or ids != self._last_ids
            elapsed = (self._last_log_ts is None or ts is None
                       or (ts - self._last_log_ts) >= LOG_EVERY_S)
            if not (changed or elapsed):
                return

            row = {
                "ts": ts,
                "lat": round(float(lat), 5),
                "lon": round(float(lon), 5),
                "heading": int(round(float(heading))),
                "mph": round(float(mph), 1),
                "src": view.get("source"),
                "conf": round(float(view.get("path_conf", 0.0)), 2),
                "ids": ids,
                "age": view.get("age_s", {}),
                "stale": view.get("stale", []),
                "ms": round(float(view.get("ms", 0.0)), 1),
            }

            parent = os.path.dirname(self.shadow_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            try:
                if os.path.getsize(self.shadow_path) > LOG_MAX_BYTES:
                    os.replace(self.shadow_path, self.shadow_path + ".1")
            except OSError:
                pass
            with open(self.shadow_path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(row, separators=(",", ":")) + "\n")
            self._last_log_ts = ts
            self._last_ids = ids
        except Exception:
            pass

    def write_shm(self, view, path="/dev/shm/canopy_lookahead.json"):
        tmp = None
        try:
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            tmp = path + ".tmp.%d" % os.getpid()
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(view, fh)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, path)
        except Exception:
            if tmp:
                try:
                    os.remove(tmp)
                except OSError:
                    pass

    def stats(self):
        try:
            pack = self._pack
            live = self._live
            traces = pack.get("traces", [])
            return {
                "pack_built_at": pack.get("built_at"),
                "traces": len(traces),
                "trace_points": sum(len(t["p"]) for t in traces),
                "items": len(pack.get("items", [])),
                "events": len(live.get("events", [])),
                "route": live.get("route") is not None,
                "loads": self._loads,
            }
        except Exception:
            return {
                "pack_built_at": None,
                "traces": 0,
                "trace_points": 0,
                "items": 0,
                "events": 0,
                "route": False,
                "loads": self._loads,
            }
