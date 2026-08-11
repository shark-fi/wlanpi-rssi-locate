#!/usr/bin/env python3
"""rssi_locator.py — collector + multilateration server for WLAN Pi sensors.

Several WLAN Pis run `rssi_sensor.py` in monitor mode and POST the RSSI of every
frame they hear from a target transmitter (an AP's beacons, or a client's
frames) to this collector. For each target heard by enough sensors, the
collector converts RSSI -> distance via a path-loss model and multilaterates a
position, then serves the results over HTTP/JSON/SSE — the same feed shape the
FTM tool used, so an existing client (e.g. an iOS app) can consume it unchanged.

Accuracy is RSSI-class: room/zone level (~metres), not FTM's sub-metre. Good for
"which AP is where" and coarse client location; not for precise ranging.

Endpoints:
    POST /report        sensors post detections here (JSON)
    GET  /positions     current estimated position per target (JSON)
    GET  /stream        Server-Sent Events; positions on each solve
    GET  /              plain-text status
    GET  /healthz       "ok"

`--mock` synthesises moving targets and sensor reports internally, so you can
drive a client with no radios deployed. Standard library only, Python 3.7+.
"""

import argparse
import json
import math
import os
import threading
import time
from collections import defaultdict, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_HERE = os.path.dirname(os.path.abspath(__file__))


# --------------------------------------------------------------------------- #
# Config
# --------------------------------------------------------------------------- #

def load_config(path):
    with open(path) as fh:
        cfg = json.load(fh)
    cfg.setdefault("area", {"xmin": 0, "ymin": 0, "xmax": 20, "ymax": 15})
    cfg.setdefault("path_loss", {"rssi_at_1m": -40.0, "exponent": 3.0})
    cfg.setdefault("window_sec", 6.0)
    cfg.setdefault("min_sensors", 3)
    cfg.setdefault("track_all_aps", True)
    cfg.setdefault("targets", [])
    if not cfg.get("sensors"):
        raise ValueError("config needs a non-empty 'sensors' array")
    # Index sensors by id, keep positions and per-sensor calibration offset.
    sensors = {}
    for s in cfg["sensors"]:
        sensors[s["id"]] = {
            "id": s["id"],
            "x": float(s["x"]),
            "y": float(s["y"]),
            "rssi_offset": float(s.get("rssi_offset", 0.0)),
        }
    cfg["_sensors"] = sensors
    cfg["_names"] = {t["mac"].lower(): t.get("name", t["mac"])
                     for t in cfg["targets"]}
    cfg["_target_macs"] = set(cfg["_names"].keys())
    return cfg


# --------------------------------------------------------------------------- #
# Path-loss + multilateration
# --------------------------------------------------------------------------- #

def pathloss_distance(rssi, rssi_at_1m, exponent):
    """Log-distance path-loss inverse: RSSI(d) = A - 10*n*log10(d)."""
    d = 10.0 ** ((rssi_at_1m - rssi) / (10.0 * exponent))
    return min(max(d, 0.3), 500.0)


def multilaterate(points, area, coarse=41):
    """Least-squares position from (x, y, distance, weight) anchors.

    Coarse grid search (robust to bad geometry / local minima) followed by a
    shrinking coordinate refine. Returns (x, y, rms_residual_m).
    """
    xmin, ymin, xmax, ymax = area

    def cost(px, py):
        s = 0.0
        for x, y, d, w in points:
            r = math.hypot(px - x, py - y) - d
            s += w * r * r
        return s

    bx, by, bc = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0, float("inf")
    for i in range(coarse):
        px = xmin + (xmax - xmin) * i / (coarse - 1)
        for j in range(coarse):
            py = ymin + (ymax - ymin) * j / (coarse - 1)
            c = cost(px, py)
            if c < bc:
                bc, bx, by = c, px, py

    step = max(xmax - xmin, ymax - ymin) / (coarse - 1)
    for _ in range(200):
        improved = False
        for dx, dy in ((step, 0), (-step, 0), (0, step), (0, -step)):
            c = cost(bx + dx, by + dy)
            if c < bc:
                bc, bx, by = c, bx + dx, by + dy
                improved = True
        if not improved:
            step *= 0.5
            if step < 0.01:
                break

    wsum = sum(w for _, _, _, w in points) or 1.0
    rms = math.sqrt(bc / wsum)
    return bx, by, rms


# --------------------------------------------------------------------------- #
# Reading store
# --------------------------------------------------------------------------- #

class Store:
    def __init__(self, cfg):
        self.cfg = cfg
        self._lock = threading.Lock()
        # mac -> sensor_id -> deque[(recv_ts, rssi)]
        self._r = defaultdict(lambda: defaultdict(lambda: deque(maxlen=64)))
        self._meta = {}       # mac -> {"kind","channel","name"}
        self._seq = 0

    def ingest(self, sensor_id, detections):
        now = time.time()
        with self._lock:
            if sensor_id not in self.cfg["_sensors"]:
                return 0
            n = 0
            for d in detections:
                mac = str(d.get("mac", "")).lower()
                if not mac:
                    continue
                if not self.cfg["track_all_aps"] \
                        and mac not in self.cfg["_target_macs"]:
                    continue
                try:
                    rssi = float(d["rssi"])
                except (KeyError, TypeError, ValueError):
                    continue
                self._r[mac][sensor_id].append((now, rssi))
                meta = self._meta.setdefault(
                    mac, {"kind": None, "channel": None, "name": None})
                if d.get("kind"):
                    meta["kind"] = d["kind"]
                if d.get("channel") is not None:
                    meta["channel"] = d["channel"]
                # Prefer a configured name, else a beacon SSID, else keep.
                if mac in self.cfg["_names"]:
                    meta["name"] = self.cfg["_names"][mac]
                elif d.get("name") and not meta["name"]:
                    meta["name"] = d["name"]
                n += 1
            return n

    def solve(self):
        cfg = self.cfg
        window = cfg["window_sec"]
        a1m = cfg["path_loss"]["rssi_at_1m"]
        n_exp = cfg["path_loss"]["exponent"]
        area = (cfg["area"]["xmin"], cfg["area"]["ymin"],
                cfg["area"]["xmax"], cfg["area"]["ymax"])
        now = time.time()
        out = []
        with self._lock:
            self._seq += 1
            for mac, per_sensor in self._r.items():
                anchors = []
                detail = []
                for sid, dq in per_sensor.items():
                    vals = [rssi for ts, rssi in dq if now - ts <= window]
                    if not vals:
                        continue
                    s = cfg["_sensors"][sid]
                    rssi = _median(vals) + s["rssi_offset"]
                    dist = pathloss_distance(rssi, a1m, n_exp)
                    # Stronger signal (shorter range) is more trustworthy.
                    w = 1.0 / max(dist, 1.0)
                    anchors.append((s["x"], s["y"], dist, w))
                    detail.append({"sensor": sid, "rssi": round(rssi, 1),
                                   "dist_m": round(dist, 2), "n": len(vals)})
                if len(anchors) < cfg["min_sensors"]:
                    continue
                x, y, rms = multilaterate(anchors, area)
                meta = self._meta.get(mac, {})
                out.append({
                    "mac": mac,
                    "name": meta.get("name") or mac,
                    "kind": meta.get("kind"),
                    "channel": meta.get("channel"),
                    "x": round(x, 2),
                    "y": round(y, 2),
                    "residual_m": round(rms, 2),
                    "n_sensors": len(anchors),
                    "sensors": sorted(detail, key=lambda d: d["sensor"]),
                    "timestamp": now,
                })
            out.sort(key=lambda t: t["name"])
            sensors = [{"id": s["id"], "x": s["x"], "y": s["y"]}
                       for s in cfg["_sensors"].values()]
            return {"seq": self._seq, "targets": out, "sensors": sensors,
                    "area": cfg["area"], "map": cfg.get("map"),
                    "timestamp": now}

    def seq(self):
        with self._lock:
            return self._seq


def _median(v):
    v = sorted(v)
    n = len(v)
    m = n // 2
    return v[m] if n % 2 else (v[m - 1] + v[m]) / 2.0


# --------------------------------------------------------------------------- #
# Mock: synthesise sensor reports for moving targets
# --------------------------------------------------------------------------- #

def mock_loop(store, cfg, stop):
    import random
    rng = random.Random(1)
    a1m = cfg["path_loss"]["rssi_at_1m"]
    n_exp = cfg["path_loss"]["exponent"]
    area = cfg["area"]
    sensors = list(cfg["_sensors"].values())
    cx = (area["xmin"] + area["xmax"]) / 2.0
    cy = (area["ymin"] + area["ymax"]) / 2.0
    rad = min(area["xmax"] - area["xmin"], area["ymax"] - area["ymin"]) * 0.3
    targets = [
        {"mac": "aa:bb:cc:00:00:01", "name": "AP-Mock-36",
         "kind": "ap", "channel": 36, "phase": 0.0, "static": (3.0, 3.0)},
        {"mac": "aa:bb:cc:00:00:02", "name": "AP-Mock-149",
         "kind": "ap", "channel": 149, "phase": 2.1,
         "static": (area["xmax"] - 3.0, area["ymax"] - 3.0)},
        {"mac": "de:ad:be:ef:00:99", "name": "Client-Mock",
         "kind": "client", "channel": 36, "phase": 0.0, "static": None},
    ]
    t0 = time.time()
    while not stop.is_set():
        t = time.time() - t0
        for tg in targets:
            if tg["static"]:
                tx, ty = tg["static"]
            else:
                theta = t * 0.15 + tg["phase"]
                tx = cx + rad * math.cos(theta)
                ty = cy + rad * math.sin(theta)
            for s in sensors:
                d = max(0.5, math.hypot(tx - s["x"], ty - s["y"]))
                rssi = a1m - 10.0 * n_exp * math.log10(d) + rng.gauss(0, 2.0)
                store.ingest(s["id"], [{
                    "mac": tg["mac"], "rssi": rssi, "kind": tg["kind"],
                    "channel": tg["channel"], "name": tg["name"]}])
        stop.wait(0.5)


# --------------------------------------------------------------------------- #
# HTTP server
# --------------------------------------------------------------------------- #

def make_handler(store, cfg):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            if cfg.get("_verbose"):
                BaseHTTPRequestHandler.log_message(self, *a)

        def _send(self, body, code=200, ctype="application/json"):
            if isinstance(body, str):
                body = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path.split("?", 1)[0] != "/report":
                self._send(json.dumps({"error": "not found"}), 404)
                return
            length = int(self.headers.get("Content-Length", 0))
            try:
                payload = json.loads(self.rfile.read(length) or b"{}")
                sid = payload.get("sensor_id", "")
                dets = payload.get("detections", [])
                n = store.ingest(sid, dets)
                self._send(json.dumps({"ok": True, "ingested": n}))
            except (ValueError, TypeError) as exc:
                self._send(json.dumps({"error": str(exc)}), 400)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/healthz":
                self._send(b"ok", ctype="text/plain")
            elif path == "/positions":
                self._send(json.dumps(store.solve()))
            elif path == "/stream":
                self._stream()
            elif path in ("/", "/map"):
                self._map()
            elif path == "/status":
                self._status()
            elif path.startswith("/static/"):
                self._static(path[len("/static/"):])
            else:
                self._send(json.dumps({"error": "not found"}), 404)

        def _map(self):
            fp = os.path.join(_HERE, "map.html")
            try:
                with open(fp, "rb") as fh:
                    self._send(fh.read(), ctype="text/html; charset=utf-8")
            except OSError:
                self._send("map.html not found next to rssi_locator.py\n",
                           404, "text/plain")

        def _static(self, name):
            # Serve a floor-plan image from the script dir (basename only).
            name = os.path.basename(name)
            fp = os.path.join(_HERE, name)
            types = {".png": "image/png", ".jpg": "image/jpeg",
                     ".jpeg": "image/jpeg", ".svg": "image/svg+xml",
                     ".webp": "image/webp", ".gif": "image/gif"}
            ext = os.path.splitext(name)[1].lower()
            if ext not in types or not os.path.isfile(fp):
                self._send(json.dumps({"error": "not found"}), 404)
                return
            with open(fp, "rb") as fh:
                self._send(fh.read(), ctype=types[ext])

        def _status(self):
            snap = store.solve()
            lines = ["rssi_locator  sensors=%d  targets=%d" %
                     (len(cfg["_sensors"]), len(snap["targets"])), ""]
            for t in snap["targets"]:
                lines.append("%-16s %-18s (%5.1f,%5.1f) ±%4.1fm  %s ch%s  n=%d"
                             % (t["name"], t["mac"], t["x"], t["y"],
                                t["residual_m"], t["kind"] or "?",
                                t["channel"], t["n_sensors"]))
            self._send("\n".join(lines) + "\n", ctype="text/plain; charset=utf-8")

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                while True:
                    self.wfile.write(("data: %s\n\n" % json.dumps(store.solve()))
                                     .encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(1.0)
            except (BrokenPipeError, ConnectionResetError):
                return

    return Handler


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", required=True,
                   help="JSON config (see sensors.example.json)")
    p.add_argument("--port", type=int, default=8090)
    p.add_argument("--bind", default="0.0.0.0")
    p.add_argument("--mock", action="store_true",
                   help="synthesise sensor reports; no real sensors needed")
    p.add_argument("--once", action="store_true",
                   help="print one solve as JSON and exit (with --mock)")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)

    cfg = load_config(args.config)
    cfg["_verbose"] = args.verbose
    store = Store(cfg)
    stop = threading.Event()

    if args.mock:
        if args.once:
            # Prime a few sweeps so a fix is available immediately.
            m = threading.Thread(target=mock_loop, args=(store, cfg, stop),
                                 daemon=True)
            m.start()
            time.sleep(1.5)
            stop.set()
            print(json.dumps(store.solve(), indent=2))
            return 0
        threading.Thread(target=mock_loop, args=(store, cfg, stop),
                         daemon=True).start()

    server = ThreadingHTTPServer((args.bind, args.port),
                                 make_handler(store, cfg))
    print("rssi_locator: %s mode, %d sensors, http://%s:%d "
          "(GET /map, /positions, /stream ; POST /report)" %
          ("mock" if args.mock else "live", len(cfg["_sensors"]),
           args.bind, args.port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
