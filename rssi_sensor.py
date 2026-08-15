#!/usr/bin/env python3
"""rssi_sensor.py — WLAN Pi capture agent for RSSI multilateration.

Runs on each WLAN Pi. Puts one or more monitor-mode interfaces on fixed
channels, captures the frames it hears, and reports the RSSI of each
transmitter (an AP's beacons, or a client's frames) to the rssi_locator
collector. Multiple interfaces let one Pi watch several channels at once, so
APs on different channels are all covered simultaneously.

    sudo ./rssi_sensor.py --collector http://LOCATOR:8090 --id pi-1 \
         --iface wlan1@36 --iface wlan2@149 --capture aps --setup

Each --iface may carry a channel as `name@chan`. --setup flips those
interfaces into monitor mode and sets the channel via `iw` first (needs root).
--capture aps|clients|all selects which transmitters to report.

--mock reports synthetic detections (no radio). --selftest validates the
radiotap/802.11 parser and exits. Standard library only, Python 3.7+.
"""

import argparse
import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import defaultdict


def _median(v):
    v = sorted(v)
    n = len(v)
    m = n // 2
    return v[m] if n % 2 else (v[m - 1] + v[m]) / 2.0


# --------------------------------------------------------------------------- #
# Radiotap + 802.11 parsing
# --------------------------------------------------------------------------- #

# bit -> (size, alignment); only bits below dBm-antsignal (5) affect its offset.
_RT_FIELDS = {
    0: (8, 8), 1: (1, 1), 2: (1, 1), 3: (4, 2), 4: (2, 2),
    5: (1, 1), 6: (1, 1), 7: (2, 2), 8: (2, 2), 9: (2, 2),
    10: (1, 1), 11: (1, 1), 12: (1, 1), 13: (1, 1), 14: (2, 2),
}


def parse_radiotap(buf):
    """Return (rt_len, signal_dbm|None, freq_mhz|None) or None if malformed."""
    if len(buf) < 8:
        return None
    rt_len = int.from_bytes(buf[2:4], "little")
    # Walk the present-flags words (bit 31 chains another word).
    off = 4
    present0 = None
    while off + 4 <= len(buf):
        word = int.from_bytes(buf[off:off + 4], "little")
        if present0 is None:
            present0 = word
        off += 4
        if not (word & 0x80000000):
            break
    if present0 is None:
        return None
    pos = off
    signal = None
    freq = None
    for bit in range(0, 15):
        if not (present0 & (1 << bit)):
            continue
        size, align = _RT_FIELDS[bit]
        if pos % align:
            pos += align - (pos % align)
        if pos + size > len(buf):
            break
        if bit == 3:
            freq = int.from_bytes(buf[pos:pos + 2], "little")
        elif bit == 5:
            signal = int.from_bytes(buf[pos:pos + 1], "little", signed=True)
        pos += size
    return rt_len, signal, freq


def _mac(b):
    return ":".join("%02x" % x for x in b)


def parse_dot11(buf, rt_len):
    """Extract transmitter, kind and (for beacons) SSID from an 802.11 frame.

    Returns dict {mac, kind, bssid, ssid} or None. `mac` is addr2, the
    transmitter — exactly whose signal the RSSI describes.
    """
    if len(buf) < rt_len + 24:
        return None
    fc0 = buf[rt_len]
    ftype = (fc0 >> 2) & 0x3
    subtype = (fc0 >> 4) & 0xF
    addr2 = _mac(buf[rt_len + 10:rt_len + 16])
    addr3 = _mac(buf[rt_len + 16:rt_len + 22])

    if ftype == 0 and subtype == 8:          # beacon
        return {"mac": addr2, "kind": "ap", "bssid": addr3,
                "ssid": _beacon_ssid(buf, rt_len)}
    if ftype == 0 and subtype == 4:          # probe request
        return {"mac": addr2, "kind": "client", "bssid": None, "ssid": None}
    if ftype == 0 and subtype == 5:          # probe response
        return {"mac": addr2, "kind": "ap", "bssid": addr3,
                "ssid": _beacon_ssid(buf, rt_len)}
    if ftype == 2:                           # data
        fromds = buf[rt_len + 1] & 0x02
        return {"mac": addr2, "kind": "ap" if fromds else "client",
                "bssid": addr3, "ssid": None}
    return None


def _beacon_ssid(buf, rt_len):
    # mgmt header 24 bytes, then fixed params 12 (tstamp8+interval2+caps2),
    # then tagged params; tag 0 = SSID.
    i = rt_len + 24 + 12
    if i + 2 > len(buf):
        return None
    tag, length = buf[i], buf[i + 1]
    if tag == 0 and i + 2 + length <= len(buf):
        try:
            return buf[i + 2:i + 2 + length].decode("utf-8", "replace") or None
        except Exception:
            return None
    return None


def freq_to_channel(freq):
    if freq is None:
        return None
    if freq == 2484:
        return 14
    if 2412 <= freq <= 2472:
        return (freq - 2407) // 5
    if 5000 <= freq <= 5900:
        return (freq - 5000) // 5
    if 5955 <= freq <= 7115:                 # 6 GHz
        return (freq - 5950) // 5
    return None


# --------------------------------------------------------------------------- #
# Interface setup
# --------------------------------------------------------------------------- #

def _run(cmd, verbose):
    if verbose:
        print("[setup] " + " ".join(cmd), file=sys.stderr)
    return subprocess.run(cmd, capture_output=True, text=True).returncode


def setup_monitor(iface, channel, verbose):
    """Best-effort: bring iface into monitor mode on `channel` via iw/ip."""
    _run(["ip", "link", "set", iface, "down"], verbose)
    rc = _run(["iw", "dev", iface, "set", "monitor", "control"], verbose)
    if rc != 0:
        _run(["iw", "dev", iface, "set", "type", "monitor"], verbose)
    _run(["ip", "link", "set", iface, "up"], verbose)
    if channel is not None:
        if _run(["iw", "dev", iface, "set", "channel", str(channel)],
                verbose) != 0:
            print("[setup] WARN: could not set %s to channel %s"
                  % (iface, channel), file=sys.stderr)


def set_channel(iface, channel, verbose):
    if channel is not None:
        _run(["iw", "dev", iface, "set", "channel", str(channel)], verbose)


# --------------------------------------------------------------------------- #
# Capture
# --------------------------------------------------------------------------- #

ETH_P_ALL = 0x0003


class Batch:
    """Accumulates detections between POSTs, averaging RSSI per transmitter."""

    def __init__(self):
        self._lock = threading.Lock()
        self._acc = defaultdict(lambda: {"sum": 0.0, "n": 0, "kind": None,
                                         "channel": None, "name": None})

    def add(self, mac, rssi, kind, channel, name):
        with self._lock:
            a = self._acc[mac]
            a["sum"] += rssi
            a["n"] += 1
            a["kind"] = kind or a["kind"]
            a["channel"] = channel if channel is not None else a["channel"]
            a["name"] = name or a["name"]

    def drain(self):
        with self._lock:
            out = []
            for mac, a in self._acc.items():
                if a["n"]:
                    out.append({"mac": mac, "rssi": round(a["sum"] / a["n"], 1),
                                "kind": a["kind"], "channel": a["channel"],
                                "name": a["name"], "samples": a["n"]})
            self._acc.clear()
            return out


def want(kind, capture, macs):
    if macs:
        return True   # allowlist is checked on mac elsewhere
    if capture == "all":
        return True
    if capture == "aps":
        return kind == "ap"
    if capture == "clients":
        return kind == "client"
    return True


def capture_iface(iface, channel, batch, args, stop):
    try:
        s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW,
                          socket.htons(ETH_P_ALL))
        s.bind((iface, 0))
        s.settimeout(1.0)
    except (OSError, PermissionError) as exc:
        print("[capture] %s: cannot open (%s) — need root + monitor mode"
              % (iface, exc), file=sys.stderr)
        return
    macs = set(m.lower() for m in args.macs) if args.macs else None
    while not stop.is_set():
        try:
            frame = s.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        rt = parse_radiotap(frame)
        if not rt:
            continue
        rt_len, signal, freq = rt
        if signal is None:
            continue
        info = parse_dot11(frame, rt_len)
        if not info:
            continue
        if macs is not None and info["mac"] not in macs:
            continue
        if not want(info["kind"], args.capture, macs):
            continue
        ch = channel if channel is not None else freq_to_channel(freq)
        batch.add(info["mac"], signal, info["kind"], ch, info.get("ssid"))
    s.close()


# --------------------------------------------------------------------------- #
# Mock
# --------------------------------------------------------------------------- #

def capture_samples(iface, seconds, macs, capture, verbose):
    """Capture for `seconds` and return mac -> {"rssi":[...], "kind", "name"}."""
    try:
        s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW,
                          socket.htons(ETH_P_ALL))
        s.bind((iface, 0))
        s.settimeout(0.5)
    except (OSError, PermissionError) as exc:
        print("cannot open %s (%s) — need root + monitor mode" % (iface, exc),
              file=sys.stderr)
        return {}
    allow = set(m.lower() for m in macs) if macs else None
    got = {}
    end = time.time() + seconds
    while time.time() < end:
        try:
            frame = s.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        rt = parse_radiotap(frame)
        if not rt or rt[1] is None:
            continue
        info = parse_dot11(frame, rt[0])
        if not info:
            continue
        if allow is not None and info["mac"] not in allow:
            continue
        if not want(info["kind"], capture, allow):
            continue
        rec = got.setdefault(info["mac"],
                             {"rssi": [], "kind": info["kind"], "name": None})
        rec["rssi"].append(rt[1])
        if info.get("ssid") and not rec["name"]:
            rec["name"] = info["ssid"]
    s.close()
    return got


def calibrate(args):
    """Measure RSSI to derive rssi_at_1m and per-sensor offsets for the config."""
    if not args.iface:
        print("calibrate needs one --iface name@channel", file=sys.stderr)
        return 2
    name, ch = parse_iface(args.iface[0])
    if args.setup:
        setup_monitor(name, ch, args.verbose)
    elif ch is not None:
        set_channel(name, ch, args.verbose)

    print("Calibrating on %s for %ds — keep the target still at %s ..."
          % (name, args.seconds,
             ("%.2f m" % args.dist) if args.dist else "a known distance"),
          file=sys.stderr)
    got = capture_samples(name, args.seconds, args.macs, args.capture,
                          args.verbose)
    if not got:
        print("No frames captured. Check monitor mode, channel, and that the "
              "target is transmitting on this channel.", file=sys.stderr)
        return 1

    rows = []
    for mac, rec in got.items():
        rows.append((mac, _median(rec["rssi"]), len(rec["rssi"]), rec["name"]))
    rows.sort(key=lambda r: r[1], reverse=True)  # strongest first

    if args.macs:
        chosen = [r for r in rows if r[0] == args.macs[0].lower()]
        if not chosen:
            print("Target %s not heard. Strongest seen:" % args.macs[0],
                  file=sys.stderr)
            _print_rows(rows[:8])
            return 1
    else:
        print("Heard %d transmitters (strongest first):" % len(rows),
              file=sys.stderr)
        _print_rows(rows[:8])
        chosen = rows[:1]
        print("\nUsing the strongest as the calibration target "
              "(pass --macs to pick one).", file=sys.stderr)

    mac, med, n, ssid = chosen[0]
    print("\nTarget %s%s: median RSSI = %.1f dBm over %d frames"
          % (mac, (" (%s)" % ssid) if ssid else "", med, n))

    if args.dist:
        a1m = med + 10.0 * args.exponent * math.log10(max(args.dist, 0.01))
        print("Derived rssi_at_1m = %.1f dBm  (assuming exponent = %.1f)"
              % (a1m, args.exponent))
        print('  -> "path_loss": { "rssi_at_1m": %.1f, "exponent": %.1f }'
              % (a1m, args.exponent))
    if args.ref is not None:
        offset = args.ref - med
        print("Per-sensor offset vs ref %.1f dBm = %.1f"
              % (args.ref, offset))
        print('  -> this sensor: "rssi_offset": %.1f' % offset)
    if not args.dist and args.ref is None:
        print("\nTip: add --dist 1.0 to derive rssi_at_1m, or --ref <dBm> "
              "(a reference sensor's median for the same target) to derive "
              "this sensor's rssi_offset.")
    return 0


def _print_rows(rows):
    for mac, med, n, ssid in rows:
        print("  %-18s %6.1f dBm  n=%-4d %s"
              % (mac, med, n, ssid or ""), file=sys.stderr)


def mock_capture(batch, args, stop):
    import math
    import random
    rng = random.Random(hash(args.id) & 0xffff)
    aps = [("aa:bb:cc:00:00:01", "AP-Mock-36", 36, -47),
           ("aa:bb:cc:00:00:02", "AP-Mock-149", 149, -63)]
    t0 = time.time()
    while not stop.is_set():
        wobble = 3.0 * math.sin((time.time() - t0) * 0.2)
        for mac, name, ch, base in aps:
            batch.add(mac, base + wobble + rng.gauss(0, 1.5), "ap", ch, name)
        stop.wait(0.3)


# --------------------------------------------------------------------------- #
# BLE capture (Linux HCI)
# --------------------------------------------------------------------------- #
#
# Why raw HCI rather than shelling out to bluetoothctl: this file is stdlib-only
# and parsing a human-facing CLI's output is a moving target. Python's socket
# module speaks AF_BLUETOOTH/BTPROTO_HCI on Linux, so the advertising reports
# arrive as bytes we decode ourselves — the same relationship this tool already
# has with radiotap.
#
# What this can and cannot hear, established by scanning a real house: devices
# that ADVERTISE are visible (BLE bulbs, phones, beacons, tags). A device already
# in a CONNECTION is not — it hops the 37 data channels with a connection-
# specific access address, and following that needs a sniffer, not an adapter.
# UniFi's UP Sense sensors are in that second group: 27 other devices showed up
# in a scan and not one sensor did.
#
# Addresses are not stable. Phones and watches rotate a random address every
# ~15 minutes, so they can be located now but not tracked across a day. Fixed
# kit (bulbs, beacons) keeps a static address and can be followed indefinitely
# — which also makes it the right thing to calibrate path loss against.

HCI_EVENT_PKT = 0x04
HCI_LE_META = 0x3E
LE_ADVERTISING_REPORT = 0x02
SOL_HCI, HCI_FILTER = 0, 2


def ble_addr(raw):
    """6 little-endian bytes -> the aa:bb:cc form the collector keys on."""
    return ":".join("%02x" % b for b in reversed(raw))


def ble_local_name(data):
    """Complete (0x09) or shortened (0x08) local name from the AD structures."""
    i = 0
    while i + 1 < len(data):
        ln = data[i]
        if ln == 0 or i + ln >= len(data) + 1:
            break
        typ = data[i + 1]
        if typ in (0x08, 0x09):
            try:
                return data[i + 2:i + 1 + ln].decode("utf-8", "replace").strip()
            except Exception:                      # noqa: BLE001 - name is optional
                return None
        i += ln + 1
    return None


def parse_le_advertising_report(pkt):
    """Decode one HCI event packet into [(mac, rssi, name), ...].

    Returns [] for anything that is not an LE Advertising Report, so the read
    loop can hand it every packet without pre-filtering.

    The RSSI is the LAST byte of each report, after a variable-length AD
    payload — read it by walking the reports, never from a fixed offset.
    """
    if len(pkt) < 5 or pkt[0] != HCI_EVENT_PKT or pkt[1] != HCI_LE_META:
        return []
    if pkt[3] != LE_ADVERTISING_REPORT:
        return []
    out, n, i = [], pkt[4], 5
    for _ in range(n):
        if i + 9 > len(pkt):
            break
        addr = pkt[i + 2:i + 8]
        data_len = pkt[i + 8]
        data = pkt[i + 9:i + 9 + data_len]
        rssi_at = i + 9 + data_len
        if rssi_at >= len(pkt):
            break
        rssi = pkt[rssi_at] - 256 if pkt[rssi_at] > 127 else pkt[rssi_at]
        out.append((ble_addr(addr), float(rssi), ble_local_name(data)))
        i = rssi_at + 1
    return out


def _hci_cmd(ogf, ocf, params=b""):
    return (bytes([0x01]) + struct.pack("<H", (ogf << 10) | ocf)
            + bytes([len(params)]) + params)


def ble_capture(batch, args, stop):
    """Passive LE scan on hci<dev>, reporting every advertiser it hears."""
    try:
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_RAW,
                             socket.BTPROTO_HCI)
        sock.bind((args.ble_dev,))
    except (AttributeError, OSError) as exc:
        print("[ble] cannot open hci%d: %s" % (args.ble_dev, exc), file=sys.stderr)
        print("[ble] needs Linux and root; check `hciconfig hci%d up`"
              % args.ble_dev, file=sys.stderr)
        return
    # Only HCI event packets, and of those only LE Meta (0x3E).
    sock.setsockopt(SOL_HCI, HCI_FILTER,
                    struct.pack("<LLLH", 1 << HCI_EVENT_PKT, 0,
                                1 << (HCI_LE_META - 32), 0))
    # passive scan, 10 ms window every 10 ms — duplicates ON, because every
    # repeat advertisement is another RSSI sample and averaging them is the
    # entire point.
    sock.send(_hci_cmd(0x08, 0x000B,
                       struct.pack("<BHHBB", 0x00, 0x0010, 0x0010, 0x00, 0x00)))
    sock.send(_hci_cmd(0x08, 0x000C, struct.pack("<BB", 0x01, 0x00)))
    if args.verbose:
        print("[ble] scanning on hci%d" % args.ble_dev, file=sys.stderr)
    sock.settimeout(1.0)
    try:
        while not stop.is_set():
            try:
                pkt = sock.recv(1024)
            except socket.timeout:
                continue
            except OSError as exc:
                print("[ble] read failed: %s" % exc, file=sys.stderr)
                return
            for mac, rssi, name in parse_le_advertising_report(pkt):
                if args.macs and mac not in args.macs:
                    continue
                batch.add(mac, rssi, "ble", None, name)
    finally:
        try:
            sock.send(_hci_cmd(0x08, 0x000C, struct.pack("<BB", 0x00, 0x00)))
            sock.close()
        except OSError:
            pass


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #

def resolve_token(args) -> str:
    """The collector token: --token, else SENSOR_TOKEN, else none.

    Environment first in practice, because a token passed as --token sits in
    `ps` output for every user on the host, continuously, for as long as the
    sensor runs. That is the same defect fixed for the exporter's console
    password in #9, and a sensor daemon has an even longer exposure than a
    scheduled subprocess.

    Empty is allowed: the standalone locator wants no token, and refusing to
    start would break the setup this tool was written for.
    """
    return (args.token or os.environ.get("SENSOR_TOKEN") or "").strip()


def report_loop(batch, args, stop):
    url = args.collector.rstrip("/") + "/report"
    token = resolve_token(args)
    headers = {"Content-Type": "application/json"}
    if token:
        headers["X-Sensor-Token"] = token
    warned = set()
    while not stop.is_set():
        stop.wait(args.interval)
        dets = batch.drain()
        if not dets:
            continue
        body = json.dumps({"sensor_id": args.id, "detections": dets}).encode()
        req = urllib.request.Request(url, body, headers)
        try:
            urllib.request.urlopen(req, timeout=5).read()
            if args.verbose:
                print("[report] %d transmitters -> %s" % (len(dets), url),
                      file=sys.stderr)
        except urllib.error.HTTPError as exc:
            # Name the auth failures once each. They repeat every interval, and
            # a wall of identical "POST failed: HTTP Error 401" buries the one
            # line that says what to do about it.
            hint = {
                401: "the collector rejected the token — set SENSOR_TOKEN to "
                     "match the bridge's",
                503: "the collector has no token configured and refuses "
                     "reports — set SENSOR_TOKEN on the bridge",
                400: "the collector rejected the report — is --id one of its "
                     "configured sensor ids?",
                404: "no /report endpoint — is SENSORS_ENABLED=true on the "
                     "bridge?",
            }.get(exc.code)
            if hint and exc.code not in warned:
                warned.add(exc.code)
                print("[report] HTTP %d: %s" % (exc.code, hint), file=sys.stderr)
            elif not hint:
                print("[report] POST failed: %s" % exc, file=sys.stderr)
        except Exception as exc:
            print("[report] POST failed: %s" % exc, file=sys.stderr)


# --------------------------------------------------------------------------- #
# Self-test
# --------------------------------------------------------------------------- #

def _ble_selftest():
    """One real-shaped LE Advertising Report, decoded.

    Two reports in one event, a name in the AD payload, and a negative RSSI as
    the LAST byte after a variable-length payload — which is the detail that
    makes a fixed offset wrong.
    """
    name = b"\x07\x09Govee1"                       # complete local name
    r1 = (bytes([0x00, 0x00]) + bytes([0x4e, 0x3d, 0x1f, 0x38, 0xc1, 0xa4])
          + bytes([len(name)]) + name + bytes([256 - 80]))     # -80 dBm
    r2 = (bytes([0x00, 0x01]) + bytes([0x40, 0x45, 0xa8, 0x79, 0x3a, 0xa8])
          + bytes([0x00]) + bytes([256 - 52]))                 # -52, no AD data
    pkt = bytes([HCI_EVENT_PKT, HCI_LE_META, 0, LE_ADVERTISING_REPORT, 2]) + r1 + r2
    got = parse_le_advertising_report(pkt)
    want = [("a4:c1:38:1f:3d:4e", -80.0, "Govee1"),
            ("a8:3a:79:a8:45:40", -52.0, None)]
    if got != want:
        print("BLE selftest FAILED\n  got  %r\n  want %r" % (got, want),
              file=sys.stderr)
        return False
    if parse_le_advertising_report(bytes([HCI_EVENT_PKT, 0x0E, 0, 0])) != []:
        print("BLE selftest FAILED: non-advertising event was decoded",
              file=sys.stderr)
        return False
    print("ble selftest OK: 2 report(s), -80/-52 dBm, name 'Govee1'")
    return True


def selftest():
    # Radiotap with Flags(1), Rate(2), Channel(3), dBm-antsignal(5) present.
    present = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5)   # 0x2E
    rt = bytearray()
    rt += bytes([0, 0])                       # version, pad
    rt += (15).to_bytes(2, "little")          # rt_len
    rt += present.to_bytes(4, "little")
    rt += bytes([0x00])                       # Flags
    rt += bytes([0x02])                       # Rate
    rt += (5180).to_bytes(2, "little") + (0).to_bytes(2, "little")  # Channel
    rt += (-55 & 0xff).to_bytes(1, "little")  # antsignal = -55 dBm
    assert len(rt) == 15, len(rt)

    # Beacon: FC=0x80,0x00; dur; a1=broadcast; a2=a3=BSSID; seq; params + SSID.
    bssid = bytes([0x00, 0x11, 0x22, 0x33, 0x44, 0x55])
    d11 = bytearray()
    d11 += bytes([0x80, 0x00]) + bytes([0, 0])
    d11 += bytes([0xff] * 6) + bssid + bssid + bytes([0, 0])
    d11 += bytes(12)                          # tstamp+interval+caps
    d11 += bytes([0x00, 0x04]) + b"TEST"      # SSID tag

    frame = bytes(rt) + bytes(d11)
    rt_len, signal, freq = parse_radiotap(frame)
    info = parse_dot11(frame, rt_len)
    assert rt_len == 15, rt_len
    assert signal == -55, signal
    assert freq == 5180, freq
    assert freq_to_channel(freq) == 36, freq_to_channel(freq)
    assert info["mac"] == "00:11:22:33:44:55", info
    assert info["kind"] == "ap", info
    assert info["ssid"] == "TEST", info
    print("selftest OK: signal=%d dBm, freq=%d (ch%d), tx=%s, ssid=%s"
          % (signal, freq, freq_to_channel(freq), info["mac"], info["ssid"]))
    return 0


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def parse_iface(spec):
    if "@" in spec:
        name, ch = spec.split("@", 1)
        return name, int(ch)
    return spec, None


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--collector", help="locator base URL, e.g. http://host:8090")
    p.add_argument("--id", help="this sensor's id (must match the locator config)")
    p.add_argument("--token", default="",
                   help="collector token, sent as X-Sensor-Token. PREFER the "
                        "SENSOR_TOKEN environment variable: an argument is "
                        "visible in `ps` to every user on this host, for as "
                        "long as the sensor runs. Not needed by the standalone "
                        "locator; required by unifi-hamina-live.")
    p.add_argument("--ble", action="store_true",
                   help="also scan BLE advertisements on hci0 and report them "
                        "as kind=ble. Hears advertisers (bulbs, phones, "
                        "beacons, tags); CANNOT hear a device already in a "
                        "connection, which is where UniFi's UP Sense sensors "
                        "live. Needs root.")
    p.add_argument("--ble-dev", type=int, default=0, metavar="N",
                   help="HCI device index for --ble (default 0 = hci0)")
    p.add_argument("--iface", action="append", default=[],
                   help="monitor interface, optionally name@channel (repeatable)")
    p.add_argument("--capture", choices=["aps", "clients", "all"],
                   default="aps", help="which transmitters to report")
    p.add_argument("--macs", default="",
                   help="comma-separated MAC allowlist (overrides --capture)")
    p.add_argument("--interval", type=float, default=1.0,
                   help="seconds between POSTs (default 1.0)")
    p.add_argument("--setup", action="store_true",
                   help="put each --iface into monitor mode + channel first")
    p.add_argument("--mock", action="store_true",
                   help="report synthetic detections; no radio")
    p.add_argument("--selftest", action="store_true",
                   help="validate the radiotap/802.11 parser and exit")
    p.add_argument("--calibrate", action="store_true",
                   help="measure RSSI to derive rssi_at_1m / rssi_offset, then exit")
    p.add_argument("--seconds", type=float, default=8.0,
                   help="--calibrate capture duration (default 8)")
    p.add_argument("--dist", type=float, default=0.0,
                   help="--calibrate: known target distance in m (derives rssi_at_1m)")
    p.add_argument("--exponent", type=float, default=3.0,
                   help="--calibrate: assumed path-loss exponent (default 3.0)")
    p.add_argument("--ref", type=float, default=None,
                   help="--calibrate: reference median dBm (derives rssi_offset)")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)
    args.macs = [m.strip() for m in args.macs.split(",") if m.strip()]

    if args.selftest:
        return 0 if (selftest() == 0 and _ble_selftest()) else 1
    if args.calibrate:
        return calibrate(args)
    if not args.collector or not args.id:
        p.error("--collector and --id are required (unless --selftest)")

    ifaces = [parse_iface(s) for s in args.iface]
    batch = Batch()
    stop = threading.Event()
    threads = []

    if args.mock:
        threads.append(threading.Thread(target=mock_capture,
                                        args=(batch, args, stop), daemon=True))
    else:
        if not ifaces and not args.ble:
            p.error("at least one --iface (or --ble) is required in live mode")
        for name, ch in ifaces:
            if args.setup:
                setup_monitor(name, ch, args.verbose)
            elif ch is not None:
                set_channel(name, ch, args.verbose)
            threads.append(threading.Thread(
                target=capture_iface, args=(name, ch, batch, args, stop),
                daemon=True))

    if args.ble and not args.mock:
        threads.append(threading.Thread(target=ble_capture,
                                        args=(batch, args, stop), daemon=True))

    threads.append(threading.Thread(target=report_loop,
                                    args=(batch, args, stop), daemon=True))
    for t in threads:
        t.start()

    print("rssi_sensor id=%s %s -> %s (capture=%s, ifaces=%s)" %
          (args.id, "mock" if args.mock else "live", args.collector,
           args.capture, ",".join(a for a, _ in ifaces) or "-"))
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
