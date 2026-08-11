# WLAN Pi RSSI location

Locate Wi-Fi **APs** (and, less reliably, **clients**) by measuring their signal
strength from several WLAN Pis in monitor mode and multilaterating a position.
No special radio required — this runs on the monitor-mode capture that WLAN Pis
and ordinary mt76/Realtek USB cards already do. It serves positions over the
same HTTP/JSON/SSE feed shape as the FTM tool, so the same client (e.g. an iOS
app) can consume it.

**Accuracy is RSSI-class:** room/zone level, roughly **2–8 m** depending on
calibration, sensor geometry, and multipath. This answers "which AP is roughly
where," not precise ranging. (FTM would be sub-metre — but no adapter you can
buy over USB does FTM; see the sibling FTM repo for that saga.)

## How it works

```
 AP beacons / client frames
        │  (RF)
        ▼
 [Pi-1] [Pi-2] [Pi-3] [Pi-4]     ← monitor mode, known positions
   │      │      │      │         each reports RSSI per transmitter
   └──────┴──POST─┴──────┘
              ▼
        rssi_locator            ← RSSI→distance (path loss) → multilaterate
              │
      GET /positions, /stream   ← JSON feed for your app
```

Each Pi hears a transmitter at some RSSI; stronger = closer. With ≥3 well-spread
sensors, the collector solves the position whose per-sensor distances best fit.

## Files

| File | Runs where | What |
|---|---|---|
| `rssi_locator.py` | one central host | Collector + multilateration + JSON/SSE server, serves the live map. `--mock` needs no sensors. |
| `rssi_sensor.py` | each WLAN Pi | Monitor-mode capture agent; reports RSSI per transmitter. `--mock`, `--selftest`, `--calibrate`. |
| `map.html` | served by locator | Self-contained live map (SSE); also embeddable in other apps. |
| `sensors.example.json` | locator | Sensor positions, path-loss model, targets, area, optional floor-plan. |

## 1. Locator config (`sensors.example.json`)

```json
{
  "area":      { "xmin": 0, "ymin": 0, "xmax": 20, "ymax": 15 },
  "path_loss": { "rssi_at_1m": -40.0, "exponent": 3.0 },
  "window_sec": 6.0,
  "min_sensors": 3,
  "track_all_aps": true,
  "sensors": [
    { "id": "pi-1", "x": 0,  "y": 0,  "rssi_offset": 0 },
    { "id": "pi-2", "x": 20, "y": 0,  "rssi_offset": 0 },
    { "id": "pi-3", "x": 20, "y": 15, "rssi_offset": 0 },
    { "id": "pi-4", "x": 0,  "y": 15, "rssi_offset": 0 }
  ],
  "targets": [ { "mac": "aa:bb:cc:dd:ee:01", "name": "AP-Lobby" } ]
}
```

- **`sensors`** — one per Pi, its `id` (must match `--id` on that Pi) and its
  measured `x`/`y` in metres on a shared floor grid. Spread them out; corners
  beat a cluster, and never put them in a straight line.
- **`track_all_aps`** — `true` auto-locates every AP whose beacons are heard.
  Set `false` to locate only the MACs in `targets`.
- **`path_loss`** — the RSSI→distance model (calibrate these; see below).
- **`rssi_offset`** — per-sensor dBm correction (calibrate; different
  cards/antennas read differently).

## 2. Run the locator

```bash
# no sensors yet — synthetic moving targets, for building the app:
python3 rssi_locator.py --config sensors.example.json --mock
python3 rssi_locator.py --config sensors.example.json --mock --once   # one JSON solve

# live:
python3 rssi_locator.py --config sensors.example.json --port 8090
```

Endpoints: `GET /` or `/map` (live map), `/positions` (JSON), `/stream` (SSE),
`/status` (text summary), `/healthz`; `POST /report` (sensors).

## 3. Run a sensor on each Pi

First validate the parser anywhere (no root/radio needed):

```bash
python3 rssi_sensor.py --selftest      # -> "selftest OK: signal=-55 dBm ..."
```

Then, on each Pi (needs **root** for monitor mode + raw capture):

```bash
sudo python3 rssi_sensor.py \
     --collector http://<locator-ip>:8090 --id pi-1 \
     --iface wlan1@36 --iface wlan2@149 \
     --capture aps --setup -v
```

- **`--iface name@channel`** — repeat per card. `--setup` flips each into
  monitor mode and sets the channel via `iw` for you. (Without `--setup`, set
  them up yourself first; see below.)
- **`--capture aps`** — report only AP beacons (best for AP location).
  `clients` = probe/data from stations; `all` = everything.
- **`--macs aa:bb:...,cc:dd:...`** — only these transmitters (overrides
  `--capture`).

### Multiple cards = multiple channels at once

This is the key to covering APs on different channels. Give each USB card its
own channel and one Pi watches them all simultaneously:

```bash
--iface wlan1@36 --iface wlan2@149 --iface wlan3@6
```

An AP on ch36 is located from every sensor's ch36 card; an AP on ch149 from
every ch149 card — concurrently. Put the **same channel set on every Pi** so all
sensors can hear the same APs. (A card can only listen to one channel at a time;
without enough cards you'd have to channel-hop, which thins out each AP's
samples.)

### Manual monitor-mode setup (if not using `--setup`)

```bash
sudo ip link set wlan1 down
sudo iw dev wlan1 set monitor control
sudo ip link set wlan1 up
sudo iw dev wlan1 set channel 36
```

## 4. Calibrate (do this once — it sets your accuracy)

The sensor has a built-in `--calibrate` mode that measures RSSI and prints the
config values to paste in.

**Derive `rssi_at_1m`** — place a target (an AP is easiest) a measured distance
from one sensor and run:
```bash
sudo python3 rssi_sensor.py --calibrate --iface wlan1@36 --dist 1.0 --setup
#   -> "Derived rssi_at_1m = -41.3 dBm (assuming exponent = 3.0)"
#   -> "path_loss": { "rssi_at_1m": -41.3, "exponent": 3.0 }
```
Pick a specific target with `--macs aa:bb:cc:...`; otherwise it uses the
strongest transmitter it hears and lists the rest.

**Derive per-sensor `rssi_offset`** — measure the *same* target from your
reference sensor, note its median dBm, then on each other sensor pass it as
`--ref`:
```bash
sudo python3 rssi_sensor.py --calibrate --iface wlan1@36 --macs aa:bb:cc:dd:ee:01 --ref -47.0
#   -> "Per-sensor offset vs ref -47.0 dBm = -3.5"
#   -> this sensor: "rssi_offset": -3.5
```

**Tune `exponent`** by hand — 2.0 = open space, 3.0–3.5 = typical office, 4+ =
lots of walls. Raise it until known distances read right.

Watch `residual_m` in the output: it's the fit error. Big residual = model or
calibration off, or a sensor mis-placed.

## 5. Live map

The locator serves a self-contained map at **`GET /map`** — sensors, located APs
(amber) and clients (blue), each with an uncertainty ring sized to its
`residual_m`, updating live over SSE. Just open `http://<locator-ip>:8090/map`.

### Overlay it on a real floor plan (the Hamina tie-in)

Drop a floor-plan image next to `rssi_locator.py` and add a `map` block to the
config, giving the pixel location of metre `(0,0)` and the image scale:

```json
"map": {
  "image_url": "/static/floor.png",
  "px_per_m": 30,
  "origin_px": [40, 520],
  "y_down": true
}
```

The locator serves the image from `/static/`, and the feed advertises the `map`
block so the page (and any other client) self-aligns metres to the plan. Two
numbers define the scale: `px_per_m` (pixels per metre in the image) and
`origin_px` (where metre 0,0 falls, in image pixels). A Hamina floor-plan export
already carries a metre scale, so those come straight from it.

## 6. Consuming the feed (iOS app + Hamina / UniFi "live")

Everything downstream reads one JSON contract, served two ways and CORS-open, so
any number of front-ends can share it:

- `GET /positions` — one-shot snapshot (poll it)
- `GET /stream` — Server-Sent Events, one snapshot per second (push)

### The payload

```json
{
  "seq": 42,
  "timestamp": 1786330000.0,
  "area": { "xmin": 0, "ymin": 0, "xmax": 20, "ymax": 15 },
  "map":  { "image_url": "/static/floor.png", "px_per_m": 30,
            "origin_px": [40, 520], "y_down": true },
  "sensors": [ { "id": "pi-1", "x": 0, "y": 0 } ],
  "targets": [
    {
      "mac": "aa:bb:cc:dd:ee:01", "name": "AP-Lobby", "kind": "ap",
      "channel": 36, "x": 6.2, "y": 4.1, "residual_m": 2.3, "n_sensors": 4,
      "sensors": [ { "sensor": "pi-1", "rssi": -58.0, "dist_m": 5.9, "n": 12 } ],
      "timestamp": 1786330000.0
    }
  ]
}
```

`--mock` emits this exact shape, so every consumer can be built before a single
Pi is deployed.

### Three ways to tie it into your other repos

1. **Embed the map as-is.** `map.html` reads its feed URL from a query param, so
   any page can drop it in an iframe pointed at the locator:
   ```html
   <iframe src="http://<locator-ip>:8090/map?feed=http://<locator-ip>:8090"
           style="border:0;width:100%;height:600px"></iframe>
   ```
2. **Render positions on your own Hamina/UniFi floor plan.** Ignore `map.html`
   and consume `/stream` directly. Each target's `x`/`y` are metres on the same
   grid you put the sensors on; multiply by your plan's pixels-per-metre and add
   your origin to drop AP markers onto an existing Hamina or UniFi floor view.
   The `residual_m` is a ready-made confidence radius.
3. **Correlate by MAC.** `target.mac` is the AP BSSID (or client MAC), so you can
   join these live positions straight onto UniFi device inventory or a Hamina AP
   list keyed by BSSID — e.g. show "planned vs measured" location per AP.

Because the coordinate system is *your* metre grid (defined by where you place
the sensors), aligning it to a Hamina plan is a single affine transform — the
same `px_per_m` + `origin_px` the `map` block already uses.

## Gotchas

- **MAC randomization** — for *clients*, modern phones randomize their MAC in
  probe requests, so you can only track a client whose real MAC you know or one
  associated to your network. **APs are unaffected** (stable BSSID) — hence AP
  location is the reliable use.
- **Same channel to be heard** — sensors only locate transmitters they're tuned
  to. Cover the channels your APs actually use.
- **RSSI is noisy** — it averages per batch and takes a median over
  `window_sec`; still expect metres of jitter. Geometry and calibration matter
  more than any code.
- **TDoA is not possible here** — the accurate time-based method needs
  nanosecond clock sync across sensors; commodity Wi-Fi + USB can't. RSSI is the
  ceiling for this hardware.
