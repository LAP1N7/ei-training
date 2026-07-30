---
name: xiao-mqtt-multinode
description: >
  Building a MULTI-NODE sensor network on XIAO ESP32S3 boards that publish both
  raw I2C sensor values and Edge Impulse inference results to one MQTT broker,
  viewed together in a single browser dashboard over MQTT-over-WebSocket. Use
  this skill whenever the user wants several boards on one dashboard ("노드 두
  개", "제스처랑 시각 같이", "대시보드에 다 띄우기", "상대팀 노드 받아오기"),
  a gateway between boards and a web page, or hits the failure modes below:
  MQTT publish silently returning false, `rc=-2` on connect, a board reporting
  "WiFi ok" while being unreachable (duplicate SSID / isolated AP), I2C
  devices not detected despite being wired, `sensor_t` redefinition when
  mixing Adafruit sensor libs with esp_camera, or serial logs vanishing on
  ESP32-S3 USB CDC. For training the models themselves use
  xiao-edgeimpulse-train / xiao-imu-gesture-train; for board basics read
  xiao-esp32s3 first.
---

# Multi-node MQTT + web dashboard on XIAO ESP32S3 (verified end-to-end)

Everything here was executed on real hardware (2026-07) with three XIAO
ESP32S3 boards — two gesture nodes (BNO055 + BH1750 + BME280) and one vision
node (OV2640) — publishing to a local mosquitto on a laptop, rendered in one
browser page. Working reference implementation: `esp32/day4/`.

The architecture is the easy part. Everything below is the part that cost
hours, written so it can be skipped next time.

## Architecture that works

```
node A (BNO055 → EI gesture)  ─┐
node B (OV2640 → EI vision)   ─┼─ MQTT 1883 ─▶ mosquitto ─ 9001 (WebSocket) ─▶ browser
node C (BNO055 → EI gesture)  ─┘                                                (MQTT.js)
```

Each board joins the same Wi-Fi and publishes to its own topic. There is **no
PC-side gateway program** — the browser speaks MQTT over WebSocket directly to
the broker. Adding a node is one firmware constant plus one line of JS.

mosquitto needs both listeners (2.x binds localhost only without them):

```
listener 1883 0.0.0.0
protocol mqtt
listener 9001 0.0.0.0
protocol websockets
allow_anonymous true
log_dest file C:\ProgramData\mosquitto.log
```

`log_dest stdout` kills the broker when it runs as a Windows service. Use a file.

### One message per node, not one per sensor

Publish raw values and inference in a **single** message, and make the sensor
map self-describing:

```json
{
  "node": "gesture", "kind": "imu", "uptime_s": 42, "rssi": -55,
  "sensors": {
    "lux": {"v": 210.8, "unit": "lx", "label": "조도", "min": 0, "max": 1000}
  },
  "inference": {"label": "running", "confidence": 0.92,
                "scores": {"idle": 0.02, "running": 0.92, "spin": 0.06},
                "dsp_ms": 3, "nn_ms": 1}
}
```

Because unit / label / range travel with the value, the dashboard hardcodes no
sensor keys — add or remove a sensor and the page just renders it. Splitting
into `…/bme280`, `…/bh1750` topics forces the page to know every sensor and is
the thing to avoid.

Node identity must be unique in **three** places: topic suffix, `NODE_ID`, and
MQTT client ID.

## Failure modes, in the order they will bite

### 1. `publish()` returns false, no error anywhere

Symptom: `[MQTT] connected` then every publish fails, payload ~1000 B.

PubSubClient's buffer must hold **payload + topic + header**, so a 1000-byte
payload does not fit in `setBufferSize(1024)`. It fails silently — `publish()`
just returns `false`.

```c
mqtt.setBufferSize(2048);   // not 1024 for a ~1 KB payload
```

Always log the length on failure; that is what makes this diagnosable:

```c
if (!mqtt.publish(TOPIC, j.c_str()))
  Serial.printf("[MQTT] publish failed (len=%u)\n", j.length());
```

### 2. `rc=-2` forever — and it is NOT the broker

`MQTT_CONNECT_FAILED (-2)` means the TCP connect failed, so people blame the
broker, the firewall, or memory. Check the network layer from the PC first:

```bash
ping <board-ip>
```

```bash
arp -a | findstr 192.168.0.
```

If the board's IP does not resolve in ARP while other devices do, **the board
is not on your L2 segment** — regardless of what it printed. In a classroom
there can be two APs with the same SSID: one real router and one isolated
software AP/hotspot whose clients cannot reach anything, not even their own
gateway. A board picks whichever is stronger at boot, so different boards land
on different networks and only some work.

Diagnose with a tiny Wi-Fi-only sketch (no camera, no EI — 1 minute to build)
that prints BSSID / channel / gateway / heap and TCP-probes the broker. See
`day4/93_net_probe/`. Real output from the broken board:

```
SSID: projectbee   BSSID: 1E:DB:D4:76:A3:E4   gateway: 192.168.0.2
TCP 192.168.0.2:80    -> FAIL      <- cannot even reach its own gateway
TCP 192.168.0.27:1883 -> FAIL
free heap: 271064 B                <- so not memory, not power
```

A BSSID with the locally-administered bit set (`1E:`, `x2:`, `xA:`, `xE:` …)
is a software AP, not a router.

**Fix — never hardcode a BSSID.** Verify reachability and move on if it fails:

```c
bool connectVerified() {
  int n = WiFi.scanNetworks();
  // Snapshot candidates FIRST: WiFi.begin() can invalidate scan results, so
  // iterating them while connecting reads garbage from the 2nd entry on.
  Cand cand[MAX_CAND]; int nc = 0;
  for (int i = 0; i < n && nc < MAX_CAND; i++) {
    if (WiFi.SSID(i) != WIFI_SSID) continue;
    memcpy(cand[nc].bssid, WiFi.BSSID(i), 6);
    cand[nc].ch = WiFi.channel(i);
    nc++;
  }
  WiFi.scanDelete();

  for (int i = 0; i < nc; i++) {
    WiFi.begin(WIFI_SSID, WIFI_PASS, cand[i].ch, cand[i].bssid);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(200);
    if (WiFi.status() != WL_CONNECTED) continue;

    WiFiClient probe;                       // the only judgement that matters
    bool ok = probe.connect(MQTT_HOST, MQTT_PORT, 3000);
    probe.stop();
    if (ok) return true;
    WiFi.disconnect(false);                 // true powers Wi-Fi OFF — next try fails
  }
  return false;
}
```

Two traps inside the fix itself: iterating live scan results, and
`WiFi.disconnect(true)` disabling the radio.

### 3. Reconnecting every loop exhausts sockets

`ensureMqtt()` called from `loop()` without a delay opens a socket per failed
attempt until LWIP runs out, after which *everything* returns `rc=-2` — the
first failure becomes permanent.

```c
if (lastMqttTry && millis() - lastMqttTry < 3000) return;
```

Also skip the expensive work (camera grab + inference) while disconnected.

### 4. Silence is ambiguous — add a heartbeat

Firmware that only prints on error cannot be told apart from firmware that has
hung. Five seconds of state costs nothing and answers every question at once:

```c
Serial.printf("[HB] up=%lus wifi=%d rssi=%d ip=%s mqtt=%d(state %d) pub ok=%lu fail=%lu heap=%u\n",
              millis()/1000, WiFi.status(), WiFi.RSSI(),
              WiFi.localIP().toString().c_str(),
              mqtt.connected(), mqtt.state(), pubOk, pubFail, ESP.getFreeHeap());
```

Add this **before** debugging anything else, not after.

### 5. `sensor_t` redefinition when a camera meets Adafruit sensors

`Adafruit_Sensor.h` (pulled in by Adafruit_BME280/BNO055) and esp_camera's
`sensor.h` both typedef `sensor_t`. Any file including both fails:

```
error: conflicting declaration 'typedef struct _sensor sensor_t'
note: previous declaration as 'typedef struct sensor_t sensor_t'
```

Split translation units — a header that leaks no esp_camera types:

```c
// camera.h — .ino sees only this
bool cameraInit();
bool cameraGrab(float *out, int w, int h, float *brightness);
```

`camera.cpp` includes `esp_camera.h`; the `.ino` includes the Adafruit libs.
They never meet. **`.cpp` files need `#include <Arduino.h>` explicitly** — only
`.ino` gets it for free (otherwise `delay` is undeclared).

### 6. I2C devices not found although "everything is wired"

Do not guess. Read the bus electrically, with internal pull-ups only:

| SDA/SCL read | Meaning |
|---|---|
| **HIGH** | nothing attached — clean bus |
| **LOW** | something IS attached but dragging the line down: unpowered slave, or a data pin landing on GND |

Unpowered I2C chips clamp SDA/SCL low through their protection diodes, so
"wired but no power" and "not wired" look completely different. If **several
adjacent pins** read LOW (e.g. D2–D5), a 4-pin module header is plugged into
the data row with VCC/GND landing on GPIOs — on XIAO the power pins are on the
**opposite side** from D4/D5, so a 4-pin header cannot go in one straight row.

Sweep pin pairs too (including swapped SDA/SCL) so a mis-identified pin is
ruled out in one shot. See `day4/94_i2c_hunt/`. Correct result:

```
D4(5) SDA = HIGH   D5(6) SCL = HIGH
[D4/D5 (표준)]  SDA=GPIO5 SCL=GPIO6
    0x23  BH1750     0x29  BNO055     0x76  BME280
```

Initialise every sensor optionally (`hasBNO = bno.begin();`) and omit missing
ones from the payload — the dashboard then simply shows fewer tiles instead of
zeros pretending to be data.

### 7. Serial output disappears on ESP32-S3

Opening the port asserts DTR → the board resets → USB-Serial/JTAG
re-enumerates → the host handle goes stale and everything printed in `setup()`
is lost. Symptom: only `ESP-ROM:esp32s3-...` and nothing else.

Two rules:

- Anything worth reading must be printed from `loop()`, repeatedly.
- To watch already-running firmware, open the port **without** touching DTR/RTS:

```powershell
$p.DtrEnable = $false
$p.RtsEnable = $false
$p.Encoding  = [System.Text.Encoding]::UTF8   # else Korean logs come out as ?
```

Ready-made: `day4/tools/listen_serial.ps1`. day3's `read_serial.ps1` resets the
board by design — use it only when a fresh boot log is what you want.

## Verification order that actually isolates faults

Check these in order; each one rules out everything below it.

1. `arduino-cli board list` — and note the **MAC** printed during upload. Boards
   get moved and COM numbers shuffle; MAC is the only stable identity. Keep a
   table when juggling three or more boards.
2. I2C scan → sensors present?
3. Serial heartbeat → WiFi status, IP, MQTT state, publish counters.
4. `ping <board-ip>` + `arp -a` → is it really on your network?
5. `mosquitto_sub -h 127.0.0.1 -t "wearable/+/#" -v` → is the payload arriving?
6. Only then open the dashboard.

Going straight to step 6 is what turns a 5-minute wiring fault into an hour.

## Practicalities

- Build with PSRAM on: `--fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi`. EI arenas
  go to PSRAM via weak `ei_malloc`/`ei_calloc`/`ei_free` overrides.
- **Never run two EI compiles at once** — they split the CPU and each takes
  ~2× longer. Serialise them.
- The broker dies quietly. Re-check `Get-NetTCPConnection -LocalPort 1883,9001`
  before blaming a board.
- Serving the dashboard over HTTP is required, not optional: browsers block
  WebSocket from `file://` pages. `python -m http.server 8000 --bind 0.0.0.0`
  also makes it reachable from a phone on the same Wi-Fi.
- If the demo network has no internet, vendor `mqtt.min.js` locally — the CDN
  `<script>` tag is a hard dependency that fails the whole page.
- A SoftAP node is isolated by construction: nothing on it can reach a broker
  on another network. Use SoftAP only for a board serving its **own** page
  (`day4/40_vision_ap/`), never as a way onto someone else's broker.
- Two boards can share one broker with zero coordination, but **duplicate MQTT
  client IDs make the broker kick the earlier connection** — the classic
  "my node keeps disconnecting when my teammate connects".
