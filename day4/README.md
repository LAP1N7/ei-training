# day4 — 센서노드 2개 + MQTT + 웹 대시보드

```
[노드1 제스처]  BNO055 → EI 추론(idle/running/spin)          ─┐
                BH1750(조도) · BME280(온·습·기압) 원시값      │  MQTT 1883
                                                              ├─▶ mosquitto ──▶ 9001(WebSocket) ──▶ 브라우저 대시보드
[노드2 시각]    OV2640 → EI 추론(band/usb)                    │                                      (MQTT.js 로 두 토픽 동시 구독)
                BH1750 · BME280 원시값 (있으면 자동 인식)    ─┘
```

두 노드가 각자 Wi-Fi 로 브로커에 직접 발행한다. 게이트웨이 프로그램은 따로 없고,
브라우저가 WebSocket 으로 브로커에 직접 붙는다.

## 토픽과 페이로드

| 토픽 | 발행자 | 주기 |
|---|---|---|
| `wearable/minseo/gesture` | 노드1 | 250 ms (4 Hz) |
| `wearable/minseo/vision`  | 노드2 | 700 ms |

두 노드가 **같은 스키마**로 보낸다. 원시 센서값과 추론 결과가 한 메시지에 들어 있다.

```json
{
  "node": "gesture", "kind": "imu", "uptime_s": 42, "rssi": -55,
  "sensors": {
    "lux":  {"v": 210.8, "unit": "lx",   "label": "조도", "min": 0, "max": 1000},
    "temp": {"v": 26.4,  "unit": "°C",   "label": "온도", "min": 0, "max": 50}
  },
  "inference": {
    "label": "running", "confidence": 0.92,
    "scores": {"idle": 0.02, "running": 0.92, "spin": 0.06},
    "dsp_ms": 3, "nn_ms": 1
  }
}
```

`sensors` 는 **키를 하드코딩하지 않는다.** 값·단위·표시이름·그래프 범위를 노드가 같이 실어
보내므로, 센서를 추가하거나 빼도 대시보드는 그대로 그린다.

## 구성

| 항목 | 값 |
|---|---|
| Wi-Fi | `projectbee` / `honeybear!` |
| 브로커 | `192.168.0.27` (내 노트북 mosquitto) |
| 포트 | 1883 = 보드, 9001 = 브라우저 WebSocket |
| I2C | SDA = D4(GPIO5), SCL = D5(GPIO6), VCC = 3V3 |

I2C 주소: `0x23` BH1750 · `0x29` BNO055 · `0x76` BME280
(`../I2cScan` 으로 확인 가능)

## 실행

1. 브로커 (이미 떠 있으면 생략). WebSocket 리스너가 필요하므로 설정 파일을 지정해서 띄운다:

```bash
net stop mosquitto
"C:\Program Files\mosquitto\mosquitto.exe" -c "C:\Users\minseo\Desktop\wearable\esp32\dashboard\mosquitto-websockets.conf" -v
```

2. 노드1 (제스처) 업로드:

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi "C:\Users\minseo\Desktop\wearable\esp32\day4\10_gesture_node"
```

```bash
arduino-cli upload -p COM6 --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi "C:\Users\minseo\Desktop\wearable\esp32\day4\10_gesture_node"
```

3. 노드2 (시각) 업로드 — 카메라 달린 보드의 포트로:

```bash
arduino-cli upload -p COM7 --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi "C:\Users\minseo\Desktop\wearable\esp32\day4\20_vision_node"
```

4. `dashboard/index.html` 을 브라우저로 열고 **연결** 클릭.
   (broker `192.168.0.27`, port `9001`, id `minseo`)

## 폴더 구성

| 경로 | 내용 |
|---|---|
| `10_gesture_node/` | 노드 1 — BNO055 제스처 추론 + BH1750·BME280 원시값 → `…/gesture` |
| `20_vision_node/` | 노드 2 — OV2640 추론 + 원시값 → `…/vision`. 카메라는 `camera.cpp` 로 분리(헤더 충돌 회피) |
| `30_gesture2_node/` | 노드 3 — 노드 1 과 동일, 토픽/ID 만 다름 → `…/gesture2` |
| `40_vision_ap/` | 시각 노드 SoftAP 단독판. 브로커 없이 `http://192.168.4.1` 에서 자체 페이지 서빙 |
| `50_gesture_ctrl/` | 노드 1 의 **원격 on/off 판** — 카메라 없는 보드(non-Sense)용 |
| `51_vision_ctrl/` | 노드 2 의 **원격 on/off 판** — 카메라 달린 보드(Sense)용 |
| `93_net_probe/` | 진단 — 붙은 AP(BSSID/게이트웨이)와 브로커 TCP 도달성 확인 |
| `94_i2c_hunt/` | 진단 — 버스 전기 상태(HIGH/LOW) + 핀 조합 전수 스캔 |
| `dashboard/index.html` | MQTT.js WebSocket 대시보드 (보기 전용) |
| `dashboard/control.html` | 같은 대시보드 + **노드별 시작/정지·주기 조절** (50/51 펌웨어와 짝) |
| `tools/listen_serial.ps1` | 보드를 리셋하지 않고 시리얼만 듣기 |
| `.claude/skills/xiao-mqtt-multinode/` | 이번 작업에서 얻은 함정과 해결법 정리 |

## 보드 2개를 대시보드에서 각각 켜고 끄기 (50 / 51 + control.html)

USB 로 두 보드에 동시에 전원을 주고, 대시보드에서 원하는 노드만 돌리는 구성.
**카메라 있는 보드(XIAO ESP32S3 Sense) = 시각**, **카메라 없는 보드 = 제스처** 로 굽는다.

```
브라우저 ──cmd/gesture──▶ mosquitto ──▶ non-Sense 보드 (BNO055)  ──gesture──┐
        ──cmd/vision ──▶            ──▶ Sense 보드   (OV2640)   ──vision ──┴─▶ 브라우저
```

지금까지는 통신이 보드 → 브로커 한 방향이라 노드를 멈추려면 USB 를 뽑아야 했다.
여기서는 보드가 명령 토픽을 **구독**해서 양방향이 된다.

| 토픽 | 방향 | 페이로드 |
|---|---|---|
| `wearable/minseo/cmd/gesture` · `…/cmd/vision` | 대시보드 → 보드 | `{"active":true,"rate_ms":250}` (retained) |
| `wearable/minseo/status/gesture` · `…/status/vision` | 보드 → 대시보드 | `online` / `offline` (LWT, retained) |
| `wearable/minseo/gesture` · `…/vision` | 보드 → 대시보드 | 기존 스키마 + `active`, `rate_ms` 추가 |

- 명령은 **retained** 로 발행한다. 보드를 리셋하거나 나중에 켜도 구독하는 순간
  마지막 명령이 되돌아오므로 켜짐/꺼짐 상태가 그대로 이어진다.
- **정지 상태에서도 원시 센서값은 1 초에 한 번 계속 올린다.** 그래야 "전원은 들어와
  있는데 추론만 멈춘 것"과 "보드가 죽은 것"이 대시보드에서 구분된다.
  건너뛰는 건 무거운 쪽 — 시각 노드는 카메라 캡처 + 추론을 통째로 생략한다.
- 제스처 노드는 꺼져 있어도 100 Hz 링버퍼를 계속 채운다. 다시 켰을 때 2 초를
  기다리지 않고 바로 제대로 된 추론이 나온다.
- 클라이언트 ID 가 `-ctrl` 로 끝나므로 기존 `10_`/`20_` 펌웨어와 ID 가 겹치지 않는다
  (같은 ID 두 개면 브로커가 먼저 붙은 쪽을 끊어버린다). 단 **토픽은 같으므로**
  옛 펌웨어와 새 펌웨어를 동시에 켜지는 말 것.

업로드 — 카메라 없는 보드와 있는 보드의 COM 을 각각 확인해서:

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi "C:\Users\minseo\Desktop\wearable\esp32\day4\50_gesture_ctrl"
```

```bash
arduino-cli upload -p COM6 --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi "C:\Users\minseo\Desktop\wearable\esp32\day4\50_gesture_ctrl"
```

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi "C:\Users\minseo\Desktop\wearable\esp32\day4\51_vision_ctrl"
```

```bash
arduino-cli upload -p COM7 --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi "C:\Users\minseo\Desktop\wearable\esp32\day4\51_vision_ctrl"
```

(EI 컴파일 두 개를 동시에 돌리지 말 것 — CPU 를 나눠 써서 각각 2배 느려진다.)

그리고 `serve_dashboard.bat` 으로 띄운 뒤 `control.html` 을 열어 **연결** → 카드마다
`시작` / `정지`, 주기(ms) 조절. 브라우저 없이 손으로 때려보려면:

```bash
"C:\Program Files\mosquitto\mosquitto_pub.exe" -h 192.168.0.27 -t wearable/minseo/cmd/vision -r -m off
```

retained 명령을 지우려면 빈 메시지를 retained 로 보낸다 (`-r -n`).

## 확인 / 문제 해결

토픽이 실제로 들어오는지 브로커에서 직접 보기:

```bash
"C:\Program Files\mosquitto\mosquitto_sub.exe" -h 192.168.0.27 -t "wearable/minseo/#" -v
```

- 대시보드 카드가 흐리게 남아 있음 → 4 초 넘게 메시지가 없다는 뜻. 위 `mosquitto_sub` 로 발행 자체를 먼저 확인.
- 보드 시리얼에 `[MQTT] connect failed rc=-2` → 브로커 IP/방화벽. `allow_firewall.bat` 참고.
- `[MQTT] publish failed (len=1002)` → 페이로드가 버퍼보다 큼. 버퍼는 페이로드 + 토픽 + 헤더를 다 담아야 하므로 `len` 보다 넉넉해야 한다 (`mqtt.setBufferSize(2048)`).
- 시리얼 로그가 안 보임 → 포트를 열면 보드가 리셋되므로, `setup()` 에서만 찍는 로그는 USB CDC 재연결 전에 날아간다. `loop()` 에서 반복 출력되는 값으로 확인할 것.
