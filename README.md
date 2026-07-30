# wearable-project — XIAO ESP32S3 센서 노드 모음

Edge Impulse 모델을 올린 XIAO ESP32S3 보드들이 MQTT 로 추론 결과를 발행하고,
브라우저 대시보드가 그걸 한 페이지에서 보여주는 구조. 폴더마다 그 구조를 조금씩
다르게 써먹은 판본이 들어 있다.

```
브라우저 ◀── WebSocket 9001 ── mosquitto ── 1883 ──▶ 보드들 (Wi-Fi)
```

PC 쪽 게이트웨이 프로그램은 없다. 브라우저가 MQTT-over-WebSocket 으로 브로커에
직접 붙는다. 보드를 늘리는 건 펌웨어 상수 하나 + 대시보드 JS 한 줄이다.

## 폴더

| 폴더 | 내용 |
|---|---|
| `alarmi/` | **미션 타이머.** 타이머가 울리면 랜덤 미션 1개(제스처 또는 사물)를 수행해야 꺼진다. 노드 3개(제스처 / 시각 / 라운드 디스플레이) + 상태머신 대시보드 |
| `day4/` | **다중 노드 + 대시보드의 원형.** 노드 2~3개가 원시 센서값과 추론 결과를 한 메시지로 발행. `50_/51_` 은 대시보드에서 노드별로 켜고 끄는 판 |
| `tellin/` | day4 에서 갈라져 나온 실험판 — 제스처 확인 전용 페이지, 알람/오디오 변형, `/jpg` 를 내주는 비전 노드 |

각 폴더의 `README.md` 에 배선·토픽·업로드 방법이 있다. 특히 `day4/README.md` 와
`day4/.claude/skills/xiao-mqtt-multinode/SKILL.md` 에 실제로 시간을 잡아먹었던
함정들(브로커에 못 닿는 동명 AP, `publish()` 가 조용히 false 를 뱉는 버퍼 크기,
Adafruit 센서 라이브러리와 esp_camera 의 `sensor_t` 충돌 등)이 정리돼 있다.

## 공통 환경

| 항목 | 값 |
|---|---|
| 보드 | XIAO ESP32S3 / XIAO ESP32S3 Sense(카메라) |
| FQBN | `esp32:esp32:XIAO_ESP32S3:PSRAM=opi` |
| I2C | SDA = D4(GPIO5), SCL = D5(GPIO6), VCC = **3V3** |
| I2C 주소 | `0x23` BH1750 · `0x29` BNO055 · `0x76` BME280 |
| 브로커 | mosquitto — 1883(보드) + 9001(브라우저 WebSocket) |

Wi-Fi SSID / 브로커 IP 는 각 `.ino` 상단 상수에 하드코딩돼 있다. 망이 바뀌면
거기부터 고쳐야 한다.

## 여기에 없는 것

- **Edge Impulse 모델 라이브러리** (`gesture111_inferencing`, `team_project_inferencing` 등).
  `Documents/Arduino/libraries/` 에 설치하는 것이고 용량이 커서 넣지 않았다.
  EI 스튜디오의 Deployment → Arduino library 로 다시 받으면 된다.
- **학습 데이터셋** (`ei-training/data/` 의 이미지·IMU JSON). 별도 폴더에 있다.
