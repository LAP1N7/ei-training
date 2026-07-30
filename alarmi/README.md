# alarmi — 알라미형 제스처·사물 미션 타이머

**타이머**로 정한 시간 뒤에 울리고, **랜덤 미션 1개**(제스처 후보 + 사물 후보를 한 풀에
합쳐 하나)를 수행해야 꺼진다.
판정은 대시보드 한 곳에서만 하고, 보드들은 추론 결과를 발행하고 `cmd` 를 구독해 모드만 바꾼다.

> 미션을 1개로 줄인 이유: 두 노드가 이제 동시에 돌기 때문에(아래 모드 전환 규칙) 어느 쪽
> 한 곳만 잡아도 되고, 2단계를 강제하면 카메라 앞으로 이동하는 시간이 그대로 벌칙이 됐다.
> 제스처만 / 사물만으로 고정하고 싶으면 대시보드의 **미션 종류** 를 바꾼다.

> 센서 노드 둘은 화면 없이 headless 로 돌고, 화면은 **세 번째 보드**(`30_clock_node`,
> XIAO + Seeed Round Display)가 맡는다. 센서도 판정 로직도 없이 `cmd`/`ui` 를 구독해
> 그리기만 한다.

```
alarmi/
  10_gesture_node/       보드 A — BNO055 제스처 추론 + cmd 구독 (tellin 사본 + 모드 전환)
  20_vision_node/        보드 B — OV2640 사물 추론 + /jpg + cmd 구독
  30_clock_node/         보드 C — 라운드 디스플레이. cmd/ui 구독 → 화면만 그림 (센서 없음)
  dashboard/
    index.html           타이머(분·초) · 상태머신 · 랜덤 미션 1개 · 판정 · 소리 ·
                         카메라(그림 위에 추론 오버레이, day3 03_web_infer_sta 구성)
    mosquitto-websockets.conf
  tools/
    img_collect.py       /jpg 폴링 → dataset/<label>/ 저장 → Edge Impulse 업로드
    collect_images.bat   위를 여는 런처 (더블클릭)
    imu_collect.py       day3 것 사본 — 제스처 재수집이 필요할 때만
  dataset/               mouse / cup / phonecase / unknown
  .env.example           EI_API_KEY=  (복사해서 .env 로)
```

## 1. 프로토콜

| 토픽 | 방향 | retain | 용도 |
|---|---|---|---|
| `wearable/minseo/gesture` | A → 브라우저 | no | 추론 결과 (250 ms) |
| `wearable/minseo/vision`  | B → 브라우저 | no | 추론 결과 (700 ms) |
| `wearable/minseo/cmd`     | 브라우저 → A,B | **yes** | 상태 전이 때만 |
| `wearable/minseo/ui`      | 브라우저 → 시계 | no | 미션 중 진행률 5 Hz `{"held_ms":1240,"need_ms":3000}` |

`cmd` 페이로드 (`type:"alarm"` 만 처리, 모르는 type 은 무시):

```json
{"type":"alarm","state":"armed","at_epoch":1785000000,"remain_s":30}
{"type":"alarm","state":"ringing"}
{"type":"alarm","state":"mission","step":1,"total":1,"kind":"gesture","label":"shaking","icon":"swing","hold_s":3}
{"type":"alarm","state":"fail","label":"shaking","next":"spin"}
{"type":"alarm","state":"success"}
{"type":"alarm","state":"shutdown"}
{"type":"alarm","state":"idle"}
```

`remain_s` 와 `fail` 은 시계 노드(`30_clock_node`)를 위해 나중에 추가된 것이다.

- **`remain_s`** — 시계 보드는 NTP 를 안 붙이므로 `at_epoch`(절대 시각)만으로는 남은 시간을
  계산할 수 없다. 남은 초를 상대값으로 같이 줘서 보드가 자기 `millis()` 로 세게 한다.
  시각 동기화 실패라는 고장 원인을 통째로 없앤다.
- **`fail`** — `failMission()` 은 라벨만 새로 뽑아 `mission` 을 다시 보내므로, 이 신호가
  없으면 시계는 "라벨이 갑자기 바뀐 미션"만 보고 실패한 줄 모른다.
  **retain 하지 않는다** — 지나간 실패가 나중에 부팅한 보드에서 현재 상태로 되살아난다.
  센서 노드들의 `onCmd` 는 mission/success/shutdown 외에는 "계속 발행"으로 처리하므로
  이 상태를 몰라도 안전하다.

**모드 전환 규칙** (양쪽 펌웨어의 `onCmd`):
`idle`/`armed`/`ringing`/`mission` → **둘 다 발행**. `success`/`shutdown` → 둘 다 침묵.

> 예전에는 `mission`+`kind:gesture` 면 A 만, `kind:object` 면 B 만 발행하고 나머지 한쪽은
> 침묵했다. 그러면 미션이 도는 동안 다른 노드 카드가 통째로 죽어서, 한 보드를 쓰는 사이
> 같이 붙어 있는 다른 보드를 볼 수도 쓸 수도 없었다. 판정은 대시보드가 노드 단위로 거르므로
> (`index.html` 의 `feed()`: `if(nodeId !== m.node) return;`) 두 노드가 동시에 떠들어도
> 엉뚱한 노드가 미션을 통과시키지는 않는다. 그래서 기본을 "둘 다 동작"으로 바꿨다.
> 예전 동작이 필요하면 양쪽 `.ino` 의 `#define EXCLUSIVE_MISSION` 을 `1` 로 두고 재컴파일.
B 의 `/jpg` 웹서버는 플래그와 무관하게 항상 돈다 (카메라 뷰 + 수집기가 쓴다).
진행률(`ui`)은 **retain 금지** — retained 로 보내면 늦게 부팅한 시계가 죽은 게이지를 복원한다.
`shutdown` retained 유령 방지: 대시보드의 "다시 알람 걸기 / 강제 정지"가 `idle` 로 덮어쓴다.

## 2. 미션 흐름 (대시보드 상태머신)

```
IDLE → ARMED(카운트다운) → RINGING → MISSION → SUCCESS → SHUTDOWN
                                        │ 제한 시간(기본 30 s) 초과 = 실패
                                        └→ 라벨 재추첨 후 재시도 (실패음 + 화면 플래시)
```

- 미션은 RINGING 진입 때 **한 개만** 뽑는다. 제스처 후보 기본 `shaking,spin`,
  사물 후보 기본 `mouse,cup,phonecase` — `idle`/`unknown` 은 자동 제외.
  **미션 종류** 를 `제스처만`/`사물만` 으로 두면 그쪽 풀에서만 뽑는다.
- 통과 조건: 해당 노드의 라벨이 임계값(기본 0.7) 이상으로 **누적 3초 유지**
  (프레임 수가 아니라 시간, 끊김 허용 600 ms).
- SUCCESS 는 성공음 + N초(기본 5) 카운트다운 후 `shutdown` 발행 → 종료 오버레이.
  deep sleep 이 아니라서 "다시 알람 걸기"를 누르면 그대로 복귀한다.

## 3. 실행 순서

1. 브로커 (관리자 PowerShell):
   ```powershell
   net stop mosquitto
   & "C:\Program Files\mosquitto\mosquitto.exe" -c "C:\Users\minseo\Desktop\alarmi\dashboard\mosquitto-websockets.conf" -v
   ```
2. 보드 업로드 (라이브러리를 갈아끼운 뒤 첫 컴파일은 `--clean` 필수):
   ```
   arduino-cli compile --clean --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 10_gesture_node
   arduino-cli upload -p COMx --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 10_gesture_node
   arduino-cli compile --clean --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 20_vision_node
   arduino-cli upload -p COMy --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 20_vision_node
   ```
3. `dashboard/index.html` 열기 → broker IP/id 확인 → **연결** → 비전 보드 시리얼의
   `[HTTP] http://…/jpg` IP 를 카메라 칸에 넣고 **켜기** → 두 노드 카드가 사는지 확인.
   카메라를 켜면 그림 위에 **추론 결과가 겹쳐서** 나온다 (라벨 색 테두리 · 좌상단
   라벨/신뢰도 태그 · 클래스별 바 · dsp/nn 지연) — day3 `03_web_infer_sta` 가 보드에서
   서빙하던 그 화면과 같은 구성이다. 다른 건 값의 출처뿐: 그쪽은 `fetch('/classify')`
   폴링, 여기는 이미 구독 중인 `vision` 토픽. 그림만 `/jpg` 로 직접 당겨온다
   (JPEG 를 MQTT 로 실으면 브로커 버퍼가 터진다).
   (브라우저는 `file://` 페이지에서 WebSocket 을 막으므로 HTTP 로 서빙해야 한다:
   `python -m http.server 8000 --bind 0.0.0.0` 를 `dashboard/` 에서.)
4. 타이머 길이(분·초)를 넣고 **시작**. 리허설은 **10초 테스트** 버튼.
5. 보드 없이 전체 흐름 확인: 연결만 하고 미션 중 **미션 통과 흉내** 버튼으로
   IDLE→…→SHUTDOWN 을 끝까지 돌려본다.

## 4. 사물 데이터 수집 → Edge Impulse 학습

1. `cp .env.example .env` 후 EI API 키 기입.
2. `tools\collect_images.bat` 더블클릭 (보드 IP 는 환경변수 `IMG_IP`, 기본 192.168.0.30).
   메뉴에서 `1=mouse 2=cup 3=phone 4=unknown` → 연사 수(기본 10) → 각도·거리를 바꿔가며 반복.
   네트워크가 막혔으면 `python tools\img_collect.py --no-upload mouse` 로 로컬만 저장.
3. 클래스당 100장 이상, **unknown 필수** (없으면 빈 책상도 마우스로 찍힌다).
   실제 사용할 조명/배경에서 수집할 것.
4. EI 웹: Impulse = Image **96×96 · RGB** · Transfer Learning (MobileNetV2 0.35).
   `camera.cpp` 가 픽셀당 packed RGB 를 float 하나로 만들고(`(r<<16)|(g<<8)|b`),
   모델 입력은 `96*96*3 = 27648` 이다. 여기서 어긋나면 추론이 통째로 망가진다.
5. Deployment → **Arduino library** → `Documents/Arduino/libraries/` 에 해제 후
   `20_vision_node.ino` 의 `#include` 를 새 헤더로 교체.
   **현재는 `team_project_inferencing`(cup/mouse/phonecase) 을 쓴다.**
   예전 `test_inferencing` 은 band/usb 라서 미션 라벨과 전혀 맞지 않았다.
   라이브러리를 갈아끼운 뒤 첫 빌드는 반드시 `--clean`.
6. ⚠️ **검증된 함정** (day3 `xiao-edgeimpulse-train/SKILL.md` Step 5): 새 EI 라이브러리에서
   `EI_MAX_OVERFLOW_BUFFER_COUNT` panic + Guru Meditation 이 나면 모델이 큰 게 아니다.
   라이브러리의 `ei_classifier_porting.h` ESP32-S3 분기가 이 값을 30 으로 하드코딩하고
   있어서 **2048 로 고치고 `--clean` 재빌드**해야 한다 (추론 6.4 s → 0.085 s 였던 그 건).

제스처 모델은 재학습 불필요 — 기존 `gesture111_inferencing`(idle/**shaking**/spin) 그대로.
대시보드 후보 입력도 `shaking,spin` 이 기본값이다 (라벨명이 `running` 이 아니라 `shaking`).

사물 라벨은 `cup` / `mouse` / `phonecase` 다. 대시보드 기본값이 한동안 `phone` 이었는데
실제 라벨은 `phonecase` 라, 그 미션은 영원히 통과되지 않았다. 후보 입력은 **모델이 실제로
내보내는 문자열과 정확히 같아야 한다** — 로그의 `라벨 발견:` 줄로 확인할 수 있다.

## 5. 라운드 디스플레이 (`30_clock_node`)

XIAO 위에 Seeed Round Display(240×240)를 그대로 꽂은 **세 번째 보드**. 센서도 모델도 없고
`cmd`/`ui` 만 구독해서 상태별 화면을 그린다.

| cmd state | 화면 |
|---|---|
| `idle` | ALARMI / STANDBY + 도는 점(살아있음 표시) |
| `armed` | 남은 시간 `mm:ss` + 줄어드는 링 (`remain_s` 를 받아 보드가 센다) |
| `ringing` | WAKE UP, 테두리 빨강 점멸 |
| `mission` | **kind 별 아이콘** + 라벨 + 해제 게이지 링(`ui` 5 Hz) |
| `fail` | X 표시 1.2 초 붙잡기 → 새 미션 화면 |
| `success` | 체크 + CLEAR! |
| `shutdown` | DONE / GOOD NIGHT |

아이콘은 `cmd` 의 `icon` 필드로 갈린다: `swing`(팔 흔들기·애니메이션) · `spin`(도는 호) ·
`mouse` · `cup` · `phone`. 모르는 이름이면 `?` 를 그려 화면이 비지 않게 한다.

- **아이콘을 JPEG 로 안 넣은 이유**: `TFT_eSPI_Clock_ex2` 의 `jpeg1.h` 하나가 108 KB 다.
  도형 몇 개면 되는 그림에 그만한 플래시를 쓸 이유가 없고, 벡터라 상태에 따라 색과
  각도를 바꿀 수 있다 (`swing` 은 팔이 실제로 흔들린다).
- **화면 문구는 전부 영문**이다. TFT_eSPI 내장 폰트에는 한글 글리프가 없어서 한글을 넣으면
  빈칸으로 나온다 (참조한 예제의 `NotoSansBold15` 도 라틴 전용). 한글이 필요하면 한글 TTF 를
  폰트 변환기로 `.h` 로 만들어 `loadFont()` 해야 하는데 글리프 수만큼 플래시를 먹는다.
  미션 라벨(`shaking`/`mouse`…)은 원래 영문이라 지금은 문제가 없다.
- **깜빡임 방지**: 240×240 스프라이트에 다 그린 뒤 한 번에 `pushSprite`. 16 bpp = 115 KB 라
  못 잡으면 8 bpp 로 물러선다 (참조 예제와 같은 처리).
- 프레임은 20 fps. `ui` 가 2 초 넘게 안 오면 게이지 링을 안 믿고 지운다 —
  멈춘 게이지가 차 있는 것처럼 보이면 안 되기 때문.
- 브로커가 끊기면 화면 아래에 `offline` 이 뜬다. 화면만 보고도 원인을 안다.

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 30_clock_node
```

```bash
arduino-cli upload -p COMz --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 30_clock_node
```

TFT_eSPI 핀 설정(`User_Setup.h`)은 라운드 디스플레이용으로 이미 잡혀 있어야 한다 —
`TFT_eSPI_Clock_ex2` 가 도는 환경이면 그대로 컴파일된다.

보드 없이 화면 전환만 확인하려면 `mosquitto_pub` 으로 상태를 직접 던져본다:

```bash
"C:\Program Files\mosquitto\mosquitto_pub.exe" -h 192.168.0.27 -t wearable/minseo/cmd -r -m "{\"type\":\"alarm\",\"state\":\"mission\",\"kind\":\"gesture\",\"label\":\"shaking\",\"icon\":\"swing\",\"hold_s\":3}"
```

## 6. 최종 리허설 체크리스트

1. `mosquitto_sub -t 'wearable/minseo/#' -v` 를 옆에 띄워둔다.
2. **10초 테스트** → RINGING 에서 `cmd` 에 `ringing` retained 가 찍히는지.
3. 미션: 지시된 라벨 3초 유지 → 게이지 차고 통과. `ui` 가 5 Hz 로 흐르는지.
4. 미션을 일부러 제한 시간까지 방치 → 실패음 + 라벨 재추첨 + 재시도되는지.
5. 미션이 사물일 때 **gesture 토픽도 같이 흐르는지**(두 노드 동시 동작).
   그래도 게이지는 vision 라벨로만 차야 정상. 반대도 마찬가지.
6. SUCCESS 5초 → `shutdown` → 두 노드 발행이 멎는지 (`/jpg` 는 계속 살아 있어야 정상).
7. "다시 시작" → `idle` retained 로 덮이고 두 노드 발행이 재개되는지.
