// alarmi — CLOCK NODE (라운드 디스플레이 상태 노드)
// Hardware: XIAO ESP32S3 + Seeed Round Display (GC9A01, 240x240)
// FQBN: esp32:esp32:XIAO_ESP32S3:PSRAM=opi
// Reference: TFT_eSPI_Clock_ex2 (Sprite double buffering & 8bpp fallback)
// GitHub: https://github.com/kimraewon04/ei-training-main
//
// 화면 연동 사양:
//   - 여백 공간: back.png (back.c) 의 올리브 그린 배경색(0x7426)으로 채움
//   - start 화면: 글자("STANDBY", "ALARMI") 및 이동 점 애니메이션 제거 -> 순수 start.c 이미지 렌더링
//   - timer 화면: 중앙에 남은 시간(00:00)만 표출
//   - mission 화면: 지정/랜덤 미션 이미지 (cup.c, mouse.c, phone.c, shake.c, circle.c) + 진행 링
//   - success / fail 화면: goodjob.c (3초) / retry.c (5초) 순수 이미지 렌더링

// [+] 사물 미션: 미션 이미지를 3 초 보여준 뒤, 라운드 디스플레이가 비전 노드의
//     카메라 화면으로 바뀐다. 카메라는 그때만 켠다 (그 외에는 기존 JPEG 화면 전환 그대로).
//     제스처 미션은 손대지 않았다 — shake.c / circle.c 가 끝까지 그대로 뜬다.
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <JPEGDecoder.h>
#include "esp_heap_caps.h"
#include "images.h"

// ---- Network & MQTT Config ----
const char *WIFI_SSID  = "projectbee";
const char *WIFI_PASS  = "honeybear!";
const char *MQTT_HOST  = "192.168.0.27";
const int   MQTT_PORT  = 1883;
const char *CMD_TOPIC  = "wearable/minseo/cmd";
const char *UI_TOPIC   = "wearable/minseo/ui";
const char *VIS_TOPIC  = "wearable/minseo/vision";   // [+] 비전 노드 IP 를 여기서 알아낸다
const char *MQTT_ID    = "xiao-minseo-node-clock";   // 노드 고유 ID

// [+] 카메라 뷰파인더. 카메라는 이 보드에 없고 비전 노드에 있으므로
// http://<비전노드IP>/jpg 를 받아 그린다 (240x240 JPEG, 실측 4.5 KB / 0.5 초).
#define CAM_INTRO_MS   3000                // 미션 이미지를 보여주는 시간, 그 뒤 카메라로 전환
#define CAM_FETCH_MS   400                 // HTTP GET 이 블로킹이라 매 프레임 받으면 MQTT 가 굶는다
#define CAM_TIMEOUT_MS 1500
#define CAM_MAX_BYTES  32768               // 240x240 JPEG 이 이보다 커질 일은 없다
#define CAM_STALE_MS   2000                // 이보다 낡은 프레임은 실시간이 아니므로 안 그린다

#define SCR            240                 // 240x240 원형 디스플레이
#define CX             (SCR / 2)
#define CY             (SCR / 2)
#define FAIL_HOLD_MS   5000                // 실패 화면 (retry.c) 5초 지연 표시
#define SUCCESS_HOLD_MS 3000               // 성공 화면 (goodjob.c) 3초 표시 후 종료
#define FRAME_MS       50                  // 20 fps (50ms)

#define minimum(a, b)  (((a) < (b)) ? (a) : (b))

// ---- Color System ----
// back.png 배경색: RGB(119, 135, 53) -> RGB565: 0x7426
#define C_BG       0x7426              // Olive Green (back.c color)
#define C_DIM      0x53E9              // Darker Olive/Gray
#define C_TXT      TFT_WHITE
#define C_ACCENT   0x455F              // Cyan Blue
#define C_OK       0x3606              // Emerald Green
#define C_WARN     0xFDA6              // Amber Orange
#define C_FAIL     0xF9E7              // Vivid Red

TFT_eSPI  tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);   // 더블버퍼 스프라이트

WiFiClient net;
PubSubClient mqtt(net);

// ---- State Machine Variables ----
enum St { S_IDLE, S_ARMED, S_RINGING, S_MISSION, S_FAIL, S_SUCCESS, S_SHUTDOWN };
static St       st          = S_IDLE;
static char     mLabel[32]  = "";       // 미션 라벨 (shaking / mouse / …)
static char     mIcon[16]   = "";       // 아이콘 이름 (swing / spin / mouse / cup / phone)
static char     mKind[16]   = "";       // gesture | object
static uint32_t armedMs     = 0;        // ARMED 진입 시각 (millis)
static uint32_t armedTotal  = 0;        // 카운트다운 총 ms
static uint32_t holdUntil   = 0;        // 특수 화면(retry 5s / goodjob 3s) 유효 완료 시각
static St       pendingSt   = S_IDLE;   // 지연 화면 종료 후 전환할 상태
static uint32_t heldMs      = 0, needMs = 3000;   // ui 진행 링
static uint32_t lastUi      = 0;        // ui 타임아웃 (2초 초과 시 링 감춤)
static uint32_t lastFrame   = 0, lastHb = 0;
static uint32_t missionAt   = 0;        // [+] MISSION 진입 시각 — 3초 인트로 계산용

// ---- [+] 사물 미션용 카메라 뷰파인더 ----
static char     visionIp[20] = "";      // 비전 노드가 MQTT 로 알려준 IP (DHCP 라 바뀐다)
static uint8_t *camBuf   = nullptr;     // JPEG 원본 (PSRAM)
static size_t   camLen   = 0;
static uint32_t camAt    = 0;           // 마지막으로 프레임을 받은 시각
static uint32_t camFetch = 0;           // 마지막 시도 시각
static uint32_t camOk = 0, camFail = 0;

// 사물 미션인가 (제스처면 카메라를 아예 안 건드린다)
static inline bool isObjectMission() { return strcmp(mKind, "gesture") != 0; }

// 인트로 3초가 지났고 카메라로 전환할 시점인가
static inline bool camPhase(uint32_t now) {
  return st == S_MISSION && isObjectMission() && (now - missionAt) >= CAM_INTRO_MS;
}

// ---- JPEG Decoder to Sprite Helper ----
static void renderJPEGToSprite(TFT_eSprite *targetSpr, int xpos, int ypos) {
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
  uint32_t min_h = minimum(mcu_h, max_y % mcu_h);
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  max_x += xpos;
  max_y += ypos;

  targetSpr->setSwapBytes(true);

  while (JpegDec.read()) {
    pImg = JpegDec.pImage;
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;

    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    if (win_w != mcu_w) {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int hh = 1; hh < (int)win_h; hh++) {
        p += mcu_w;
        for (int w = 0; w < (int)win_w; w++) {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }

    if ((mcu_x + (int)win_w) <= targetSpr->width() && (mcu_y + (int)win_h) <= targetSpr->height()) {
      targetSpr->pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    } else if ((mcu_y + (int)win_h) >= targetSpr->height()) {
      JpegDec.abort();
    }
  }
  targetSpr->setSwapBytes(false);
}

static void loadBackgroundJpeg(const uint8_t arrayname[], uint32_t array_size) {
  JpegDec.decodeArray(arrayname, array_size);
  int x = ((int)spr.width()  - (int)JpegDec.width)  / 2;
  int y = ((int)spr.height() - (int)JpegDec.height) / 2;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  renderJPEGToSprite(&spr, x, y);
}

// ---- [+] 카메라 프레임 ----
// 비전 노드의 /jpg 를 한 프레임 받아 camBuf 에 담는다. 블로킹이므로 호출 빈도는
// 호출자가 CAM_FETCH_MS 로 제한한다.
static bool camFetchFrame() {
  if (!camBuf || !visionIp[0] || WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  char url[64];
  snprintf(url, sizeof(url), "http://%s/jpg", visionIp);
  http.setConnectTimeout(CAM_TIMEOUT_MS);
  http.setTimeout(CAM_TIMEOUT_MS);
  if (!http.begin(url)) { camFail++; return false; }

  bool ok = false;
  if (http.GET() == HTTP_CODE_OK) {
    int len = http.getSize();
    size_t cap = (len > 0 && (size_t)len < CAM_MAX_BYTES) ? (size_t)len : CAM_MAX_BYTES;
    size_t got = http.getStream().readBytes(camBuf, cap);
    // JPEG 인지 확인하고 받는다 — 오류 페이지를 디코더에 넘기면 조용히 깨진 화면이 된다.
    if (got > 4 && camBuf[0] == 0xFF && camBuf[1] == 0xD8) {
      camLen = got; camAt = millis(); camOk++; ok = true;
    } else camFail++;
  } else camFail++;
  http.end();
  return ok;
}

// camBuf 의 JPEG 를 화면에 깐다. 기존 loadBackgroundJpeg 과 같은 경로를 쓴다.
static void camDrawFrame() {
  if (!camLen) return;
  if (!JpegDec.decodeArray(camBuf, camLen)) return;
  int x = ((int)spr.width()  - (int)JpegDec.width)  / 2;
  int y = ((int)spr.height() - (int)JpegDec.height) / 2;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  renderJPEGToSprite(&spr, x, y);
}

// ---- Lightweight JSON Parsers ----
static bool jsonStr(const char *buf, const char *key, char *out, size_t n) {
  char pat[24];
  snprintf(pat, sizeof(pat), "\"%s\":\"", key);
  const char *p = strstr(buf, pat);
  if (!p) return false;
  p += strlen(pat);
  const char *e = strchr(p, '"');
  if (!e) return false;
  size_t len = (size_t)(e - p);
  if (len >= n) len = n - 1;
  memcpy(out, p, len);
  out[len] = 0;
  return true;
}

static bool jsonNum(const char *buf, const char *key, long *out) {
  char pat[24];
  snprintf(pat, sizeof(pat), "\"%s\":", key);
  const char *p = strstr(buf, pat);
  if (!p) return false;
  *out = atol(p + strlen(pat));
  return true;
}

// ---- Text Helper (English only, safe scaling) ----
static void centerText(const char *s, int y, uint16_t c, int font, int size = 1) {
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(c);
  spr.setTextSize(size);
  spr.drawString(s, CX, y, font);
  spr.setTextSize(1);
}

// Countdown time using Font 4 (scaled x2) centered on screen
static void bigTime(uint32_t ms, int y, uint16_t c) {
  uint32_t s = (ms + 999) / 1000;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)(s / 60), (unsigned)(s % 60));
  centerText(buf, y, c, 4, 2);
}

// ---- Render Screens ----
// 1. Idle (Power On / Standby) -> start.c ONLY (No text, no dot)
static void screenIdle(uint32_t now) {
  loadBackgroundJpeg(start_map, 9093);
}

// 2. Armed (Alarm Countdown) -> timer.c + Center mm:ss
static void screenArmed(uint32_t now) {
  loadBackgroundJpeg(timer_map, 12140);
  uint32_t el = now - armedMs;
  uint32_t left = (el >= armedTotal) ? 0 : armedTotal - el;
  int deg = armedTotal ? (int)(360.0f * left / armedTotal) : 0;
  
  spr.drawSmoothArc(CX, CY, 114, 106, 0, 359, C_DIM, C_BG);
  if (deg > 0) spr.drawSmoothArc(CX, CY, 114, 106, 0, deg, C_ACCENT, C_BG, true);
  
  bigTime(left, CY, C_TXT);
}

// 3. Ringing (Alarm Firing) -> timer.c + Flash Border
static void screenRinging(uint32_t now) {
  loadBackgroundJpeg(timer_map, 12140);
  bool on = (now / 350) % 2;
  spr.drawSmoothArc(CX, CY, 114, 106, 0, 359, on ? C_FAIL : C_DIM, C_BG);
}

// 4. Mission (Random/Assigned Image + Progress Ring)
static void screenMission(uint32_t now) {
  // [+] 사물 미션은 미션 이미지를 CAM_INTRO_MS 동안 보여준 뒤 카메라 화면으로 넘어간다.
  // 프레임이 낡았으면(비전 노드 미응답) 넘어가지 않고 미션 이미지를 유지한다 — 멈춘
  // 그림을 실시간인 척 보여주면 화면만 보고 고장을 알 수 없다.
  if (camPhase(now) && camLen && (now - camAt) < CAM_STALE_MS) {
    camDrawFrame();
    // 해제 게이지는 카메라 위에도 그대로 올린다 (아래 공통 경로와 같은 링)
    spr.drawSmoothArc(CX, CY, 116, 108, 0, 359, C_DIM, C_BG);
    uint32_t need = needMs ? needMs : 1;
    int deg = (int)(360.0f * (heldMs > need ? need : heldMs) / need);
    if ((now - lastUi) < 2000 && deg > 0)
      spr.drawSmoothArc(CX, CY, 116, 108, 0, deg, deg >= 359 ? C_OK : C_WARN, C_BG, true);
    return;
  }

  // Load background JPEG matching icon/label
  if (!strcmp(mIcon, "cup") || !strcmp(mLabel, "cup")) {
    loadBackgroundJpeg(cup_map, 18816);
  } else if (!strcmp(mIcon, "mouse") || !strcmp(mLabel, "mouse")) {
    loadBackgroundJpeg(mouse_map, 21970);
  } else if (!strcmp(mIcon, "phone") || !strcmp(mLabel, "phone")) {
    loadBackgroundJpeg(phone_map, 24128);
  } else if (!strcmp(mIcon, "shake") || !strcmp(mIcon, "swing") || !strcmp(mLabel, "shaking")) {
    loadBackgroundJpeg(shake_map, 34503);
  } else if (!strcmp(mIcon, "circle") || !strcmp(mIcon, "spin") || !strcmp(mLabel, "spin")) {
    loadBackgroundJpeg(circle_map, 32588);
  } else {
    loadBackgroundJpeg(start_map, 9093); // Fallback
  }

  // Draw unlock progress arc ring (ui 5 Hz)
  spr.drawSmoothArc(CX, CY, 116, 108, 0, 359, C_DIM, C_BG);
  uint32_t need = needMs ? needMs : 1;
  int deg = (int)(360.0f * (heldMs > need ? need : heldMs) / need);
  bool fresh = (now - lastUi) < 2000;              // Erase gauge if ui > 2 sec stale
  if (fresh && deg > 0)
    spr.drawSmoothArc(CX, CY, 116, 108, 0, deg, deg >= 359 ? C_OK : C_WARN, C_BG, true);
}

// 5. Fail -> retry.c (Held for 5s)
static void screenFail(uint32_t now) {
  loadBackgroundJpeg(retry_map, 41217);
}

// 6. Success -> goodjob.c (Held for 3s then Shutdown)
static void screenSuccess(uint32_t now) {
  loadBackgroundJpeg(goodjob_map, 39457);
}

// 7. Shutdown
static void screenShutdown() {
  spr.fillSprite(C_BG);
}

static void render() {
  uint32_t now = millis();

  // Hold timer check (Retry 5s / Goodjob 3s)
  if (holdUntil && now >= holdUntil) {
    holdUntil = 0;
    st = pendingSt;
  }

  // Fill background margin with back.c color (0x7426)
  spr.fillSprite(C_BG);

  switch (st) {
    case S_ARMED:    screenArmed(now);    break;
    case S_RINGING:  screenRinging(now);  break;
    case S_MISSION:  screenMission(now);  break;
    case S_FAIL:     screenFail(now);     break;
    case S_SUCCESS:  screenSuccess(now);  break;
    case S_SHUTDOWN: screenShutdown();    break;
    default:         screenIdle(now);     break;
  }

  if (!mqtt.connected()) centerText("offline", CY + 100, C_FAIL, 2);

  spr.pushSprite(0, 0);
}

// ---- MQTT Callback & State Control ----
static void setState(St s) {
  if (holdUntil) { pendingSt = s; return; }
  st = s;
}

void onMqtt(char *topic, byte *payload, unsigned int len) {
  static char buf[512];
  if (len >= sizeof(buf)) len = sizeof(buf) - 1;
  memcpy(buf, payload, len);
  buf[len] = 0;

  if (!strcmp(topic, UI_TOPIC)) {
    long v;
    if (jsonNum(buf, "held_ms", &v)) heldMs = (uint32_t)(v < 0 ? 0 : v);
    if (jsonNum(buf, "need_ms", &v) && v > 0) needMs = (uint32_t)v;
    lastUi = millis();
    return;
  }

  // [+] 비전 노드 발행은 IP 만 꺼내 쓴다. 추론 판정은 대시보드가 하므로 여기서 볼 일이 없다.
  if (!strcmp(topic, VIS_TOPIC)) {
    char ip[20];
    if (jsonStr(buf, "ip", ip, sizeof(ip)) && strcmp(ip, visionIp)) {
      strncpy(visionIp, ip, sizeof(visionIp) - 1);
      visionIp[sizeof(visionIp) - 1] = 0;
      Serial.printf("[CAM] vision node at %s\n", visionIp);
    }
    return;
  }

  // cmd topic processing
  if (!strstr(buf, "\"type\":\"alarm\"")) return;
  char state[24] = "";
  if (!jsonStr(buf, "state", state, sizeof(state))) return;
  Serial.printf("[CMD] %s\n", buf);

  if (!strcmp(state, "armed")) {
    long r = 0;
    jsonNum(buf, "remain_s", &r);
    armedMs = millis();
    armedTotal = (uint32_t)(r > 0 ? r : 0) * 1000UL;
    holdUntil = 0;
    setState(S_ARMED);
  } else if (!strcmp(state, "ringing")) {
    holdUntil = 0;
    setState(S_RINGING);
  } else if (!strcmp(state, "mission")) {
    jsonStr(buf, "label", mLabel, sizeof(mLabel));
    jsonStr(buf, "icon",  mIcon,  sizeof(mIcon));
    jsonStr(buf, "kind",  mKind,  sizeof(mKind));
    long h;
    if (jsonNum(buf, "hold_s", &h) && h > 0) needMs = (uint32_t)h * 1000UL;
    heldMs = 0;
    // [+] 인트로 타이머를 새 미션마다 다시 시작한다. camLen 도 버려야 한다 —
    // 안 버리면 직전 미션의 마지막 프레임이 인트로 없이 한 순간 스쳐 보인다.
    missionAt = millis();
    camLen = 0;
    setState(S_MISSION);
  } else if (!strcmp(state, "fail")) {
    // Fail: display retry.c for 5 seconds (5000ms), then return to mission retry
    st = S_FAIL;
    pendingSt = S_MISSION;
    holdUntil = millis() + FAIL_HOLD_MS;
    heldMs = 0;
  } else if (!strcmp(state, "success")) {
    // Success: display goodjob.c for 3 seconds (3000ms), then shutdown
    st = S_SUCCESS;
    pendingSt = S_SHUTDOWN;
    holdUntil = millis() + SUCCESS_HOLD_MS;
    heldMs = 0;
  } else if (!strcmp(state, "shutdown")) {
    holdUntil = 0;
    st = S_SHUTDOWN;
  } else if (!strcmp(state, "idle")) {
    holdUntil = 0;
    st = S_IDLE;
    heldMs = 0;
  }
}

// Enforce 3-second reconnect interval to prevent socket depletion (rc=-2)
static uint32_t lastMqttTry = 0;
void ensureMqtt() {
  if (mqtt.connected()) return;
  uint32_t now = millis();
  if (lastMqttTry && now - lastMqttTry < 3000) return;
  lastMqttTry = now;
  if (mqtt.connect(MQTT_ID)) {
    Serial.println("[MQTT] connected");
    mqtt.subscribe(CMD_TOPIC);
    mqtt.subscribe(UI_TOPIC);
    mqtt.subscribe(VIS_TOPIC);     // [+] 비전 노드 IP 를 알아내기 위해서만
  } else {
    Serial.printf("[MQTT] connect failed rc=%d\n", mqtt.state());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(200);
  Serial.println("\n==== alarmi CLOCK NODE ====");

  // 백라이트 전원 켜기 (Seeed Round Display: D6 / GPIO 43)
#ifdef D6
  pinMode(D6, OUTPUT);
  digitalWrite(D6, HIGH);
#endif
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif
  pinMode(43, OUTPUT);
  digitalWrite(43, HIGH);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(C_BG);

  // 16bpp sprite allocation fallback to 8bpp (TFT_eSPI_Clock_ex2 pattern)
  if (spr.createSprite(SCR, SCR) == nullptr) {
    Serial.println("[TFT] 16bpp failed - fallback to 8bpp");
    spr.setColorDepth(8);
    spr.createSprite(SCR, SCR);
  }
  spr.fillSprite(C_BG);

  // 부팅 즉시 start.c 화면 렌더링
  render();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setBufferSize(1024);

  // [+] JPEG 버퍼는 PSRAM 에서 잡는다. 스프라이트가 이미 내부 RAM 을 115 KB 쓰고 있다.
  // 못 잡으면 뷰파인더만 조용히 꺼지고(camFetchFrame 이 바로 false) 시계는 그대로 돈다.
  camBuf = (uint8_t *)heap_caps_malloc(CAM_MAX_BYTES, MALLOC_CAP_SPIRAM);
  if (!camBuf) camBuf = (uint8_t *)malloc(CAM_MAX_BYTES);
  Serial.printf("[CAM] buffer %s\n", camBuf ? "ok" : "실패 - 뷰파인더 비활성");
}

void loop() {
  uint32_t now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    ensureMqtt();
    mqtt.loop();
  }

  // [+] 카메라는 사물 미션의 인트로가 끝난 뒤에만 받아온다. 다른 상태에서 받으면
  // 비전 노드의 카메라와 Wi-Fi 를 공짜로 갉아먹는다 (그 보드는 700 ms 마다 추론도 돈다).
  if (camPhase(now) && now - camFetch >= CAM_FETCH_MS) {
    camFetch = now;
    camFetchFrame();
    if (mqtt.connected()) mqtt.loop();   // GET 이 블로킹이라 그동안 쌓인 cmd 를 즉시 비운다
  }

  // 5-second Serial Heartbeat
  if (now - lastHb >= 5000) {
    lastHb = now;
    Serial.printf("[HB] up=%lus wifi=%d mqtt=%d(state %d) st=%d held=%lu/%lu "
                  "cam ok=%lu fail=%lu ip=%s heap=%u\n",
                  now / 1000, WiFi.status(), mqtt.connected(), mqtt.state(),
                  (int)st, heldMs, needMs, camOk, camFail,
                  visionIp[0] ? visionIp : "-", ESP.getFreeHeap());
  }

  if (now - lastFrame >= FRAME_MS) { lastFrame = now; render(); }
}
