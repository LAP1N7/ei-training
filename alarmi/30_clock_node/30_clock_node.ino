// alarmi — CLOCK NODE (라운드 디스플레이).  ★ XIAO ESP32S3 + Seeed Round Display ★
//
// 센서는 하나도 안 붙는다. 이 보드는 오직 "지금 무슨 상태인지"를 화면에 그린다.
// 판정은 대시보드가 하고, 이 보드는 그 결과를 받아 보여주기만 한다 (headless 반대편).
//
//   대시보드 ──cmd(retained)──▶ 이 보드   wearable/minseo/cmd   상태 전이
//   대시보드 ──ui (5 Hz)──────▶ 이 보드   wearable/minseo/ui    {"held_ms":1240,"need_ms":3000}
//
// 화면 (cmd 의 state 로 갈린다):
//   idle      💤 대기 시계
//   armed     ⏳ 남은 시간 카운트다운       ← remain_s 를 받아 자기가 센다
//   ringing   🔔 알람 (테두리 점멸)
//   mission   🎯 미션 — kind 별로 다른 아이콘 + 라벨 + 진행 링(ui) + 제한시간
//   fail      ❌ 실패 플래시 (1.2 초) 후 새 미션으로
//   success   🎉 클리어
//   shutdown  🌙 종료
//
// 왜 NTP 를 안 쓰나: 이 보드는 절대 시각이 필요 없다. 대시보드가 armed 에 remain_s(남은 초)를
// 같이 실어 보내므로 그걸 받아서 자기 millis() 로 세면 된다. 시각 동기화 실패라는 고장
// 원인을 통째로 없애는 쪽이 낫다.
//
// 배선: Seeed Round Display 를 XIAO 위에 그대로 꽂는다 (SPI + 백라이트).
//   TFT_eSPI 의 핀 설정은 User_Setup.h 에 이미 잡혀 있어야 한다
//   — TFT_eSPI_Clock_ex2 가 도는 환경이면 그대로 컴파일된다.
//
// 빌드:
//   arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 30_clock_node
//   arduino-cli upload -p COMx --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 30_clock_node
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ---- config ----
const char *WIFI_SSID  = "projectbee";
const char *WIFI_PASS  = "honeybear!";
const char *MQTT_HOST  = "192.168.0.27";
const int   MQTT_PORT  = 1883;
const char *CMD_TOPIC  = "wearable/minseo/cmd";
const char *UI_TOPIC   = "wearable/minseo/ui";
const char *MQTT_ID    = "xiao-minseo-node-clock";   // 노드마다 반드시 달라야 한다

#define SCR        240                 // 원형 패널 240x240
#define CX         (SCR / 2)
#define CY         (SCR / 2)
#define FAIL_MS    1200                // 실패 화면을 붙잡아두는 시간
#define FRAME_MS   50                  // 20 fps

// 색
#define C_BG       TFT_BLACK
#define C_DIM      0x39C7              // 회색
#define C_TXT      TFT_WHITE
#define C_ACCENT   0x455F              // 파랑
#define C_OK       0x3606              // 초록
#define C_WARN     0xFDA6              // 주황
#define C_FAIL     0xF9E7              // 빨강

TFT_eSPI  tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);   // 깜빡임 방지용 — 항상 스프라이트에 그려서 한 번에 push

WiFiClient net;
PubSubClient mqtt(net);

// ---- 대시보드가 알려준 상태 ----
enum St { S_IDLE, S_ARMED, S_RINGING, S_MISSION, S_FAIL, S_SUCCESS, S_SHUTDOWN };
static St       st         = S_IDLE;
static char     mLabel[32] = "";       // 미션 라벨 (shaking / mouse / …)
static char     mIcon[16]  = "";       // 아이콘 이름 (swing / spin / mouse / cup / phone)
static char     mKind[16]  = "";       // gesture | object
static uint32_t armedMs    = 0;        // ARMED 진입 시각
static uint32_t armedTotal = 0;        // 카운트다운 총 길이 (ms)
static uint32_t failUntil  = 0;        // 이 시각까지는 실패 화면을 유지
static St       pendingSt  = S_IDLE;   // 실패 화면 도중에 도착한 다음 상태
static uint32_t heldMs = 0, needMs = 3000;   // ui 진행 링
static uint32_t lastUi = 0;            // ui 가 끊기면 링을 지우기 위해
static uint32_t lastFrame = 0, lastHb = 0;

// ---- 아주 작은 JSON 판독기 ----
// ArduinoJson 을 쓸 만큼 복잡하지 않다. 다른 노드들도 strstr 로 처리한다.
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

// =============================== 아이콘 ===============================
// JPEG 를 헤더로 박으면 파일 하나에 100 KB 씩 먹는다 (TFT_eSPI_Clock_ex2 의 jpeg1.h 가 108 KB).
// 미션 아이콘은 도형 몇 개면 되므로 벡터로 그린다 — 용량 0, 색도 상태 따라 바꿀 수 있다.
// t 는 애니메이션용 위상(0~1).
static void iconSwing(int cx, int cy, int r, uint16_t c, float t) {
  // 팔 흔들기: 머리 + 몸통 + 좌우로 흔들리는 팔
  float a = sinf(t * 2 * PI) * 0.6f;              // ±34°
  spr.fillSmoothCircle(cx, cy - r / 2, r / 5, c, C_BG);
  spr.drawWideLine(cx, cy - r / 4, cx, cy + r / 2, 5, c, C_BG);
  for (int s = -1; s <= 1; s += 2) {
    float ang = s * (0.9f + a);
    spr.drawWideLine(cx, cy - r / 8,
                     cx + sinf(ang) * r * 0.8f, cy - r / 8 - cosf(ang) * r * 0.55f, 5, c, C_BG);
  }
}
static void iconSpin(int cx, int cy, int r, uint16_t c, float t) {
  // 회전: 도는 호 + 끝에 점
  int start = (int)(t * 360) % 360;
  spr.drawSmoothArc(cx, cy, r, r - 6, start, start + 250, c, C_BG, true);
  float a = (start + 250) * DEG_TO_RAD;
  spr.fillSmoothCircle(cx + sinf(a) * (r - 3), cy - cosf(a) * (r - 3), 6, c, C_BG);
}
static void iconMouse(int cx, int cy, int r, uint16_t c) {
  spr.drawSmoothRoundRect(cx - r / 2, cy - r, r, r * 2, r / 2, r / 2 - 4, c, C_BG);
  spr.drawWideLine(cx, cy - r * 0.75f, cx, cy - r * 0.2f, 4, c, C_BG);
}
static void iconCup(int cx, int cy, int r, uint16_t c) {
  // 사다리꼴 종이컵
  spr.drawWideLine(cx - r * 0.7f, cy - r, cx + r * 0.7f, cy - r, 5, c, C_BG);
  spr.drawWideLine(cx - r * 0.7f, cy - r, cx - r * 0.45f, cy + r, 5, c, C_BG);
  spr.drawWideLine(cx + r * 0.7f, cy - r, cx + r * 0.45f, cy + r, 5, c, C_BG);
  spr.drawWideLine(cx - r * 0.45f, cy + r, cx + r * 0.45f, cy + r, 5, c, C_BG);
}
static void iconPhone(int cx, int cy, int r, uint16_t c) {
  spr.drawSmoothRoundRect(cx - r * 0.55f, cy - r, r * 1.1f, r * 2, 8, 4, c, C_BG);
  spr.fillSmoothCircle(cx, cy + r * 0.72f, 5, c, C_BG);
}
static void drawIcon(const char *icon, int cx, int cy, int r, uint16_t c, float t) {
  if      (!strcmp(icon, "swing")) iconSwing(cx, cy, r, c, t);
  else if (!strcmp(icon, "spin"))  iconSpin (cx, cy, r, c, t);
  else if (!strcmp(icon, "mouse")) iconMouse(cx, cy, r, c);
  else if (!strcmp(icon, "cup"))   iconCup  (cx, cy, r, c);
  else if (!strcmp(icon, "phone")) iconPhone(cx, cy, r, c);
  else {   // 모르는 아이콘이면 라벨만 크게 — 새 라벨이 생겨도 화면이 비지 않게
    spr.setTextDatum(MC_DATUM);
    spr.setTextColor(c, C_BG);
    spr.drawString("?", cx, cy, 4);
  }
}

// =============================== 화면 ===============================
// 화면 문구는 전부 영문이다. TFT_eSPI 내장 폰트에는 한글 글리프가 없어서 한글을 넣으면
// 네모나 빈칸으로 나온다 (참조한 TFT_eSPI_Clock_ex2 의 NotoSansBold15 도 라틴 전용).
// 한글이 꼭 필요하면 한글 TTF 를 폰트 변환기로 .h 로 만들어 loadFont() 해야 하는데,
// 글리프가 많아 플래시를 크게 먹는다. 미션 라벨(shaking/mouse…)도 원래 영문이라 지금은 불필요.
static void centerText(const char *s, int y, uint16_t c, int font, int size = 1) {
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(c, C_BG);
  spr.setTextSize(size);
  spr.drawString(s, CX, y, font);
  spr.setTextSize(1);
}

// mm:ss. 폰트 7(7세그)은 User_Setup 에서 빠져 있을 수 있어 안 쓴다 —
// 폰트 4 를 2배로 키우면 어느 설정에서든 뜬다.
static void bigTime(uint32_t ms, int y, uint16_t c) {
  uint32_t s = (ms + 999) / 1000;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)(s / 60), (unsigned)(s % 60));
  centerText(buf, y, c, 4, 2);
}

static void screenIdle(uint32_t now) {
  spr.drawSmoothCircle(CX, CY, 112, C_DIM, C_BG);
  centerText("ALARMI", CY - 26, C_DIM, 4);
  centerText("STANDBY", CY + 18, C_DIM, 2);
  // 살아있다는 표시 — 12시 위치를 도는 점
  float a = (now % 4000) / 4000.0f * 2 * PI;
  spr.fillSmoothCircle(CX + sinf(a) * 100, CY - cosf(a) * 100, 4, C_ACCENT, C_BG);
}

static void screenArmed(uint32_t now) {
  uint32_t el = now - armedMs;
  uint32_t left = (el >= armedTotal) ? 0 : armedTotal - el;
  // 남은 비율만큼 링이 줄어든다
  int deg = armedTotal ? (int)(360.0f * left / armedTotal) : 0;
  spr.drawSmoothArc(CX, CY, 112, 102, 0, 359, C_DIM, C_BG);
  if (deg > 0) spr.drawSmoothArc(CX, CY, 112, 102, 0, deg, C_ACCENT, C_BG, true);
  centerText("TIMER", CY - 52, C_DIM, 2);
  bigTime(left, CY, C_TXT);
  centerText("REMAINING", CY + 48, C_DIM, 2);
}

static void screenRinging(uint32_t now) {
  bool on = (now / 350) % 2;                       // 테두리 점멸
  spr.drawSmoothArc(CX, CY, 112, 100, 0, 359, on ? C_FAIL : C_DIM, C_BG);
  centerText("WAKE UP", CY - 20, on ? C_FAIL : C_TXT, 4);
  centerText("GET READY", CY + 26, C_DIM, 2);
}

static void screenMission(uint32_t now) {
  // 바깥 링 = 해제 게이지 (ui 로 5 Hz 마다 갱신)
  spr.drawSmoothArc(CX, CY, 114, 104, 0, 359, C_DIM, C_BG);
  uint32_t need = needMs ? needMs : 1;
  int deg = (int)(360.0f * (heldMs > need ? need : heldMs) / need);
  bool fresh = (now - lastUi) < 2000;              // ui 가 끊기면 링을 믿지 않는다
  if (fresh && deg > 0)
    spr.drawSmoothArc(CX, CY, 114, 104, 0, deg, deg >= 359 ? C_OK : C_WARN, C_BG, true);

  bool isGesture = !strcmp(mKind, "gesture");
  uint16_t c = isGesture ? C_ACCENT : C_OK;
  drawIcon(mIcon, CX, CY - 22, 40, c, (now % 1200) / 1200.0f);

  centerText(mLabel, CY + 44, C_TXT, 4);
  centerText(isGesture ? "GESTURE" : "SHOW OBJECT", CY + 78, C_DIM, 2);
}

static void screenFail(uint32_t now) {
  bool on = (now / 150) % 2;
  spr.drawSmoothArc(CX, CY, 114, 100, 0, 359, on ? C_FAIL : C_DIM, C_BG);
  // X 표시
  for (int i = -1; i <= 1; i += 2)
    spr.drawWideLine(CX - 34 * i, CY - 34, CX + 34 * i, CY + 34, 9, C_FAIL, C_BG);
  centerText("MISS - RETRY", CY + 74, C_FAIL, 2);
}

static void screenSuccess(uint32_t now) {
  spr.drawSmoothArc(CX, CY, 114, 100, 0, 359, C_OK, C_BG);
  // 체크 표시
  spr.drawWideLine(CX - 40, CY, CX - 10, CY + 30, 10, C_OK, C_BG);
  spr.drawWideLine(CX - 10, CY + 30, CX + 44, CY - 32, 10, C_OK, C_BG);
  centerText("CLEAR!", CY - 56, C_OK, 4);
}

static void screenShutdown() {
  spr.drawSmoothCircle(CX, CY, 100, C_DIM, C_BG);
  centerText("DONE", CY - 12, C_DIM, 4);
  centerText("GOOD NIGHT", CY + 26, C_DIM, 2);
}

static void render() {
  uint32_t now = millis();

  // 실패 화면은 잠깐 붙잡아둔다. 대시보드가 fail 직후 곧바로 mission 을 다시 보내기
  // 때문에, 안 붙잡으면 실패 화면이 한 프레임도 못 보이고 지나간다.
  if (failUntil && now >= failUntil) { failUntil = 0; st = pendingSt; }

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

  // Wi-Fi / 브로커가 끊기면 화면 아래에 조용히 표시 — 화면만 보고도 원인을 안다
  if (!mqtt.connected()) centerText("offline", CY + 100, C_FAIL, 2);

  spr.pushSprite(0, 0);
}

// =============================== MQTT ===============================
static void setState(St s) {
  if (failUntil) { pendingSt = s; return; }   // 실패 화면 중이면 끝난 뒤에 반영
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

  // cmd
  if (!strstr(buf, "\"type\":\"alarm\"")) return;   // 모르는 type 은 무시
  char state[24] = "";
  if (!jsonStr(buf, "state", state, sizeof(state))) return;
  Serial.printf("[CMD] %s\n", buf);

  if (!strcmp(state, "armed")) {
    long r = 0;
    jsonNum(buf, "remain_s", &r);
    armedMs = millis();
    armedTotal = (uint32_t)(r > 0 ? r : 0) * 1000UL;
    setState(S_ARMED);
  } else if (!strcmp(state, "ringing")) {
    setState(S_RINGING);
  } else if (!strcmp(state, "mission")) {
    jsonStr(buf, "label", mLabel, sizeof(mLabel));
    jsonStr(buf, "icon",  mIcon,  sizeof(mIcon));
    jsonStr(buf, "kind",  mKind,  sizeof(mKind));
    long h;
    if (jsonNum(buf, "hold_s", &h) && h > 0) needMs = (uint32_t)h * 1000UL;
    heldMs = 0;
    setState(S_MISSION);
  } else if (!strcmp(state, "fail")) {
    // 실패만은 즉시 화면을 뺏는다 (setState 를 안 거친다)
    st = S_FAIL;
    pendingSt = S_MISSION;
    failUntil = millis() + FAIL_MS;
    heldMs = 0;
  } else if (!strcmp(state, "success")) {
    failUntil = 0;                 // 성공이 실패 화면에 가려지면 안 된다
    st = S_SUCCESS;
    heldMs = 0;
  } else if (!strcmp(state, "shutdown")) {
    failUntil = 0;
    st = S_SHUTDOWN;
  } else if (!strcmp(state, "idle")) {
    failUntil = 0;
    st = S_IDLE;
    heldMs = 0;
  }
}

// 재연결은 간격을 두고 시도한다. loop 마다 connect() 를 부르면 실패할 때마다 소켓이
// 하나씩 물려서 LWIP 소켓이 고갈되고, 그때부터는 무조건 rc=-2 로 떨어진다.
static uint32_t lastMqttTry = 0;
void ensureMqtt() {
  if (mqtt.connected()) return;
  uint32_t now = millis();
  if (lastMqttTry && now - lastMqttTry < 3000) return;
  lastMqttTry = now;
  if (mqtt.connect(MQTT_ID)) {
    Serial.println("[MQTT] connected");
    mqtt.subscribe(CMD_TOPIC);     // retained 라 붙자마자 현재 상태를 받는다
    mqtt.subscribe(UI_TOPIC);
  } else {
    Serial.printf("[MQTT] connect failed rc=%d\n", mqtt.state());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(300);
  Serial.println("\n==== alarmi CLOCK NODE ====");

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(C_BG);

  // 240x240 16bpp = 115 KB. 못 잡으면 8bpp 로 물러선다 (TFT_eSPI_Clock_ex2 와 같은 처리).
  if (spr.createSprite(SCR, SCR) == nullptr) {
    Serial.println("[TFT] 16bpp 실패 - 8bpp 로 재시도");
    spr.setColorDepth(8);
    spr.createSprite(SCR, SCR);
  }
  spr.fillSprite(C_BG);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
  Serial.printf("[WIFI] %s IP=%s\n",
                WiFi.status() == WL_CONNECTED ? "ok" : "실패(loop 에서 재시도)",
                WiFi.localIP().toString().c_str());

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setBufferSize(1024);
}

void loop() {
  uint32_t now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    ensureMqtt();
    mqtt.loop();                   // 여기서 cmd/ui 가 들어온다
  }

  // 정상 동작할 때 아무것도 안 찍으면 "멈춤"과 "정상"을 구분할 수 없다
  if (now - lastHb >= 5000) {
    lastHb = now;
    Serial.printf("[HB] up=%lus wifi=%d mqtt=%d(state %d) st=%d held=%lu/%lu heap=%u\n",
                  now / 1000, WiFi.status(), mqtt.connected(), mqtt.state(),
                  (int)st, heldMs, needMs, ESP.getFreeHeap());
  }

  if (now - lastFrame >= FRAME_MS) { lastFrame = now; render(); }
}
