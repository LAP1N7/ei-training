// GESTURE + CLOCK NODE — 10_gesture_node 에 라운드 디스플레이를 얹은 판본.
//
// 왜 합쳤나: alarmi 의 원래 설계는 보드 3 개(제스처 / 비전 / 화면)지만 실제 하드웨어는
// XIAO 2 개이고, 라운드 디스플레이가 제스처 보드에 달려 있다. 그래서 이 보드 하나가
// 제스처 추론과 화면을 같이 담당한다.
//
// 기준은 10_gesture_node 다 — 추론 경로(sampleTick / publishState / ring / features)는
// 손대지 않았다. 화면 코드는 30_clock_node 에서 가져와 얹었고, 추가분은 [D] 로 표시했다.
//
// 화면 (cmd 의 state 로 갈린다. 이미지는 30_clock_node 와 같은 .c 배열):
//   idle→start.c   armed/ringing→timer.c   fail→retry.c   success→goodjob.c
//   mission→ 제스처면 shake.c / circle.c (그대로 유지)
//           사물이면 cup/mouse/phone.c 를 3 초 보여준 뒤 비전 노드의 카메라 화면으로 전환
//
// 카메라는 이 보드에 없다. 사물 미션 중에만 http://<비전노드IP>/jpg 를 받아 그리고,
// 그 외에는 아예 건드리지 않는다.
//
// 원본(10_gesture_node) 주석 시작 ↓
// SENSOR NODE 1 (gesture) — day4/10_gesture_node 의 사본.
//
// BNO055 로 제스처를 추론하고, 같은 I2C 버스에 붙은 BH1750(조도) / BME280(온·습·기압)
// 의 "가공하지 않은" 센서값을 함께 실어서 MQTT 로 한 번에 발행한다.
// dashboard.html / gesture-alarm.html / gesture-audio.html 이 이 토픽을 구독한다.
//
// 원본 대비 바뀐 곳은 [+] 로 표시했다 — 모델 라이브러리 하나뿐이다.
// 발행 스키마(node/kind/sensors/inference)는 21_vision_node_jpg 와 동일하므로
// 대시보드는 제스처 노드든 비전 노드든 같은 코드로 읽는다.
//
// 배선 (I2C0):  SDA -> D4(GPIO5),  SCL -> D5(GPIO6),  VCC -> 3V3,  GND -> GND
//   0x23  BH1750  조도
//   0x29  BNO055  9축 IMU   <- 추론 입력
//   0x76  BME280  온도/습도/기압
//
// 빌드 (라이브러리를 갈아끼운 뒤 첫 컴파일은 --clean 이 필요하다. arduino-cli 가
// 라이브러리를 이름+버전으로 캐시해서, 재빌드된 동명 라이브러리의 낡은 오브젝트를
// 그대로 링크해 예전 모델을 조용히 굽는다):
//   arduino-cli compile --clean --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 10_gesture_node
//   arduino-cli upload -p COM6 --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 10_gesture_node
#include <gesture111_inferencing.h>   // [+] EI 프로젝트 gesture111 — idle/shaking/spin
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <WiFi.h>
#include <HTTPClient.h>              // [D] 비전 노드의 /jpg 를 받아오기 위해
#include <PubSubClient.h>
#include <math.h>
#include <TFT_eSPI.h>                // [D]
#include <SPI.h>                     // [D]
#include <JPEGDecoder.h>             // [D]
#include "images.h"                  // [D] start/timer/cup/mouse/phone/shake/circle/goodjob/retry
#include "esp_heap_caps.h"

// EI 텐서 아레나를 PSRAM 으로 (weak symbol 오버라이드)
void *ei_malloc(size_t size) {
  void *p = heap_caps_aligned_alloc(16, size, MALLOC_CAP_SPIRAM);
  if (!p) p = heap_caps_aligned_alloc(16, size, MALLOC_CAP_DEFAULT);
  return p;
}
void *ei_calloc(size_t n, size_t s) { void *p = ei_malloc(n * s); if (p) memset(p, 0, n * s); return p; }
void ei_free(void *ptr) { heap_caps_free(ptr); }

// ---- config ----
const char *WIFI_SSID  = "projectbee";
const char *WIFI_PASS  = "honeybear!";
const char *MQTT_HOST  = "192.168.0.27";   // 내 노트북의 mosquitto (1883=보드, 9001=브라우저)
const int   MQTT_PORT  = 1883;
const char *MQTT_TOPIC = "wearable/minseo/gesture";
const char *CMD_TOPIC  = "wearable/minseo/cmd";     // [+] 대시보드 상태머신 다운링크 (retained)
const char *UI_TOPIC   = "wearable/minseo/ui";      // [D] 해제 게이지 (5 Hz)
const char *VIS_TOPIC  = "wearable/minseo/vision";  // [D] 비전 노드 IP 를 여기서 알아낸다
const char *MQTT_ID    = "xiao-minseo-node-gesture";
const char *NODE_ID    = "gesture";

#define SDA_PIN     5
#define SCL_PIN     6
#define PUBLISH_MS  250    // 추론 + 발행 주기 (4 Hz)
#define SLOW_READ_MS 1000  // BME280 / BH1750 갱신 주기

// ---- [D] 화면 ----
#define SCR             240
#define CX              (SCR / 2)
#define CY              (SCR / 2)
#define FRAME_MS        50                 // 20 fps
#define FAIL_HOLD_MS    5000               // retry.c 유지
#define SUCCESS_HOLD_MS 3000               // goodjob.c 유지
#define minimum(a, b)   (((a) < (b)) ? (a) : (b))

#define C_BG       0x7426                  // back.png 올리브 그린
#define C_DIM      0x53E9
#define C_TXT      TFT_WHITE
#define C_ACCENT   0x455F
#define C_OK       0x3606
#define C_WARN     0xFDA6
#define C_FAIL     0xF9E7

// ---- [D] 사물 미션용 카메라 뷰파인더 ----
// 비전 노드 주소는 두 경로로 얻는다:
//   1) MQTT 의 vision 발행에 있는 "ip" (있으면 이게 우선 — DHCP 로 바뀌어도 따라간다)
//   2) 없으면 아래 폴백. 지금 돌고 있는 비전 노드 펌웨어는 ip 를 안 실어 보내므로
//      이 값으로 붙는다. 비전 노드를 다시 구우면 1) 이 자동으로 덮어쓴다.
#define VISION_IP_FALLBACK "192.168.0.94"
#define CAM_INTRO_MS   3000                // 미션 이미지를 보여주는 시간, 그 뒤 카메라로 전환
#define CAM_FETCH_MS   400
#define CAM_TIMEOUT_MS 1500
#define CAM_MAX_BYTES  32768
#define CAM_STALE_MS   2000                // 이보다 낡은 프레임은 실시간이 아니므로 안 그린다

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BME280 bme;
BH1750          lux;
WiFiClient net;
PubSubClient mqtt(net);

bool hasBNO = false, hasBME = false, hasBH = false;

// ---- [D] 화면 상태 (30_clock_node 와 같은 상태머신) ----
TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

enum St { S_IDLE, S_ARMED, S_RINGING, S_MISSION, S_FAIL, S_SUCCESS, S_SHUTDOWN };
static St       dSt        = S_IDLE;
static char     mLabel[32] = "";
static char     mIcon[16]  = "";
static char     mKind[16]  = "";
static uint32_t armedAt    = 0;      // ARMED 진입 시각
static uint32_t armedTotal = 0;
static uint32_t holdUntil  = 0;      // retry/goodjob 유지 완료 시각
static St       pendingSt  = S_IDLE;
static uint32_t heldMs = 0, needMs = 3000;
static uint32_t lastUi = 0;
static uint32_t lastFrame = 0, missionAt = 0;

// ---- [D] 카메라 뷰파인더 ----
static char     visionIp[20] = VISION_IP_FALLBACK;
static uint8_t *camBuf   = nullptr;
static size_t   camLen   = 0;
static uint32_t camAt    = 0;
static uint32_t camFetch = 0;
static uint32_t camOk = 0, camFail = 0;

static inline bool isObjectMission() { return strcmp(mKind, "gesture") != 0; }
static inline bool camPhase(uint32_t now) {
  return dSt == S_MISSION && isObjectMission() && (now - missionAt) >= CAM_INTRO_MS;
}

// ---- 100 Hz 링버퍼 (선형가속도 3축) ----
#define RAW EI_CLASSIFIER_RAW_SAMPLE_COUNT       // 200 = 2 s @ 100 Hz
static float ring[RAW][3];
static int   rhead = 0;
static bool  rfull = false;
static uint32_t nextUs = 0;

// ---- 최근 원시 센서값 캐시 ----
static float sAx = 0, sAy = 0, sAz = 0, sMag = 0;     // 선형가속도
static float sYaw = 0, sRoll = 0, sPitch = 0;         // 오일러각
static float sLux = NAN, sTemp = NAN, sHum = NAN, sPres = NAN;
static uint32_t lastSlow = 0, lastPub = 0;

static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static int get_feature_data(size_t offset, size_t length, float *out) {
  memcpy(out, features + offset, length * sizeof(float));
  return 0;
}

// 100 Hz 로 IMU 를 긁어 링버퍼에 넣는다. loop() 에서 매번 호출.
void sampleTick() {
  uint32_t now = micros();
  if ((int32_t)(now - nextUs) < 0) return;
  nextUs += 1000000UL / 100;
  if (!hasBNO) return;

  sensors_event_t la;
  bno.getEvent(&la, Adafruit_BNO055::VECTOR_LINEARACCEL);
  sAx = la.acceleration.x; sAy = la.acceleration.y; sAz = la.acceleration.z;
  ring[rhead][0] = sAx; ring[rhead][1] = sAy; ring[rhead][2] = sAz;
  rhead = (rhead + 1) % RAW;
  if (rhead == 0) rfull = true;
  sMag = sqrtf(sAx * sAx + sAy * sAy + sAz * sAz);
}

// BME280 / BH1750 / 오일러각은 느려도 되므로 1 초에 한 번만
void readSlowSensors() {
  if (hasBH) {
    float v = lux.readLightLevel();
    if (v >= 0) sLux = v;
  }
  if (hasBME) {
    sTemp = bme.readTemperature();
    sHum  = bme.readHumidity();
    sPres = bme.readPressure() / 100.0f;
  }
  if (hasBNO) {
    sensors_event_t o;
    bno.getEvent(&o, Adafruit_BNO055::VECTOR_EULER);
    sYaw = o.orientation.x; sRoll = o.orientation.y; sPitch = o.orientation.z;
  }
}

// 대시보드가 센서 종류를 하드코딩하지 않아도 되도록,
// 값 + 단위 + 표시이름 + 그래프 범위를 함께 실어 보낸다.
static void addSensor(String &j, bool &first, const char *key, float v,
                      const char *unit, const char *label, float lo, float hi) {
  if (isnan(v)) return;
  if (!first) j += ",";
  first = false;
  j += "\"" + String(key) + "\":{\"v\":" + String(v, 2) +
       ",\"unit\":\"" + unit + "\",\"label\":\"" + label +
       "\",\"min\":" + String(lo, 1) + ",\"max\":" + String(hi, 1) + "}";
}

void publishState() {
  // 링버퍼 -> EI 특징벡터 (오래된 것이 앞, 덜 찼으면 앞을 0 으로 패딩)
  int n = rfull ? RAW : rhead;
  for (int i = 0; i < RAW - n; i++) {
    features[i * 3 + 0] = 0; features[i * 3 + 1] = 0; features[i * 3 + 2] = 0;
  }
  for (int i = 0; i < n; i++) {
    int idx = rfull ? (rhead + i) % RAW : i;
    int fp  = (RAW - n) + i;
    features[fp * 3 + 0] = ring[idx][0];
    features[fp * 3 + 1] = ring[idx][1];
    features[fp * 3 + 2] = ring[idx][2];
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = &get_feature_data;
  ei_impulse_result_t result = {0};
  if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) return;

  int best = 0;
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > result.classification[best].value) best = i;
  }

  String j = "{\"node\":\"" + String(NODE_ID) + "\",\"kind\":\"imu\"";
  j += ",\"uptime_s\":" + String(millis() / 1000);
  j += ",\"rssi\":" + String(WiFi.RSSI());

  // ---- 원시 센서값 ----
  j += ",\"sensors\":{";
  bool first = true;
  addSensor(j, first, "lux",      sLux,   "lx",   "조도",       0, 1000);
  addSensor(j, first, "temp",     sTemp,  "°C",   "온도",       0, 50);
  addSensor(j, first, "humidity", sHum,   "%",    "습도",       0, 100);
  addSensor(j, first, "pressure", sPres,  "hPa",  "기압",       950, 1050);
  addSensor(j, first, "accel",    sMag,   "m/s²", "가속도 |a|", 0, 20);
  addSensor(j, first, "ax",       sAx,    "m/s²", "가속도 X",   -20, 20);
  addSensor(j, first, "ay",       sAy,    "m/s²", "가속도 Y",   -20, 20);
  addSensor(j, first, "az",       sAz,    "m/s²", "가속도 Z",   -20, 20);
  addSensor(j, first, "yaw",      sYaw,   "°",    "Yaw",        0, 360);
  addSensor(j, first, "roll",     sRoll,  "°",    "Roll",       -90, 90);
  addSensor(j, first, "pitch",    sPitch, "°",    "Pitch",      -180, 180);
  j += "}";

  // ---- 추론 결과 ----
  j += ",\"inference\":{\"label\":\"" + String(ei_classifier_inferencing_categories[best]) + "\"";
  j += ",\"confidence\":" + String(result.classification[best].value, 3);
  j += ",\"scores\":{";
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (i) j += ",";
    j += "\"" + String(ei_classifier_inferencing_categories[i]) + "\":" +
         String(result.classification[i].value, 3);
  }
  j += "},\"dsp_ms\":" + String(result.timing.dsp);
  j += ",\"nn_ms\":" + String(result.timing.classification) + "}}";

  if (!mqtt.publish(MQTT_TOPIC, j.c_str())) {
    Serial.printf("[MQTT] publish failed (len=%u)\n", j.length());
  }
}

// [+] 대시보드 상태머신에 따른 모드 전환.
//   idle/armed/ringing/mission 은 계속 발행하고, success/shutdown 에서만 멈춘다.
//   샘플링(sampleTick)은 어느 쪽이든 계속 돌린다 — 링버퍼가 식으면 다시 켰을 때
//   2초를 기다려야 한다. 모르는 type(예: 예전 timer)은 무시한다.
//
// [+2] 예전에는 "사물 미션 중에는 제스처 노드가 침묵" 이었다. 그러면 미션이 도는
//   동안 다른 노드 카드가 통째로 죽어서, 한 보드를 쓰는 사이 같이 붙어 있는 다른
//   보드를 쳐다볼 수도, 같이 쓸 수도 없었다. 판정은 대시보드가 노드 단위로 거르므로
//   (index.html 의 feed(): `if(nodeId !== m.node) return;`) 두 노드가 동시에 떠들어도
//   엉뚱한 노드가 미션을 통과시키는 일은 생기지 않는다. 그래서 기본을 "둘 다 동작"
//   으로 바꿨다. 예전 동작으로 되돌리려면 아래를 1 로.
#define EXCLUSIVE_MISSION 0

static bool inferEnabled = true;

// =========================== [D] 화면 ===========================
// 아래 렌더링 블록은 30_clock_node 에서 그대로 가져왔다. 이미지 배열(.c)과 크기 상수도
// 같으므로 화면 결과물은 동일하다.

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

static void renderJPEGToSprite(TFT_eSprite *t, int xpos, int ypos) {
  uint16_t mcu_w = JpegDec.MCUWidth, mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width,    max_y = JpegDec.height;
  uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
  uint32_t min_h = minimum(mcu_h, max_y % mcu_h);
  uint32_t win_w = mcu_w, win_h = mcu_h;
  max_x += xpos; max_y += ypos;
  t->setSwapBytes(true);
  while (JpegDec.read()) {
    uint16_t *pImg = JpegDec.pImage;
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;
    win_w = (mcu_x + mcu_w <= (int)max_x) ? mcu_w : min_w;
    win_h = (mcu_y + mcu_h <= (int)max_y) ? mcu_h : min_h;
    if (win_w != mcu_w) {
      uint16_t *cImg = pImg + win_w;
      int p = 0;
      for (int hh = 1; hh < (int)win_h; hh++) {
        p += mcu_w;
        for (int w = 0; w < (int)win_w; w++) { *cImg = *(pImg + w + p); cImg++; }
      }
    }
    if ((mcu_x + (int)win_w) <= t->width() && (mcu_y + (int)win_h) <= t->height())
      t->pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    else if ((mcu_y + (int)win_h) >= t->height()) JpegDec.abort();
  }
  t->setSwapBytes(false);
}

static void loadBackgroundJpeg(const uint8_t arr[], uint32_t size) {
  JpegDec.decodeArray(arr, size);
  int x = ((int)spr.width()  - (int)JpegDec.width)  / 2;
  int y = ((int)spr.height() - (int)JpegDec.height) / 2;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  renderJPEGToSprite(&spr, x, y);
}

static void centerText(const char *s, int y, uint16_t c, int font, int size = 1) {
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(c);
  spr.setTextSize(size);
  spr.drawString(s, CX, y, font);
  spr.setTextSize(1);
}
static void bigTime(uint32_t ms, int y, uint16_t c) {
  uint32_t s = (ms + 999) / 1000;
  char b[16];
  snprintf(b, sizeof(b), "%02u:%02u", (unsigned)(s / 60), (unsigned)(s % 60));
  centerText(b, y, c, 4, 2);
}

// ---- [D] 카메라 프레임 ----
// 사물 미션 중에만 호출된다. 블로킹이라 호출 빈도는 loop 가 CAM_FETCH_MS 로 제한한다.
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

static void camDrawFrame() {
  if (!camLen) return;
  if (!JpegDec.decodeArray(camBuf, camLen)) return;
  int x = ((int)spr.width()  - (int)JpegDec.width)  / 2;
  int y = ((int)spr.height() - (int)JpegDec.height) / 2;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  renderJPEGToSprite(&spr, x, y);
}

static void ringGauge(uint32_t now) {
  spr.drawSmoothArc(CX, CY, 116, 108, 0, 359, C_DIM, C_BG);
  uint32_t need = needMs ? needMs : 1;
  int deg = (int)(360.0f * (heldMs > need ? need : heldMs) / need);
  if ((now - lastUi) < 2000 && deg > 0)
    spr.drawSmoothArc(CX, CY, 116, 108, 0, deg, deg >= 359 ? C_OK : C_WARN, C_BG, true);
}

static void screenMission(uint32_t now) {
  // 사물 미션: 미션 이미지를 3 초 보여준 뒤 카메라로 전환. 프레임이 낡았으면(비전 노드
  // 미응답) 넘어가지 않고 미션 이미지를 유지한다 — 멈춘 그림을 실시간인 척 보여주면
  // 화면만 보고 고장을 알 수 없다.
  if (camPhase(now) && camLen && (now - camAt) < CAM_STALE_MS) {
    camDrawFrame();
    ringGauge(now);
    return;
  }
  // 제스처 미션은 여기로만 온다 — 30_clock_node 와 완전히 같은 경로.
  if (!strcmp(mIcon, "cup") || !strcmp(mLabel, "cup"))                    loadBackgroundJpeg(cup_map, 18816);
  else if (!strcmp(mIcon, "mouse") || !strcmp(mLabel, "mouse"))           loadBackgroundJpeg(mouse_map, 21970);
  else if (!strcmp(mIcon, "phone") || !strcmp(mLabel, "phone"))           loadBackgroundJpeg(phone_map, 24128);
  else if (!strcmp(mIcon, "shake") || !strcmp(mIcon, "swing") || !strcmp(mLabel, "shaking"))
                                                                          loadBackgroundJpeg(shake_map, 34503);
  else if (!strcmp(mIcon, "circle") || !strcmp(mIcon, "spin") || !strcmp(mLabel, "spin"))
                                                                          loadBackgroundJpeg(circle_map, 32588);
  else                                                                    loadBackgroundJpeg(start_map, 9093);
  ringGauge(now);
}

static void render() {
  uint32_t now = millis();
  if (holdUntil && now >= holdUntil) { holdUntil = 0; dSt = pendingSt; }

  spr.fillSprite(C_BG);
  switch (dSt) {
    case S_ARMED: {
      loadBackgroundJpeg(timer_map, 12140);
      uint32_t el = now - armedAt;
      uint32_t left = (el >= armedTotal) ? 0 : armedTotal - el;
      int deg = armedTotal ? (int)(360.0f * left / armedTotal) : 0;
      spr.drawSmoothArc(CX, CY, 114, 106, 0, 359, C_DIM, C_BG);
      if (deg > 0) spr.drawSmoothArc(CX, CY, 114, 106, 0, deg, C_ACCENT, C_BG, true);
      bigTime(left, CY, C_TXT);
      break;
    }
    case S_RINGING:
      loadBackgroundJpeg(timer_map, 12140);
      spr.drawSmoothArc(CX, CY, 114, 106, 0, 359, ((now / 350) % 2) ? C_FAIL : C_DIM, C_BG);
      break;
    case S_MISSION:  screenMission(now);                    break;
    case S_FAIL:     loadBackgroundJpeg(retry_map, 41217);   break;
    case S_SUCCESS:  loadBackgroundJpeg(goodjob_map, 39457); break;
    case S_SHUTDOWN: spr.fillSprite(C_BG);
                     centerText("GOOD NIGHT", CY, C_DIM, 4); break;
    default:         loadBackgroundJpeg(start_map, 9093);    break;
  }
  if (!mqtt.connected()) centerText("offline", CY + 100, C_FAIL, 2);
  spr.pushSprite(0, 0);
}

void onCmd(char *topic, byte *payload, unsigned int len) {
  static char buf[512];
  if (len >= sizeof(buf)) len = sizeof(buf) - 1;
  memcpy(buf, payload, len);
  buf[len] = 0;

  // [D] ui 는 해제 게이지 전용 — cmd 파싱 전에 처리하고 빠진다.
  if (!strcmp(topic, UI_TOPIC)) {
    long v;
    if (jsonNum(buf, "held_ms", &v)) heldMs = (uint32_t)(v < 0 ? 0 : v);
    if (jsonNum(buf, "need_ms", &v) && v > 0) needMs = (uint32_t)v;
    lastUi = millis();
    return;
  }

  // [D] 비전 노드 발행은 IP 만 꺼내 쓴다. 추론 판정은 대시보드가 한다.
  if (!strcmp(topic, VIS_TOPIC)) {
    char ip[20];
    if (jsonStr(buf, "ip", ip, sizeof(ip)) && strcmp(ip, visionIp)) {
      strncpy(visionIp, ip, sizeof(visionIp) - 1);
      visionIp[sizeof(visionIp) - 1] = 0;
      Serial.printf("[CAM] vision node at %s\n", visionIp);
    }
    return;
  }

  if (!strstr(buf, "\"type\":\"alarm\"")) return;

  // [D] 화면 상태 갱신. 아래 inferEnabled 판정은 원본 그대로 두고, 그 앞에 화면용
  // 상태만 따로 뽑는다 — 추론 제어 로직에는 손대지 않는다.
  char state[24] = "";
  if (jsonStr(buf, "state", state, sizeof(state))) {
    if (!strcmp(state, "armed")) {
      long r = 0;
      jsonNum(buf, "remain_s", &r);
      armedAt = millis();
      armedTotal = (uint32_t)(r > 0 ? r : 0) * 1000UL;
      holdUntil = 0; dSt = S_ARMED;
    } else if (!strcmp(state, "ringing")) {
      holdUntil = 0; dSt = S_RINGING;
    } else if (!strcmp(state, "mission")) {
      jsonStr(buf, "label", mLabel, sizeof(mLabel));
      jsonStr(buf, "icon",  mIcon,  sizeof(mIcon));
      jsonStr(buf, "kind",  mKind,  sizeof(mKind));
      long h;
      if (jsonNum(buf, "hold_s", &h) && h > 0) needMs = (uint32_t)h * 1000UL;
      heldMs = 0;
      // 인트로 타이머를 미션마다 다시 시작하고 직전 프레임을 버린다 — 안 버리면
      // 이전 미션의 마지막 카메라 화면이 인트로 없이 한 순간 스쳐 보인다.
      missionAt = millis();
      camLen = 0;
      holdUntil = 0; dSt = S_MISSION;
    } else if (!strcmp(state, "fail")) {
      dSt = S_FAIL; pendingSt = S_MISSION;
      holdUntil = millis() + FAIL_HOLD_MS; heldMs = 0;
    } else if (!strcmp(state, "success")) {
      dSt = S_SUCCESS; pendingSt = S_SHUTDOWN;
      holdUntil = millis() + SUCCESS_HOLD_MS; heldMs = 0;
    } else if (!strcmp(state, "shutdown")) {
      holdUntil = 0; dSt = S_SHUTDOWN;
    } else if (!strcmp(state, "idle")) {
      holdUntil = 0; dSt = S_IDLE; heldMs = 0;
    }
  }

  bool en = inferEnabled;
  if (strstr(buf, "\"state\":\"mission\"")) {
#if EXCLUSIVE_MISSION
    en = strstr(buf, "\"kind\":\"gesture\"") != NULL;  // 사물 미션 중엔 침묵
#else
    en = true;                                        // 사물 미션 중에도 계속 발행
#endif
  } else if (strstr(buf, "\"state\":\"shutdown\"") || strstr(buf, "\"state\":\"success\"")) {
    en = false;
  } else {                                             // idle / armed / ringing
    en = true;
  }
  if (en != inferEnabled) {
    inferEnabled = en;
    Serial.printf("[CMD] infer %s\n", en ? "ON" : "OFF");
  }
}

void ensureMqtt() {
  if (mqtt.connected()) return;
  if (mqtt.connect(MQTT_ID)) {
    Serial.println("[MQTT] connected");
    mqtt.subscribe(CMD_TOPIC);       // [+] retained 라 붙자마자 현재 상태를 받는다
    mqtt.subscribe(UI_TOPIC);        // [D] 해제 게이지
    mqtt.subscribe(VIS_TOPIC);       // [D] 비전 노드 IP 를 알아내기 위해서만
  } else {
    Serial.printf("[MQTT] connect failed rc=%d\n", mqtt.state());
  }
}

void initSensors() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  hasBNO = bno.begin();
  if (hasBNO) { bno.setExtCrystalUse(true); delay(800); }

  hasBME = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
  hasBH  = lux.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);

  Serial.printf("[I2C] BNO055=%d  BME280=%d  BH1750=%d\n", hasBNO, hasBME, hasBH);
  if (!hasBNO) Serial.println("[I2C] BNO055 없음 - 추론 불가. 배선 확인 (SDA=5 SCL=6)");
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);      // 시리얼 모니터를 안 열어도 loop 가 멈추지 않게
  delay(500);
  Serial.println("\n==== alarmi GESTURE + CLOCK NODE ====");

  // [D] 화면을 센서보다 먼저 올린다 — I2C 스캔이 실패해도 화면은 뜨게 해서
  // "보드가 죽었나 센서가 없나" 를 화면만 보고 구분할 수 있다.
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(C_BG);
  // 240x240 16bpp = 115 KB. 못 잡으면 8bpp 로 물러선다 (30_clock_node 와 같은 처리).
  if (spr.createSprite(SCR, SCR) == nullptr) {
    Serial.println("[TFT] 16bpp 실패 - 8bpp 로 재시도");
    spr.setColorDepth(8);
    spr.createSprite(SCR, SCR);
  }
  spr.fillSprite(C_BG);

  initSensors();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WIFI] ok IP="); Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WIFI] 실패 - 계속 재시도");
  }

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onCmd);       // [+] cmd 다운링크
  // 원시센서(11종) + 추론결과를 한 메시지에 담으면 페이로드가 ~1 KB 나온다.
  // PubSubClient 버퍼는 페이로드뿐 아니라 토픽 + 헤더까지 같이 써서, 딱 1024 로 잡으면
  // publish 가 조용히 false 를 뱉는다. 여유 있게 2 KB.
  mqtt.setBufferSize(2048);
  Serial.printf("[MQTT] %s:%d  topic=%s\n", MQTT_HOST, MQTT_PORT, MQTT_TOPIC);

  // [D] JPEG 버퍼는 PSRAM 에서. 스프라이트가 이미 내부 RAM 을 115 KB 쓰고,
  // EI 아레나도 ei_malloc 으로 PSRAM 을 쓴다. 못 잡으면 뷰파인더만 조용히 꺼진다.
  camBuf = (uint8_t *)heap_caps_malloc(CAM_MAX_BYTES, MALLOC_CAP_SPIRAM);
  if (!camBuf) camBuf = (uint8_t *)malloc(CAM_MAX_BYTES);
  Serial.printf("[CAM] buffer %s\n", camBuf ? "ok" : "실패 - 뷰파인더 비활성");

  nextUs = micros();
  readSlowSensors();
}

void loop() {
  sampleTick();

  uint32_t now = millis();
  if (now - lastSlow >= SLOW_READ_MS) { lastSlow = now; readSlowSensors(); }

  // [D] 화면은 Wi-Fi 가 끊겨도 그린다 — 아래 early return 보다 앞에 둔다.
  // 안 그러면 브로커가 죽었을 때 화면이 마지막 프레임에 얼어붙는다.
  if (now - lastFrame >= FRAME_MS) { lastFrame = now; render(); }

  if (WiFi.status() != WL_CONNECTED) return;
  ensureMqtt();
  mqtt.loop();

  // [D] 카메라는 사물 미션의 인트로가 끝난 뒤에만 받아온다. 다른 상태에서는 비전 노드를
  // 건드리지 않는다 (그 보드는 700 ms 마다 추론도 돌린다).
  //
  // GET 이 최대 CAM_TIMEOUT_MS 동안 블로킹이라 그 사이 sampleTick 이 멈춘다. 사물 미션
  // 중이라 제스처 판정이 필요 없는 구간이지만, 링버퍼에 시간 간격이 어긋난 샘플이
  // 섞이면 미션이 제스처로 되돌아왔을 때 첫 추론이 틀린다. 그래서 GET 뒤에 nextUs 를
  // 다시 맞춰서, 밀린 만큼을 몰아서 읽지 않고 다음 주기부터 정상 간격으로 재개한다.
  if (camPhase(now) && now - camFetch >= CAM_FETCH_MS) {
    camFetch = now;
    camFetchFrame();
    nextUs = micros();                  // 밀린 샘플을 버스트로 채우지 않는다
    if (mqtt.connected()) mqtt.loop();  // 블로킹 동안 쌓인 cmd 를 즉시 비운다
  }

  // [+] inferEnabled 가 꺼져 있으면 추론·발행을 건너뛴다 (샘플링은 위에서 계속 돈다).
  if (inferEnabled && now - lastPub >= PUBLISH_MS) { lastPub = now; publishState(); }
}
