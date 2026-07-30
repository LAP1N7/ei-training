// day4 — 시각 노드 (원격 활성/정지판).  ★ 카메라 달린 보드(XIAO ESP32S3 Sense) 용 ★
//
// 20_vision_node 와 하는 일은 같지만, 대시보드에서 이 노드만 켜고 끌 수 있다.
//
//   대시보드 --(cmd)--> 보드     wearable/minseo/cmd/vision      {"active":true,"rate_ms":700}
//   보드 --(data)-----> 대시보드 wearable/minseo/vision          평소의 센서+추론 페이로드
//   보드 --(생사)-----> 대시보드 wearable/minseo/status/vision   "online" / "offline"(LWT)
//
// 명령은 retained 로 보내므로, 보드를 리셋해도 마지막 상태(켜짐/꺼짐)를 그대로 이어받는다.
// 꺼진 동안에는 카메라 캡처와 추론을 통째로 건너뛴다 — 이게 이 노드에서 제일 무거운
// 부분이라, 두 보드를 같이 켜 놓고 시연할 때 필요한 쪽만 돌릴 수 있다.
// 대신 I2C 원시 센서값은 1 초에 한 번 계속 올려서 "전원은 들어와 있음" 을 보여준다.
//
// 모델: test_inferencing  (2 class: band / usb, 96x96)
// I2C 원시센서(있으면 자동 인식):  0x23 BH1750,  0x76/0x77 BME280
//   카메라는 SCCB(GPIO40/39)를 쓰므로 Wire(GPIO5/6)와 충돌하지 않는다.
//
// 빌드:
//   arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 51_vision_ctrl
//   arduino-cli upload -p COM7 --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 51_vision_ctrl
#include <test_inferencing.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include "camera.h"          // esp_camera.h 는 camera.cpp 안에만 있다 (헤더 충돌 회피)
#include "esp_heap_caps.h"

// EI 텐서 아레나를 PSRAM 으로 (MobileNet 은 내부 SRAM 에 안 들어간다)
void *ei_malloc(size_t size) {
  void *p = heap_caps_aligned_alloc(16, size, MALLOC_CAP_SPIRAM);
  if (!p) p = heap_caps_aligned_alloc(16, size, MALLOC_CAP_DEFAULT);
  return p;
}
void *ei_calloc(size_t n, size_t s) { void *p = ei_malloc(n * s); if (p) memset(p, 0, n * s); return p; }
void ei_free(void *ptr) { heap_caps_free(ptr); }

// ---- config ----
const char *WIFI_SSID   = "projectbee";
const char *WIFI_PASS   = "honeybear!";
const char *MQTT_HOST   = "192.168.0.27";
const int   MQTT_PORT   = 1883;
const char *NODE_ID     = "vision";
const char *MQTT_ID     = "xiao-minseo-node-vision-ctrl";     // 노드마다 반드시 달라야 한다
const char *TOPIC_DATA  = "wearable/minseo/vision";
const char *TOPIC_CMD   = "wearable/minseo/cmd/vision";
const char *TOPIC_STATE = "wearable/minseo/status/vision";

#define SDA_PIN      5
#define SCL_PIN      6
#define DEFAULT_RATE 700    // 카메라 추론은 무거워서 제스처보다 느리게
#define IDLE_RATE    1000
#define RATE_MIN     300
#define RATE_MAX     10000
#define SLOW_READ_MS 1000

Adafruit_BME280 bme;
BH1750          lux;
WiFiClient net;
PubSubClient mqtt(net);

bool hasBME = false, hasBH = false;

// ---- 원격 제어 상태 ----
static bool     gActive = true;              // 명령이 오기 전 기본값: 켜짐
static uint32_t gRateMs = DEFAULT_RATE;

static float sLux = NAN, sTemp = NAN, sHum = NAN, sPres = NAN;
static float sBright = NAN;                  // 프레임 평균 밝기 (0~255) — 카메라 자체의 원시값
static uint32_t lastSlow = 0, lastPub = 0, lastHb = 0;
static uint32_t pubOk = 0, pubFail = 0;

static float features[EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT];
static int get_feature_data(size_t offset, size_t length, float *out) {
  memcpy(out, features + offset, length * sizeof(float));
  return 0;
}

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
}

static void addSensor(String &j, bool &first, const char *key, float v,
                      const char *unit, const char *label, float lo, float hi) {
  if (isnan(v)) return;
  if (!first) j += ",";
  first = false;
  j += "\"" + String(key) + "\":{\"v\":" + String(v, 2) +
       ",\"unit\":\"" + unit + "\",\"label\":\"" + label +
       "\",\"min\":" + String(lo, 1) + ",\"max\":" + String(hi, 1) + "}";
}

// ---- 명령 수신 ----
//
// 대시보드는 JSON 을 보내지만, mosquitto_pub 으로 손으로 때려볼 때가 훨씬 많으므로
// "on" / "off" / "1" / "0" 같은 맨몸 페이로드도 그대로 받아준다.
//
//   mosquitto_pub -h 192.168.0.27 -t wearable/minseo/cmd/vision -r -m off
//   mosquitto_pub -h 192.168.0.27 -t wearable/minseo/cmd/vision -r -m "{\"active\":true,\"rate_ms\":1500}"
void onCommand(char *topic, byte *payload, unsigned int len) {
  String p;
  p.reserve(len + 1);
  for (unsigned int i = 0; i < len; i++) p += (char)payload[i];
  p.trim();
  Serial.printf("[CMD] %s <- %s\n", topic, p.c_str());
  if (!p.length()) return;

  String low = p; low.toLowerCase();

  if (low == "on" || low == "1" || low == "true" || low == "start") {
    gActive = true;
  } else if (low == "off" || low == "0" || low == "false" || low == "stop") {
    gActive = false;
  } else {
    int a = low.indexOf("\"active\"");
    if (a >= 0) {
      int c = low.indexOf(':', a);
      if (c >= 0) {
        // 값의 첫 글자만 보면 된다: t/1 -> 켜짐, f/0 -> 꺼짐
        int v = c + 1;
        while (v < (int)low.length() && (low[v] == ' ' || low[v] == '"')) v++;
        if (v < (int)low.length()) gActive = (low[v] == 't' || low[v] == '1');
      }
    }
    int r = low.indexOf("\"rate_ms\"");
    if (r >= 0) {
      int c = low.indexOf(':', r);
      if (c >= 0) {
        long ms = low.substring(c + 1).toInt();
        if (ms >= RATE_MIN && ms <= RATE_MAX) gRateMs = (uint32_t)ms;
      }
    }
  }

  Serial.printf("[CMD] -> active=%d rate=%lums\n", gActive, gRateMs);
  lastPub = 0;                    // 다음 loop 에서 곧바로 한 번 발행해 반응을 즉시 보여준다
}

void publishState() {
  String j = "{\"node\":\"" + String(NODE_ID) + "\",\"kind\":\"camera\"";
  j += ",\"active\":" + String(gActive ? "true" : "false");
  j += ",\"rate_ms\":" + String(gRateMs);
  j += ",\"uptime_s\":" + String(millis() / 1000);
  j += ",\"rssi\":" + String(WiFi.RSSI());

  // 꺼져 있으면 카메라 캡처 자체를 건너뛴다 (이 노드에서 제일 비싼 부분)
  bool haveFrame = false;
  if (gActive) {
    haveFrame = cameraGrab(features, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, &sBright);
    if (!haveFrame) Serial.println("[CAM] 프레임 획득 실패");
  }

  j += ",\"sensors\":{";
  bool first = true;
  addSensor(j, first, "lux",        sLux,    "lx",  "조도",        0, 1000);
  addSensor(j, first, "temp",       sTemp,   "°C",  "온도",        0, 50);
  addSensor(j, first, "humidity",   sHum,    "%",   "습도",        0, 100);
  addSensor(j, first, "pressure",   sPres,   "hPa", "기압",        950, 1050);
  if (gActive) addSensor(j, first, "brightness", sBright, "", "프레임 밝기", 0, 255);
  j += "}";

  if (haveFrame) {
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &get_feature_data;
    ei_impulse_result_t result = {0};
    if (run_classifier(&signal, &result, false) == EI_IMPULSE_OK) {
      int best = 0;
      for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > result.classification[best].value) best = i;
      }

      j += ",\"inference\":{\"label\":\"" + String(ei_classifier_inferencing_categories[best]) + "\"";
      j += ",\"confidence\":" + String(result.classification[best].value, 3);
      j += ",\"scores\":{";
      for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (i) j += ",";
        j += "\"" + String(ei_classifier_inferencing_categories[i]) + "\":" +
             String(result.classification[i].value, 3);
      }
      j += "},\"dsp_ms\":" + String(result.timing.dsp);
      j += ",\"nn_ms\":" + String(result.timing.classification) + "}";
    }
  }
  j += "}";

  if (mqtt.publish(TOPIC_DATA, j.c_str())) pubOk++;
  else { pubFail++; Serial.printf("[MQTT] publish failed (len=%u)\n", j.length()); }
}

// 재연결은 반드시 간격을 두고 시도한다. loop 마다 connect() 를 부르면 실패할 때마다
// 소켓이 하나씩 물려서 LWIP 소켓이 고갈되고, 그때부터는 무조건 rc=-2 로 떨어진다.
static uint32_t lastMqttTry = 0;
void ensureMqtt() {
  if (mqtt.connected()) return;
  uint32_t now = millis();
  if (lastMqttTry && now - lastMqttTry < 3000) return;
  lastMqttTry = now;

  // LWT: 보드가 말없이 사라지면 브로커가 대신 offline 을 뿌려준다.
  if (mqtt.connect(MQTT_ID, TOPIC_STATE, 0, true, "offline")) {
    Serial.println("[MQTT] connected");
    mqtt.publish(TOPIC_STATE, "online", true);
    // retained 명령이 걸려 있으면 구독하는 즉시 되돌아온다 = 리셋해도 상태 유지
    mqtt.subscribe(TOPIC_CMD);
    Serial.printf("[MQTT] subscribe %s\n", TOPIC_CMD);
  } else {
    Serial.printf("[MQTT] connect failed rc=%d\n", mqtt.state());
  }
}

// 교실에 "projectbee" 라는 이름의 AP 가 두 대 떠 있다. 한 대는 정상(게이트웨이
// 192.168.0.1), 다른 한 대는 자기 게이트웨이조차 TCP 가 안 열리는 고립된 소프트웨어
// AP(192.168.0.2)다. 그래서 "붙었다"로 끝내지 않고 브로커까지 TCP 가 열리는지 확인한다.
#define MAX_CAND 8
struct Cand { uint8_t bssid[6]; int32_t ch; int32_t rssi; };

bool connectVerified() {
  Serial.println("[WIFI] AP 스캔...");
  int n = WiFi.scanNetworks();
  if (n <= 0) { Serial.println("[WIFI] AP 없음"); WiFi.scanDelete(); return false; }

  // 후보를 먼저 복사해둔다. WiFi.begin() 을 부르면 스캔 결과가 무효화될 수 있어서,
  // 스캔 결과를 들고 순회하면 두 번째 후보부터 쓰레기 값을 읽게 된다.
  Cand cand[MAX_CAND];
  int nc = 0;
  for (int i = 0; i < n && nc < MAX_CAND; i++) {
    if (WiFi.SSID(i) != WIFI_SSID) continue;
    memcpy(cand[nc].bssid, WiFi.BSSID(i), 6);
    cand[nc].ch   = WiFi.channel(i);
    cand[nc].rssi = WiFi.RSSI(i);
    nc++;
  }
  WiFi.scanDelete();
  Serial.printf("[WIFI] '%s' 후보 %d 개\n", WIFI_SSID, nc);
  if (!nc) return false;

  for (int i = 0; i < nc; i++) {
    char b[18];
    snprintf(b, sizeof(b), "%02X:%02X:%02X:%02X:%02X:%02X",
             cand[i].bssid[0], cand[i].bssid[1], cand[i].bssid[2],
             cand[i].bssid[3], cand[i].bssid[4], cand[i].bssid[5]);
    Serial.printf("[WIFI] 후보 %s ch=%d RSSI=%d\n", b, (int)cand[i].ch, (int)cand[i].rssi);

    WiFi.begin(WIFI_SSID, WIFI_PASS, cand[i].ch, cand[i].bssid);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(200);
    if (WiFi.status() != WL_CONNECTED) { Serial.println("[WIFI]   연결 실패"); continue; }

    WiFiClient probe;
    bool reachable = probe.connect(MQTT_HOST, MQTT_PORT, 3000);
    probe.stop();
    Serial.printf("[WIFI]   IP=%s gw=%s broker=%s\n",
                  WiFi.localIP().toString().c_str(),
                  WiFi.gatewayIP().toString().c_str(),
                  reachable ? "OK" : "unreachable");
    if (reachable) return true;

    WiFi.disconnect(false);      // true 로 주면 Wi-Fi 자체가 꺼져 다음 시도가 실패한다
    delay(300);
  }
  Serial.println("[WIFI] 브로커에 닿는 AP 를 못 찾음");
  return false;
}

// 정상 동작할 때 아무것도 안 찍으면 "멈춤"과 "정상"을 구분할 수 없다.
void heartbeat() {
  Serial.printf("[HB] up=%lus active=%d rate=%lums wifi=%d rssi=%d ip=%s mqtt=%d(state %d) pub ok=%lu fail=%lu heap=%u\n",
                millis() / 1000, gActive, gRateMs, WiFi.status(), WiFi.RSSI(),
                WiFi.localIP().toString().c_str(),
                mqtt.connected(), mqtt.state(), pubOk, pubFail, ESP.getFreeHeap());
}

void initSensors() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  hasBME = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
  hasBH  = lux.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  Serial.printf("[I2C] BME280=%d  BH1750=%d\n", hasBME, hasBH);
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(500);
  Serial.println("\n==== day4 VISION NODE (remote on/off) ====");

  if (!cameraInit()) {
    Serial.println("[CAM] init 실패 - 카메라 모듈/PSRAM 확인 (Sense 확장보드가 끝까지 꽂혔는지)");
    while (true) delay(1000);
  }
  Serial.println("[CAM] ok");
  initSensors();

  WiFi.mode(WIFI_STA);
  if (!connectVerified()) Serial.println("[WIFI] 일단 진행 - loop 에서 재시도");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onCommand);
  mqtt.setBufferSize(2048);      // 페이로드 + 토픽 + 헤더가 다 들어가야 한다
  Serial.printf("[MQTT] %s:%d  data=%s  cmd=%s\n", MQTT_HOST, MQTT_PORT, TOPIC_DATA, TOPIC_CMD);

  readSlowSensors();
}

void loop() {
  uint32_t now = millis();
  if (now - lastHb   >= 5000)         { lastHb = now;   heartbeat(); }
  if (now - lastSlow >= SLOW_READ_MS) { lastSlow = now; readSlowSensors(); }

  // 끊기면 다시 "브로커에 닿는 AP" 부터 고른다 (30 초에 한 번)
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastScan = 0;
    if (now - lastScan >= 30000) { lastScan = now; connectVerified(); }
    return;
  }
  ensureMqtt();
  if (!mqtt.connected()) return;      // 끊긴 상태에서 카메라 추론까지 돌릴 이유가 없다
  mqtt.loop();                        // 명령은 여기서 들어온다 — 빠뜨리면 구독이 죽은 것처럼 보인다

  uint32_t period = gActive ? gRateMs : IDLE_RATE;
  if (now - lastPub >= period) { lastPub = now; publishState(); }
}
