// 비전 노드 + 카메라 화면 (day4/20_vision_node 의 사본 + 웹서버)
//
// 원본 20_vision_node 는 STA + MQTT 라 대시보드에 붙지만 그림을 못 준다.
// 40_vision_ap 는 /jpg 로 그림을 주지만 WiFi.mode(WIFI_AP) 라 브로커에 못 붙는다.
// 이 스케치는 둘을 합친다 — STA 로 브로커에 발행하면서, 동시에 자기 IP 의
// /jpg 로 카메라 프레임을 내준다. 대시보드는 <img> 로 그걸 폴링한다.
//
//   추론 결과 → MQTT (wearable/minseo/vision)
//   카메라 그림 → HTTP  http://<보드IP>/jpg
//
// JPEG 를 MQTT 로 보내지 않는 이유: 프레임 하나가 10~20 KB 라 브로커 버퍼를
// 그냥 터뜨린다 (이미 len=1002 에서 겪은 그 문제).
//
// 원본 대비 바뀐 곳은 [+] 로 표시했다.
//
// 빌드:
//   arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 21_vision_node_jpg
//   arduino-cli upload -p COMx --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 21_vision_node_jpg
#include <test_inferencing.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WebServer.h>       // [+]
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
const char *WIFI_SSID  = "projectbee";
const char *WIFI_PASS  = "honeybear!";
const char *MQTT_HOST  = "192.168.0.27";
const int   MQTT_PORT  = 1883;
const char *MQTT_TOPIC = "wearable/minseo/vision";
const char *MQTT_ID    = "xiao-minseo-node-vision";
const char *NODE_ID    = "vision";

#define SDA_PIN      5
#define SCL_PIN      6
#define PUBLISH_MS   700    // 카메라 추론은 무거워서 제스처보다 느리게
#define SLOW_READ_MS 1000

Adafruit_BME280 bme;
BH1750          lux;
WiFiClient net;
PubSubClient mqtt(net);
WebServer server(80);        // [+]

bool hasBME = false, hasBH = false;
static float sLux = NAN, sTemp = NAN, sHum = NAN, sPres = NAN;
static float sBright = NAN;                  // 프레임 평균 밝기 (0~255) — 카메라 자체의 원시값
static uint32_t lastSlow = 0, lastPub = 0, lastHb = 0;
static uint32_t pubOk = 0, pubFail = 0;
static uint32_t jpgOk = 0, jpgFail = 0;      // [+]

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

void publishState() {
  if (!cameraGrab(features, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, &sBright)) {
    Serial.println("[CAM] 프레임 획득 실패");
    return;
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &get_feature_data;
  ei_impulse_result_t result = {0};
  if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) return;

  int best = 0;
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > result.classification[best].value) best = i;
  }

  String j = "{\"node\":\"" + String(NODE_ID) + "\",\"kind\":\"camera\"";
  j += ",\"uptime_s\":" + String(millis() / 1000);
  j += ",\"rssi\":" + String(WiFi.RSSI());

  j += ",\"sensors\":{";
  bool first = true;
  addSensor(j, first, "lux",        sLux,    "lx",  "조도",        0, 1000);
  addSensor(j, first, "temp",       sTemp,   "°C",  "온도",        0, 50);
  addSensor(j, first, "humidity",   sHum,    "%",   "습도",        0, 100);
  addSensor(j, first, "pressure",   sPres,   "hPa", "기압",        950, 1050);
  addSensor(j, first, "brightness", sBright, "",    "프레임 밝기", 0, 255);
  j += "}";

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

  if (mqtt.publish(MQTT_TOPIC, j.c_str())) pubOk++;
  else { pubFail++; Serial.printf("[MQTT] publish failed (len=%u)\n", j.length()); }
}

// [+] 카메라 그림. 인코딩 자체는 camera.cpp 가 한다 (esp_camera 타입이 여기 오면 안 된다).
void handleJpg() {
  uint8_t *jpg = NULL;
  size_t   len = 0;
  if (!cameraJpeg(&jpg, &len)) {
    jpgFail++;
    server.send(503, "text/plain", "no frame");
    return;
  }
  server.setContentLength(len);
  server.send(200, "image/jpeg", "");
  server.client().write(jpg, len);
  cameraJpegFree(jpg);            // 안 돌려주면 PSRAM 이 프레임마다 샌다
  jpgOk++;
}

// [+] 대시보드에 넣을 IP 를 브라우저로 바로 확인할 수 있게
void handleRoot() {
  String s = "vision node\n";
  s += "ip: "   + WiFi.localIP().toString() + "\n";
  s += "jpg: http://" + WiFi.localIP().toString() + "/jpg\n";
  s += "mqtt: " + String(mqtt.connected() ? "connected" : "down") + "\n";
  server.send(200, "text/plain", s);
}

// 재연결은 반드시 간격을 두고 시도한다. loop 마다 connect() 를 부르면 실패할 때마다
// 소켓이 하나씩 물려서 LWIP 소켓이 고갈되고, 그때부터는 무조건 rc=-2 로 떨어진다.
static uint32_t lastMqttTry = 0;
void ensureMqtt() {
  if (mqtt.connected()) return;
  uint32_t now = millis();
  if (lastMqttTry && now - lastMqttTry < 3000) return;
  lastMqttTry = now;
  if (mqtt.connect(MQTT_ID)) Serial.println("[MQTT] connected");
  else Serial.printf("[MQTT] connect failed rc=%d\n", mqtt.state());
}

// 교실에 "projectbee" 라는 이름의 AP 가 두 대 떠 있다. 한 대는 정상(게이트웨이
// 192.168.0.1), 다른 한 대는 자기 게이트웨이조차 TCP 가 안 열리는 고립된 소프트웨어
// AP(192.168.0.2)다. 이 보드는 계속 고립된 쪽에 잡혀서 브로커에 못 붙었다.
//
// 그래서 "붙었다"로 끝내지 않고 브로커까지 실제로 TCP 가 열리는지 확인한다.
// 안 열리면 그 AP 를 버리고 다음 후보로 넘어간다.
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
  Serial.printf("[HB] up=%lus wifi=%d rssi=%d ip=%s mqtt=%d(state %d) pub ok=%lu fail=%lu jpg ok=%lu fail=%lu heap=%u\n",
                millis() / 1000, WiFi.status(), WiFi.RSSI(),
                WiFi.localIP().toString().c_str(),
                mqtt.connected(), mqtt.state(), pubOk, pubFail, jpgOk, jpgFail, ESP.getFreeHeap());
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
  Serial.println("\n==== VISION NODE + /jpg ====");

  if (!cameraInit()) {
    Serial.println("[CAM] init 실패 - 카메라 모듈/PSRAM 확인");
    while (true) delay(1000);
  }
  Serial.println("[CAM] ok");
  initSensors();

  WiFi.mode(WIFI_STA);
  if (!connectVerified()) Serial.println("[WIFI] 일단 진행 - loop 에서 재시도");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(2048);      // 페이로드 + 토픽 + 헤더가 다 들어가야 한다
  Serial.printf("[MQTT] %s:%d  topic=%s\n", MQTT_HOST, MQTT_PORT, MQTT_TOPIC);

  // [+] 웹서버. MQTT 와 무관하게 뜬다.
  server.on("/",    handleRoot);
  server.on("/jpg", handleJpg);
  server.begin();
  Serial.printf("[HTTP] http://%s/jpg  ← 대시보드 카메라 칸에 이 IP\n",
                WiFi.localIP().toString().c_str());

  readSlowSensors();
}

void loop() {
  uint32_t now = millis();

  // [+] 반드시 아래 early return 들보다 먼저. MQTT 가 끊긴 동안에도 카메라 화면은
  //     계속 나와야 하는데, 원본 loop 는 !mqtt.connected() 에서 그냥 return 해버린다.
  server.handleClient();

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
  mqtt.loop();

  // 추론이 도는 동안에는 handleClient() 가 못 돌아서 화면이 잠깐 멈춘다.
  // 화면이 너무 끊기면 PUBLISH_MS 를 늘려라 (추론 주기 ↔ 화면 부드러움 맞바꾸기).
  if (now - lastPub >= PUBLISH_MS) { lastPub = now; publishState(); }
}
