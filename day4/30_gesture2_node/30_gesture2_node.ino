// day4 — SENSOR NODE 3 (gesture2). "상대팀 노드" 역할.
//
// 10_gesture_node 와 동작은 똑같고 MQTT 토픽 / 클라이언트 ID / 노드 이름만 다르다.
// 같은 망의 같은 브로커에 각자 publish 하면 대시보드가 카드를 따로 그린다 —
// 다른 팀 노드를 받아오는 것도 정확히 이 방식이다.
//
// 클라이언트 ID 가 겹치면 브로커가 먼저 붙어 있던 쪽을 끊어버리므로 반드시 달라야 한다.
//
// 배선 (I2C0):  SDA -> D4(GPIO5),  SCL -> D5(GPIO6),  VCC -> 3V3,  GND -> GND
//   0x23  BH1750  조도
//   0x29  BNO055  9축 IMU   <- 추론 입력
//   0x76  BME280  온도/습도/기압
//
// 빌드:
//   arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 30_gesture2_node
//   arduino-cli upload -p COM10 --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 30_gesture2_node
#include <gesture_inferencing.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
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
const char *MQTT_TOPIC = "wearable/minseo/gesture2";
const char *MQTT_ID    = "xiao-minseo-node-gesture2";   // 1번 노드와 절대 겹치면 안 된다
const char *NODE_ID    = "gesture2";

#define SDA_PIN     5
#define SCL_PIN     6
#define PUBLISH_MS  250    // 추론 + 발행 주기 (4 Hz)
#define SLOW_READ_MS 1000  // BME280 / BH1750 갱신 주기

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BME280 bme;
BH1750          lux;
WiFiClient net;
PubSubClient mqtt(net);

bool hasBNO = false, hasBME = false, hasBH = false;

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
static uint32_t lastSlow = 0, lastPub = 0, lastHb = 0;
static uint32_t pubOk = 0, pubFail = 0;

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

  if (mqtt.publish(MQTT_TOPIC, j.c_str())) pubOk++;
  else { pubFail++; Serial.printf("[MQTT] publish failed (len=%u)\n", j.length()); }
}

// 정상 동작할 때 아무것도 안 찍으면 "멈춤"과 "정상"을 구분할 수 없다.
// 5 초에 한 번 상태를 뱉어서 살아있는지, 어디서 막혔는지 바로 보이게 한다.
void heartbeat() {
  Serial.printf("[HB] up=%lus wifi=%d rssi=%d ip=%s mqtt=%d(state %d) pub ok=%lu fail=%lu heap=%u\n",
                millis() / 1000, WiFi.status(), WiFi.RSSI(),
                WiFi.localIP().toString().c_str(),
                mqtt.connected(), mqtt.state(), pubOk, pubFail, ESP.getFreeHeap());
}

// 재연결은 간격을 두고 시도한다. loop 마다 connect() 를 부르면 실패할 때마다 소켓이
// 하나씩 물려서 LWIP 소켓이 고갈되고, 그다음부터는 무조건 rc=-2 로 떨어진다.
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
// AP(192.168.0.2)다. 이 자리에서는 고장난 쪽 신호가 더 세서, SSID 만 보고 붙으면
// 매번 그쪽에 잡힌다. (93_net_probe 로 확인)
//
// 그래서 "붙었다"로 끝내지 않고 브로커까지 실제로 TCP 가 열리는지 확인한다.
// 안 열리면 그 AP 를 버리고 다음 후보로 넘어간다. BSSID 를 박아두지 않으므로
// 자리를 옮기거나 AP 가 바뀌어도 그대로 동작한다.
#define MAX_CAND 8
struct Cand { uint8_t bssid[6]; int32_t ch; int32_t rssi; };

bool connectVerified() {
  Serial.println("[WIFI] AP 스캔...");
  int n = WiFi.scanNetworks();
  if (n <= 0) { Serial.println("[WIFI] AP 없음"); WiFi.scanDelete(); return false; }

  // 후보를 먼저 복사해둔다. WiFi.begin() 을 부르면 스캔 결과가 무효화될 수 있어서,
  // 스캔 결과를 들고 다니면서 순회하면 두 번째 후보부터 쓰레기 값을 읽게 된다.
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

    // 진짜 판정 기준: 브로커까지 TCP 가 열리는가.
    // 이 교실엔 같은 SSID 로 뜬 고립 AP 가 있어서 "붙었다"만으론 부족하다.
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
  Serial.println("\n==== day4 GESTURE NODE ====");

  initSensors();

  WiFi.mode(WIFI_STA);
  if (!connectVerified()) Serial.println("[WIFI] 일단 진행 - loop 에서 재시도");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  // 원시센서(11종) + 추론결과를 한 메시지에 담으면 페이로드가 ~1 KB 나온다.
  // PubSubClient 버퍼는 페이로드뿐 아니라 토픽 + 헤더까지 같이 써서, 딱 1024 로 잡으면
  // publish 가 조용히 false 를 뱉는다. 여유 있게 2 KB.
  mqtt.setBufferSize(2048);
  Serial.printf("[MQTT] %s:%d  topic=%s\n", MQTT_HOST, MQTT_PORT, MQTT_TOPIC);

  nextUs = micros();
  readSlowSensors();
}

void loop() {
  sampleTick();

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
  mqtt.loop();
  if (now - lastPub >= PUBLISH_MS) { lastPub = now; publishState(); }
}
