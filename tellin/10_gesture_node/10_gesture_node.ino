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
const char *MQTT_TOPIC = "wearable/minseo/gesture";
const char *MQTT_ID    = "xiao-minseo-node-gesture";
const char *NODE_ID    = "gesture";

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

void ensureMqtt() {
  if (mqtt.connected()) return;
  if (mqtt.connect(MQTT_ID)) Serial.println("[MQTT] connected");
  else Serial.printf("[MQTT] connect failed rc=%d\n", mqtt.state());
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
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WIFI] ok IP="); Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WIFI] 실패 - 계속 재시도");
  }

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
  if (now - lastSlow >= SLOW_READ_MS) { lastSlow = now; readSlowSensors(); }

  if (WiFi.status() != WL_CONNECTED) return;
  ensureMqtt();
  mqtt.loop();
  if (now - lastPub >= PUBLISH_MS) { lastPub = now; publishState(); }
}
