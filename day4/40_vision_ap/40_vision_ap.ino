// day4 — 시각 노드 (SoftAP 단독 동작 버전).
//
// 왜 이걸 따로 만들었나:
//   교실에 "projectbee" 라는 이름의 AP 가 두 대 떠 있고, 그중 하나는 자기 게이트웨이
//   조차 TCP 가 안 열리는 고립된 소프트웨어 AP 다. 보드가 부팅할 때 어느 쪽을 잡을지는
//   운이라서, 시각 보드는 계속 고립된 쪽(gateway 192.168.0.2)에 붙어 브로커에 닿지
//   못했다. (93_net_probe 로 확인: 힙 271KB 여유, 전원/메모리 문제 아님)
//
//   그래서 교실 AP 를 아예 안 쓴다. 이 보드가 자기 AP 를 띄우고 페이지도 직접 서빙한다.
//   폰이나 노트북을 이 AP 에 붙여 http://192.168.4.1 만 열면 끝 — 브로커도, 라우터도,
//   PC 쪽 프로그램도 필요 없다.
//
// 모델: test_inferencing (2 class: band / usb, 96x96)
// 카메라 코드는 day3/03_web_infer_sta 에서 검증된 것을 그대로 쓴다.
//
// 접속:  Wi-Fi "XIAO_VISION" / 비번 12345678  ->  http://192.168.4.1
//
// 빌드:
//   arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 40_vision_ap
//   arduino-cli upload -p COM9 --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi 40_vision_ap
#include <test_inferencing.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"

// EI 텐서 아레나를 PSRAM 으로 (MobileNet 은 내부 SRAM 에 안 들어간다)
void *ei_malloc(size_t size) {
  void *p = heap_caps_aligned_alloc(16, size, MALLOC_CAP_SPIRAM);
  if (!p) p = heap_caps_aligned_alloc(16, size, MALLOC_CAP_DEFAULT);
  return p;
}
void *ei_calloc(size_t n, size_t s) { void *p = ei_malloc(n * s); if (p) memset(p, 0, n * s); return p; }
void ei_free(void *ptr) { heap_caps_free(ptr); }

const char *AP_SSID = "XIAO_VISION";
const char *AP_PASS = "12345678";

// XIAO ESP32S3 Sense 카메라 핀맵
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

WebServer server(80);
static float features[EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT];
static float lastBright = 0;

static int get_feature_data(size_t offset, size_t length, float *out) {
  memcpy(out, features + offset, length * sizeof(float));
  return 0;
}

static const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>XIAO Vision Node</title>
<style>
:root{--bg:#0b0d12;--card:rgba(255,255,255,.04);--line:rgba(255,255,255,.09);
      --text:#eef1f7;--muted:#7d879c;--tint:#35d07f}
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;background:var(--bg);color:var(--text);padding:24px 16px 48px;
     font:15px/1.55 -apple-system,"Segoe UI",system-ui,sans-serif}
.wrap{max-width:460px;margin:0 auto;display:flex;flex-direction:column;gap:16px}
h1{font-size:19px;font-weight:680;letter-spacing:-.02em}
h1 span{color:var(--tint)}
.tag{font-size:11px;color:var(--muted);letter-spacing:.08em;text-transform:uppercase}
.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:16px;
      position:relative;overflow:hidden}
.card::before{content:"";position:absolute;inset:0 0 auto 0;height:2px;
              background:linear-gradient(90deg,var(--tint),transparent 72%)}
.sec{font-size:10.5px;font-weight:700;letter-spacing:.12em;text-transform:uppercase;
     color:var(--muted);margin-bottom:10px;display:flex;align-items:center;gap:8px}
.sec::after{content:"";flex:1;height:1px;background:var(--line)}
img{width:100%;aspect-ratio:1;object-fit:cover;border-radius:12px;display:block;background:#000}
.infer{display:flex;gap:16px;align-items:center}
.ring{position:relative;width:96px;height:96px;flex:none}
.ring svg{width:100%;height:100%;transform:rotate(-90deg)}
.ring .bg{fill:none;stroke:var(--line);stroke-width:9}
.ring .fg{fill:none;stroke:var(--tint);stroke-width:9;stroke-linecap:round;
          transition:stroke-dashoffset .4s cubic-bezier(.2,.8,.2,1)}
.ring .txt{position:absolute;inset:0;display:flex;flex-direction:column;
           align-items:center;justify-content:center}
.pct{font-size:21px;font-weight:650;font-variant-numeric:tabular-nums}
.cap{font-size:9.5px;color:var(--muted);letter-spacing:.1em;text-transform:uppercase}
.big{font-size:28px;font-weight:650;text-transform:uppercase;letter-spacing:-.02em}
.lat{font-size:11.5px;color:var(--muted);margin-top:5px;font-variant-numeric:tabular-nums}
.srow{display:grid;grid-template-columns:78px 1fr 42px;align-items:center;gap:10px;margin-top:9px}
.nm{font-size:12.5px;font-weight:550}
.tr{height:9px;border-radius:999px;background:#12151d;overflow:hidden;
    box-shadow:inset 0 0 0 1px var(--line)}
.fl{height:100%;width:0;border-radius:999px;background:var(--tint);
    transition:width .35s cubic-bezier(.2,.8,.2,1)}
.pc{font-size:12px;text-align:right;color:var(--muted);font-variant-numeric:tabular-nums}
.s{display:flex;justify-content:space-between;align-items:baseline;padding:7px 0}
.s+.s{border-top:1px solid var(--line)}
.k{font-size:13px;color:var(--muted)}
.v{font-size:17px;font-weight:550;font-variant-numeric:tabular-nums}
</style></head><body>
<div class="wrap">
  <div>
    <h1>XIAO <span>Vision Node</span></h1>
    <div class="tag">SoftAP · 192.168.4.1</div>
  </div>

  <div class="card">
    <div class="sec">카메라</div>
    <img id="cam" src="/jpg">
  </div>

  <div class="card">
    <div class="sec">추론 결과</div>
    <div class="infer">
      <div class="ring">
        <svg viewBox="0 0 100 100">
          <circle class="bg" cx="50" cy="50" r="43"></circle>
          <circle class="fg" id="ring" cx="50" cy="50" r="43"
                  stroke-dasharray="270.18" stroke-dashoffset="270.18"></circle>
        </svg>
        <div class="txt"><span class="pct" id="pct">—</span><span class="cap">conf</span></div>
      </div>
      <div>
        <div class="big" id="label">—</div>
        <div class="lat" id="lat">대기 중</div>
      </div>
    </div>
    <div id="scores"></div>
  </div>

  <div class="card">
    <div class="sec">원시 센서값</div>
    <div class="s"><span class="k">프레임 평균 밝기</span><span class="v" id="bright">—</span></div>
    <div class="s"><span class="k">업타임</span><span class="v" id="up">—</span></div>
  </div>
</div>
<script>
const RING_C = 2*Math.PI*43;
setInterval(()=>{document.getElementById('cam').src='/jpg?t='+Date.now()},1200);
async function tick(){
  try{
    const d = await (await fetch('/classify')).json();
    const conf = Math.max(0,Math.min(1,d.confidence||0));
    document.getElementById('label').textContent = d.label||'—';
    document.getElementById('pct').textContent = Math.round(conf*100)+'%';
    document.getElementById('ring').style.strokeDashoffset = (RING_C*(1-conf)).toFixed(2);
    const dsp=d.dsp_ms|0, nn=d.nn_ms|0;
    document.getElementById('lat').textContent = `dsp ${dsp}ms + nn ${nn}ms = ${dsp+nn}ms`;
    let h='';
    for(const [k,v] of Object.entries(d.scores||{})){
      h += `<div class="srow"><span class="nm">${k}</span>`+
           `<div class="tr"><div class="fl" style="width:${(v*100).toFixed(1)}%"></div></div>`+
           `<span class="pc">${Math.round(v*100)}%</span></div>`;
    }
    document.getElementById('scores').innerHTML = h;
    document.getElementById('bright').textContent = (d.brightness||0).toFixed(1);
    document.getElementById('up').textContent = (d.uptime_s|0)+'s';
  }catch(e){}
  setTimeout(tick,400);
}
tick();
</script></body></html>
)HTML";

void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleJpg() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "no frame"); return; }
  uint8_t *jpg = NULL; size_t jpgLen = 0;
  bool ok = frame2jpg(fb, 85, &jpg, &jpgLen);
  esp_camera_fb_return(fb);
  if (!ok) { server.send(500, "text/plain", "jpeg failed"); return; }
  server.setContentLength(jpgLen);
  server.send(200, "image/jpeg", "");
  server.client().write(jpg, jpgLen);
  free(jpg);
}

void handleClassify() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "application/json", "{\"error\":\"no frame\"}"); return; }

  // CW 회전 보정 — day3/03_web_infer_sta 에서 실측으로 맞춘 방향
  const int W = EI_CLASSIFIER_INPUT_WIDTH, H = EI_CLASSIFIER_INPUT_HEIGHT;
  const uint16_t *src = (const uint16_t *)fb->buf;
  double sum = 0;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      int sx = y * fb->width / H;
      int sy = fb->height - 1 - (x * fb->height / W);
      uint16_t px = src[sy * fb->width + sx];
      px = (px >> 8) | (px << 8);
      uint8_t r = ((px >> 11) & 0x1F) << 3;
      uint8_t g = ((px >> 5)  & 0x3F) << 2;
      uint8_t b = ( px        & 0x1F) << 3;
      features[y * W + x] = (float)((r << 16) | (g << 8) | b);
      sum += (r * 0.299 + g * 0.587 + b * 0.114);
    }
  }
  esp_camera_fb_return(fb);
  lastBright = (float)(sum / (W * H));

  signal_t signal;
  signal.total_length = W * H;
  signal.get_data = &get_feature_data;
  ei_impulse_result_t result = {0};
  if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) {
    server.send(500, "application/json", "{\"error\":\"classifier\"}");
    return;
  }

  int best = 0;
  String json = "{\"scores\":{";
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > result.classification[best].value) best = i;
    json += "\"" + String(ei_classifier_inferencing_categories[i]) + "\":" +
            String(result.classification[i].value, 3);
    if (i < EI_CLASSIFIER_LABEL_COUNT - 1) json += ",";
  }
  json += "},\"label\":\"" + String(ei_classifier_inferencing_categories[best]) + "\"";
  json += ",\"confidence\":" + String(result.classification[best].value, 3);
  json += ",\"brightness\":" + String(lastBright, 1);
  json += ",\"uptime_s\":" + String(millis() / 1000);
  json += ",\"dsp_ms\":" + String(result.timing.dsp);
  json += ",\"nn_ms\":" + String(result.timing.classification) + "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(500);

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;     config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;   config.pin_href  = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = -1; config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_240X240;
  config.pixel_format = PIXFORMAT_RGB565;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.fb_count     = 2;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("[CAM] init 실패 - 카메라 모듈/PSRAM 확인");
    while (true) delay(1000);
  }
  // AE/AWB 가 수렴할 때까지 몇 프레임 버린다
  for (int i = 0; i < 10; i++) {
    camera_fb_t *w = esp_camera_fb_get();
    if (w) esp_camera_fb_return(w);
    delay(60);
  }
  Serial.println("[CAM] ok");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", handleRoot);
  server.on("/jpg", handleJpg);
  server.on("/classify", handleClassify);
  server.begin();
}

void loop() {
  server.handleClient();

  // 시리얼 포트를 열면 보드가 리셋되므로, setup 에서만 찍는 로그는 USB CDC 재연결 전에
  // 날아간다. 접속 안내는 loop 에서 주기적으로 다시 찍는다.
  static uint32_t last = 0;
  if (millis() - last > 3000) {
    last = millis();
    Serial.printf("[AP] ssid=%s  http://%s  clients=%d  heap=%u\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str(),
                  WiFi.softAPgetStationNum(), ESP.getFreeHeap());
  }
}
