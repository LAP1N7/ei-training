// 카메라 전용 번역 단위. 여기서는 esp_camera.h 만 보고, Adafruit_Sensor.h 는
// 절대 include 하지 않는다 (camera.h 주석 참고).
#include <Arduino.h>          // .ino 와 달리 .cpp 는 자동으로 안 붙는다 (delay 등)
#include "camera.h"
#include "esp_camera.h"

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

bool cameraInit() {
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

  if (esp_camera_init(&config) != ESP_OK) return false;

  // 자동노출/화이트밸런스가 수렴할 때까지 몇 프레임 버린다
  for (int i = 0; i < 10; i++) {
    camera_fb_t *w = esp_camera_fb_get();
    if (w) esp_camera_fb_return(w);
    delay(60);
  }
  return true;
}

bool cameraGrab(float *out, int w, int h, float *brightness) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;

  const uint16_t *src = (const uint16_t *)fb->buf;
  double sum = 0;
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      // CW 회전 보정 — day3/03_web_infer_sta 에서 실측으로 맞춘 방향
      int sx = y * fb->width / h;
      int sy = fb->height - 1 - (x * fb->height / w);
      uint16_t px = src[sy * fb->width + sx];
      px = (px >> 8) | (px << 8);                 // RGB565 는 빅엔디안으로 들어온다
      uint8_t r = ((px >> 11) & 0x1F) << 3;
      uint8_t g = ((px >> 5)  & 0x3F) << 2;
      uint8_t b = ( px        & 0x1F) << 3;
      out[y * w + x] = (float)((r << 16) | (g << 8) | b);
      sum += (r * 0.299 + g * 0.587 + b * 0.114);
    }
  }
  esp_camera_fb_return(fb);

  if (brightness) *brightness = (float)(sum / (w * h));
  return true;
}
