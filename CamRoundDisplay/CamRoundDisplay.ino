/*
  CamRoundDisplay.ino - Color Swap Fix & Horizontal Mirror Fix
  
  1. Fixes color degradation by enabling tft.setSwapBytes(true)
  2. Fixes horizontal mirror orientation via sensor set_hmirror(1)
  3. High quality JPEG quality (10) for clear image
*/

#define BOARD_SCREEN_COMBO 501

#include <TFT_eSPI.h>
#include <JPEGDecoder.h>
#include "esp_camera.h"

#define USER_LED 21

TFT_eSPI tft = TFT_eSPI();
camera_config_t cam_cfg;

void renderJPEG(uint8_t *buf, size_t len) {
  if (!JpegDec.decodeArray(buf, len)) return;

  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  while (JpegDec.read()) {
    pImg = JpegDec.pImage;
    int mcu_x = JpegDec.MCUx * mcu_w;
    int mcu_y = JpegDec.MCUy * mcu_h;

    uint32_t win_w = (mcu_x + mcu_w <= max_x) ? mcu_w : (max_x - mcu_x);
    uint32_t win_h = (mcu_y + mcu_h <= max_y) ? mcu_h : (max_y - mcu_y);

    if (mcu_x < 240 && mcu_y < 240) {
      tft.pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    }
  }
}

bool initCamera() {
  cam_cfg.ledc_channel = LEDC_CHANNEL_0;
  cam_cfg.ledc_timer   = LEDC_TIMER_0;
  cam_cfg.pin_pwdn     = -1;
  cam_cfg.pin_reset    = -1;
  cam_cfg.pin_xclk     = 10;
  cam_cfg.pin_sccb_sda = 40;
  cam_cfg.pin_sccb_scl = 39;
  cam_cfg.pin_d0       = 15;
  cam_cfg.pin_d1       = 17;
  cam_cfg.pin_d2       = 18;
  cam_cfg.pin_d3       = 16;
  cam_cfg.pin_d4       = 14;
  cam_cfg.pin_d5       = 12;
  cam_cfg.pin_d6       = 11;
  cam_cfg.pin_d7       = 48;
  cam_cfg.pin_vsync    = 38;
  cam_cfg.pin_href     = 47;
  cam_cfg.pin_pclk     = 13;
  cam_cfg.xclk_freq_hz = 20000000;
  cam_cfg.frame_size   = FRAMESIZE_240X240;
  cam_cfg.pixel_format = PIXFORMAT_JPEG;
  cam_cfg.grab_mode    = CAMERA_GRAB_LATEST;
  cam_cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  cam_cfg.jpeg_quality = 10; // High quality
  cam_cfg.fb_count     = 2;

  esp_err_t err = esp_camera_init(&cam_cfg);
  if (err != ESP_OK) return false;

  // Sensor Orientation Adjustments (Fix Mirroring & Orientation)
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_hmirror(s, 1); // Fix Horizontal Mirror (Flip left <-> right)
    s->set_vflip(s, 0);   // Vertical flip (0 = normal)
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(USER_LED, OUTPUT);

  // Initialize Display
  tft.init();
  tft.setRotation(0);
  tft.setSwapBytes(true); // Fix Color Distortion (RGB565 Byte Order Swap)
  tft.fillScreen(TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setTextSize(2);
  tft.setCursor(20, 110);
  tft.print("Starting Cam...");

  // Initialize Camera
  if (!initCamera()) {
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setCursor(20, 110);
    tft.print("Cam Init Fail!");
    while (true) {
      digitalWrite(USER_LED, LOW);
      delay(100);
      digitalWrite(USER_LED, HIGH);
      delay(100);
    }
  }

  tft.fillScreen(TFT_BLACK);
  Serial.println("Color Fixed & Mirror Corrected Camera Stream Ready!");
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  // Render High Quality JPEG Frame
  renderJPEG(fb->buf, fb->len);
  esp_camera_fb_return(fb);

  digitalWrite(USER_LED, (millis() / 200) % 2 ? LOW : HIGH);
}
