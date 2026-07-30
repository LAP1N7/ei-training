// 카메라를 별도 번역 단위로 격리한다.
//
// esp_camera.h 가 끌어오는 sensor.h 와 Adafruit_Sensor.h 가 둘 다 `sensor_t` 를
// 정의해서, 한 파일에서 같이 include 하면 conflicting declaration 으로 깨진다.
// 그래서 이 헤더는 esp_camera 타입을 하나도 노출하지 않는다 — .ino 에서는
// BME280/BH1750(= Adafruit_Sensor.h)을, camera.cpp 에서는 esp_camera.h 만 본다.
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// 카메라 초기화 + AE/AWB 수렴용 워밍업. 성공하면 true.
bool cameraInit();

// 한 프레임을 잡아 Edge Impulse 입력 형식(픽셀당 packed RGB 를 float 하나로)으로
// out[w*h] 에 채운다. 데스크에 눕혀 놓은 모듈 방향에 맞춰 CW 회전 보정을 적용한다.
// brightness 에는 프레임 평균 밝기(0~255)를 돌려준다 — 카메라의 원시 센서값.
bool cameraGrab(float *out, int w, int h, float *brightness);

// 한 프레임을 JPEG 로 인코딩해 *out / *len 에 돌려준다. 성공하면 true.
// 호출한 쪽이 반드시 cameraJpegFree() 로 돌려줘야 한다 (안 하면 PSRAM 이 샌다).
//
// 이 함수가 .ino 가 아니라 여기 있는 이유: esp_camera_fb_get()/frame2jpg() 가
// esp_camera.h 를 필요로 하는데, .ino 는 Adafruit_Sensor.h 때문에 그걸 못 본다.
// 그래서 esp_camera 타입이 안 새는 형태(uint8_t/size_t)로만 뚫어준다.
bool cameraJpeg(uint8_t **out, size_t *len);
void cameraJpegFree(uint8_t *buf);
