// 카메라를 별도 번역 단위로 격리한다.
//
// esp_camera.h 가 끌어오는 sensor.h 와 Adafruit_Sensor.h 가 둘 다 `sensor_t` 를
// 정의해서, 한 파일에서 같이 include 하면 conflicting declaration 으로 깨진다.
// 그래서 이 헤더는 esp_camera 타입을 하나도 노출하지 않는다 — .ino 에서는
// BME280/BH1750(= Adafruit_Sensor.h)을, camera.cpp 에서는 esp_camera.h 만 본다.
#pragma once
#include <stdbool.h>

// 카메라 초기화 + AE/AWB 수렴용 워밍업. 성공하면 true.
bool cameraInit();

// 한 프레임을 잡아 Edge Impulse 입력 형식(픽셀당 packed RGB 를 float 하나로)으로
// out[w*h] 에 채운다. 데스크에 눕혀 놓은 모듈 방향에 맞춰 CW 회전 보정을 적용한다.
// brightness 에는 프레임 평균 밝기(0~255)를 돌려준다 — 카메라의 원시 센서값.
bool cameraGrab(float *out, int w, int h, float *brightness);
