// IMU data-collection node — XIAO ESP32S3 + BNO055
// Streams linear acceleration (gravity removed, m/s^2) as CSV "ax,ay,az"
// over serial at a FIXED 100 Hz. The PC-side Python tool slices this into
// labeled windows and uploads them to Edge Impulse.
//
// Wiring (I2C0): SDA -> D4/GPIO5, SCL -> D5/GPIO6, VCC -> 3V3, GND -> GND
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define SDA_PIN   5
#define SCL_PIN   6
#define SAMPLE_HZ 100

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
uint32_t nextUs = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);          // keep running even if no monitor attached
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);             // 400 kHz so 100 Hz reads keep up
  if (!bno.begin()) {
    Serial.println("# ERROR: BNO055 not found at 0x29");
    while (true) delay(1000);
  }
  bno.setExtCrystalUse(true);
  delay(1000);                        // let fusion settle
  Serial.println("# BNO055 ready: ax,ay,az linear-accel m/s^2 @100Hz");
  nextUs = micros();
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextUs) >= 0) {
    nextUs += 1000000UL / SAMPLE_HZ;
    sensors_event_t la;
    bno.getEvent(&la, Adafruit_BNO055::VECTOR_LINEARACCEL);
    Serial.printf("%.3f,%.3f,%.3f\n",
                  la.acceleration.x, la.acceleration.y, la.acceleration.z);
  }
}
