// day4 — I2C 배선 추적기.
//
// "물리적으로 다 연결했는데 스캔이 0개" 일 때 쓴다. 두 가지를 본다:
//
//  1) 버스 전기 상태 — 내부 풀업만 켜고 각 핀을 읽는다.
//       HIGH = 풀업이 살아있음 (정상 대기 상태)
//       LOW  = 누가 버스를 잡고 있음 (합선, 또는 GND 에 붙음)
//     둘 다 HIGH 인데 장치가 안 잡히면 그 핀엔 아무것도 안 달린 것이다.
//
//  2) 핀 조합 전수 스캔 — D0~D10 중 그럴듯한 SDA/SCL 쌍을 전부 시도한다.
//     엉뚱한 핀에 꽂혀 있으면 여기서 잡힌다.
//
// XIAO ESP32S3 핀 대응:
//   D0=1  D1=2  D2=3  D3=4  D4=5  D5=6  D6=43  D7=44  D8=7  D9=8  D10=9
#include <Wire.h>

struct PinPair { int sda; int scl; const char *name; };

// 흔한 오배선까지 포함해서 넉넉히 시도한다 (뒤바꿔 꽂은 경우 포함)
static const PinPair PAIRS[] = {
  {5, 6,  "D4/D5   (표준)"},
  {6, 5,  "D5/D4   (SDA/SCL 뒤바뀜)"},
  {4, 3,  "D3/D2"},
  {3, 4,  "D2/D3"},
  {2, 1,  "D1/D0"},
  {1, 2,  "D0/D1"},
  {43, 44, "D6/D7"},
  {44, 43, "D7/D6"},
  {7, 8,  "D8/D9"},
  {8, 7,  "D9/D8"},
  {8, 9,  "D9/D10"},
  {9, 8,  "D10/D9"},
};
static const int NPAIRS = sizeof(PAIRS) / sizeof(PAIRS[0]);

// 이 프로젝트에서 기대하는 주소
static const char *nameOf(uint8_t a) {
  switch (a) {
    case 0x23: return "BH1750 (조도)";
    case 0x29: return "BNO055 (IMU)";
    case 0x28: return "BNO055 (ADR low)";
    case 0x76: return "BME280 (온습기압)";
    case 0x77: return "BME280 (SDO high)";
    default:   return "";
  }
}

void checkLevels() {
  Serial.println("--- 버스 전기 상태 (내부 풀업만) ---");
  const int pins[] = {5, 6, 4, 3, 43, 44};
  const char *nm[] = {"D4(5) SDA", "D5(6) SCL", "D3(4)", "D2(3)", "D6(43)", "D7(44)"};
  for (int i = 0; i < 6; i++) {
    pinMode(pins[i], INPUT_PULLUP);
    delay(2);
    int v = digitalRead(pins[i]);
    Serial.printf("  %-10s = %s%s\n", nm[i], v ? "HIGH" : "LOW",
                  v ? "" : "   <- 누가 버스를 잡고 있음 (합선/GND 접촉 의심)");
  }
}

int scanPair(const PinPair &p) {
  Wire.end();
  delay(20);
  Wire.begin(p.sda, p.scl);
  Wire.setClock(100000);
  delay(30);

  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      if (!found) Serial.printf("  [%s]  SDA=GPIO%d SCL=GPIO%d\n", p.name, p.sda, p.scl);
      Serial.printf("      0x%02X  %s\n", a, nameOf(a));
      found++;
    }
  }
  return found;
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(1500);
}

void loop() {
  Serial.println("\n========== I2C HUNT ==========");
  checkLevels();

  Serial.println("--- 핀 조합 전수 스캔 ---");
  int total = 0;
  for (int i = 0; i < NPAIRS; i++) total += scanPair(PAIRS[i]);

  if (!total) {
    Serial.println("  어떤 핀 조합에서도 응답 없음");
    Serial.println("  => 센서에 전원이 안 들어가고 있을 가능성이 가장 높다.");
    Serial.println("     VCC 는 반드시 3V3 핀 (5V 패드는 USB 급전 시 죽어있음).");
    Serial.println("     GND 도 보드와 공통으로 물려야 한다.");
  } else {
    Serial.printf("  -> 총 %d 개 응답\n", total);
  }
  Serial.println("==============================");
  delay(4000);
}
