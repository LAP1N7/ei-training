// ============================================================
// 아두이노 LED 3 패턴 제어 프로그램
// 보드: 아두이노 우노 / 언어: C++
// 패턴: 전체 점멸 / 순차 점등 / 교차 점멸
// ============================================================

// --------------------- 설정 변수 ---------------------
// LED 핀 배열 (원하면 핀 번호/개수를 자유롭게 수정)
const int ledPins[] = {2, 3, 4, 5, 6};
const int ledCount  = sizeof(ledPins) / sizeof(ledPins[0]);

// 딜레이 시간 (ms)
const unsigned long blinkDelay     = 500;   // 전체 점멸 속도
const unsigned long sequentialDelay = 300;  // 순차 점등 속도
const unsigned long alternatingDelay = 500; // 교차 점멸 속도

// 각 패턴 반복 횟수
const int patternRepeats = 3;

// --------------------- 초기 설정 ---------------------
void setup() {
  // 모든 LED 핀을 출력 모드로 설정
  for (int i = 0; i < ledCount; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW); // 시작 시 모두 꺼둠
  }
}

// --------------------- 메인 루프 ---------------------
void loop() {
  allBlink();       // 패턴 1: 전체 점멸
  sequential();     // 패턴 2: 순차 점등
  alternating();    // 패턴 3: 교차 점멸
}

// ============================================================
// 패턴 1: 전체 점멸 (All Blink)
// 모든 LED가 동시에 켜졌다가 꺼지는 것을 반복
// ============================================================
void allBlink() {
  for (int r = 0; r < patternRepeats; r++) {
    // 모든 LED 켜기
    for (int i = 0; i < ledCount; i++) {
      digitalWrite(ledPins[i], HIGH);
    }
    delay(blinkDelay);

    // 모든 LED 끄기
    for (int i = 0; i < ledCount; i++) {
      digitalWrite(ledPins[i], LOW);
    }
    delay(blinkDelay);
  }
}

// ============================================================
// 패턴 2: 순차 점등 (Sequential)
// 1번 → 5번까지 하나씩 켜지고, 다시 5번 → 1번으로 하나씩 꺼짐
// ============================================================
void sequential() {
  for (int r = 0; r < patternRepeats; r++) {
    // 정방향으로 하나씩 켜기 (이전에 켜진 LED는 유지)
    for (int i = 0; i < ledCount; i++) {
      digitalWrite(ledPins[i], HIGH);
      delay(sequentialDelay);
    }

    // 역방향으로 하나씩 끄기
    for (int i = ledCount - 1; i >= 0; i--) {
      digitalWrite(ledPins[i], LOW);
      delay(sequentialDelay);
    }
  }
}

// ============================================================
// 패턴 3: 교차 점멸 (Alternating)
// 홀수 번째 LED와 짝수 번째 LED가 번갈아 가며 켜지고 꺼짐
// [1,3,5] 켜짐 ↔ [2,4] 켜짐 교대로 동작
// ============================================================
void alternating() {
  for (int r = 0; r < patternRepeats; r++) {
    // 홀수 인덱스(0,2,4) LED 켜고 짝수 인덱스(1,3) LED 끄기
    for (int i = 0; i < ledCount; i++) {
      digitalWrite(ledPins[i], (i % 2 == 0) ? HIGH : LOW);
    }
    delay(alternatingDelay);

    // 홀수 인덱스(0,2,4) LED 끄고 짝수 인덱스(1,3) LED 켜기
    for (int i = 0; i < ledCount; i++) {
      digitalWrite(ledPins[i], (i % 2 == 0) ? LOW : HIGH);
    }
    delay(alternatingDelay);
  }
}
