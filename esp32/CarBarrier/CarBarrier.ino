#include <Servo.h>

#define TRIG_PIN    9
#define ECHO_PIN    10
#define SERVO_PIN   11
#define LED_GREEN   6
#define LED_RED     5
#define BUTTON_PIN  2
#define POT_PIN     A0

#define BARRIER_OPEN_ANGLE   90
#define BARRIER_CLOSE_ANGLE  0
#define MEASURE_INTERVAL     100
#define SERIAL_INTERVAL      500
#define SERVO_MOVE_DELAY     15
#define DEBOUNCE_MS          300

Servo barrierServo;

volatile bool buttonPressed = false;
bool barrierOpen = false;
bool manualMode = false;

unsigned long lastMeasureTime = 0;
unsigned long lastSerialTime = 0;
unsigned long lastServoTime = 0;
unsigned long buttonPressTime = 0;

int targetAngle = BARRIER_CLOSE_ANGLE;
int currentAngle = BARRIER_CLOSE_ANGLE;

void buttonISR() {
  unsigned long now = millis();
  if (now - buttonPressTime > DEBOUNCE_MS) {
    buttonPressed = true;
    buttonPressTime = now;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  barrierServo.attach(SERVO_PIN);
  barrierServo.write(BARRIER_CLOSE_ANGLE);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);

  Serial.println("=== Car Barrier System ===");
  Serial.println("Potentiometer: calibration threshold");
  Serial.println("Button: toggle manual mode");
}

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999.0;
  return duration * 0.034 / 2.0;
}

void updateBarrier() {
  if (currentAngle == targetAngle) return;

  unsigned long now = millis();
  if (now - lastServoTime < SERVO_MOVE_DELAY) return;
  lastServoTime = now;

  if (currentAngle < targetAngle) {
    currentAngle++;
  } else {
    currentAngle--;
  }
  barrierServo.write(currentAngle);
}

void updateLEDs() {
  if (currentAngle >= BARRIER_OPEN_ANGLE) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
    barrierOpen = true;
  } else if (currentAngle <= BARRIER_CLOSE_ANGLE) {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
    barrierOpen = false;
  }
}

void loop() {
  unsigned long now = millis();

  // --- Button interrupt handling ---
  if (buttonPressed) {
    buttonPressed = false;
    manualMode = !manualMode;
    if (manualMode) {
      targetAngle = barrierOpen ? BARRIER_CLOSE_ANGLE : BARRIER_OPEN_ANGLE;
      Serial.println("[MANUAL] Mode ON");
    } else {
      Serial.println("[AUTO] Mode ON");
    }
  }

  // --- Manual mode: toggle barrier ---
  if (manualMode) {
    // targetAngle set by button, nothing else to do
  }

  // --- Auto mode: ultrasonic + potentiometer ---
  if (!manualMode) {
    if (now - lastMeasureTime >= MEASURE_INTERVAL) {
      lastMeasureTime = now;

      float distance = measureDistance();
      int potVal = analogRead(POT_PIN);
      float threshold = map(potVal, 0, 1023, 10, 200);

      if (distance < threshold) {
        targetAngle = BARRIER_OPEN_ANGLE;
      } else {
        targetAngle = BARRIER_CLOSE_ANGLE;
      }

      if (now - lastSerialTime >= SERIAL_INTERVAL) {
        lastSerialTime = now;
        Serial.print("Dist: ");
        Serial.print(distance, 1);
        Serial.print("cm | Threshold: ");
        Serial.print(threshold, 1);
        Serial.print("cm | Angle: ");
        Serial.println(currentAngle);
      }
    }
  }

  updateBarrier();
  updateLEDs();
}
