const int redPin = 4;
const int yellowPin = 5;
const int greenPin = 6;
const int btnPin = 2;

volatile bool emergency = false;

unsigned long prevMillis = 0;
int state = 0;
const unsigned long interval = 3000;

void btnISR() {
  emergency = true;
}

void setLights(bool r, bool y, bool g) {
  digitalWrite(redPin, r);
  digitalWrite(yellowPin, y);
  digitalWrite(greenPin, g);
}

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(btnPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(btnPin), btnISR, FALLING);

  Serial.begin(9600);
  Serial.println("Traffic Light Started");

  setLights(LOW, LOW, HIGH);
  prevMillis = millis();
}

void loop() {
  if (emergency) {
    emergency = false;
    setLights(LOW, LOW, HIGH);
    setLights(HIGH, LOW, LOW);
    Serial.println("EMERGENCY: RED");
    prevMillis = millis();
    return;
  }

  if (millis() - prevMillis >= interval) {
    prevMillis += interval;
    state = (state + 1) % 3;

    switch (state) {
      case 0:
        setLights(LOW, LOW, HIGH);
        Serial.println("GREEN");
        break;
      case 1:
        setLights(LOW, HIGH, LOW);
        Serial.println("YELLOW");
        break;
      case 2:
        setLights(HIGH, LOW, LOW);
        Serial.println("RED");
        break;
    }
  }
}
