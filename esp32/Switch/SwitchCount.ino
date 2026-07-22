const int btnPin = 1;

bool stableState = HIGH;
bool lastReading = HIGH;
unsigned long lastDebounce = 0;
const unsigned long debounceDelay = 50;

int count = 0;

void setup() {
  pinMode(btnPin, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.println("System Started");
}

void loop() {
  bool reading = digitalRead(btnPin);

  if (reading != lastReading) {
    lastDebounce = millis();
  }

  if ((millis() - lastDebounce) > debounceDelay) {
    if (reading != stableState) {
      stableState = reading;
      if (stableState == LOW) {
        count++;
        Serial.println(count);
      }
    }
  }

  lastReading = reading;
}
