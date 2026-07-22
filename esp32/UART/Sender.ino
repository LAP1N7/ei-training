#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11);

const int btnPin = 2;

void setup() {
  pinMode(btnPin, INPUT_PULLUP);
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println("Sender Started");
}

void loop() {
  if (digitalRead(btnPin) == LOW) {
    mySerial.println("ON");
    Serial.println("Button Pressed -> Sent ON");
    delay(500);
  }
}
