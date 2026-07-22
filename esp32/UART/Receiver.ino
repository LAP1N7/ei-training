#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11);

const int ledPin = 13;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println("Receiver Started");
}

void loop() {
  if (mySerial.available()) {
    String cmd = mySerial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "ON") {
      digitalWrite(ledPin, HIGH);
      Serial.println("LED ON");
      delay(3000);
      digitalWrite(ledPin, LOW);
      Serial.println("LED OFF");
    }
  }
}
