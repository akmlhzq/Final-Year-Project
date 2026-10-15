#include <Arduino.h>

const int RELAY1 = 23;
const int RELAY3 = 22;

void setup() {
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  digitalWrite(RELAY1, HIGH);   // start OFF (most modules are active LOW)
  digitalWrite(RELAY3, HIGH);   // start OFF (most modules are active LOW)
}

void loop() {
  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY3, LOW);
      // relay ON (click)
  delay(2000);
  digitalWrite(RELAY1, HIGH);   // relay OFF (click)
  digitalWrite(RELAY3, HIGH);   // relay OFF (click)
  delay(2000);
}