// Blink test on GPIO36. Connect an LED (with resistor) to GPIO36 and GND.
// Toggles every 500 ms to verify the pin and wiring.
#include <Arduino.h>

static const int LED_PIN = 36; // or 41

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}
