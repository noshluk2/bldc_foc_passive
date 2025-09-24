#include <Arduino.h>
#include <SPI.h>

const int PIN_SCK  = 4;
const int PIN_MISO = 5;
const int PIN_MOSI = 6;
const int PIN_CS   = 7;

void setup() {
  Serial.begin(115200);
  delay(500);

  // bring up SPI on your pins
  SPI.end();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  Serial.println("Starting SPI sanity check…");
}

void loop() {
  digitalWrite(PIN_CS, LOW);
  uint8_t r = SPI.transfer(0x55);  // clock out 0x55 pattern
  digitalWrite(PIN_CS, HIGH);

  Serial.printf("Wrote 0x55, got 0x%02X\n", r);
  delay(500);
}
