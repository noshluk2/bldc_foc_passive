// SPI sanity check: toggles CS and transfers 0x55 to verify wiring on custom pins.
// Prints returned byte every 500 ms to confirm clock/MISO/MOSI behavior.
// Use before sensor bring-up to rule out wiring mistakes.
#include <Arduino.h>
#include <SPI.h>

static const int PIN_CS   = 10;  // CS   , white
static const int PIN_SCK  = 12;  // SCK  , blue
static const int PIN_MISO = 13;  // MISO , green
static const int PIN_MOSI = 11;  // MOSI , yellow

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
