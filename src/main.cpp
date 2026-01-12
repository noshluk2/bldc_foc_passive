#include <Arduino.h>


#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(7, 8);          // SDA=7, SCL=8
  Wire.setClock(100000);     // Start conservative: 100 kHz

  Serial.println("I2C scan...");
    delay(2000);

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("Found device at 0x%02X\n", addr);
    }
  }
  Serial.println("Done.");
}

void loop() {}
