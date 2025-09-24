// Minimal AS5048A readout (ESP32 + SimpleFOC): ticks + angle_rad
#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

static const int PIN_CS   = 7;  // CS   , white
static const int PIN_SCK  = 4;  // SCK  , blue
static const int PIN_MISO = 5;  // MISO , green
static const int PIN_MOSI = 6;  // MOSI , yellow



MagneticSensorSPI sensor(AS5048_SPI, PIN_CS);

const uint32_t REPORT_MS = 50;                 // 20 Hz output
const float    K_RAD2TICKS = 16384.0f / (2.0f * PI);  // rad -> ticks

unsigned long last_report = 0;

void setup() {
  Serial.begin(115200);
  delay(50);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  sensor.init(&SPI);

  Serial.println(F("ticks,angle_rad"));
}

void loop() {
  sensor.update();

  // Current angle in radians [0, 2π)
  float angle_rad = sensor.getAngle();

  // Convert to native 14-bit ticks [0..16383]
  uint16_t ticks = (uint16_t)(angle_rad * K_RAD2TICKS + 0.5f) & 0x3FFF;

  unsigned long now = millis();
  if (now - last_report >= REPORT_MS) {
    last_report = now;
    Serial.print(ticks);
    Serial.print(",");
    Serial.println(angle_rad, 9);  // high precision print
  }
}
