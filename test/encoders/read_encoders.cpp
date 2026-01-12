#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

// ESP32-C3 Super Mini (common working SPI mapping)
static const int PIN_CS   = 14;
static const int PIN_SCK  = 12;
static const int PIN_MISO = 13;
static const int PIN_MOSI = 11;

MagneticSensorSPI sensor(AS5048_SPI, PIN_CS);

const uint32_t REPORT_MS = 200; // slower print for stability

unsigned long last_report = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\nBooting...");

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  // Optionally slow down SPI (SimpleFOC uses SPISettings internally, but start stable)
  sensor.init(&SPI);

  Serial.println("ticks,angle_rad");
}

void loop() {
  sensor.update();
  float angle = sensor.getAngle();

  // Convert radians to 14-bit ticks
  uint16_t ticks = (uint16_t)(angle * (16384.0f / (2.0f * PI)) + 0.5f) & 0x3FFF;

  unsigned long now = millis();
  if (now - last_report >= REPORT_MS) {
    last_report = now;
    Serial.print(ticks);
    Serial.print(",");
    Serial.println(angle, 6);
  }

  delay(1); // yield to avoid watchdog pressure
}
