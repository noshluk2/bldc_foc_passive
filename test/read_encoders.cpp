// Simple encoder readout test for manual rotation (ESP32 + AS5048A SPI)
#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

static const int PIN_CS   = 7;  // white
static const int PIN_SCK  = 4;  // blue
static const int PIN_MISO = 5;  // green
static const int PIN_MOSI = 6;  // yellow

MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_CS);

// Optional motor/driver placeholders (kept disabled)
BLDCDriver3PWM driver = BLDCDriver3PWM(26, 25, 33, 32);
BLDCMotor motor = BLDCMotor(7);

unsigned long last_report = 0;
const uint32_t report_ms = 50;

void setup() {
  Serial.begin(115200);
  delay(50);

  // SPI with explicit pin mapping (ESP32)
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  // Sensor init on the SPI bus we just configured
  sensor.init(&SPI);

  // Link to motor only if you want SimpleFOC helpers later
  motor.linkSensor(&sensor);

  Serial.println("Encoder Read Test (AS5048A SPI) — rotate the shaft by hand");
  Serial.println("time_ms,angle_rad,velocity_rad_s");
}

void loop() {
  // Update sensor (needed for correct velocity)
  sensor.update();

  float angle = sensor.getAngle();
  float velocity = sensor.getVelocity();

  unsigned long now = millis();
  if (now - last_report >= report_ms) {
    last_report = now;
    Serial.print(now);
    Serial.print(",");
    Serial.print(angle, 6);
    Serial.print(",");
    Serial.println(velocity, 6);
  }
}
