// Simple encoder readout test for manual rotation (ESP32 + AS5048A SPI)
#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

// AS5048A on SPI, CS = GPIO27 (white)
// g ,y,b,w
// miso mosi clk csn
static const int PIN_CS   = 27;  // white
static const int PIN_SCK  = 13;  // blue
static const int PIN_MISO = 12;  // green
static const int PIN_MOSI = 14;  // yellow

MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_CS);

// Optional motor/driver placeholders (kept disabled)
BLDCDriver3PWM driver = BLDCDriver3PWM(26, 25, 33, 32);
BLDCMotor motor = BLDCMotor(7);

unsigned long last_report = 0;
const uint32_t report_ms = 50; // 20 Hz

void setup() {
  Serial.begin(115200);
  delay(50);

  // SPI with explicit pin mapping (ESP32)
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);   // idle high (not selected)

  // Sensor init on the SPI bus we just configured
  sensor.init(&SPI);

  // Link to motor only if you want SimpleFOC helpers later
  motor.linkSensor(&sensor);

  // Keep driver & motor uninitialized to avoid torque output
  // If you ever power the driver, uncomment:
  // driver.voltage_power_supply = 12;
  // driver.init();
  // motor.linkDriver(&driver);

  Serial.println("Encoder Read Test (AS5048A SPI) — rotate the shaft by hand");
  Serial.println("time_ms,angle_rad,velocity_rad_s");
}

void loop() {
  // Update sensor (needed for correct velocity)
  sensor.update();

  float angle = sensor.getAngle();       // radians [0, 2π)
  float velocity = sensor.getVelocity(); // rad/s

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
