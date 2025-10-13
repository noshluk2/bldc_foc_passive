// === Cubemars gimbal + SimpleFOC + AS5048A (SPI) ===
// Mode: Zero-torque (passive) — buttery free rotation (controller adds no steps)

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

// ---------- YOUR PINS ----------
static const int PIN_CS   = 7;  // AS5048A CS   (white)
static const int PIN_SCK  = 4;  // SCK          (blue)
static const int PIN_MISO = 5;  // MISO         (green)
static const int PIN_MOSI = 6;  // MOSI         (yellow)

// Motor driver (3-PWM). EN = -1 means always enabled or not used.
BLDCDriver3PWM driver(/*Ua*/0, /*Ub*/1, /*Uc*/3, /*EN*/-1);

// Motor: Cubemars GL30 variant — try 7 pole pairs first (some variants are 11)
BLDCMotor motor(11);

// AS5048A over SPI (SimpleFOC provides AS5048_SPI preset: 14-bit, mode 1)
MagneticSensorSPI sensor(AS5048_SPI, PIN_CS);

// ---------- USER KNOBS (start conservative) ----------
static const float BUS_VOLTAGE     = 12.0f;  // your 12V supply
static const float VOLTAGE_LIMIT_V = 1.2f;   // keep low for gimbals (0.8..1.5 typical)
static const uint32_t PWM_HZ       = 25000;  // 25 kHz to reduce ripple/whine
static const float    VEL_LPF_Tf   = 0.02f;  // 20 ms velocity LPF (cleaner readings)

void setup() {
  delay(200);
  Serial.begin(115200);

  // --- SPI with explicit pins (ESP32) ---
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  sensor.init();
  motor.linkSensor(&sensor);

  // --- Driver setup ---
  driver.voltage_power_supply = BUS_VOLTAGE;
  driver.voltage_limit        = VOLTAGE_LIMIT_V;
  driver.pwm_frequency        = PWM_HZ;
  driver.init();

  // --- Motor setup ---
  motor.controller        = MotionControlType::torque;     // we command torque directly
  motor.torque_controller = TorqueControlType::voltage;    // voltage-mode (no current sensors)
  motor.voltage_limit     = VOLTAGE_LIMIT_V;
  motor.foc_modulation    = FOCModulationType::SpaceVectorPWM;

  // Optional smoothing for velocity readout
  motor.LPF_velocity.Tf = VEL_LPF_Tf;

  // If it "fights" you after first try, uncomment ONE of these and re-test:
  // motor.sensor_direction = Direction::CCW;  // or Direction::CW; set before initFOC()

  motor.init();
  motor.initFOC();  // aligns using the sensor

  Serial.println("FOC ready. Zero-torque passive mode.");
  Serial.println("If feel is 'fighty' or buzzy, try sensor_direction flip or pole pairs 7↔11.");
}

void loop() {
  // Run FOC and commutation
  motor.loopFOC();

  // Zero torque (passive): controller adds no extra torque
  motor.move(0.0f);

  // Optional: quick telemetry
  // static uint32_t t0=0; if (millis()-t0>200) { t0=millis();
  //   Serial.printf("theta=%.3f rad, omega=%.3f rad/s\n", sensor.getAngle(), motor.getVelocity());
  // }
}
