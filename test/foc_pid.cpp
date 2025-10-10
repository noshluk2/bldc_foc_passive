// ESP32-C3 + SimpleFOC + AS5048A
// Minimal soft-spring position hold at a fixed angle (no serial, no logs)

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

// ===================== USER SETTINGS =====================
static const float TARGET_DEG      = 45.0f;  // << set your desired angle here
static const float KP              = 2.0f;   // spring [V/rad]
static const float KD              = 0.05f;  // damper [V/(rad/s)]
static const float DEADBAND        = 0.02f;  // [rad] ~1.15°
static const float VOLTAGE_LIMIT   = 2.0f;   // [V] keep low for gimbal motors
static const float BUS_VOLTAGE     = 12.0f;  // supply
static const int   POLE_PAIRS      = 7;      // try 7; if odd behavior, try 11

// ===================== PINS (change to your board) =====================
// Use non-strapping pins on ESP32-C3 for PWM (avoid GPIO0/1/3)
static const int PIN_UH = 8;
static const int PIN_VH = 9;
static const int PIN_WH = 10;
static const int PIN_EN = -1;  // set a GPIO if your driver has EN, else -1

// AS5048A SPI (ok to keep these)
static const int PIN_SCK  = 4;
static const int PIN_MISO = 5;
static const int PIN_MOSI = 6;
static const int PIN_CS   = 7;

// ===================== OBJECTS =====================
BLDCMotor motor(POLE_PAIRS);
BLDCDriver3PWM driver(PIN_UH, PIN_VH, PIN_WH, PIN_EN);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_CS);

// ===================== INTERNALS =====================
const float TARGET_RAD = TARGET_DEG * DEG_TO_RAD;
const float LPF_ALPHA  = 0.2f;   // velocity filter
const float TAU_SLEW   = 0.02f;  // V/loop torque slew
float lpf_qd = 0.0f;
float tau_out = 0.0f;

// Deadband + soft saturation
inline float shapedError(float e) {
  float ae = fabsf(e);
  if (ae <= DEADBAND) return 0.0f;
  float sign = (e > 0) ? 1.0f : -1.0f;
  float mag  = ae - DEADBAND;
  return sign * tanhf(mag * 0.8f);
}

void setup() {
  // SPI + sensor
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  sensor.init(&SPI);

  // Motor/driver
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = BUS_VOLTAGE;
  driver.pwm_frequency = 25000;
  driver.init();
  motor.linkDriver(&driver);

  if (PIN_EN >= 0) { pinMode(PIN_EN, OUTPUT); digitalWrite(PIN_EN, HIGH); }

  motor.controller        = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit     = VOLTAGE_LIMIT;
  motor.foc_modulation    = FOCModulationType::SinePWM;

  motor.init();
  motor.initFOC();
}

void loop() {
  motor.loopFOC();

  // State
  float q  = motor.shaft_angle;     // rad
  float qd = motor.shaft_velocity;  // rad/s (estimated)
  lpf_qd = (1.0f - LPF_ALPHA) * lpf_qd + LPF_ALPHA * qd;

  // Soft-spring torque
  float err      = TARGET_RAD - q;
  float e_shape  = shapedError(err);
  float tau_cmd  = KP * e_shape - KD * lpf_qd;

  // Clamp to voltage limit
  float vmax = VOLTAGE_LIMIT;
  if (tau_cmd >  vmax) tau_cmd =  vmax;
  if (tau_cmd < -vmax) tau_cmd = -vmax;

  // Slew-limit for smoothness
  if (tau_out < tau_cmd)       tau_out = min(tau_cmd, tau_out + TAU_SLEW);
  else if (tau_out > tau_cmd)  tau_out = max(tau_cmd, tau_out - TAU_SLEW);

  // Apply
  motor.move(tau_out);
}
