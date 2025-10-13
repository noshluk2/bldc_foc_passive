// === Cubemars gimbal + SimpleFOC + AS5048A (SPI) ===
// Mode: Smooth passive feel with viscous damping (no steppy feel)
// ESP32-C3 (e.g., LOLIN C3 Mini) + your pin map

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <math.h>

// ---------- YOUR PINS ----------
static const int PIN_CS   = 7;  // AS5048A CS   (white)
static const int PIN_SCK  = 4;  // SCK          (blue)
static const int PIN_MISO = 5;  // MISO         (green)
static const int PIN_MOSI = 6;  // MOSI         (yellow)

// 3-PWM driver pins (ESP32-C3)
BLDCDriver3PWM driver(/*Ua*/0, /*Ub*/1, /*Uc*/3, /*EN*/-1);

// Cubemars GL30: start with 7 pole pairs (some variants are 11)
BLDCMotor motor(7);

// AS5048A over SPI (SimpleFOC preset: 14-bit, mode 1)
MagneticSensorSPI sensor(AS5048_SPI, PIN_CS);

// ---------- USER KNOBS ----------
static const float BUS_VOLTAGE     = 12.0f;
static const float VOLTAGE_LIMIT_V = 1.0f;    // start 0.8–1.2 V
static const uint32_t PWM_HZ       = 25000;   // 25 kHz
static const float    VEL_LPF_Tf   = 0.03f;   // 20–40 ms helps smooth very low speed

// Viscous damping (main anti-step knob): tau = -Kd * omega
static const float Kd_visc         = 0.07f;   // try 0.05–0.10 V/(rad/s)

// Smooth Coulomb term (optional): prevents micro stick-slip near zero speed
static const float Coulomb_V       = 0.06f;   // 0.03–0.10 V; keep small
static const float Coulomb_scale   = 0.3f;    // slope of tanh near zero

// Tiny low-freq dither (optional): gentle nudge through residual cogging
static const float DITHER_V        = 0.03f;   // 0.02–0.05 V max
static const float DITHER_HZ       = 7.0f;    // low & inaudible

static inline float clampV(float x, float lim) {
  return x >  lim ?  lim : (x < -lim ? -lim : x);
}

void setup() {
  delay(200);
  Serial.begin(115200);

  // SPI with explicit pins on ESP32
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  sensor.init();
  motor.linkSensor(&sensor);

  // Driver
  driver.voltage_power_supply = BUS_VOLTAGE;
  driver.voltage_limit        = VOLTAGE_LIMIT_V;
  driver.pwm_frequency        = PWM_HZ;
  driver.init();

  // Motor
  motor.controller        = MotionControlType::torque;     // command torque directly
  motor.torque_controller = TorqueControlType::voltage;    // voltage-mode (no current sensors)
  motor.voltage_limit     = VOLTAGE_LIMIT_V;
  motor.foc_modulation    = FOCModulationType::SpaceVectorPWM;
  motor.LPF_velocity.Tf   = VEL_LPF_Tf;

  // If it "fights" you after first try, set direction BEFORE initFOC():
  // motor.sensor_direction = Direction::CCW;  // or Direction::CW;

  motor.init();
  motor.initFOC();

  Serial.println("FOC ready. Viscous damping mode (SimpleFOC >= 2.x API).");
}

void loop() {
  motor.loopFOC();

  // Velocity from SimpleFOC (already LPF'ed): rad/s
  const float omega = motor.shaftVelocity();

  // 1) Viscous damping: continuous opposing torque while moving
  float tau = -Kd_visc * omega;

  // 2) Tiny smooth Coulomb-like term near zero speed
  tau += -Coulomb_V * tanh(omega / Coulomb_scale);

  // 3) Low-frequency micro-dither
  static uint32_t t0 = millis();
  float t = 0.001f * (millis() - t0);
  tau += DITHER_V * sinf(2.0f * PI * DITHER_HZ * t);

  // Safety clamp
  tau = clampV(tau, VOLTAGE_LIMIT_V);

  motor.move(tau);

  // // Optional debug:
  // static uint32_t last=0; if (millis()-last>200) { last=millis();
  //   Serial.printf("omega=%.3f rad/s  tau=%.3f V\n", omega, tau);
  // }
}
