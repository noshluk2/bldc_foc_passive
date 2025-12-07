// Adaptive hold/haptic feel: spring-damper that softens as you spin faster.
// Reads AS5048A via SPI and drives a SimpleFOC BLDC with voltage-mode torque.
// Serial: set base %, re-center hold, tweak torque cap; streams state logs.
// Adaptive Hold: resistance drops as you rotate (ESP32 + SimpleFOC + AS5048A SPI)
// Serial:
//   - Send a number like "50" or "50%" to set base strength (default 60%)
//   - Send "H" to re-center the hold at the current position
//   - Optional: "U=4" to set UQ_MAX volts (torque cap)

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

// ====== SPI wiring (your working mapping) ======
static const int PIN_CS   = 10;  // CS   , white
static const int PIN_SCK  = 12;  // SCK  , blue
static const int PIN_MISO = 13;  // MISO , green
static const int PIN_MOSI = 11;  // MOSI , yellow

MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_CS);

// ====== Motor/driver ======
BLDCDriver3PWM driver = BLDCDriver3PWM(/*Ua*/26, /*Ub*/25, /*Uc*/33, /*EN*/32);
BLDCMotor motor = BLDCMotor(/*pole pairs*/7);

// ====== User knobs ======
float base_pct = 60.0f;      // base stiffness at rest (0..100)
float UQ_MAX   = 4.0f;       // max q-axis voltage (torque cap, Volts)

// Virtual spring-damper gains at 100% (tuned for 12V gimbal)
// Effective gains = base_pct% * speed_factor * these
const float KP_100 = 3.0f;   // V/rad
const float KD_100 = 0.15f;  // V*s/rad

// Speed → resistance shaping
const float VEL_START = 0.30f;  // rad/s: begin softening here
const float VEL_HIGH  = 8.00f;  // rad/s: reach minimum here
const float FACTOR_MIN = 0.10f; // 10% of base when very fast

// ====== Internals ======
float hold_angle = 0.0f;     // mechanical angle to hold (rad)
float vel_filt   = 0.0f;
const float VEL_ALPHA = 0.2f;

unsigned long last_log = 0;
const uint32_t log_ms = 100;
String line;

// -------- helpers --------
static inline float wrap_pi(float x){
  while (x >  PI) x -= 2.0f*PI;
  while (x < -PI) x += 2.0f*PI;
  return x;
}
static inline float clampf(float x, float lo, float hi){
  return x < lo ? lo : (x > hi ? hi : x);
}

void parseSerial(){
  while (Serial.available()){
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n'){
      if (line.length()){
        if (line.equalsIgnoreCase("H")){
          hold_angle = sensor.getAngle();
          Serial.println("Hold angle re-centered.");
        } else if (line.startsWith("U=") || line.startsWith("u=")){
          float v = line.substring(2).toFloat();
          if (v > 0.5f && v <= 8.0f){
            UQ_MAX = v;
            motor.voltage_limit = UQ_MAX;
            Serial.print("UQ_MAX set to "); Serial.print(UQ_MAX,2); Serial.println(" V");
          } else {
            Serial.println("U invalid (0.5..8 V).");
          }
        } else {
          if (line.endsWith("%")) line.remove(line.length()-1);
          float p = line.toFloat();
          p = clampf(p, 0.0f, 100.0f);
          base_pct = p;
          Serial.print("Base strength set to "); Serial.print(base_pct,1); Serial.println("%");
        }
      }
      line = "";
    } else {
      line += c;
    }
  }
}

void setup(){
  Serial.begin(115200);
  delay(80);

  // SPI + sensor
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  sensor.init(&SPI);

  // Driver
  driver.voltage_power_supply = 12.0f;
  driver.pwm_frequency = 25000;
  driver.init();

  // Motor (FOC with torque=voltage)
  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = UQ_MAX;
  motor.init();
  motor.initFOC();
  motor.enable();

  // Lock current position as the hold center
  hold_angle = sensor.getAngle();

  Serial.println("== Adaptive Hold (resistance decreases with speed) ==");
  Serial.println("Send: 0..100(%) to set base, 'H' to re-center, 'U=4' to set torque cap.");
  Serial.println("time_ms,angle_deg,vel_deg_s,factor,pct,torque_V");
}

void loop(){
  parseSerial();

  // FOC commutation & sensor update
  motor.loopFOC();

  // Read state
  float angle = sensor.getAngle();
  float vel   = sensor.getVelocity();                   // rad/s (updated by loopFOC)
  vel_filt += VEL_ALPHA * (vel - vel_filt);

  // Position error around hold center
  float err = wrap_pi(angle - hold_angle);             // rad

  // Speed-dependent softening factor
  float speed = fabsf(vel_filt);
  float factor;
  if (speed <= VEL_START) {
    factor = 1.0f;
  } else if (speed >= VEL_HIGH) {
    factor = FACTOR_MIN;
  } else {
    float t = (speed - VEL_START) / (VEL_HIGH - VEL_START); // 0..1
    factor = 1.0f - t*(1.0f - FACTOR_MIN);                  // linear fade
  }

  // Effective gains
  float gain_scale = (base_pct/100.0f) * factor;
  float Kp = KP_100 * gain_scale;
  float Kd = KD_100 * gain_scale;

  // Virtual spring-damper torque (sign resists motion)
  float torque_v = clampf( -Kp*err - Kd*vel_filt, -UQ_MAX, UQ_MAX );

  // Apply torque command (voltage-mode torque)
  motor.move(torque_v);

  // Log
  unsigned long now = millis();
  if (now - last_log >= log_ms){
    last_log = now;
    Serial.print(now); Serial.print(',');
    Serial.print(angle * RAD_TO_DEG, 2); Serial.print(',');
    Serial.print(vel_filt * RAD_TO_DEG, 2); Serial.print(',');
    Serial.print(factor, 2); Serial.print(',');
    Serial.print(base_pct, 1); Serial.print(',');
    Serial.println(torque_v, 3);
  }
}
