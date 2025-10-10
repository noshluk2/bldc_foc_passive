// Sensorless static hold with 0..100% strength (ESP32 + BLDCDriver3PWM)
// Applies a fixed stator field at a chosen electrical angle to "lock" the rotor.
// Serial: send "50" or "50%" to set strength, "A=60" to set angle in degrees.

#include <Arduino.h>
#include <SimpleFOC.h>

// -------- Driver + Motor (adjust pins to your board) --------
BLDCDriver3PWM driver(/*Ua*/0, /*Ub*/1, /*Uc*/3, /*EN*/-1);
BLDCMotor motor = BLDCMotor(/*pole pairs*/7);   // 7 fo      r 14-pole gimbal

// -------- User controls --------
float resist_pct = 50.0f;           // 0..100% hold strength
const float UQ_MAX = 4.0f;          // volts at 100% (tune to taste; start low)
float hold_angle_el = 0.0f;         // electrical angle (rad). Change with "A=deg".

// -------- Internals --------
unsigned long last_report = 0;
const uint32_t report_ms = 100;     // 10 Hz log
String line;
float Uq_target = 0.0f;
float Uq_actual = 0.0f;             // ramped for smoothness

// simple slew limiter (V per loop)
const float UQ_SLEW = 0.05f;        // increase/decrease per loop (tune)

void parseSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (line.length()) {
        // angle command e.g. "A=60"
        if (line.startsWith("A=") || line.startsWith("a=")) {
          float deg = line.substring(2).toFloat();
          hold_angle_el = deg * DEG_TO_RAD;
          Serial.print("Hold electrical angle set to ");
          Serial.print(deg, 1);
          Serial.println(" deg");
        } else {
          // percentage e.g. "35" or "35%"
          if (line.endsWith("%")) line.remove(line.length() - 1);
          float p = line.toFloat();
          if (p < 0) p = 0; if (p > 100) p = 100;
          resist_pct = p;
          Serial.print("Hold strength set to ");
          Serial.print(resist_pct, 1);
          Serial.println("%");
        }
      }
      line = "";
    } else {
      line += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);

  // Driver setup
  driver.voltage_power_supply = 12.0f; // your bus voltage
  driver.pwm_frequency = 25000;        // quieter
  driver.init();

  // Motor setup for direct phase voltage control (no sensor / no FOC loop)
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SinePWM;  // clean sine hold
  motor.voltage_limit = UQ_MAX;                       // clamp
  motor.init();
  motor.enable();                                     // enable gate driver

  Serial.println("== Sensorless Static Hold ==");
  Serial.println("Type: 0..100 (or %) for strength, A=deg to change electrical angle.");
  Serial.println("Columns: time_ms,Uq[V],pct,angle_el[deg]");
}

void loop() {
  parseSerial();

  // map % -> target q-axis voltage
  Uq_target = (resist_pct / 100.0f) * UQ_MAX;

  // ramp to avoid step jolts
  if (Uq_actual < Uq_target)       Uq_actual = min(Uq_target, Uq_actual + UQ_SLEW);
  else if (Uq_actual > Uq_target)  Uq_actual = max(Uq_target, Uq_actual - UQ_SLEW);

  // Apply static stator field at fixed electrical angle
  // Ud=0 (no d-axis), Uq = Uq_actual, angle = hold_angle_el
  motor.setPhaseVoltage(Uq_actual, 0.0f, hold_angle_el);

  // periodic log
  unsigned long now = millis();
  if (now - last_report >= report_ms) {
    last_report = now;
    Serial.print(now); Serial.print(',');
    Serial.print(Uq_actual, 3); Serial.print(',');
    Serial.print(resist_pct, 1); Serial.print(',');
    Serial.println(hold_angle_el * RAD_TO_DEG, 1);
  }
}
