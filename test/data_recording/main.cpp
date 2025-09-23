// Data recording test: stream CSV of time, angle, velocity, torque
#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

BLDCDriver3PWM driver = BLDCDriver3PWM(26, 25, 33, 32);
BLDCMotor motor = BLDCMotor(7);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 27);

// Configure mode: 0 = passive (no torque), 1 = damping torque
int mode = 1; // default to damping for more interesting data
float damping_K = 0.05f;

// Report rate
unsigned long last_report = 0;
const uint32_t report_ms = 10; // 100 Hz

static void printHelp() {
  Serial.println("Commands:");
  Serial.println("  MODE=0     passive sensor only");
  Serial.println("  MODE=1     damping torque (-K*vel)");
  Serial.println("  K=0.05     set damping gain (0..2)");
}

static float parseAssignF(const String &line, const char *key) {
  int idx = line.indexOf('=');
  if (idx < 0) return NAN;
  String k = line.substring(0, idx); k.trim();
  if (!k.equalsIgnoreCase(key)) return NAN;
  String v = line.substring(idx + 1); v.trim();
  return v.toFloat();
}

void setup() {
  Serial.begin(115200);
  SPI.begin(14, 12, 13);

  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = 5.0;
  motor.init();
  motor.initFOC();
  motor.enable();

  Serial.println("Data Recording Test");
  printHelp();
  Serial.println("CSV header: time_ms,angle_rad,velocity_rad_s,torque_cmd,mode");
}

String cmd;

void loop() {
  motor.loopFOC();

  // Serial commands
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd.length()) {
        float v;
        v = parseAssignF(cmd, "MODE");
        if (!isnan(v)) {
          int m = (int)v;
          if (m == 0 || m == 1) { mode = m; Serial.print("MODE="); Serial.println(mode); }
          else Serial.println("Invalid MODE (0 or 1)");
          cmd = ""; continue;
        }
        v = parseAssignF(cmd, "K");
        if (!isnan(v)) {
          if (v >= 0 && v <= 2.0f) { damping_K = v; Serial.print("K="); Serial.println(damping_K, 4); }
          else Serial.println("Invalid K (0..2)");
          cmd = ""; continue;
        }
        if (cmd.equalsIgnoreCase("HELP")) printHelp();
      }
      cmd = "";
    } else {
      cmd += c;
    }
  }

  float torque = 0.0f;
  float vel = motor.shaft_velocity;
  if (mode == 1) {
    torque = -damping_K * vel;
  }
  motor.move(torque);

  unsigned long now = millis();
  if (now - last_report >= report_ms) {
    last_report = now;
    Serial.print(now);
    Serial.print(",");
    Serial.print(motor.shaft_angle, 6);
    Serial.print(",");
    Serial.print(vel, 6);
    Serial.print(",");
    Serial.print(torque, 6);
    Serial.print(",");
    Serial.println(mode);
  }
}

