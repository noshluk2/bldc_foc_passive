#include <SimpleFOC.h>
#include <SPI.h>

// --- Hardware definitions ---
BLDCDriver3PWM driver = BLDCDriver3PWM(26, 25, 33, 32);
BLDCMotor motor = BLDCMotor(7);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 27);

// --- Final Tuned Haptic Feedback Values ---
float target_angle = 0.0f;
bool is_locked = false;
float velocity_lock_threshold = 1.00; // Lock engages below this speed
float base_damping_factor = 0.05;     // Constant rotational resistance
float filter_alpha = 0.07;            // Velocity smoothing factor
float smoothed_velocity = 0.0f;

// PID Controller with the final tuned P, I, and D values
PIDController pid_position(30.00, 0.05, 0.00, 1000, 5.0);

// --- Position Reporting Variables ---
unsigned long last_report_time = 0;   // Stores the last time position was reported
const int report_interval = 100;      // Report position every 100 milliseconds

// Forward declaration for local functions
float normalizeAngle(float angle);
void reportPosition();

void setup() {
  Serial.begin(115200);
  SPI.begin(14, 12, 13); // SCK, MISO, MOSI

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

  Serial.println("\n--- Haptic Knob Initialized ---");
}

void loop() {
  motor.loopFOC();

  // Get raw velocity and apply our simple anti-vibration filter
  float raw_velocity = motor.shaft_velocity;
  smoothed_velocity = (filter_alpha * raw_velocity) + ((1.0 - filter_alpha) * smoothed_velocity);

  // --- Calculate the two components of torque ---

  // 1. Damping Torque: Always active, provides a "heavy" feel during motion.
  float damping_torque = -base_damping_factor * smoothed_velocity;

  // 2. PID Torque: Only active when the motor is in a locked state.
  float pid_torque = 0;

  // Check if we should be in a locked state
  if (abs(smoothed_velocity) < velocity_lock_threshold) {
    if (!is_locked) {
      // Engage the lock: capture the current angle as our new target.
      target_angle = motor.shaft_angle;
      is_locked = true;
    }

    // Calculate the error from our captured target angle.
    float error = normalizeAngle(target_angle - motor.shaft_angle);

    // The PID controller calculates the torque to hold the position.
    pid_torque = pid_position(error);

  } else {
    // Breakaway: disengage the lock.
    is_locked = false;
    // pid_torque remains 0.
  }

  // --- Combine the torques for the final output ---
  float total_torque = damping_torque + pid_torque;
  motor.move(total_torque);

  // --- Periodically report the motor's position ---
  reportPosition();
}

/**
 * @brief Normalizes an angle to the range -PI to PI for shortest-path calculations.
 */
float normalizeAngle(float angle){
  float a = fmod(angle + PI, 2*PI);
  return a >= 0 ? (a - PI) : (a + PI);
}

/**
 * @brief Prints the motor's current shaft angle to the Serial Monitor at a fixed interval.
 */
void reportPosition() {
  if (millis() - last_report_time > report_interval) {
    last_report_time = millis(); // Update the timer
    Serial.print("Position [rad]: ");
    Serial.println(motor.shaft_angle);
  }
}