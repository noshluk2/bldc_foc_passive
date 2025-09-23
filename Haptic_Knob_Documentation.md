# Haptic Knob Implementation Documentation

## Overview
This document provides a comprehensive explanation of a haptic knob implementation using the SimpleFOC library to control a Brushless DC (BLDC) motor with a magnetic encoder. The system delivers a detent-like haptic feedback experience by combining damping torque and position-locking torque via a Proportional-Integral-Derivative (PID) controller. The code is designed for an Arduino-compatible microcontroller and leverages Field-Oriented Control (FOC) for precise motor control.

## Table of Contents
1. [System Architecture](#system-architecture)
2. [Key Concepts](#key-concepts)
3. [Code Structure and Workflow](#code-structure-and-workflow)
4. [Implementation Details](#implementation-details)
5. [Usage Instructions](#usage-instructions)
6. [Potential Enhancements](#potential-enhancements)
7. [Annotated Source Code](#annotated-source-code)

## System Architecture

### Hardware Components
- **BLDC Motor**: Configured with 7 pole pairs, controlled using FOC for smooth and efficient operation.
- **BLDCDriver3PWM**: A 3-phase PWM driver utilizing pins 26, 25, 33 (phase pins), and 32 (enable pin).
- **MagneticSensorSPI**: An AS5048 magnetic encoder connected via SPI (pins 14, 12, 13 for SCK, MISO, MOSI, and 27 for chip select) to provide high-resolution rotor angle and velocity feedback.
- **Microcontroller**: An Arduino-compatible board running the control loop and interfacing with the motor and sensor.

### Software Dependencies
- **SimpleFOC Library**: Facilitates FOC for BLDC motor control, handling phase commutation and sensor integration.
- **SPI Library**: Manages communication with the magnetic sensor.

## Key Concepts

### Field-Oriented Control (FOC)
FOC is an advanced motor control technique that aligns the stator's magnetic field with the rotor's magnetic field to maximize torque efficiency. The SimpleFOC library abstracts:
- **Phase Voltage Control**: Generates precise PWM signals for the motor's three phases.
- **Sensor Integration**: Utilizes encoder data to determine rotor position and velocity.

### Haptic Feedback Mechanism
The haptic knob delivers two distinct torque components:
1. **Damping Torque**: A velocity-proportional resistance that provides a consistent "heavy" feel during rotation.
2. **PID Torque**: Activates when the motor's velocity falls below a predefined threshold, locking the knob to a specific angle using a PID controller.

### PID Controller
The PID controller maintains the knob's position when locked:
- **Proportional Gain (P=30.00)**: Scales the torque response to the angular error.
- **Integral Gain (I=0.05)**: Compensates for steady-state error by accumulating error over time.
- **Derivative Gain (D=0.00)**: Disabled to avoid noise amplification in this application.
- **Output Constraints**: Limited to ±1000 units with a maximum voltage of 5V to ensure safe operation.

### Velocity Smoothing
A low-pass filter reduces noise in velocity measurements:
- **Filter Alpha (0.07)**: Determines the balance between responsiveness and noise suppression.
- **Formula**: `smoothed_velocity = (alpha * raw_velocity) + (1 - alpha) * smoothed_velocity`

### Angle Normalization
The `normalizeAngle` function constrains angle errors to the range [-π, π], ensuring the shortest-path correction for PID control.

## Code Structure and Workflow

### Initialization (setup)
- Configures serial communication (115200 baud) for debugging.
- Initializes SPI, sensor, driver, and motor.
- Sets FOC parameters: sine PWM modulation, torque control mode, and a 5V voltage limit.
- Initializes and enables the FOC algorithm.

### Main Loop (loop)
- Updates FOC calculations with `motor.loopFOC()`.
- Applies a low-pass filter to smooth velocity measurements.
- Computes damping torque continuously.
- Engages PID torque when velocity drops below 1 rad/s, locking the knob to the current angle.
- Combines torques and sends the command to the motor.
- Reports the shaft angle every 100ms via serial output.

### Helper Functions
- **normalizeAngle**: Normalizes angles to [-π, π] for accurate error calculation.
- **reportPosition**: Logs the motor's shaft angle at regular intervals.

## Implementation Details

### Parameter Configuration
- **Velocity Lock Threshold (1.00 rad/s)**: Determines when the position lock engages.
- **Base Damping Factor (0.05)**: Sets the strength of velocity-based resistance.
- **Filter Alpha (0.07)**: Controls velocity smoothing.
- **PID Parameters**: Tuned for stable and responsive position locking.

### Safety Features
- **Voltage Limit (5V)**: Prevents excessive torque, protecting the motor and hardware.
- **Velocity Threshold**: Ensures the lock disengages during intentional rotation, preventing abrupt resistance.

### Serial Reporting
- Position updates are sent every 100ms, providing real-time feedback for debugging and monitoring.

## Usage Instructions
1. **Hardware Setup**:
   - Connect the BLDC motor to the driver (pins 26, 25, 33, 32).
   - Wire the AS5048 sensor to SPI pins (14, 12, 13, 27).
   - Power the driver with a 12V supply.

2. **Software Setup**:
   - Install the SimpleFOC and SPI libraries in the Arduino IDE.
   - Upload the provided code to the microcontroller.

3. **Operation**:
   - Open the Serial Monitor (115200 baud) to view position updates.
   - Rotate the knob to experience damping and locking behavior.
   - Adjust parameters (e.g., PID gains, damping factor) to tune the haptic feel.

4. **Tuning Tips**:
   - Increase `base_damping_factor` for a heavier feel.
   - Adjust `velocity_lock_threshold` to change lock engagement sensitivity.
   - Modify PID gains for faster or smoother locking.

## Potential Enhancements
- **Detent Simulation**: Introduce periodic torque variations to mimic mechanical detents.
- **Dynamic Parameter Adjustment**: Implement serial commands to modify PID gains or damping in real time.
- **Error Handling**: Add checks for sensor or driver failures to improve robustness.
- **Multi-Position Locking**: Allow predefined lock points for a stepped haptic effect.

## Annotated Source Code

<xaiArtifact artifact_id="5ac701a3-345f-4305-a705-091431f37ada" artifact_version_id="0cb2e878-8303-416d-b880-5eef1b1e8f29" title="Haptic_Knob.ino" contentType="text/x-arduino">

#include <SimpleFOC.h>
#include <SPI.h>

// --- Hardware Configuration ---
// Instantiate 3-phase PWM driver with phase pins (A, B, C) and enable pin
BLDCDriver3PWM driver = BLDCDriver3PWM(26, 25, 33, 32);
// Define BLDC motor with 7 pole pairs for FOC calculations
BLDCMotor motor = BLDCMotor(7);
// Configure AS5048 magnetic encoder with SPI interface (CS pin 27)
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 27);

// --- Haptic Feedback Parameters ---
// Target angle for position locking in radians
float target_angle = 0.0f;
// Flag indicating whether the motor is locked to a position
bool is_locked = false;
// Velocity threshold (rad/s) for engaging position lock
float velocity_lock_threshold = 1.00;
// Damping factor for velocity-proportional resistance
float base_damping_factor = 0.05;
// Low-pass filter coefficient for velocity smoothing
float filter_alpha = 0.07;
// Smoothed velocity to reduce noise and vibrations
float smoothed_velocity = 0.0f;

// --- PID Controller Configuration ---
// PID controller with P=30.0, I=0.05, D=0.0, ramp=1000, output limit=5.0V
PIDController pid_position(30.00, 0.05, 0.00, 1000, 5.0);

// --- Position Reporting Variables ---
// Timestamp of the last position report (milliseconds)
unsigned long last_report_time = 0;
// Interval for reporting position (milliseconds)
const int report_interval = 100;

// Forward declarations for helper functions
float normalizeAngle(float angle);
void reportPosition();

void setup() {
  // Initialize serial communication for debugging and monitoring
  Serial.begin(115200);
  // Configure SPI with custom pins (SCK=14, MISO=12, MOSI=13)
  SPI.begin(14, 12, 13);

  // Initialize magnetic sensor for rotor position feedback
  sensor.init();
  // Link sensor to motor for angle and velocity calculations
  motor.linkSensor(&sensor);
  // Set driver power supply voltage to 12V
  driver.voltage_power_supply = 12;
  // Initialize PWM driver
  driver.init();
  // Link driver to motor for phase control
  motor.linkDriver(&driver);

  // Configure FOC with sine PWM modulation and torque control mode
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.controller = MotionControlType::torque;
  // Set voltage limit to 5V for safe operation
  motor.voltage_limit = 5.0;

  // Initialize motor and FOC algorithm
  motor.init();
  motor.initFOC();
  // Enable motor control
  motor.enable();

  // Confirm initialization via serial output
  Serial.println("\n--- Haptic Knob Initialized ---");
}

void loop() {
  // Update FOC calculations for motor commutation and sensor reading
  motor.loopFOC();

  // --- Velocity Smoothing ---
  // Retrieve raw shaft velocity from motor
  float raw_velocity = motor.shaft_velocity;
  // Apply low-pass filter to smooth velocity and reduce noise
  smoothed_velocity = (filter_alpha * raw_velocity) + ((1.0 - filter_alpha) * smoothed_velocity);

  // --- Torque Calculation ---
  // Compute damping torque proportional to velocity for consistent resistance
  float damping_torque = -base_damping_factor * smoothed_velocity;

  // Initialize PID torque (active only when locked)
  float pid_torque = 0;
  // Check if velocity is below threshold to engage position lock
  if (abs(smoothed_velocity) < velocity_lock_threshold) {
    if (!is_locked) {
      // Engage lock by setting current angle as target
      target_angle = motor.shaft_angle;
      is_locked = true;
    }
    // Calculate normalized error between target and current angle
    float error = normalizeAngle(target_angle - motor.shaft_angle);
    // Compute PID torque to maintain position
    pid_torque = pid_position(error);
  } else {
    // Disengage lock when velocity exceeds threshold
    is_locked = false;
  }

  // --- Apply Combined Torque ---
  // Sum damping and PID torques for final motor command
  float total_torque = damping_torque + pid_torque;
  // Send torque command to motor
  motor.move(total_torque);

  // --- Periodic Position Reporting ---
  // Log shaft angle at regular intervals
  reportPosition();
}

/**
 * @brief Normalizes an angle to the range [-π, π] for shortest-path error calculation.
 * @param angle Input angle in radians.
 * @return Normalized angle in radians.
 */
float normalizeAngle(float angle) {
  float a = fmod(angle + PI, 2*PI);
  return a >= 0 ? (a - PI) : (a + PI);
}

/**
 * @brief Reports the motor's shaft angle to the Serial Monitor every 100ms.
 */
void reportPosition() {
  if (millis() - last_report_time > report_interval) {
    last_report_time = millis();
    Serial.print("Position [rad]: ");
    Serial.println(motor.shaft_angle);
  }
}