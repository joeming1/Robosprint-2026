// ============================================================
//  encoder.cpp
//  Encoder ISRs + PD straight-line correction
//  + acceleration / deceleration ramping
//  + HARD STOP anti-ghost motion fix
// ============================================================

#include "Arduino.h"
#include "encoder.hpp"
#include "controller.hpp"
#include "motor.hpp"

#define Encoder_L_A 18
#define Encoder_L_B 19
#define Encoder_R_A 20
#define Encoder_R_B 21

// ============================================================
// ENCODER COUNTS
// ============================================================
volatile long encodervalue_L = 0;
volatile long encodervalue_R = 0;

// ============================================================
// STRAIGHT-LINE PD SETTINGS
// ============================================================
const int DEADBAND = 15;

const float Kp_straight = 0.5f;
const float Kd_straight = 1.5f;

// ============================================================
// ACCELERATION / DECELERATION
// ============================================================
const int ACCEL_STEP = 3;
const int DECEL_STEP = 5;

// ============================================================
// PWM STATE
// ============================================================
int current_pwm_left = 0;
int current_pwm_right = 0;

// ============================================================
// STATE
// ============================================================
bool wasDrivingStraight = false;

long previousError = 0;
unsigned long previousTime = 0;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void isEncoderLeft();
void isEncoderRight();

// ============================================================
// HARD STOP (ANTI-GHOST MOTION)
// ============================================================
void hardStop() {
  current_pwm_left = 0;
  current_pwm_right = 0;
  pwm_left = 0;
  pwm_right = 0;
}

// ============================================================
// SETUP
// ============================================================
void encoder_setup() {

  pinMode(Encoder_L_A, INPUT_PULLUP);
  pinMode(Encoder_L_B, INPUT_PULLUP);
  pinMode(Encoder_R_A, INPUT_PULLUP);
  pinMode(Encoder_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Encoder_L_A), isEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_L_B), isEncoderLeft, CHANGE);

  attachInterrupt(digitalPinToInterrupt(Encoder_R_A), isEncoderRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_R_B), isEncoderRight, CHANGE);
}

// ============================================================
// ISRs
// ============================================================
void isEncoderLeft() { encodervalue_L++; }
void isEncoderRight() { encodervalue_R++; }

// ============================================================
// RESET
// ============================================================
void resetEncoder() {

  noInterrupts();
  encodervalue_L = 0;
  encodervalue_R = 0;
  interrupts();

  previousError = 0;
  previousTime = millis();
}

// ============================================================
// RAMP
// ============================================================
int rampPWM(int current, int target) {

  if (current < target) {
    current += ACCEL_STEP;
    if (current > target) current = target;
  }
  else if (current > target) {
    current -= DECEL_STEP;
    if (current < target) current = target;
  }

  return current;
}

// ============================================================
// MAIN CONTROLLER
// ============================================================
void encoder_pid() {

  int target_left = demand_left;
  int target_right = demand_right;

  // ========================================================
  // DEADZONE
  // ========================================================
  if (abs(target_left) < DEADBAND) target_left = 0;
  if (abs(target_right) < DEADBAND) target_right = 0;

  // ========================================================
  // DIRECTION DETECTION
  // ========================================================
  bool goForward  = (target_left > 0 && target_right > 0);
  bool goBackward = (target_left < 0 && target_right < 0);

  bool drivingStraight = (goForward || goBackward);

  bool turning =
      (target_left > 0 && target_right < 0) ||
      (target_left < 0 && target_right > 0);

  // ========================================================
  // HARD STOP CONDITIONS (FIXES GHOST MOTION)
  // ========================================================
  static int last_left = 0;
  static int last_right = 0;

  bool directionChanged =
      (last_left > 0) != (target_left > 0) ||
      (last_right > 0) != (target_right > 0);

  last_left = target_left;
  last_right = target_right;

  if ((target_left == 0 && target_right == 0) || directionChanged) {
    hardStop();
    return;
  }

  // ========================================================
  // RESET ENCODERS ON STRAIGHT ENTRY
  // ========================================================
  if (drivingStraight && !wasDrivingStraight) {
    resetEncoder();
  }

  wasDrivingStraight = drivingStraight;

  // ========================================================
  // PD CORRECTION (STRAIGHT ONLY)
  // ========================================================
  if (drivingStraight) {

    long snapL, snapR;

    noInterrupts();
    snapL = encodervalue_L;
    snapR = encodervalue_R;
    interrupts();

    long error = snapL - snapR;

    unsigned long now = millis();
    float dt = (now - previousTime) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;

    float derivative = (error - previousError) / dt;

    int correction =
        (int)((Kp_straight * error) +
              (Kd_straight * derivative));

    if (goBackward) {
      target_left += correction;
      target_right -= correction;
    } else {
      target_left -= correction;
      target_right += correction;
    }

    previousError = error;
    previousTime = now;
  }

  // ========================================================
  // CLAMP
  // ========================================================
  target_left = constrain(target_left, -128, 128);
  target_right = constrain(target_right, -128, 128);

  // ========================================================
  // RAMP (ONLY WHEN NOT TURNING)
  // ========================================================
  if (turning) {
    current_pwm_left = target_left;
    current_pwm_right = target_right;
  }
  else {
    current_pwm_left = rampPWM(current_pwm_left, target_left);
    current_pwm_right = rampPWM(current_pwm_right, target_right);
  }

  // ========================================================
  // OUTPUT
  // ========================================================
  pwm_left = current_pwm_left;
  pwm_right = current_pwm_right;
}