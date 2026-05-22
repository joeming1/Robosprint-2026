#include <Arduino.h>
// ============================================================
//  motor.cpp
//  Motor driver pin control
//  EN is ACTIVE-LOW: digitalWrite LOW = driver enabled
//
//  pwm_left / pwm_right are written by encoder_pid() (manual)
//  or encoder_auto() (autonomous), then motor_run() drives pins.
// ============================================================

#include "Arduino.h"
#include "motor.hpp"

#define MOTOR_L_ENABLE 5
#define MOTOR_L_PWM1   6
#define MOTOR_L_PWM2   7

#define MOTOR_R_ENABLE 2
#define MOTOR_R_PWM1   4
#define MOTOR_R_PWM2   3

// Defined here, extern'd in motor.hpp
int pwm_left  = 0;
int pwm_right = 0;

void motor_setup() {
  pinMode(MOTOR_L_ENABLE, OUTPUT);
  pinMode(MOTOR_L_PWM1,   OUTPUT);
  pinMode(MOTOR_L_PWM2,   OUTPUT);
  pinMode(MOTOR_R_ENABLE, OUTPUT);
  pinMode(MOTOR_R_PWM1,   OUTPUT);
  pinMode(MOTOR_R_PWM2,   OUTPUT);

  digitalWrite(MOTOR_L_ENABLE, LOW);  // active-low: LOW = enabled
  digitalWrite(MOTOR_R_ENABLE, LOW);
}

// Single motor_run() used by BOTH manual and autonomous modes
// Just set pwm_left / pwm_right before calling this
void motor_run() {
  if (pwm_left >= 0) {
    analogWrite(MOTOR_L_PWM1, pwm_left);
    analogWrite(MOTOR_L_PWM2, 0);
  } else {
    analogWrite(MOTOR_L_PWM1, 0);
    analogWrite(MOTOR_L_PWM2, -pwm_left);
  }

  if (pwm_right >= 0) {
    analogWrite(MOTOR_R_PWM1, pwm_right);
    analogWrite(MOTOR_R_PWM2, 0);
  } else {
    analogWrite(MOTOR_R_PWM1, 0);
    analogWrite(MOTOR_R_PWM2, -pwm_right);
  }
}

void motor_stop() {
  pwm_left  = 0;
  pwm_right = 0;
  motor_run();
}
