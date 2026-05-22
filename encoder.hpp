#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "Arduino.h"

// ============================================================
// ENCODER COUNTS
// ============================================================
extern volatile long encodervalue_L;
extern volatile long encodervalue_R;

// ============================================================
// PWM STATE
// ============================================================
extern int current_pwm_left;
extern int current_pwm_right;

// ============================================================
// STATE
// ============================================================
extern bool wasDrivingStraight;

// ============================================================
// FUNCTIONS
// ============================================================
void encoder_setup();
void isEncoderLeft();
void isEncoderRight();
void resetEncoder();
void encoder_pid();

#endif