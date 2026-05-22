#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP

#include <Arduino.h>

// =====================================================
//  PIN DEFINITIONS
// =====================================================
extern const int STEP_PIN;
extern const int DIR_PIN;
extern const int M0;
extern const int M1;
extern const int M2;
extern const int STEP_EN;

// =====================================================
//  MICROSTEP MODES
// =====================================================
enum MicrostepMode {
  FULL_STEP,
  HALF_STEP,
  QUARTER_STEP,
  EIGHTH_STEP,
  SIXTEENTH_STEP,
  THIRTY_SECOND
};

// =====================================================
//  FUNCTION DECLARATIONS
// =====================================================
void stepper_setup();

// Set microstepping resolution
void setMicrostep(MicrostepMode mode);

// Manual control — move while button held, stop when released
void stepper_manual(bool clockwise);
void stepper_stop_manual();

// Autonomous control — move fixed number of steps then stop
// Usage: stepper_auto(200, true, 800)
//   steps            — number of steps to move
//   clockwise        — true = CW, false = CCW
//   speedDelayMicros — delay between steps (lower = faster)
void stepper_auto(int steps, bool clockwise, int speedDelayMicros);

#endif
