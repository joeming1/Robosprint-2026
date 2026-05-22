#ifndef LINEAR_ACTUATOR_HPP
#define LINEAR_ACTUATOR_HPP

#include <Arduino.h>

// =====================================================
//  BTS7960 PIN DEFINITIONS
//  LEN/REN  — EN HIGH = enabled, EN LOW = disabled
//  LPWM/RPWM — PWM pins (44, 46 — both PWM-capable)
//  Limit switches — stop at physical end positions
// =====================================================

extern const int LEN;
extern const int REN;
extern const int LPWM;
extern const int RPWM;
// extern bool extendLocked;

// extern const int LIMIT_EXTEND;   // switch pin at full extend
// extern const int LIMIT_RETRACT;  // switch pin at full retract

// =====================================================
//  FUNCTION DECLARATIONS
// =====================================================

void actuator_setup();

// Manual control — called directly from controller
void extend();
void retract();
void stopActuator();
void changeDirectionDelay();

// Autonomous control — call with duration in milliseconds
// Stops early if limit switch triggers
void extend_auto(unsigned long durationMs);
void retract_auto(unsigned long durationMs);

#endif
