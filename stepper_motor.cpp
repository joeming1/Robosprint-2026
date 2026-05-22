// ============================================================
//  stepper_motor.cpp
//  DRV8825 stepper motor driver
//
//  Manual  : stepper_manual() — steps one pulse per loop call
//            while button is held, stop when released
//  Auto    : stepper_auto()   — runs fixed steps then returns
// ============================================================

#include "stepper_motor.hpp"
#include <Arduino.h>

const int STEP_PIN = 39;
const int DIR_PIN  = 41;
const int M0       = 47;
const int M1       = 45;
const int M2       = 43;
const int STEP_EN       = 35;

// ---- Manual step timing ----
// How fast the stepper moves during manual button hold
// Lower = faster. 800µs = safe starting speed
const int MANUAL_STEP_DELAY = 100;

// ============================================================
//  SETUP
// ============================================================
void stepper_setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN,  LOW);
  digitalWrite(STEP_EN,LOW);

  // Default to full step — change if needed
  setMicrostep(FULL_STEP);
}

// ============================================================
//  MICROSTEP MODE
// ============================================================
void setMicrostep(MicrostepMode mode) {
  switch (mode) {
    case FULL_STEP:       digitalWrite(M0,LOW);  digitalWrite(M1,LOW);  digitalWrite(M2,LOW);  break;
    case HALF_STEP:       digitalWrite(M0,HIGH); digitalWrite(M1,LOW);  digitalWrite(M2,LOW);  break;
    case QUARTER_STEP:    digitalWrite(M0,LOW);  digitalWrite(M1,HIGH); digitalWrite(M2,LOW);  break;
    case EIGHTH_STEP:     digitalWrite(M0,HIGH); digitalWrite(M1,HIGH); digitalWrite(M2,LOW);  break;
    case SIXTEENTH_STEP:  digitalWrite(M0,HIGH); digitalWrite(M1,LOW);  digitalWrite(M2,HIGH); break;
    case THIRTY_SECOND:   digitalWrite(M0,HIGH); digitalWrite(M1,HIGH); digitalWrite(M2,HIGH); break;
  }
}

// ============================================================
//  MANUAL CONTROL
//
//  Call stepper_manual(true/false) every loop while button held
//  Each call sends one step pulse
//  Call stepper_stop_manual() when button released
//
//  In controller.cpp:
//    if (ps2x.Button(PSB_L1)) stepper_manual(true);   // CW
//    if (ps2x.Button(PSB_R1)) stepper_manual(false);  // CCW
//    if (ps2x.ButtonReleased(PSB_L1) ||
//        ps2x.ButtonReleased(PSB_R1)) stepper_stop_manual();
// ============================================================
void stepper_manual(bool clockwise) {

  static bool lastDir = false;

  if (clockwise != lastDir) {
    digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
    lastDir = clockwise;
  }

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN, LOW);

  delayMicroseconds(MANUAL_STEP_DELAY);
}

void stepper_stop_manual() {
  digitalWrite(STEP_PIN, LOW);
}

// ============================================================
//  AUTONOMOUS CONTROL
//
//  Blocking — runs all steps then returns.
//  Call from your autonomous sequence.
//
//  Usage examples:
//    stepper_auto(200,  true,  800);  // 200 steps CW,  ~800rpm
//    stepper_auto(400,  false, 500);  // 400 steps CCW, faster
//    stepper_auto(1600, true,  1200); // 1600 steps CW, slower
//
//  steps            — total step pulses to send
//  clockwise        — true = CW, false = CCW
//  speedDelayMicros — microseconds between pulses (lower = faster)
//                     minimum safe value ~200, comfortable ~800
// ============================================================
void stepper_auto(int steps, bool clockwise, int speedDelayMicros) {
  Serial.print("Stepper: ");
  Serial.print(steps);
  Serial.print(" steps ");
  Serial.println(clockwise ? "CW" : "CCW");

  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(speedDelayMicros);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(speedDelayMicros);
  }

  Serial.println("Stepper: done");
}
