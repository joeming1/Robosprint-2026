// ============================================================
//  LinearActuator.cpp
//  BTS7960 motor driver for linear actuator
//
//  EN HIGH = driver enabled, EN LOW = disabled
//  LPWM → extend direction PWM
//  RPWM → retract direction PWM
//
//  Limit switches (INPUT_PULLUP):
//    LIMIT_EXTEND  — reads LOW when fully extended
//    LIMIT_RETRACT — reads LOW when fully retracted
// ============================================================

#include "LinearActuator.hpp"
#include <Arduino.h>

// ---- Pin definitions ----
const int LEN  = 31;
const int REN  = 31;   // tied together — one pin enables both sides
const int LPWM = 46;
const int RPWM = 44;
bool extendLocked = false;

// ---- Limit switch pins (change to your actual wiring) ----
// INPUT_PULLUP: switch connects pin to GND when triggered → reads LOW
// const int LIMIT_EXTEND  = 37;

// ============================================================
//  SETUP
// ============================================================
void actuator_setup() {
  pinMode(LEN,  OUTPUT);
  pinMode(REN,  OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);

  // Start disabled
  digitalWrite(LEN, HIGH);
  digitalWrite(REN, HIGH);

  stopActuator();
  delay(1000);

}

// ============================================================
//  INTERNAL HELPERS
// ============================================================
static void enableDriver() {
  digitalWrite(LEN, HIGH);
  digitalWrite(REN, HIGH);
}

static void disableDriver() {
  digitalWrite(LEN, LOW);
  digitalWrite(REN, LOW);
}

// ============================================================
//  MANUAL CONTROL
//  Called directly from controller buttons (L2/R2)
//  Caller must check limit switches before calling
// ============================================================
void retract() {
  extendLocked=false;
  enableDriver();
  analogWrite(LPWM, 200);
  analogWrite(RPWM, 0);
}


void extend() {

  // if (extendLocked) {
  //   Serial.println("EXTEND LOCKED");
  //   stopActuator();
  //   return;
  // }

  enableDriver();
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 200);
}

void stopActuator() {
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 0);
  disableDriver();
}

// Safe direction change — always stop briefly before reversing
void changeDirectionDelay() {
  stopActuator();
  delay(300);
}

// ============================================================
//  AUTONOMOUS CONTROL
//
//  extend_auto(durationMs)  — extend for up to durationMs ms
//  retract_auto(durationMs) — retract for up to durationMs ms
//
//  Both stop immediately if the corresponding limit switch
//  triggers before the duration expires.
//
//  Usage:
//    extend_auto(3000);    // extend for up to 3 seconds
//    retract_auto(3000);   // retract for up to 3 seconds
// ============================================================
void extend_auto(unsigned long durationMs) {

  Serial.print("Actuator: extending for ");
  Serial.print(durationMs);
  Serial.println("ms");

  enableDriver();
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 200);

  unsigned long startTime = millis();

  // Run until duration expires OR limit switch triggers
  while (millis() - startTime < durationMs) {

  }

  stopActuator();
  Serial.println("Actuator: extend done");
}

void retract_auto(unsigned long durationMs) {


  Serial.print("Actuator: retracting for ");
  Serial.print(durationMs);
  Serial.println("ms");

  enableDriver();
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 200);

  unsigned long startTime = millis();

  while (millis() - startTime < durationMs) {
      break;
  }

  stopActuator();
  Serial.println("Actuator: retract done");
}
