// ============================================================
//  controller.cpp
//  PS2 controller input handling
//
//  Joystick / D-pad  → demand_left / demand_right (drive)
//  L1 / R1           → stepper CW / CCW
//  L2 / R2           → actuator extend / retract
//  Square (PINK)     → toggle autonomous mode
// ============================================================

#include <PS2X_lib.h>
#include "Arduino.h"
#include "controller.hpp"
#include "stepper_motor.hpp"
#include "LinearActuator.hpp"

#define PS2_DAT 48
#define PS2_CMD 49
#define PS2_SEL 51
#define PS2_CLK 53

#define pressures false
#define rumble    false

PS2X ps2x;
int  error = 0;

// ---- Globals defined here, extern'd in controller.hpp ----
int  demand_left  = 0;
int  demand_right = 0;
bool autonomous   = false;


void controller_setup() {
  Serial.begin(57600);
  delay(500);

  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  if (error == 0)      Serial.println("PS2: Controller linked OK");
  else if (error == 1) Serial.println("PS2 ERROR: No controller found.");
  else if (error == 2) Serial.println("PS2 ERROR: Controller refusing commands.");
}

void controller_feedback() {
  if (error == 1) return;

  ps2x.read_gamepad(false, 0);

  // ---- Drive (left/right stick Y axes) ----
  demand_left  = 128 - ps2x.Analog(PSS_LY);
  demand_right = 128 - ps2x.Analog(PSS_RY);

  // ---- D-pad overrides ----
  if (ps2x.Button(PSB_PAD_UP))    { demand_left =  128; demand_right =  128; }
  if (ps2x.Button(PSB_PAD_DOWN))  { demand_left = -128; demand_right = -128; }
  if (ps2x.Button(PSB_PAD_LEFT))  { demand_left = -128; demand_right =  128; }
  if (ps2x.Button(PSB_PAD_RIGHT)) { demand_left =  128; demand_right = -128; }

  // ---- Square — toggle autonomous mode ----
  if (ps2x.ButtonPressed(PSB_PINK)) {
    autonomous = !autonomous;
    Serial.print("Autonomous: ");
    Serial.println(autonomous ? "ON" : "OFF");
  }

  // if(ps2x.ButtonPressed(PSB_GREEN)){
  //   autonomous = 0;
  // }

  // ---- Block drive demands during autonomous ----
  if (autonomous) {
    demand_left  = 0;
    demand_right = 0;
    return;  // skip manual stepper/actuator during autonomous
  }

  // ---- L1 / R1 — stepper manual control ----
  // L1 held = clockwise, R1 held = counterclockwise
  if (ps2x.Button(PSB_L1))      stepper_manual(true);
  else if (ps2x.Button(PSB_R1)) stepper_manual(false);
  else                          stepper_stop_manual();

  // ---- L2 / R2 — linear actuator manual control ----
  // L2 held = extend, R2 held = retract
  // Releasing both stops the actuator
  if (ps2x.Button(PSB_L2))      {Serial.println("l2 presssed"); extend();}
  else if (ps2x.Button(PSB_R2)) retract();
  else if (ps2x.Button(PSB_GREEN)) extend_auto(2100);
  else if (ps2x.Button(PSB_RED)) extend_auto(3500);
  else                          stopActuator();
}
