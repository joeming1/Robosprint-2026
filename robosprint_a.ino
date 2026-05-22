// ============================================================
//  main.ino
//  Differential Drive Robot
//  Manual (PS2) + Autonomous (Square button toggle)
//
//  Manual mode:
//    Joystick/D-pad → drive motors (encoder straight correction)
//    L1/R1          → stepper CW/CCW
//    L2/R2          → actuator extend/retract
//
//  Autonomous mode (Square toggles on/off):
//    Runs predefined sequence then returns to manual
// ============================================================

#include "controller.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include "encoder_auto.hpp"
#include "stepper_motor.hpp"
#include "LinearActuator.hpp"

void setup() {
  controller_setup();
  motor_setup();
  encoder_setup();
  stepper_setup();
  actuator_setup();
}

void loop() {
  controller_feedback();  // reads PS2, handles button logic
  // continuously monitor limit sw

  if (autonomous) {
    Serial.println("RASP,CAMERA");
    // if (Serial.available()) {  
    // input = Serial.readStringUntil('\n');
    // input.trim();
    // }
    // ============================================================
    //  AUTONOMOUS SEQUENCE
    //  Add or reorder steps here to change the sequence.
    //  Each function blocks until complete then moves to next.
    // ============================================================
    // controller_feedback();
    // 1. Drive forward 1 revolution
    // encodermove_auto(1320, 1320, true);
    // encodermove_auto(1320, 1320, false);
    // autonomous=false;

    // // 2. Extend actuator (up to 3 seconds or until limit switch)
    // extend_auto(3000);

    // // 3. Rotate stepper 200 steps clockwise
    // stepper_auto(200, true, 800);

    // // 4. Retract actuator
    // retract_auto(3000);

    // // Sequence complete — return to manual
    // autonomous = false;
    // Serial.println("Autonomous sequence complete — returning to manual");

  } else {
    // retract();
    // ============================================================
    //  MANUAL MODE
    //  encoder_pid writes pwm_left/pwm_right with correction
    //  motor_run drives the pins
    //  Stepper and actuator handled inside controller_feedback()
    // ============================================================
    encoder_pid();
    motor_run();
  }
}
