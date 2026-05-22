// ============================================================
//  encoder_auto.cpp
//  Autonomous PID move — writes pwm_left/pwm_right directly
//  and calls motor_run() each step, same as manual mode.
// ============================================================

#include "Arduino.h"
#include "encoder.hpp"     // encodervalue_L/R, resetEncoder()
#include "motor.hpp"       // pwm_left, pwm_right, motor_run(), motor_stop()
#include "controller.hpp"

const long COUNTS_PER_REV  = 1320;
const int  INTERVALS       = 10;
const long PULSES_PER_STEP = COUNTS_PER_REV / INTERVALS;  // 132

long INERTIA_COUNTS_L = 330;
long INERTIA_COUNTS_R = 330;

const int BASE_PWM_L = 150;
const int BASE_PWM_R = 150;

float Kp_L = 1.2, Ki_L = 0.05, Kd_L = 0.8;
float Kp_R = 1.2, Ki_R = 0.05, Kd_R = 0.8;

float pidIntegral_L = 0, pidLastError_L = 0;
float pidIntegral_R = 0, pidLastError_R = 0;

float computePID_L(float expectedRate, float actualRate) {
  float error      = expectedRate - actualRate;
  pidIntegral_L   += error;
  float derivative = error - pidLastError_L;
  pidLastError_L   = error;
  return (Kp_L * error) + (Ki_L * pidIntegral_L) + (Kd_L * derivative);
}

float computePID_R(float expectedRate, float actualRate) {
  float error      = expectedRate - actualRate;
  pidIntegral_R   += error;
  float derivative = error - pidLastError_R;
  pidLastError_R   = error;
  return (Kp_R * error) + (Ki_R * pidIntegral_R) + (Kd_R * derivative);
}

void resetPID() {
  pidIntegral_L = pidLastError_L = 0;
  pidIntegral_R = pidLastError_R = 0;
}

// ============================================================
//  encodermove_auto()
//  Sets pwm_left/pwm_right each step → calls motor_run()
//  Exact same motor path as manual mode — no extra functions.
// ============================================================
void encodermove_auto(long targetPulses_L, long targetPulses_R, bool forward) {

  long effective_L = targetPulses_L - INERTIA_COUNTS_L;
  long effective_R = targetPulses_R - INERTIA_COUNTS_R;

  if (effective_L <= 0 || effective_R <= 0) {
    Serial.println("ERROR: INERTIA_COUNTS too large for target!");
    return;
  }

  resetEncoder();
  resetPID();

  long totalSteps  = min(effective_L, effective_R) / PULSES_PER_STEP;
  long remainder_L = effective_L - (totalSteps * PULSES_PER_STEP);
  long remainder_R = effective_R - (totalSteps * PULSES_PER_STEP);

  // Use local vars during PID loop, write to globals each iteration
  int  cur_pwm_L = BASE_PWM_L;
  int  cur_pwm_R = BASE_PWM_R;
  long last_L    = 0;
  long last_R    = 0;

  Serial.println("--- Autonomous Move Start ---");
  Serial.print("Target L: ");    Serial.print(targetPulses_L);
  Serial.print("  Target R: ");  Serial.println(targetPulses_R);

  // ============================================================
  //  STEP LOOP
  // ============================================================
  for (int step = 0; step < totalSteps; step++) {
    controller_feedback();  

    long stepTarget       = (step + 1) * PULSES_PER_STEP;
    unsigned long stepStart   = millis();
    unsigned long stepTimeout = stepStart + 2000;

    // Write PWM to globals and drive motors — same path as manual
    pwm_left  = forward ?  cur_pwm_L : -cur_pwm_L;
    pwm_right = forward ?  cur_pwm_R : -cur_pwm_R;
    motor_run();

    // Wait for both wheels to reach step checkpoint
    while (true) {
      long count_L, count_R;
      noInterrupts();
      count_L = encodervalue_L;
      count_R = encodervalue_R;
      interrupts();

      if (count_L >= stepTarget && count_R >= stepTarget) break;
      if (millis() > stepTimeout) { Serial.println("Step timeout!"); break; }
    }

    // Measure speed this step
    unsigned long stepTime = millis() - stepStart;
    if (stepTime == 0) stepTime = 1;

    long snap_L, snap_R;
    noInterrupts();
    snap_L = encodervalue_L;
    snap_R = encodervalue_R;
    interrupts();

    long pulses_L = snap_L - last_L;
    long pulses_R = snap_R - last_R;
    last_L = snap_L;
    last_R = snap_R;

    float actual_L = (float)pulses_L / stepTime;
    float actual_R = (float)pulses_R / stepTime;
    float expected = (float)PULSES_PER_STEP / stepTime;

    // PID update
    cur_pwm_L = constrain((int)(cur_pwm_L + computePID_L(expected, actual_L) * 20), 40, 255);
    cur_pwm_R = constrain((int)(cur_pwm_R + computePID_R(expected, actual_R) * 20), 40, 255);

    Serial.print("Step "); Serial.print(step + 1);
    Serial.print(" | L: "); Serial.print(pulses_L);
    Serial.print(" R: ");   Serial.print(pulses_R);
    Serial.print(" | PWM L: "); Serial.print(cur_pwm_L);
    Serial.print(" R: ");       Serial.println(cur_pwm_R);
  }

  // ============================================================
  //  REMAINDER pulses after last full step
  // ============================================================
  if (remainder_L > 0 || remainder_R > 0) {
    unsigned long remTimeout = millis() + 2000;

    pwm_left  = forward ?  cur_pwm_L : -cur_pwm_L;
    pwm_right = forward ?  cur_pwm_R : -cur_pwm_R;
    motor_run();

    while (true) {
      long count_L, count_R;
      noInterrupts();
      count_L = encodervalue_L;
      count_R = encodervalue_R;
      interrupts();

      if (count_L >= effective_L && count_R >= effective_R) break;
      if (millis() > remTimeout) break;
    }
  }

  // ============================================================
  //  CUT POWER — coast to real target using inertia buffer
  // ============================================================
  motor_stop();
  Serial.println("Power cut — coasting...");
  delay(500);

  long final_L, final_R;
  noInterrupts();
  final_L = encodervalue_L;
  final_R = encodervalue_R;
  interrupts();

  long error_L = final_L - targetPulses_L;
  long error_R = final_R - targetPulses_R;

  Serial.println("--- Move Complete ---");
  Serial.print("Final L: "); Serial.print(final_L);
  Serial.print(" Error L: "); Serial.println(error_L);
  Serial.print("Final R: "); Serial.print(final_R);
  Serial.print(" Error R: "); Serial.println(error_R);

  // Auto-tune inertia based on real overshoot/undershoot
  INERTIA_COUNTS_L = constrain(INERTIA_COUNTS_L + error_L / 2, 0, targetPulses_L / 2);
  INERTIA_COUNTS_R = constrain(INERTIA_COUNTS_R + error_R / 2, 0, targetPulses_R / 2);

  Serial.print("Next inertia L: "); Serial.print(INERTIA_COUNTS_L);
  Serial.print("  R: ");            Serial.println(INERTIA_COUNTS_R);
}
