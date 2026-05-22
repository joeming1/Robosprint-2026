#ifndef MOTOR_HPP
#define MOTOR_HPP

extern int pwm_left;
extern int pwm_right;

void motor_setup();
void motor_run();
void motor_stop();

#endif
