#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

// Shared joystick demands — written by controller, read by encoder + motor
extern int demand_left;
extern int demand_right;


// Autonomous mode flag — toggled by Square button in controller_feedback()
extern bool autonomous;

void controller_setup();
void controller_feedback();

#endif
