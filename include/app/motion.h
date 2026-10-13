#ifndef APP_MOTION_H
#define APP_MOTION_H

#include "vmath.h"

void motion_init(void);
void motion_update(void);

// Relative move: delta_distance in m (+forward), delta_angle in rad (+CCW). Either may be 0.
void motion_move(double delta_distance, double delta_angle);
void motion_turn_test(bool left);
void motion_test(bool forward);
void motion_zero_position(void);
// Step-response test to calibrate Ka; place the robot on blocks (wheels free) and read the result from the debug output.
void motion_ka_test(void);

extern double acceleration;//m/s^2
extern double velocity;//m/s
extern double distance;//m
extern double target_v; //desired linear velocity (of the robot) in m/s
extern double target_w; //desired angular velocity or rotational velocity (of the robot) in rad/s
extern double target_a; //desired linear acceleration (of the robot) in m/s^2
extern double max_achieved_velocity; // m/s
extern double distance_at_max_velocity; // m
extern bool voltage_saturated; // Wheel voltage hit the supply limit during the last move
extern double max_tracking_error; // Peak |target_v - velocity| during the last move, m/s
extern double max_slip_accel; // Peak encoder accel minus IMU accel during the last move, m/s^2
extern double heading; //rad
extern double ka_test_w_ss; // Ka test: steady-state wheel speed, rad/s
extern double ka_test_tau; // Ka test: time constant, s
extern double ka_test_Kv_meas; // Ka test: measured Kv
extern double ka_test_Ka; // Ka test: Kv * tau
extern double ka_test_Ka_meas; // Ka test: Kv_meas * tau
extern double target_heading; //rad

#endif // APP_MOTION_H