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
extern double start_delay_L_ms; // Move command to left wheel moving, ms; -1 if not yet
extern double start_delay_R_ms; // Move command to right wheel moving, ms; -1 if not yet
extern double max_heading_error; // Peak |target_heading - yaw| during the last move, rad
extern double odometry_heading; // Heading from the wheel difference since the last zero, rad
struct KaTestResult
{
    double w_ss; // Steady-state wheel speed, rad/s
    double tau; // Time constant, s; -1 if 63.2% was never reached
    double Kv; // Measured Kv, V/(rad/s)
    double Ka; // Kv * tau
};
extern KaTestResult ka_test_results[4]; // Left forward, left reverse, right forward, right reverse
extern double target_heading; //rad

#endif // APP_MOTION_H