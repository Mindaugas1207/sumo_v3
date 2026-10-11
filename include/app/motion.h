#ifndef APP_MOTION_H
#define APP_MOTION_H

#include "vmath.h"

void motion_init(void);
void motion_update(void);

void motion_test(bool forward);
void motion_zero_position(void);

extern double acceleration;//m/s^2
extern double velocity;//m/s
extern double distance;//m
extern double target_v; //desired linear velocity (of the robot) in m/s
extern double target_w; //desired angular velocity or rotational velocity (of the robot) in rad/s
extern double target_a; //desired linear acceleration (of the robot) in m/s^2
extern bool max_speed_reached;
extern double max_achieved_velocity; // m/s
extern double distance_at_max_velocity; // m

#endif // APP_MOTION_H