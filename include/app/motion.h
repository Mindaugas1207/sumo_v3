#ifndef APP_MOTION_H
#define APP_MOTION_H

#include "vmath.h"

struct MotionData
{
    //IMU
    vmath::vector3d<double> linear_acceleration;
    vmath::vector3d<double> angular_velocity;
    vmath::vector3d<double> orientation;
    bool stationary;
    //Wheel encoders
    double right_wheel_angle;
    double left_wheel_angle;
    double right_wheel_velocity;
    double left_wheel_velocity;
    double right_wheel_distance;
    double left_wheel_distance;
    double forward_distance;
};

void motion_init(void);
void motion_update(void);
void motion_reset(void);

void motion_set_motors_enabled(bool enabled);
void move_linear(double distance);
void move_rotational_degrees(double angle);
void move_rotational(double angle);
bool is_move_complete(void);

MotionData get_motion_data(void);

#endif // APP_MOTION_H