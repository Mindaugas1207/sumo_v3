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
};

extern double left_setpoint;//rad
extern double right_setpoint;//rad

void motion_init(void);
void motion_update(void);
void motion_reset(void);

MotionData get_motion_data(void);

#endif // APP_MOTION_H