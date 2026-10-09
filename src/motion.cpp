
#include "config.h"
#include "motion.h"
#include "hardware.h"
#include "vmath.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "pid.h"

constexpr double NominalMotorRPM = 600.0;
constexpr double NominalMotorVoltage = 6.0;
constexpr double SystemVoltage = 12.0;

constexpr double NominalMotorRadPerSec = NominalMotorRPM * 2.0 * M_PI / 60.0;

constexpr double WHEEL_RADIUS = 0.030 / 2.0; // Wheel radius in meters
constexpr double WHEEL_BASE = 0.06; // Distance between the wheels in meters

constexpr double Ks = 0.05;
constexpr double Ke = NominalMotorVoltage / (NominalMotorRadPerSec * SystemVoltage);

constexpr double SurfaceSlipCorrection = 6.0; // Correction factor for surface slip, dimensionless
constexpr double AngularSlipCorrection = 2; // Correction factor for angular slip, dimensionless
constexpr double LinearAccelerationMiddle = 0.7; //Larger value means less time for acceleration and more for deceleration
constexpr double AngularAccelerationMiddle = 0.7; //Larger value means less time for acceleration and more for deceleration

constexpr double MaxAcceleration = SurfaceSlipCorrection * NominalMotorRadPerSec * WHEEL_RADIUS; // Maximum linear acceleration of the robot in m/s^2, assuming the motor can reach its nominal speed instantly
constexpr double MaxDeacceleration = 2.0 * MaxAcceleration;
constexpr double MaxVelocity = 0.8 * NominalMotorRadPerSec * WHEEL_RADIUS; // Maximum linear velocity of the robot in m/s, assuming the motor can reach its nominal speed instantly
constexpr double MaxAngularAcceleration = AngularSlipCorrection * NominalMotorRadPerSec * WHEEL_RADIUS / (WHEEL_BASE / 2.0); // Maximum angular acceleration of the robot in rad/s^2
constexpr double MaxAngulardeacceleration = 2 * MaxAngularAcceleration;
constexpr double MaxAngularVelocity = 0.6 * NominalMotorRadPerSec * WHEEL_RADIUS / (WHEEL_BASE / 2.0); // Maximum angular velocity of the robot in rad/s

constexpr double G = 9.80665; // Gravity constant in m/s^2, used to convert accelerometer readings from g to m/s^2. The standard value is 9.80665 m/s^2.
constexpr double a_cutoff = 10.0; //Hz
constexpr double a_rc_coef = 1.0 / (2.0 * M_PI * a_cutoff);
constexpr double a_min = 0.02; // Minimum acceleration magnitude to consider the robot as non-stationary, in m/s^2.
constexpr double w_min = 0.02; // Minimum angular velocity magnitude to consider the robot as non-stationary, in rad/s.
constexpr double v_min = 0.01; // Minimum linear velocity magnitude to consider the robot as non-stationary, in rad/s.
constexpr vmath::vector3d g = {0.0, 0.0, 1.0}; // Gravity vector in the IMU frame, assuming the IMU is mounted flat with Z axis up

constexpr double madgwick_beta = LSM6DSR::LSM6DSR_GYRO_NOISE_DPS_PER_ROOTHZ * (M_PI / 180.0) * vmath::sqrt<double>(3.0/4.0) * vmath::sqrt<double>(imu_sample_rate_hz);
vmath::MadgwickFilter<double> imu_filter(madgwick_beta);

vmath::vector3d<double> accelBias = {0.0};
vmath::vector3d<double> gyroBias = {0.0};
vmath::vector3d<double> linear_acceleration = {0.0}; // In the robot-heading-aligned frame with Z fixed up, in m/s^2, referenced to initial orientation at power on
vmath::vector3d<double> angular_velocity = {0.0}; // In the robot-heading-aligned frame with Z fixed up, in rad/s, referenced to initial orientation at power on
vmath::vector3d<double> orientation = {0.0}; // Euler angles in radians, in ZYX order (yaw-pitch-roll), referenced to initial orientation at power on
bool stationary = true; // Whether the robot is stationary based on IMU readings, used to determine whether to trust orientation and velocity information from the IMU or not

bool motors_enabled = false;

bool move_started = false;
bool move_complete = true;
bool move_stopped = true; // Whether the robot stopped before completing the current move
int move_type = 0; // 1 for linear move, 0 for rotational move, 2 constant velocity move
double move_target_distance = 0.0; // How far the robot should move for the current move, in meters
double move_target_angle = 0.0; // Target angle for the current move, in radians
double move_start_distance = 0.0; // Distance at the start of the current move, in meters
double move_start_angle = 0.0; // Angle at the start of the current move, in radians

//PID controller for maintaining the robot's heading
PID heading_pid(1.0, 0.0, 0.0); // Example PID gains: Kp = 0.1, Ki = 0.01, Kd = 0.05


utils::time_t average_compute_time = 0;

double angleL_total = 0.0;//rad
double angleR_total = 0.0;//rad
double velocityL = 0.0;//rad/s
double velocityR = 0.0;//rad/s
double accelerationL = 0.0;//m/s^2
double accelerationR = 0.0;//m/s^2
double distanceL = 0.0;//m
double distanceR = 0.0;//m

double acceleration = 0.0;//m/s^2
double velocity = 0.0;//m/s
double distance = 0.0;//m

double target_velocity = 0.0;//m/s
double target_angular_velocity = 0.0;//rad/s

void move_linear(double distance)
{
    move_started = true;
    move_complete = false;
    move_stopped = false;
    move_type = 1; // Linear move
    move_target_distance = distance;
    move_start_distance = distance_forward;
    move_target_angle = 0;
    move_start_angle = orientation.yaw();
}

void move_rotational_degrees(double angle)
{
    move_rotational(angle * M_PI / 180.0);
}

void move_rotational(double angle)
{
    move_started = true;
    move_complete = false;
    move_stopped = false;
    move_type = 0; // Rotational move
    move_target_angle = angle;
    move_start_angle = orientation.yaw();
}

void move_cancel(void)
{
    target_velocity = 0.0;
    target_angular_velocity = 0.0;
    move_started = false;
    move_complete = true;
    move_stopped = true;
}

void move_constant_velocity(double v, double w)
{
    move_started = true;
    move_complete = false;
    move_stopped = false;
    move_type = 2; // Constant velocity move
    move_target_distance = v;
    move_target_angle = w;
}

void set_velocity(double v, double w)
{
    target_velocity = v;
    target_angular_velocity = w;
}

bool is_move_complete(void)
{
    return move_complete;
}

MotionData get_motion_data(void)
{
    MotionData data;
    data.linear_acceleration = linear_acceleration;
    data.angular_velocity = angular_velocity;
    data.orientation = orientation;
    data.stationary = stationary;
    data.right_wheel_angle = right_angle;
    data.left_wheel_angle = left_angle;
    data.right_wheel_velocity = right_velocity;
    data.left_wheel_velocity = left_velocity;
    data.right_wheel_distance = right_distance;
    data.left_wheel_distance = left_distance;
    data.forward_distance = distance_forward;
    return data;
}


void motion_set_motors_enabled(bool enabled)
{
    motors_enabled = enabled;
    if (!enabled)
    {
        imu_filter.reset();
        left_angle = 0.0;
        right_angle = 0.0;
        left_velocity = 0.0;
        right_velocity = 0.0;
        target_velocity = 0.0;
        target_angular_velocity = 0.0;
        move_target_angle = 0;
        move_start_angle = orientation.yaw();
        move_target_distance = 0.0;
        move_start_distance = 0.0;
        left_distance = 0.0;
        right_distance = 0.0;
        distance_forward = 0.0;
        move_started = false;
        move_complete = true;
        move_stopped = true;
        move_type = 0;
    }
}

void motion_init(void)
{
    accelBias = active_config.accelBias;
    gyroBias = active_config.gyroBias;
}

void motion_reset(void)
{
    imu_filter.reset();
    left_angle = 0.0;
    right_angle = 0.0;
    left_velocity = 0.0;
    right_velocity = 0.0;
    target_velocity = 0.0;
    target_angular_velocity = 0.0;
    move_target_angle = 0;
    move_start_angle = orientation.yaw();
    move_target_distance = 0.0;
    move_start_distance = 0.0;
    left_distance = 0.0;
    right_distance = 0.0;
    distance_forward = 0.0;
    move_started = false;
    move_complete = true;
    move_stopped = true;
    move_type = 0;
}



void motion_update(void)
{
    static utils::time_t last_compute_time = 0;
    static vmath::vector3d a_filt_prev = {0.0};
    static double last_angleL = 0.0;
    static double last_angleR = 0.0;
    static double last_velocityL = 0.0;
    static double last_velocityR = 0.0;
    static double last_velocity = 0.0;
    bool timeout = utils::hasElapsed_us(last_compute_time, imu_sample_period_us * 2);
    bool data_ready = imu.isDataReady();
    utils::time_t t = utils::now();

    if (!(data_ready && timeout)) return;

    vmath::vector3d a;
    vmath::vector3d w;
    double angleL, angleR;

    if (!data_ready && timeout) utils::error_printf("Motion: IMU data read timeout\n");
    if (!imu.readData(a, w))
    {
        utils::error_printf("Motion: Error reading IMU data\n");
        return;
    }
    
    if (!left_encoder.readAngleRadians(angleL))
    {
        utils::error_printf("Failed to read angle from left_encoder\n");
        return;
    }
    if (!right_encoder.readAngleRadians(angleR))
    {
        utils::error_printf("Failed to read angle from right_encoder\n");
        return;
    }
    

    

    

    t = utils::now();
    double dt = (t - last_compute_time) / 1000000.0;
    last_compute_time = t;

    a -= accelBias;
    w -= gyroBias;
    auto q = imu_filter.compute(w, a, dt);
    double a_alpha = dt / (a_rc_coef + dt);
    auto a_filt = a * a_alpha + a_filt_prev * (1.0 - a_alpha);
    a_filt_prev = a_filt;

    double d_angleL = vmath::angleDifference(angleL, last_angleL);
    last_angleL = angleL;
    double d_angleR = -vmath::angleDifference(angleR, last_angleR);
    last_angleR = angleR; 

    angleL_total += d_angleL;
    angleR_total += d_angleR;
    velocityL = d_angleL / dt;
    velocityR = d_angleR / dt;
    accelerationL = (velocityL - last_velocityL) / dt;
    accelerationR = (velocityR - last_velocityR) / dt;
    last_velocityL = velocityL;
    last_velocityR = velocityR;
    distanceL += d_angleL * WHEEL_RADIUS;
    distanceR += d_angleR * WHEEL_RADIUS;

    velocity = (velocityL + velocityR) / 2.0;
    acceleration = (velocity - last_velocity) / dt;
    last_velocity = velocity;
    distance = (distanceL + distanceR) / 2.0;

    stationary = fabs(a_filt.length() - 1.0) < a_min && fabs(w.length()) < w_min && fabs(velocity) < v_min && fabs(acceleration) < a_min;

    //Update orientation even when stationary as the madgwic filter keeps maintains stable orientation estimation even without movement.
    orientation = q.eulerAngles(); // Orientation in world frame, referenced to initial orientation at power on, ZYX order (yaw-pitch-roll)

    if (stationary)
    {
        // If the robot is stationary, we can't get meaningful velocity information from the IMU, so we just set them to zero to avoid drift and noise.
        linear_acceleration = {0.0, 0.0, 0.0};
        angular_velocity = {0.0, 0.0, 0.0};
    }
    else
    {
        auto q_tilt = vmath::quaternion<>::fromRotationZ(-orientation.yaw()) * q; // Remove heading to get tilt-compensated acceleration and angular velocity in the robot-heading-aligned frame
        linear_acceleration = ((q_tilt * a_filt * q_tilt.conjugate()).toVector() - g) * G; // Convert from g to m/s^2 and remove gravity
        angular_velocity = (q_tilt * w * q_tilt.conjugate()).toVector(); // Rotate angular velocity to robot-heading-aligned frame, no need to remove gravity since it's a vector and doesn't affect rotation
    }

    double left_setpoint = (target_velocity - target_angular_velocity * WHEEL_BASE / 2.0) / WHEEL_RADIUS;
    double right_setpoint = (target_velocity + target_angular_velocity * WHEEL_BASE / 2.0) / WHEEL_RADIUS;

    double power_left = left_setpoint >= 0 ? Ks + Ke * left_setpoint : -Ks + Ke * left_setpoint;
    double power_right = right_setpoint >= 0 ? Ks + Ke * right_setpoint : -Ks + Ke * right_setpoint;

    
    if (move_started)
    {
        if (move_type == 2) // Constant velocity move
        {
            // No specific distance or angle target, just maintain the target velocities, until canceled
            target_velocity = move_target_distance;
            target_angular_velocity = move_target_angle;
        }
        if (move_type == 1) // Linear move
        {
            double distance_moved = distance_forward - move_start_distance;
            double distance_remaining = move_target_distance - distance_moved;
            //utils::debug_printf("distance forward: %f, move start distance: %f, Distance moved: %f, Target distance: %f\n", distance_forward, move_start_distance, distance_moved, move_target_distance);

            if (fabs(distance_remaining) <= 0 || move_stopped)
            {
                move_complete = true;
                move_started = false;
                move_stopped = false;
                target_velocity = 0.0;
                target_angular_velocity = 0.0;
                heading_pid.reset(); // Reset the heading PID controller when the move is complete
            }
            else
            {
                move_complete = false;

                if (fabs(distance_remaining) > fabs(move_target_distance) * LinearAccelerationMiddle) //Halfway not yet reached
                {
                    if (distance_remaining < 0)
                        target_velocity = std::max(target_velocity - MaxAcceleration * dt, -MaxVelocity); // Accelerate towards the maximum backward velocity (backward)
                    else
                        target_velocity = std::min(target_velocity + MaxAcceleration * dt, MaxVelocity); // Accelerate towards the maximum forward velocity (forward)
                }
                else // Past halfway point
                {
                    if (distance_remaining < 0)
                        target_velocity = std::min(target_velocity + MaxDeacceleration * dt, 0.0); // Decelerate towards stopping
                    else
                        target_velocity = std::max(target_velocity - MaxDeacceleration * dt, 0.0); // Decelerate towards stopping

                    if (target_velocity == 0.0)
                        move_stopped = true;
                }

                double heading_correction = heading_pid.compute_radians(move_start_angle, orientation.yaw(), dt); // Maintain current heading during linear move
                target_angular_velocity = heading_correction;
                //utils::debug_printf("target_velocity: %f, Target angular velocity: %f\n", target_velocity, target_angular_velocity);
                //target_angular_velocity = std::max(std::min(heading_correction, MaxAngularVelocity), -MaxAngularVelocity); // Apply heading correction during linear move
            }
        }
        else if (move_type == 0) // Rotational move
        {
            double angle_turned = vmath::angleDifference(orientation.yaw(), move_start_angle);
            double angle_remaining = vmath::angleDifference(move_target_angle, angle_turned);

            if (fabs(angle_remaining) <= 0 || move_stopped)
            {
                move_complete = true;
                move_started = false;
                move_stopped = false;
                target_velocity = 0.0;
                target_angular_velocity = 0.0;
                heading_pid.reset(); // Reset the heading PID controller when the rotational move is complete
            }
            else
            {
                move_complete = false;

                if (fabs(angle_remaining) > fabs(move_target_angle) * AngularAccelerationMiddle) // Halfway not yet reached
                {
                    if (angle_remaining < 0)
                        target_angular_velocity = std::max(target_angular_velocity - MaxAngularAcceleration * dt, -MaxAngularVelocity); // Accelerate towards the maximum negative angular velocity (turning left)
                    else
                        target_angular_velocity = std::min(target_angular_velocity + MaxAngularAcceleration * dt, MaxAngularVelocity); // Accelerate towards the maximum positive angular velocity (turning right)
                }
                else // Past halfway point
                {
                    if (angle_remaining < 0)
                        target_angular_velocity = std::min(target_angular_velocity + MaxAngulardeacceleration * dt, 0.0); // Decelerate towards stopping
                    else
                        target_angular_velocity = std::max(target_angular_velocity - MaxAngulardeacceleration * dt, 0.0); // Decelerate towards stopping

                    if (target_angular_velocity == 0.0)
                        move_stopped = true;
                }

                //utils::debug_printf("Angle turned: %f, Angle remaining: %f, Target angular velocity: %f\n", angle_turned, angle_remaining, target_angular_velocity);
            }
        }
    }
    else
    {
        target_velocity = 0.0;
        target_angular_velocity = 0.0;
    }


    if (motors_enabled)
    {
        if (target_velocity == 0.0 && target_angular_velocity == 0.0)
        {
            left_motor.setPower(0.0);
            right_motor.setPower(0.0);
        }
        else
        {
            

            left_motor.setPower(power_left);
            right_motor.setPower(power_right);
        }
    }
    else
    {
        left_motor.setPower(0.0);
        right_motor.setPower(0.0);
    }
}

