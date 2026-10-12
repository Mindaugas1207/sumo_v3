
#include "config.h"
#include "motion.h"
#include "hardware.h"
#include "vmath.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "pid.h"
/*
enum class ControlCommands
{
    DISTANCE, // Command to move a specific distance
    ANGLE, // Command to rotate by a specific angle
    LINEAR_VELOCITY, // Command to move at a specific linear velocity
    ANGULAR_VELOCITY, // Command to move at a specific angular velocity
    BREAK // Command to apply braking function to immediately stop the robot
};
*/
constexpr double NominalMotorRPM = 600.0;
constexpr double NominalMotorVoltage = 6.0;
constexpr double SystemVoltage = 12.0;

constexpr double NominalMotorRadPerSec = NominalMotorRPM * SystemVoltage / NominalMotorVoltage * 2.0 * M_PI / 60.0;

constexpr double MinMotorRadPerSec = 10;

constexpr double WHEEL_RADIUS = 0.030 / 2.0; // Wheel radius in meters
constexpr double WHEEL_BASE = 0.06; // Distance between the wheels in meters

constexpr double Ks = 1.0; // Static friction voltage constant, in volts
constexpr double Kv = 0.0937; // Velocity constant, in volts per rad/s
constexpr double Ka = 0.0; // Acceleration constant, in volts per rad/s^2


constexpr double MaxWheelMps = NominalMotorRadPerSec * WHEEL_RADIUS; // Maximum wheel speed in meters per second
constexpr double MinWheelMps = MinMotorRadPerSec * WHEEL_RADIUS; // Minimum wheel speed in meters per second

//Measured values
//2.0V -> 10-12 rad/s
//3.0V -> 23-25 rad/s
//4.0V -> 31-34 rad/s
//5.0V -> 38-45 rad/s
//6.0V -> 49-56 rad/s
//7.0V -> 61-70 rad/s
//8.0V -> 71-78 rad/s
//9.0V -> 80-90 rad/s
//10.0V -> 90-100 rad/s
//11.0V -> 99-110 rad/s
//12.0V -> 119-125 rad/s

constexpr double MaxAcceleration = 20.0; // Maximum acceleration in m/s^2
constexpr double MaxDeceleration = 10.0; // Maximum deceleration in m/s^2
constexpr double MaxVelocity = 1.0; // Maximum velocity in m/s
constexpr double MaxAngularVelocity = 40.0; // Maximum angular velocity in rad/s (tune)
constexpr double MaxAngularAcceleration = 600.0; // Maximum angular acceleration in rad/s^2 (tune)
constexpr double MaxAngularDeceleration = 200.0; // Maximum angular deceleration in rad/s^2 (tune)

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
vmath::vector3d<double> angular_acceleration = {0.0}; // In the robot-heading-aligned frame with Z fixed up, in rad/s^2, referenced to initial orientation at power on
vmath::vector3d<double> orientation = {0.0}; // Euler angles in radians, in ZYX order (yaw-pitch-roll), referenced to initial orientation at power on
bool stationary = true; // Whether the robot is stationary based on IMU readings, used to determine whether to trust orientation and velocity information from the IMU or not
double angleL_total = 0.0;//rad
double angleR_total = 0.0;//rad
double wL = 0.0;//rad/s
double wR = 0.0;//rad/s
double aL = 0.0;//m/s^2
double aR = 0.0;//m/s^2
double distanceL = 0.0;//m
double distanceR = 0.0;//m

double acceleration = 0.0;//m/s^2
double velocity = 0.0;//m/s
double distance = 0.0;//m

double target_v = 0; //desired linear velocity (of the robot) in m/s
double target_w = 0; //desired angular velocity or rotational velocity (of the robot) in rad/s
double target_a = 0; //desired linear acceleration (of the robot) in m/s^2
double target_alpha = 0; //desired angular acceleration (of the robot) in rad/s^2
double target_distance = 0.0;//m
double target_heading = 0.0;//rad
bool move = false;
double max_achieved_velocity = 0.0; // m/s
double distance_at_max_velocity = 0.0; // m

double heading_offset = 0.0; //rad

double heading = 0.0;

PID pid_wL(0.08, 0.06, 0.0, 0.5, 0.5);
PID pid_wR(0.08, 0.06, 0.0, 0.5, 0.5);

void motion_init(void)
{
    accelBias = active_config.accelBias;
    gyroBias = active_config.gyroBias;
}

void motion_move(double delta_distance, double delta_angle)
{
    target_distance = distance + delta_distance;
    target_heading = vmath::constrainAngle(orientation.yaw() + delta_angle);
    move = true;
    max_achieved_velocity = 0.0;
    distance_at_max_velocity = 0.0;
}

void motion_test(bool forward)
{
    motion_move(forward ? 0.5 : -0.5, 0.0);
}

void motion_turn_test(bool left)
{
    motion_move(0.0, left ? 90.0 / 180.0 * M_PI : -90.0 / 180.0 * M_PI);
}

void motion_zero_position(void)
{
    angleL_total = 0.0;
    angleR_total = 0.0;
    distanceL = 0.0;
    distanceR = 0.0;
    heading_offset = orientation.yaw();
}

void motion_update(void)
{
    static utils::time_t last_compute_time = 0;
    static vmath::vector3d a_filt_prev = {0.0};
    static double last_angleL = 0.0;
    static double last_angleR = 0.0;
    static double last_wL = 0.0;
    static double last_wR = 0.0;
    static double last_velocity = 0.0;
    utils::time_t t = utils::now();
    bool data_ready = imu.isDataReady() || gpio_get(IMU_INT_PIN); // INT1 is level-latched, so a high pin means unread data even if the edge was missed
    bool timeout = utils::hasElapsed_us(last_compute_time, imu_sample_period_us * 2, t);

    if (!(data_ready || timeout)) return;

    vmath::vector3d a_imu;
    vmath::vector3d w_imu;
    double angleL, angleR;

    if (!data_ready && timeout) utils::error_printf("Motion: IMU data read timeout\n");
    if (!imu.readData(a_imu, w_imu))
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

    double dt = (t - last_compute_time) / 1000000.0;
    last_compute_time = t;

    a_imu -= accelBias;
    w_imu -= gyroBias;
    auto q = imu_filter.compute(w_imu, a_imu, dt);
    double a_alpha = dt / (a_rc_coef + dt);
    auto a_filt = a_imu * a_alpha + a_filt_prev * (1.0 - a_alpha);
    a_filt_prev = a_filt;

    double d_angleL = vmath::angleDifference(angleL, last_angleL);
    last_angleL = angleL;
    double d_angleR = -vmath::angleDifference(angleR, last_angleR);
    last_angleR = angleR; 

    angleL_total += d_angleL;
    angleR_total += d_angleR;
    wL = d_angleL / dt;
    wR = d_angleR / dt;
    aL = (wL - last_wL) / dt;
    aR = (wR - last_wR) / dt;
    last_wL = wL;
    last_wR = wR;
    distanceL += d_angleL * WHEEL_RADIUS;
    distanceR += d_angleR * WHEEL_RADIUS;

    velocity = (wL + wR) * WHEEL_RADIUS / 2.0;
    if (velocity > max_achieved_velocity)
    {
        max_achieved_velocity = velocity;
        distance_at_max_velocity = distance;
    }
    acceleration = (velocity - last_velocity) / dt;
    last_velocity = velocity;
    distance = (distanceL + distanceR) / 2.0;

    stationary = std::abs(a_filt.length() - 1.0) < a_min && std::abs(w_imu.length()) < w_min && std::abs(velocity) < v_min && std::abs(acceleration) < a_min;

    //Update orientation even when stationary as the madgwic filter keeps maintains stable orientation estimation even without movement.
    orientation = q.eulerAngles(); // Orientation in world frame, referenced to initial orientation at power on, ZYX order (yaw-pitch-roll)

    if (stationary)
    {
        // If the robot is stationary, we can't get meaningful velocity information from the IMU, so we just set them to zero to avoid drift and noise.
        linear_acceleration = {0.0, 0.0, 0.0};
        angular_velocity = {0.0, 0.0, 0.0};
        angular_acceleration = {0.0, 0.0, 0.0};
    }
    else
    {
        auto q_tilt = vmath::quaternion<>::fromRotationZ(-orientation.yaw()) * q; // Remove heading to get tilt-compensated acceleration and angular velocity in the robot-heading-aligned frame
        linear_acceleration = ((q_tilt * a_filt * q_tilt.conjugate()).toVector() - g) * G; // Convert from g to m/s^2 and remove gravity
        auto new_angular_velocity = (q_tilt * w_imu * q_tilt.conjugate()).toVector(); // Rotate angular velocity to robot-heading-aligned frame, no need to remove gravity since it's a vector and doesn't affect rotation
        angular_acceleration = (new_angular_velocity - angular_velocity) / dt; // Compute angular acceleration in the robot-heading-aligned frame
        angular_velocity = new_angular_velocity; // Update angular velocity
    }
    double w_heading = angular_velocity.yaw();
    heading = vmath::angleDifference(orientation.yaw(), heading_offset);
    double remaining_angle = vmath::angleDifference(target_heading, heading); // Remaining angle to the target (method handles wrapping)
    double stopping_angle = (target_w * target_w) / (MaxAngularDeceleration); // Angle required to stop the robot from current angular velocity
    stopping_angle = target_w >= 0 ? stopping_angle : -stopping_angle;

    

    double remaining_distance = target_distance - distance;
    double stopping_distance = (target_v * target_v) / (2 * MaxDeceleration); // Distance required to stop the robot from current velocity
    stopping_distance = target_v >= 0 ? stopping_distance : -stopping_distance;

    if (move)
    {
       // if (remaining_distance < 0.001 && remaining_angle < 0.01 && stationary)
       // {
         //   target_a = 0;
        //    target_v = 0;
         //   target_w = 0;
         //   target_alpha = 0;
          //  move = false;
        //}
        //else
        ////{
/*
            if (remaining_distance > 0.005 && remaining_distance > stopping_distance)
            {
                target_a = std::clamp(MaxAcceleration * MaxAcceleration * remaining_distance, -MaxAcceleration, MaxAcceleration);
                target_v += target_a * dt;
            }
            else if (remaining_distance < -0.005 && remaining_distance < stopping_distance)
            {
                target_a = std::clamp(MaxAcceleration * MaxAcceleration * remaining_distance, -MaxAcceleration, MaxAcceleration);
                target_v += target_a * dt;
            }
            else if (std::abs(velocity) > 0.1) //Decelerate if the velocity is non zero
            {
                target_a = velocity > 0 ? -MaxDeceleration : MaxDeceleration;
                target_v += target_a * dt;
            }
            else*/
            {
                target_a = 0;
                target_v = 0;
            }
 
            //utils::debug_printf("remaining_angle: %f, stopping_angle: %f, target_w: %f, w_heading: %f\n", remaining_angle, stopping_angle, target_w, w_heading);

            if (remaining_angle > 0.1 && remaining_angle > stopping_angle)
            {
                target_alpha = MaxAngularAcceleration;
                target_w += target_alpha * dt;
            }
            else if (remaining_angle < -0.1 && remaining_angle < stopping_angle)
            {
                target_alpha = -MaxAngularAcceleration;
                target_w += target_alpha * dt;
            }
            else if (remaining_angle > 0.1 && remaining_angle <= stopping_angle)
            {
                target_alpha = -MaxAngularDeceleration;
                target_w += target_alpha * dt;
            }
            else if (remaining_angle < -0.1 && remaining_angle >= stopping_angle)
            {
                target_alpha = MaxAngularDeceleration;
                target_w += target_alpha * dt;
            }
            else
            {
                target_alpha = 0;
                target_w = 0;
            }
        //}
    }
    else
    {
        target_a = 0;
        target_v = 0;
        target_w = 0;
        target_alpha = 0;
    }
    
    target_v = std::clamp(target_v, -MaxVelocity, MaxVelocity); // Clamp target velocity to the maximum allowed velocity
    target_w = std::clamp(target_w, -MaxAngularVelocity, MaxAngularVelocity); // Clamp target angular velocity to the maximum allowed angular velocity

    double target_wL = (target_v - target_w * WHEEL_BASE / 2.0) / WHEEL_RADIUS; //desired angular velocity for left wheel
    double target_wR = (target_v + target_w * WHEEL_BASE / 2.0) / WHEEL_RADIUS; //desired angular velocity for right wheel
    double target_aL = (target_a - target_alpha * WHEEL_BASE / 2.0) / WHEEL_RADIUS; //desired angular acceleration for left wheel
    double target_aR = (target_a + target_alpha * WHEEL_BASE / 2.0) / WHEEL_RADIUS; //desired angular acceleration for right wheel

    //Compute the required motor voltages based on the target angular velocities and accelerations
    double VoltageL = (target_wL >= MinMotorRadPerSec ? Ks : (target_wL <= -MinMotorRadPerSec ? -Ks : 0)) + Kv * target_wL + Ka * target_aL + pid_wL.compute(target_wL, wL, dt);
    double VoltageR = (target_wR >= MinMotorRadPerSec ? Ks : (target_wR <= -MinMotorRadPerSec ? -Ks : 0)) + Kv * target_wR + Ka * target_aR + pid_wR.compute(target_wR, wR, dt);

    double dutyCycleL = VoltageL / SystemVoltage;
    double dutyCycleR = VoltageR / SystemVoltage;

    left_motor.setPower(dutyCycleL); //The duty cycle is clamped in the setPower method
    right_motor.setPower(dutyCycleR);

    //utils::debug_printf("Test Angular Velocity: %.3f, Duty Cycle L: %.3f, Duty Cycle R: %.3f, wL: %.3f, wR: %.3f\n", test_w, dutyCycleL, dutyCycleR, wL, wR);

    //utils::debug_printf("Started: %d, Distance: %.3f, Target Distance: %.3f, remaining distance: %.3f, stopping distance: %.3f, Target Velocity: %.3f, Target Acceleration: %.3f\nTarget wL: %.3f, Target wR: %.3f, Target aL: %.3f, Target aR: %.3f, VoltageL: %.3f, VoltageR: %.3f, dutyCycleL: %.3f, dutyCycleR: %.3f\n"
    //    , move_distance, distance, target_distance, remaining_distance, stopping_distance, target_v, target_a, target_wL, target_wR, target_aL, target_aR, VoltageL, VoltageR, dutyCycleL, dutyCycleR);
}

