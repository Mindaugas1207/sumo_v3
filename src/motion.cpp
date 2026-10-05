
#include "config.h"
#include "motion.h"
#include "hardware.h"
#include "vmath.h"

constexpr double G = 9.80665; // Gravity constant in m/s^2, used to convert accelerometer readings from g to m/s^2. The standard value is 9.80665 m/s^2.
constexpr double a_cutoff = 10.0; //Hz
constexpr double a_rc_coef = 1.0 / (2.0 * M_PI * a_cutoff);
constexpr double a_min = 0.02; // Minimum acceleration magnitude to consider the robot as non-stationary, in m/s^2.
constexpr double w_min = 0.02; // Minimum angular velocity magnitude to consider the robot as non-stationary, in rad/s.
constexpr vmath::vector3d g = {0.0, 0.0, 1.0}; // Gravity vector in the IMU frame, assuming the IMU is mounted flat with Z axis up

constexpr double madgwick_beta = LSM6DSR::LSM6DSR_GYRO_NOISE_DPS_PER_ROOTHZ * (M_PI / 180.0) * vmath::sqrt<double>(3.0/4.0) * vmath::sqrt<double>(imu_sample_rate_hz);
vmath::MadgwickFilter<double> imu_filter(madgwick_beta);

vmath::vector3d<double> linear_acceleration = {0.0}; // In the robot-heading-aligned frame with Z fixed up, in m/s^2, referenced to initial orientation at power on
vmath::vector3d<double> angular_velocity = {0.0}; // In the robot-heading-aligned frame with Z fixed up, in rad/s, referenced to initial orientation at power on
vmath::vector3d<double> orientation = {0.0}; // Euler angles in radians, in ZYX order (yaw-pitch-roll), referenced to initial orientation at power on
bool stationary = true; // Whether the robot is stationary based on IMU readings, used to determine whether to trust orientation and velocity information from the IMU or not

utils::time_t average_compute_time = 0;

double left_angle = 0.0;//rad
double right_angle = 0.0;//rad
double left_velocity = 0.0;//rad/s
double right_velocity = 0.0;//rad/s

double left_setpoint = 0.0;//rad
double right_setpoint = 0.0;//rad

MotionData get_motion_data(void)
{
    MotionData data;
    data.linear_acceleration = linear_acceleration;
    data.angular_velocity = angular_velocity;
    data.orientation = orientation;
    data.stationary = stationary;
    data.right_wheel_angle = right_angle;
    data.left_wheel_angle = left_angle;
    return data;
}

void motion_init(void)
{
}

void motion_reset(void)
{
    imu_filter.reset();
    left_angle = 0.0;
    right_angle = 0.0;
    left_velocity = 0.0;
    right_velocity = 0.0;
    left_setpoint = 0.0;
    right_setpoint = 0.0;
}

void motion_update(void)
{
    static utils::time_t last_compute_time = 0;
    static utils::time_t last_read_time = 0;
    static vmath::vector3d a_comp = {0.0}, w_comp = {0.0}, a_filt_prev = {0.0};
    static bool initialized = false;
    static double last_left_angle = 0.0;
    static double last_right_angle = 0.0;
    bool timeout = utils::hasElapsed_us(last_read_time, imu_sample_period_us * 2);
    bool data_ready = imu.isDataReady();
    utils::time_t t = utils::now();
    // If data is available or if it's been too long since the last read (indicating a possible missed data ready signal), attempt to read data
    if (data_ready || timeout)
    {
        last_read_time = t;
        vmath::vector3d a_raw;
        vmath::vector3d w_raw;
        double angle_left_raw, angle_right_raw;
        
        if (imu.readData(a_raw, w_raw))
        {
            a_comp = a_raw - active_config.accelBias;
            w_comp = w_raw - active_config.gyroBias;
            initialized = true;
        }
        else
        {
            utils::error_printf("Motion: Error reading IMU data\n");
        }
        
        if (left_encoder.readAngleRadians(angle_left_raw))
        {
            left_angle += vmath::angleDifference(angle_left_raw, last_left_angle);
            last_left_angle = angle_left_raw;
        }
        else
        {
            utils::error_printf("Failed to read angle from left_encoder\n");
        }
        if (right_encoder.readAngleRadians(angle_right_raw))
        {
            right_angle += vmath::angleDifference(angle_right_raw, last_right_angle);
            last_right_angle = angle_right_raw;
        }
        else
        {
            utils::error_printf("Failed to read angle from right_encoder\n");
        }

        if (!data_ready && timeout)
        {
          utils::error_printf("Motion: IMU data read timeout\n");
        }

        if (!initialized)
        {
            return;
        }

        t = utils::now();
        double dt = (t - last_compute_time) / 1000000.0;
        last_compute_time = t;

        auto q = imu_filter.compute(w_comp, a_comp, dt);
        double a_alpha = dt / (a_rc_coef + dt);
        auto a_filt = a_comp * a_alpha + a_filt_prev * (1.0 - a_alpha);
        a_filt_prev = a_filt;

        stationary = fabs(a_filt.length() - 1.0) < a_min && fabs(w_comp.length()) < w_min;

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
            angular_velocity = (q_tilt * w_comp * q_tilt.conjugate()).toVector(); // Rotate angular velocity to robot-heading-aligned frame, no need to remove gravity since it's a vector and doesn't affect rotation
        }

        left_motor.setPower(left_setpoint);
        right_motor.setPower(right_setpoint);

    #if PRINT_IMU_DATA
        static utils::time_t last_print_time = 0;
        if (utils::hasElapsed_us(last_print_time, 1000000 / PRINT_RATE))
        {
            last_print_time = utils::now();
            utils::debug_printf("%s, a: (%.2f, %.2f, %.2f) m/s^2, w: (%.2f, %.2f, %.2f) deg/s, o: (%.2f, %.2f, %.2f) deg\n",
                stationary ? "Stationary" : "Moving",
                linear_acceleration.X, linear_acceleration.Y, linear_acceleration.Z,
                angular_velocity.X * 180.0 / M_PI, angular_velocity.Y * 180.0 / M_PI, angular_velocity.Z * 180.0 / M_PI,
                orientation.X * 180.0 / M_PI, orientation.Y * 180.0 / M_PI, orientation.Z * 180.0 / M_PI);
        }
    #endif
    }
}

