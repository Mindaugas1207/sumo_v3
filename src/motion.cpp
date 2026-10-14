
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
constexpr double Kv = 0.0955; // Velocity constant, in volts per rad/s
constexpr double Ka = 0.0016;//0.0034; // Acceleration constant, in volts per rad/s^2
struct MotorModel
{
    double Ks; // V
    double Kv; // V per rad/s
    double Ka; // V per rad/s^2
};

struct MotorConstants
{
    MotorModel forward;
    MotorModel reverse;
};

// Per-motor, per-direction feedforward constants; all start equal to the shared values above, tune individually.
constexpr MotorConstants LeftMotorModel = {{Ks, Kv, Ka}, {Ks + 0.1, Kv, Ka}};
constexpr MotorConstants RightMotorModel = {{Ks, Kv, Ka}, {Ks, Kv, Ka}};

constexpr double MaxWheelMps = NominalMotorRadPerSec * WHEEL_RADIUS; // Maximum wheel speed in meters per second
constexpr double MinWheelMps = MinMotorRadPerSec * WHEEL_RADIUS; // Minimum wheel speed in meters per second

// Voltage-limited model: V = Ks + Kv*w + Ka*alpha. Accel/decel limits are 0 if Ka is not set.
constexpr double VoltageLimitedMaxWheelRadPerSec = (SystemVoltage - Ks) / Kv; // Steady-state speed at full voltage, in rad/s
constexpr double VoltageLimitedMaxVelocity = VoltageLimitedMaxWheelRadPerSec * WHEEL_RADIUS; // in m/s
constexpr double VoltageLimitedMaxAcceleration = Ka > 0.0 ? (SystemVoltage - Ks) / Ka * WHEEL_RADIUS : 0.0; // From standstill, in m/s^2
constexpr double VoltageLimitedMinDeceleration = Ka > 0.0 ? (SystemVoltage + Ks) / Ka * WHEEL_RADIUS : 0.0; // Full reverse voltage near standstill (worst case), in m/s^2
constexpr double VoltageLimitedMaxDeceleration = Ka > 0.0 ? (SystemVoltage + Ks + Kv * VoltageLimitedMaxWheelRadPerSec) / Ka * WHEEL_RADIUS : 0.0; // Full reverse voltage at top speed, back-EMF adds to braking (best case), in m/s^2

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

constexpr double DecelPlanMargin = 0.8; // Plan braking at this fraction of the deceleration limit so there is headroom to correct
constexpr double PositionKp = 20.0; // 1/s, caps v_desired near the target so the sqrt profile's infinite slope doesn't cause chatter
constexpr double MaxVelocity = 0.9 * VoltageLimitedMaxVelocity; // Maximum velocity in m/s, 10% voltage headroom left for the PID

struct MotionLimits
{
    double acceleration; // Speeding up in this direction of travel, m/s^2
    double deceleration; // Braking while travelling in this direction, m/s^2
    double velocity; // Top speed in this direction, m/s
};

// Wheels are at the rear and the centre of mass is forward, so grip differs by direction of travel and by speeding up vs braking.
constexpr MotionLimits ForwardLimits = {10.0, 10.0, MaxVelocity};
constexpr MotionLimits ReverseLimits = {10.0, 10.0, MaxVelocity};
constexpr double MaxAngularVelocity = 40.0; // Maximum angular velocity in rad/s (tune)
constexpr double MaxAngularAcceleration = 600.0; // Maximum angular acceleration in rad/s^2 (tune)
constexpr double MaxAngularDeceleration = 200.0; // Maximum angular deceleration in rad/s^2 (tune)
constexpr double HeadingMaxW = 2.0; // rad/s, limits the heading correction so a wheel that can't follow doesn't wind it into a spin

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
bool hold_heading = false; // True for straight moves (no commanded turn)
double max_achieved_velocity = 0.0; // m/s
double distance_at_max_velocity = 0.0; // m
bool voltage_saturated = false; // Latched during a move if a commanded wheel voltage exceeded SystemVoltage
double max_tracking_error = 0.0; // Peak |target_v - velocity| during a move, m/s
double max_slip_accel = 0.0; // Peak (encoder accel - IMU accel) during a move, m/s^2; large means slipping

double heading_offset = 0.0; //rad

double heading = 0.0;

PID pid_wL(0.05, 1.0, 0.0, 0.5, 1.0);
PID pid_wR(0.05, 1.0, 0.0, 0.5, 1.0);
PID pid_heading(8.0, 1.0, 0.0, 0.5, HeadingMaxW); // Heading error in rad, output is target_w in rad/s

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
    hold_heading = (delta_angle == 0.0);
    pid_heading.reset();
    max_achieved_velocity = 0.0;
    distance_at_max_velocity = 0.0;
    voltage_saturated = false;
    max_tracking_error = 0.0;
    max_slip_accel = 0.0;
}

void motion_test(bool forward)
{
    motion_move(forward ? 0.5 : -0.5, 0.0);
}

void motion_turn_test(bool left)
{
    motion_move(0.0, left ? 90.0 / 180.0 * M_PI : -90.0 / 180.0 * M_PI);
}

// Ka step-response test: for each motor and direction in turn, apply a voltage step from rest to that motor alone,
// then fit tau = Ka/Kv from the speed rise (w = w_ss * (1 - exp(-t/tau))). Run with the wheels free.
constexpr double KaTestVoltage = 6.0; // Volts, keep well below SystemVoltage so the rise is slow enough to sample
constexpr double KaTestDuration = 0.6; // Seconds, several time constants
constexpr double KaTestSettleTime = 0.5; // Seconds with the motors off between runs so the wheel stops
constexpr size_t KaTestMaxSamples = 1024;
constexpr int KaTestRuns = 4; // left forward, left reverse, right forward, right reverse
static int ka_test_run = -1; // -1 when idle
static bool ka_test_settling = false;
static double ka_test_time = 0.0;
static size_t ka_test_count = 0;
static float ka_test_t[KaTestMaxSamples];
static float ka_test_w[KaTestMaxSamples];

KaTestResult ka_test_results[KaTestRuns];

void motion_ka_test(void)
{
    move = false;
    ka_test_run = 0;
    ka_test_settling = false;
    ka_test_time = 0.0;
    ka_test_count = 0;
}

// Returns true while the test owns the motors; VL and VR are the voltages to apply.
static bool ka_test_step(double dt, double wL, double wR, double& VL, double& VR)
{
    if (ka_test_run < 0) return false;

    VL = VR = 0.0;
    if (ka_test_settling)
    {
        ka_test_time += dt;
        if (ka_test_time >= KaTestSettleTime)
        {
            ka_test_settling = false;
            ka_test_time = 0.0;
            ka_test_count = 0;
        }
        return true;
    }

    bool right = ka_test_run >= 2;
    bool reverse = (ka_test_run & 1) != 0;
    double sign = reverse ? -1.0 : 1.0;
    double w_avg = sign * (right ? wR : wL); // Positive in the driven direction

    if (ka_test_time < KaTestDuration && ka_test_count < KaTestMaxSamples)
    {
        ka_test_t[ka_test_count] = ka_test_time;
        ka_test_w[ka_test_count] = w_avg;
        ka_test_count++;
        ka_test_time += dt;
        (right ? VR : VL) = sign * KaTestVoltage;
        return true;
    }

    KaTestResult& result = ka_test_results[ka_test_run];
    result = {0.0, -1.0, 0.0, 0.0};
    if (ka_test_count >= 10)
    {

        // Steady-state speed from the last 10% of samples
        size_t tail = ka_test_count / 10;
        double w_ss = 0.0;
        for (size_t i = ka_test_count - tail; i < ka_test_count; i++) w_ss += ka_test_w[i];
        w_ss /= tail;

        // Time to reach 63.2% of steady state, interpolated
        double target = 0.632 * w_ss;
        double tau = -1.0;
        for (size_t i = 1; i < ka_test_count; i++)
        {
            if (ka_test_w[i] >= target && ka_test_w[i] > ka_test_w[i - 1])
            {
                double f = (target - ka_test_w[i - 1]) / (ka_test_w[i] - ka_test_w[i - 1]);
                tau = ka_test_t[i - 1] + f * (ka_test_t[i] - ka_test_t[i - 1]);
                break;
            }
        }

        const MotorConstants& motor = right ? RightMotorModel : LeftMotorModel;
        double Ks_model = (reverse ? motor.reverse : motor.forward).Ks;
        double Kv_meas = w_ss > 0.0 ? (KaTestVoltage - Ks_model) / w_ss : 0.0;
        result = {w_ss, tau, Kv_meas, Kv_meas * tau};
        utils::debug_printf("Ka test %c%c: w_ss=%.2f rad/s, tau=%.4f s, Kv=%.4f, Ka=%.5f\n",
            right ? 'R' : 'L', reverse ? 'R' : 'F', w_ss, tau, Kv_meas, Kv_meas * tau);
    }

    ka_test_run++;
    if (ka_test_run >= KaTestRuns)
    {
        ka_test_run = -1;
        return false;
    }
    ka_test_settling = true;
    ka_test_time = 0.0;
    return true;
}

void motion_zero_position(void)
{
    angleL_total = 0.0;
    angleR_total = 0.0;
    distanceL = 0.0;
    distanceR = 0.0;
    heading_offset = orientation.yaw();
}

// Feedforward voltage for one motor; the direction (and so the constants) follows the sign of target_w, or target_a at standstill.
static double motor_feedforward(const MotorConstants& m, double target_w, double target_a)
{
    const MotorModel& d = (target_w != 0.0 ? target_w : target_a) >= 0.0 ? m.forward : m.reverse;
    double ks = target_w >= MinMotorRadPerSec ? d.Ks : (target_w <= -MinMotorRadPerSec ? -d.Ks : 0.0);
    return ks + d.Kv * target_w + d.Ka * target_a;
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
    static double acc_filt = 0.0;
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
    acc_filt = acceleration * a_alpha + acc_filt * (1.0 - a_alpha);
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
    //double w_heading = angular_velocity.yaw();
    heading = vmath::angleDifference(orientation.yaw(), heading_offset);
    //double remaining_angle = vmath::angleDifference(target_heading, heading); // Remaining angle to the target (method handles wrapping)
    //double stopping_angle = (target_w * target_w) / (MaxAngularDeceleration); // Angle required to stop the robot from current angular velocity
    //stopping_angle = target_w >= 0 ? stopping_angle : -stopping_angle;

    

    double remaining_distance = target_distance - distance;

    //double motors_w_avg = (wL + wR) / 2.0;
    //double motors_bemf_avg = std::min(Kv * std::abs(motors_w_avg), SystemVoltage - Ks); // Ks is already part of MinDeceleration
    //double possible_deceleration = (motors_bemf_avg / (SystemVoltage - Ks)) * (VoltageLimitedMaxDeceleration - VoltageLimitedMinDeceleration) + VoltageLimitedMinDeceleration;

    if (move)
    {
            // Limits for the direction we are heading in, and for the direction we are currently travelling (matters when braking or reversing).
            const MotionLimits& toward = remaining_distance >= 0 ? ForwardLimits : ReverseLimits;
            const MotionLimits& travelling = (target_v != 0.0 ? target_v : remaining_distance) >= 0 ? ForwardLimits : ReverseLimits;

            // Fastest speed from which we can still stop within the remaining distance; sign points toward the target.
            double v_limit = std::min({toward.velocity, std::sqrt(2.0 * DecelPlanMargin * toward.deceleration * std::abs(remaining_distance)), PositionKp * std::abs(remaining_distance)});
            double v_desired = remaining_distance >= 0 ? v_limit : -v_limit;

            // Slew toward v_desired: brake rate when reducing speed or reversing, accel rate when speeding up.
            bool speeding_up = std::abs(v_desired) > std::abs(target_v) && (v_desired * target_v >= 0);
            double max_step = (speeding_up ? toward.acceleration : travelling.deceleration) * dt;
            double step = std::clamp(v_desired - target_v, -max_step, max_step);
            target_v += step;
            target_a = step / dt;

            target_w = hold_heading ? pid_heading.compute_radians(target_heading, orientation.yaw(), dt) : 0.0;

            if (std::abs(remaining_distance) < 0.005 && std::abs(velocity) < 0.05)
            {
                target_a = 0;
                target_v = 0;
                target_w = 0;
                pid_heading.reset();
                pid_wL.reset(); // Stop the integrator winding up against static friction and hunting
                pid_wR.reset();
            }
    }
    else
    {
        target_a = 0;
        target_v = 0;
        target_w = 0;
        target_alpha = 0;
    }
    
    double clamped_v = std::clamp(target_v, -MaxVelocity, MaxVelocity); // Clamp target velocity to the maximum allowed velocity
    if (clamped_v != target_v) target_a = 0; // No acceleration feedforward while saturated
    target_v = clamped_v;
    target_w = std::clamp(target_w, -MaxAngularVelocity, MaxAngularVelocity); // Clamp target angular velocity to the maximum allowed angular velocity
    double target_wL = (target_v - target_w * WHEEL_BASE / 2.0) / WHEEL_RADIUS; //desired angular velocity for left wheel
    double target_wR = (target_v + target_w * WHEEL_BASE / 2.0) / WHEEL_RADIUS; //desired angular velocity for right wheel
    double target_aL = (target_a - target_alpha * WHEEL_BASE / 2.0) / WHEEL_RADIUS; //desired angular acceleration for left wheel
    double target_aR = (target_a + target_alpha * WHEEL_BASE / 2.0) / WHEEL_RADIUS; //desired angular acceleration for right wheel

    //Compute the required motor voltages based on the target angular velocities and accelerations
    double VoltageL = motor_feedforward(LeftMotorModel, target_wL, target_aL) + pid_wL.compute(target_wL, wL, dt);
    double VoltageR = motor_feedforward(RightMotorModel, target_wR, target_aR) + pid_wR.compute(target_wR, wR, dt);

    double ka_VL, ka_VR;
    if (ka_test_step(dt, wL, wR, ka_VL, ka_VR))
    {
        VoltageL = ka_VL;
        VoltageR = ka_VR;
    }

    if (move)
    {
        if (std::max(std::abs(VoltageL), std::abs(VoltageR)) > SystemVoltage) voltage_saturated = true;
        max_tracking_error = std::max(max_tracking_error, std::abs(target_v - velocity));
        max_slip_accel = std::max(max_slip_accel, std::abs(acc_filt) - linear_acceleration.length());
    }

    //utils::debug_printf("VoltageL: %.3f, VoltageR: %.3f, target_wL: %.3f, target_wR: %.3f, target_aL: %.3f, target_aR: %.3f\n, target_v: %.3f, target_w: %.3f, target_a: %.3f, target_alpha: %.3f\n", VoltageL, VoltageR, target_wL, target_wR, target_aL, target_aR, target_v, target_w, target_a, target_alpha);

    double dutyCycleL = VoltageL / SystemVoltage;
    double dutyCycleR = VoltageR / SystemVoltage;

    left_motor.setPower(dutyCycleL); //The duty cycle is clamped in the setPower method
    right_motor.setPower(dutyCycleR);

}

