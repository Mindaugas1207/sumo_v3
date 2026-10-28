
#include "config.h"
#include "motion.h"
#include "hardware.h"
#include "vmath.h"
#include "pid.h"
constexpr double NominalMotorRPM = 600.0;
constexpr double NominalMotorVoltage = 6.0;
constexpr double SystemVoltage = 12.0;

constexpr double NominalMotorRadPerSec = NominalMotorRPM * SystemVoltage / NominalMotorVoltage * 2.0 * M_PI / 60.0;

constexpr double MinMotorRadPerSec = 10;

constexpr double WHEEL_RADIUS = 0.0147; // Effective rolling radius of the soft tire under load, in meters (measured: 2% under the 15 mm nominal)
constexpr double WHEEL_BASE = 0.06; // Distance between the wheels in meters

constexpr double Ks = 1.0; // Static friction voltage constant, in volts
constexpr double Kv = 0.1; // Velocity constant, in volts per rad/s
constexpr double Ka = 0.003; // Acceleration constant, in volts per rad/s^2
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
constexpr MotorConstants LeftMotorModel = {{Ks, Kv, Ka}, {Ks, Kv, Ka}};
constexpr MotorConstants RightMotorModel = {{Ks, Kv, Ka}, {Ks, Kv, Ka}};

constexpr double MaxWheelMps = NominalMotorRadPerSec * WHEEL_RADIUS; // Maximum wheel speed in meters per second
constexpr double MinWheelMps = MinMotorRadPerSec * WHEEL_RADIUS; // Minimum wheel speed in meters per second

constexpr double MinRobotTurningRadS = MinWheelMps * 2.0 / WHEEL_BASE; // Minimum robot turning rate in rad/s

// Voltage-limited model: V = Ks + Kv*w + Ka*alpha. Accel/decel limits are 0 if Ka is not set.
constexpr double VoltageLimitedMaxWheelRadPerSec = (SystemVoltage - Ks) / Kv; // Steady-state speed at full voltage, in rad/s
constexpr double VoltageLimitedMaxVelocity = VoltageLimitedMaxWheelRadPerSec * WHEEL_RADIUS; // in m/s
constexpr double VoltageLimitedMaxAcceleration = Ka > 0.0 ? (SystemVoltage - Ks) / Ka * WHEEL_RADIUS : 0.0; // From standstill, in m/s^2
constexpr double VoltageLimitedMinDeceleration = Ka > 0.0 ? (SystemVoltage + Ks) / Ka * WHEEL_RADIUS : 0.0; // Full reverse voltage near standstill (worst case), in m/s^2
constexpr double VoltageLimitedMaxDeceleration = Ka > 0.0 ? (SystemVoltage + Ks + Kv * VoltageLimitedMaxWheelRadPerSec) / Ka * WHEEL_RADIUS : 0.0; // Full reverse voltage at top speed, back-EMF adds to braking (best case), in m/s^2
constexpr double VoltageLimitedMaxAngularVelocity = VoltageLimitedMaxVelocity * 2.0 / WHEEL_BASE; // in rad/s
constexpr double VoltageLimitedMaxAngularAcceleration = VoltageLimitedMaxAcceleration * 2.0 / WHEEL_BASE; // in rad/s^2
constexpr double VoltageLimitedMinAngularDeceleration = VoltageLimitedMinDeceleration * 2.0 / WHEEL_BASE; // in rad/s^2
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
constexpr double PositionKp = 50.0; // 1/s, caps v_desired near the target so the sqrt profile's infinite slope doesn't cause chatter
constexpr double HeadingKp = 50.0; // 1/s, proportional gain for heading control
constexpr double MaxVelocity = 1.0 * VoltageLimitedMaxVelocity; // Maximum velocity in m/s, 10% voltage headroom left for the PID
constexpr double MaxAngularVelocity = 1.0 * VoltageLimitedMaxAngularVelocity; // Maximum angular velocity in rad/s (tune)

struct MotionLimits
{
    double acceleration; // Speeding up in this direction of travel, m/s^2
    double deceleration; // Braking while travelling in this direction, m/s^2
    double velocity; // Top speed in this direction, m/s
};

// Wheels are at the rear and the centre of mass is forward, so grip differs by direction of travel and by speeding up vs braking.
constexpr MotionLimits ForwardLimits = {14.0, 9.0, MaxVelocity};
constexpr MotionLimits ReverseLimits = {5.0, 5.0, MaxVelocity};
constexpr MotionLimits RotationalLimits = {300, 120, MaxAngularVelocity};

constexpr double MinDistance = 0.005; // Minimum distance to consider the robot as having reached its target, in meters.
constexpr double MinHeadingError = 1.0 * M_PI / 180.0; // Minimum heading error to consider the robot as having reached its target, in rad.
constexpr double MinVelocity = MinWheelMps / 5; // Minimum velocity to consider the robot as having reached its target, in m/s.
constexpr double MinAngularVelocity = MinRobotTurningRadS / 4; // Minimum angular velocity to consider the robot as having reached its target, in rad/s.
constexpr double StopDeceleration = 20.0; // Deceleration used during an emergency stop, in m/s^2.
constexpr double LineSafetyMargin = 0.05; //meters
constexpr double LineReturnDistance = 0.1; // Distance to move back after detecting a line, in meters.

constexpr double RingDiameter = 0.77;
constexpr double RingRadius = RingDiameter / 2.0;
constexpr double RingWhiteWidth = 0.025; // Width of the white line on the ring, in meters.
constexpr double RingInnerRadius = RingRadius - RingWhiteWidth; // Inner radius of the ring, in meters.
constexpr double RingDriftK = 0.03;
constexpr double RingRadialRecovery = 0.5; // Fraction of uncertainty reduced after a radial update
// Line sensor positions relative to the robot's rotation center.
constexpr double LineSensorX = 0.043;
constexpr double LineSensorY = 0.082;



struct RingTracker
{
    //Robot forward and reverse driving direction is the Y axis.
    vmath::vector2d<double> circle_position; //The robot is the origin, the circle position is relative to the robot
    double lastDistance = 0;
    double sigma = 0; // Uncertainty of the circle position estimate, used to adjust the effective ring radius in distance calculations

    vmath::vector2d<double> line_sensor_left_position;
    vmath::vector2d<double> line_sensor_right_position;
    bool line_sensor_left_hit = false;
    bool line_sensor_right_hit = false;

    double distanceToLine(double heading) const
    {
        auto u = vmath::vector2d<double>::fromAngle(heading + M_PI_2);
        double cu = circle_position.dot(u);
        double c2 = circle_position.dot(circle_position);
        double R = RingInnerRadius - sigma;
        double D = R * R - c2 + cu * cu;
        if (D < 0.0)
            return 0.0; // No intersection, return 0 distance
        double root = vmath::sqrt(D);
        double t0 = cu - root;
        if (t0 >= 0.0) return t0;
        double t1 = cu + root;
        if (t1 >= 0.0) return t1;

        return 0.0; // No valid intersection in the forward direction
    }

    double angleToCenter(double heading) const
    {
        return vmath::angleDifference(heading + M_PI_2, std::atan2(circle_position.Y, circle_position.X));
    }

    void updatePosition(double distance, double heading)
    {
        double deltaDistance = distance - lastDistance;
        lastDistance = distance;
        auto v = vmath::vector2d<double>::fromAngle(heading + M_PI_2, deltaDistance);
        circle_position -= v; //Heading is referenced from the forward driving direction (Y axis)
        sigma = std::clamp(sigma + std::abs(deltaDistance) * RingDriftK, 0.0, RingInnerRadius); // Update uncertainty based on distance traveled
        if (line_sensor_left_hit) line_sensor_left_position -= v;
        if (line_sensor_right_hit) line_sensor_right_position -= v;
    }

    void updateLineSensorLeft(double heading) //Left sensor has "hit"
    {
        if (line_sensor_left_hit) return;
        line_sensor_left_hit = true;
        line_sensor_left_position = vmath::vector2d{-LineSensorX, LineSensorY}.rotate(heading);

    }
    void updateLineSensorRight(double heading) //Right sensor has "hit"
    {
        if (line_sensor_right_hit) return;
        line_sensor_right_hit = true;
        line_sensor_right_position = vmath::vector2d{LineSensorX, LineSensorY}.rotate(heading);
    }
    void finalizeLineSensorPositions(double heading)
    {
        if (line_sensor_left_hit && line_sensor_right_hit)
            chordUpdate(line_sensor_left_position, line_sensor_right_position, heading);
        else if (line_sensor_left_hit)
            radialUpdate(line_sensor_left_position);
        else if (line_sensor_right_hit)
            radialUpdate(line_sensor_right_position);

        line_sensor_left_hit = false;
        line_sensor_right_hit = false;
    }

    void clearLineSensors(void)
    {
        line_sensor_left_hit = false;
        line_sensor_right_hit = false;
    }

    void reset(void)
    {
        line_sensor_left_hit = false;
        line_sensor_right_hit = false;
        circle_position = vmath::vector2d{0.0, RingInnerRadius - WHEEL_RADIUS}; //At start the ring center is infront of the robot
        sigma = 0;
        lastDistance = 0;
    }

private:
    void radialUpdate(vmath::vector2d<double> Sp)
    {
        //From a single line sensor position we can only determine that we are at the edge of the ring, so we project the circle center along the direction from the sensor to the current estimated center.
        //This assumes that our heading estimate is reasonably accurate, so the direction from the sensor to the current estimated center is a good approximation of the true radial direction.
        auto v = circle_position - Sp;
        circle_position = v.normalize() * RingInnerRadius + Sp;
        sigma *= 1.0 - RingRadialRecovery; // Reduce uncertainty after radial update
    }

    void chordUpdate(vmath::vector2d<double> A, vmath::vector2d<double> B, double heading)
    {
        //When both line sensors have detected the line, we can estimate the circle center using the chord formed by the two sensor positions.
        //Since the only way that both sensors can see the line is if the robot is facing the edge then the center of the ring is neceseraly behind the robot at a distance R, along the direction perpendicular to the chord formed by the two sensor positions.
        //We find the angle between the robot's heading and the chord's perpendicular direction using the known line sensor's X seperation and the chord length.
        //The accuracy of this method only depends on the sensor precision and the odometry precision during the movement from one line sensor detection to the other, since the white line is very thin the odometry error is negligible.
        auto chord = B - A;
        auto midpoint = (A + B) * 0.5;

        auto normal = chord.rotate(M_PI_2).normalize();

        double L = chord.length();
        double h = std::sqrt(RingInnerRadius * RingInnerRadius - 0.25 * L * L);

        auto center1 = midpoint + normal * h;
        auto center2 = midpoint - normal * h;

        auto forward = vmath::vector2d<double>::fromAngle(heading + M_PI_2);

        circle_position = (center1.dot(forward) < center2.dot(forward)) ? center1 : center2;
        
        sigma = 0; // Reset uncertainty after chord update
    }
};


constexpr double maxVelocityForStoppingDistance(
    double stopping_distance,
    double deceleration,
    double margin = 0.5)
{
    return vmath::sqrt(
        2.0 * deceleration *
        stopping_distance * margin);
}

constexpr double v_edge =
    maxVelocityForStoppingDistance(
        0.02,                       // 2 cm
        10.0,
        0.5);                       // 50% margin

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
vmath::vector3d<double> orientation = {0.0}; // Euler angles in radians, in ZYX order (yaw-pitch-roll), referenced to initial orientation at power on

double angleL_total = 0.0;//rad
double angleR_total = 0.0;//rad
double distance = 0.0;//m
double total_heading = 0.0; // Total accumulated heading change in radians
double line_detected_distance = 0.0; // Distance at which a line was detected, in meters
bool line_was_detected = false; // True if a line was detected
double delta_heading = 0.0;
double target_v = 0; //desired linear velocity (of the robot) in m/s
double target_w = 0; //desired angular velocity or rotational velocity (of the robot) in rad/s
double target_a = 0; //desired linear acceleration (of the robot) in m/s^2
double target_alpha = 0; //desired angular acceleration (of the robot) in rad/s^2
double target_distance = 0.0;//m
double target_speed = 0.0; // Desired speed at the end of the move
double target_heading = 0.0;//rad
bool hold_heading = false; // True for straight moves (no commanded turn)

constexpr double vel_alpha = 0.1; // Low-pass filter coefficient for velocity filtering
double velocity_filtered = 0.0;

PID pid_wL(0.5, 2.0, 0.0, 1.5, 2.5);
PID pid_wR(0.5, 2.0, 0.0, 1.5, 2.5);
PID pid_heading(10.0, 0.5, 0.0, 5.0, 20.0); // Heading error in rad, output is target_w in rad/s

int initialized = 0;
constexpr int InitializationDelay = 500; // in milliseconds
utils::time_t initialization_start_time = 0;
bool repeated_line = false;
bool enabled = false;
bool line_avoidance_enabled = false;
LineAvoidanceState line_avoidance_state = LineAvoidanceState::None;
double distance_to_line = 0.0;

RingTracker ring_tracker;

MotionState motion_state = MotionState::Idle;

vmath::vector2d<double> get_ring_position(void)
{
    return ring_tracker.circle_position;
}

double get_ring_sigma(void)
{
    return ring_tracker.sigma;
}

double get_ring_angle_to_center(void)
{
    return ring_tracker.angleToCenter(total_heading);
}

bool get_ring_left_hit(void)
{
    return ring_tracker.line_sensor_left_hit;
}

bool get_ring_right_hit(void)
{
    return ring_tracker.line_sensor_right_hit;
}

vmath::vector2d<double> get_line_sensor_left_pos(void)
{
    return ring_tracker.line_sensor_left_position;
}

vmath::vector2d<double> get_line_sensor_right_pos(void)
{
    return ring_tracker.line_sensor_right_position;
}

double get_distance_to_line(void)
{
    return distance_to_line;
}

void motion_init(void)
{
    accelBias = active_config.accelBias;
    gyroBias = active_config.gyroBias;
    // Initialize line sensors to false
    ring_tracker.reset();
}

void motion_move(double delta_distance, double delta_angle, double coasting_velocity)
{
    if (motion_state != MotionState::Idle)
        return;

    if (delta_distance == 0.0 && delta_angle == 0.0)
        return;
    if (delta_distance == 0.0)
    {
        target_heading = total_heading + delta_angle;
        motion_state = MotionState::Turning; // If only an angle is specified, we are turning
        target_speed = 0.0;
        return; // No need to set target distance for pure turn
    }
    else
    {
        target_distance = distance + delta_distance;
        target_heading = total_heading;
        target_speed = delta_distance > 0 ? coasting_velocity : 0;
        motion_state = MotionState::Moving; // If a distance is specified, we are moving
    }
    
}

void motion_stop(void)
{
    if (motion_state == MotionState::Idle || line_avoidance_state != LineAvoidanceState::None)
        return;
    motion_state = MotionState::Stop;
}

void motion_test(bool forward)
{
    motion_move(forward ? 0.4 : -0.1, 0.0);
    //tst = forward;
}

void motion_turn_test(bool left)
{
    motion_move(0.0, left ? 90.0 / 180.0 * M_PI : -90.0 / 180.0 * M_PI);
}

void motion_zero_position(void)
{
    angleL_total = 0.0;
    angleR_total = 0.0;
    distance = 0.0;
    total_heading = 0.0;
}

// Feedforward voltage for one motor; the direction (and so the constants) follows the sign of target_w, or target_a at standstill.
static double motor_feedforward(const MotorConstants& m, double target_w, double target_a)
{
    const MotorModel& d = (target_w != 0.0 ? target_w : target_a) >= 0.0 ? m.forward : m.reverse;
    double ks = target_w >= MinMotorRadPerSec ? d.Ks : (target_w <= -MinMotorRadPerSec ? -d.Ks : 0.0);
    return ks + d.Kv * target_w + d.Ka * target_a;
}

MotionState get_motion_state(void)
{
    return motion_state;
}


void motion_set_line_avoidance_enabled(bool en)
{
    line_avoidance_enabled = en;
}

LineAvoidanceState get_line_avoidance_state(void)
{
    return line_avoidance_state;
}

void motion_set_enabled(bool en)
{
    enabled = en;
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
    static double last_heading = 0;
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
    double wL = d_angleL / dt;
    double wR = d_angleR / dt;
    double aL = (wL - last_wL) / dt;
    double aR = (wR - last_wR) / dt;
    last_wL = wL;
    last_wR = wR;
    double d_distanceL = d_angleL * WHEEL_RADIUS;
    double d_distanceR = d_angleR * WHEEL_RADIUS;

    double velocity = (wL + wR) * WHEEL_RADIUS / 2.0;
    double acceleration = (velocity - last_velocity) / dt;
    last_velocity = velocity;

    velocity_filtered = velocity * vel_alpha + velocity_filtered * (1.0 - vel_alpha);
    acc_filt = acceleration * a_alpha + acc_filt * (1.0 - a_alpha);
    distance += (d_distanceL + d_distanceR) / 2.0;

    bool stationary = std::abs(a_filt.length() - 1.0) < a_min && std::abs(w_imu.length()) < w_min && std::abs(velocity) < v_min && std::abs(acceleration) < a_min;

    vmath::vector3d<double> linear_acceleration = {0.0}; // In the robot-heading-aligned frame with Z fixed up, in m/s^2, referenced to initial orientation at power on
    vmath::vector3d<double> angular_velocity = {0.0}; // In the robot-heading-aligned frame with Z fixed up, in rad/s, referenced to initial orientation at power on
    vmath::vector3d<double> angular_acceleration = {0.0}; // In the robot-heading-aligned frame with Z fixed up, in rad/s^2, referenced to initial orientation at power on

    //Update orientation even when stationary as the madgwic filter keeps maintains stable orientation estimation even without movement.
    orientation = q.eulerAngles(); // Orientation in world frame, referenced to initial orientation at power on, ZYX order (yaw-pitch-roll)
    double d_heading = vmath::angleDifference(last_heading, orientation.yaw()); // Change in heading since last update
    last_heading = orientation.yaw();
    if (!stationary)
    {
        auto q_tilt = vmath::quaternion<>::fromRotationZ(-orientation.yaw()) * q; // Remove heading to get tilt-compensated acceleration and angular velocity in the robot-heading-aligned frame
        linear_acceleration = ((q_tilt * a_filt * q_tilt.conjugate()).toVector() - g) * G; // Convert from g to m/s^2 and remove gravity
        auto new_angular_velocity = (q_tilt * w_imu * q_tilt.conjugate()).toVector(); // Rotate angular velocity to robot-heading-aligned frame, no need to remove gravity since it's a vector and doesn't affect rotation
        angular_acceleration = (new_angular_velocity - angular_velocity) / dt; // Compute angular acceleration in the robot-heading-aligned frame
        angular_velocity = new_angular_velocity; // Update angular velocity
        total_heading += d_heading; // Accumulate total heading change, doing this here prevents drift when stationary
    }

    bool line_left = gpio_get(LINE_SENSOR_LEFT_PIN);
    bool line_right = gpio_get(LINE_SENSOR_RIGHT_PIN);
    bool line_detected = line_left || line_right;
    bool moving = std::abs(velocity) > MinVelocity || std::abs(angular_velocity.yaw()) > MinAngularVelocity;
    bool moving_forward = moving && velocity > MinVelocity;

    ring_tracker.updatePosition(distance, total_heading);

    //double motors_w_avg = (wL + wR) / 2.0;
    //double motors_bemf_avg = std::min(Kv * std::abs(motors_w_avg), SystemVoltage - Ks); // Ks is already part of MinDeceleration
    //double possible_deceleration = (motors_bemf_avg / (SystemVoltage - Ks)) * (VoltageLimitedMaxDeceleration - VoltageLimitedMinDeceleration) + VoltageLimitedMinDeceleration;
    
    //utils::debug_printf("Remaining distance: %f, Heading error: %f, Moving: %d, Line detected: %d\nmoving_forward: %d, rotating_left: %d, angular_velocity: %f, velocity: %f\n", remaining_distance, heading_error, moving, line_detected, moving_forward, rotating_left, angular_velocity.yaw(), velocity);

    if (line_left)
    {
        ring_tracker.updateLineSensorLeft(total_heading);
    }
    if (line_right)
    {
        ring_tracker.updateLineSensorRight(total_heading);
    }

    if (line_detected && !line_was_detected)
    {
        line_was_detected = true;
        line_detected_distance = distance;
    }
    else if (!line_detected && line_was_detected)
    {
        if (distance - line_detected_distance < -MinDistance)
        {
            line_was_detected = false;
            ring_tracker.finalizeLineSensorPositions(total_heading);
        }
    }

    bool on_line = line_detected || line_was_detected;

    switch (line_avoidance_state)
    {
        case LineAvoidanceState::None:
        {
            if (on_line && line_avoidance_enabled)
            {
                if (!repeated_line)
                {
                    motion_state = MotionState::Stop;
                    line_avoidance_state = LineAvoidanceState::Stoping;
                }
            }
            
            if (!line_detected && repeated_line)
            {
                repeated_line = false;
                line_was_detected = false;
                ring_tracker.clearLineSensors();
            }
            break;
        }
        case LineAvoidanceState::Stoping:
        {
            if (motion_state == MotionState::Idle)
            {
                target_distance = line_detected_distance - LineReturnDistance;
                target_heading = total_heading;
                target_speed = 0;
                motion_state = MotionState::Moving;
                line_avoidance_state = LineAvoidanceState::Returning;
            }
            break;
        }
        case LineAvoidanceState::Returning:
        {
            if (motion_state == MotionState::Idle)
            {
                
                if (line_detected)
                {
                    repeated_line = true;
                    line_was_detected = false;
                }
                target_distance = 0;
                target_heading = total_heading + ring_tracker.angleToCenter(total_heading);
                target_speed = 0;
                motion_state = MotionState::Turning;
                line_avoidance_state = LineAvoidanceState::None;
            }
            break;
        }
    }

    distance_to_line = ring_tracker.distanceToLine(total_heading);
    double remaining_distance = target_distance - distance;
    if (remaining_distance > 0)
    {
        if (distance_to_line < remaining_distance)
        {
            // Adjust remaining distance to avoid crossing the line
            remaining_distance = std::max(0.0, distance_to_line - LineSafetyMargin);
        }
    }

    
    double heading_error = target_heading - total_heading; // Remaining heading error (method handles wrapping)
    delta_heading = heading_error;

    switch (motion_state)
    {
        case MotionState::Idle:
        {
            //distance = 0;
            target_speed = 0;
            target_distance = distance;
            target_heading = total_heading;
            

            target_a = 0;
            target_v = 0;
            target_w = 0;
            target_alpha = 0;
            pid_heading.reset();
            pid_wL.reset();
            pid_wR.reset();
            break;
        }
        case MotionState::Stop:
        {
            if (moving_forward)
            {
                target_a = -StopDeceleration;
                target_v += target_a * dt;
                target_w = 0;
                target_alpha = 0;
            }
            else
            {
                target_a = 0;
                target_v = 0;
                target_w = 0;
                target_alpha = 0;
                pid_heading.reset();
                pid_wL.reset();
                pid_wR.reset();
                motion_state = MotionState::Idle;
            }
            break;
        }
        case MotionState::Moving:
        {
            // Limits for the direction we are heading in, and for the direction we are currently travelling (matters when braking or reversing).
            const MotionLimits& toward = remaining_distance >= 0 ? ForwardLimits : ReverseLimits;
            const MotionLimits& travelling = (target_v != 0.0 ? target_v : remaining_distance) >= 0 ? ForwardLimits : ReverseLimits;

            double target_velocity = std::abs(target_speed) < MinVelocity ? 0 : target_speed;

            // Fastest speed from which we can still stop within the remaining distance; sign points toward the target.
            double v_limit = std::min({toward.velocity, std::sqrt(target_velocity * target_velocity + 2.0 * DecelPlanMargin * toward.deceleration * std::abs(remaining_distance)), PositionKp * std::abs(remaining_distance) + std::abs(target_velocity)});
            double v_desired = remaining_distance >= 0 ? v_limit : -v_limit;

            // Slew toward v_desired: brake rate when reducing speed or reversing, accel rate when speeding up.
            bool speeding_up = std::abs(v_desired) > std::abs(target_v) && (v_desired * target_v >= 0);
            double max_step = (speeding_up ? toward.acceleration : travelling.deceleration) * dt;
            double step = std::clamp(v_desired - target_v, -max_step, max_step);
            target_v += step;
            target_a = step / dt;
            target_w = pid_heading.compute(target_heading, total_heading, dt);
            target_alpha = 0;
            if (std::abs(remaining_distance) < MinDistance)
            {
                if (target_velocity != 0)
                {
                    motion_state = MotionState::Coasting;
                }
                else
                {
                    motion_state = MotionState::Stop;
                }
            }
            break;
        }
        case MotionState::Turning:
        {
            double w_limit = std::min({RotationalLimits.velocity, std::sqrt(2.0 * DecelPlanMargin * RotationalLimits.deceleration * std::abs(heading_error)), HeadingKp * std::abs(heading_error)});
            double w_desired = heading_error >= 0 ? w_limit : -w_limit;
            bool speeding_up = std::abs(w_desired) > std::abs(target_w) && (w_desired * target_w >= 0);
            double max_step = (speeding_up ? RotationalLimits.acceleration : RotationalLimits.deceleration) * dt;
            double step = std::clamp(w_desired - target_w, -max_step, max_step);
            target_v = 0;
            target_a = 0;
            target_w += step;
            target_alpha = step / dt;
            if (std::abs(heading_error) < MinHeadingError)
            {
                motion_state = MotionState::Stop;
            }
            break;
        }
        case MotionState::Coasting:
        {
            double target_velocity = std::abs(target_speed) < MinVelocity ? 0 : target_speed;
            double v_current = target_v; // Average the measured velocity and the setpoint to reduce oscillations
            double step = std::clamp(target_velocity - v_current, -ForwardLimits.deceleration / 2 * dt, ForwardLimits.acceleration / 2 * dt);
            target_v += step;
            target_a = step / dt;
            target_w = pid_heading.compute(target_heading, total_heading, dt);
            target_alpha = 0;

            if (!moving && std::abs(target_velocity) < MinVelocity)
            {
                motion_state = MotionState::Stop;
            }
            break;
        }
    }

    if (!enabled || initialized != 2)
    {
        distance = 0;
        total_heading = 0;
        target_speed = 0;
        target_distance = distance;
        target_heading = total_heading;

        target_a = 0;
        target_v = 0;
        target_w = 0;
        target_alpha = 0;
        pid_heading.reset();
        pid_wL.reset();
        pid_wR.reset();
        ring_tracker.reset();
        motion_state = MotionState::Idle;
        line_avoidance_state = LineAvoidanceState::None;
        line_was_detected = false;
        line_detected_distance = distance;

        if (initialized == 0)
        {
            initialization_start_time = utils::now();
            initialized = 1;
        }
        else if (initialized == 1)
        {
            if (utils::hasElapsed_ms(initialization_start_time, InitializationDelay))
            {
                initialized = 2;
            }
        }
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

    double dutyCycleL = VoltageL / SystemVoltage;
    double dutyCycleR = VoltageR / SystemVoltage;

    left_motor.setPower(dutyCycleL); //The duty cycle is clamped in the setPower method
    right_motor.setPower(dutyCycleR);
}
