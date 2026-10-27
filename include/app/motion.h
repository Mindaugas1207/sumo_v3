#ifndef APP_MOTION_H
#define APP_MOTION_H

#include "vmath.h"

void motion_init(void);
void motion_update(void);

// Relative move: delta_distance in m (+forward), delta_angle in rad (+CCW). Either may be 0.
constexpr double CoastingVelocity = 0.4; // Velocity to maintain during coasting, in m/s.
void motion_move(double delta_distance, double delta_angle, double coasting_velocity = CoastingVelocity);
void motion_turn_test(bool left);
void motion_test(bool forward);
void motion_zero_position(void);
void motion_stop(void);

enum class MotionState
{
    Idle,
    Stop,
    Moving,
    Turning,
    Coasting
};
enum class LineAvoidanceState
{
    None,
    Stoping,
    Returning
};
extern double delta_heading;
extern double target_heading;
extern double total_heading;
extern double distance;

MotionState get_motion_state(void);
LineAvoidanceState get_line_avoidance_state(void);
void motion_set_enabled(bool);
double get_ring_angle_to_center(void);
vmath::vector2d<double> get_ring_position(void);
double get_ring_sigma(void);

bool get_ring_left_hit(void);
bool get_ring_right_hit(void);
vmath::vector2d<double> get_line_sensor_left_pos(void);
vmath::vector2d<double> get_line_sensor_right_pos(void);

double get_distance_to_line(void);


#endif // APP_MOTION_H