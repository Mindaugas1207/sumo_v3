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
    Returning,
    Turning
};
extern double delta_heading;
extern double target_heading;
extern double total_heading;

extern MotionState motion_state;
extern LineAvoidanceState line_avoidance_state;
extern double distance;
extern bool tst;

MotionState get_motion_state(void);

#endif // APP_MOTION_H