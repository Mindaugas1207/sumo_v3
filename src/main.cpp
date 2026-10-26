
//#include "pico_utils.h"
#include "app/hardware.h"


#include <stdio.h>
#include <stdarg.h>
#include "pico/stdlib.h"

#include "display.h"
#include "app/remote.h"
#include "app/motion.h"
#include "app/sensors.h"

enum class CombatState
{
    Search,
    Attack,
    Push
};

CombatState combat_state = CombatState::Search;

SensorData sensorData;
LineAvoidanceState lineAvoidanceState;
MotionState motionState;

void handle_combat();

int main()
{
    hardware_init();
    motion_init();

    while (true)
    {
        bool start_state = gpio_get(START_PIN);
        motion_set_enabled(start_state);

        sensorData = get_sensor_data();
        lineAvoidanceState = get_line_avoidance_state();
        motionState = get_motion_state();

        if (start_state)
        {
            handle_combat();
        }
        else
        {
            handle_remote();
            handle_display();
            STATUS_Led.update();
        }

        motion_update();
    }
}

void second_core_main(void)
{
    sensors_init();
    while (true)
    {
        handle_sensors();
        utils::sleep_ms(10);
    }
}

void handle_search()
{
    if (lineAvoidanceState == LineAvoidanceState::None)
    {
        if (sensorData.targetDetected)
        {
            motion_stop();
            combat_state = CombatState::Attack;
        }
        else if (motionState == MotionState::Idle)
        {
            motion_move(0.4, 0.0);
        }
    }
}

void handle_attack()
{
    if (lineAvoidanceState == LineAvoidanceState::None)
    {
        if (!sensorData.targetDetected)
        {
            motion_stop();
            combat_state = CombatState::Search;
        }
    }
}


void handle_combat()
{
    switch (combat_state)
    {
        case CombatState::Search:
            handle_search();
            break;
        case CombatState::Attack:
            handle_attack();
            break;
        case CombatState::Push:
            // Handle push behavior
            break;
    }

}


/*
if (start_state)
        {
            auto sensorData = get_sensor_data();
            bool line_sensor_left_gpio_state = gpio_get(LINE_SENSOR_LEFT_PIN);
            bool line_sensor_right_gpio_state = gpio_get(LINE_SENSOR_RIGHT_PIN);
            if (!line_sensor_triggered && (line_sensor_left_gpio_state || line_sensor_right_gpio_state))
            {
                line_sensor_triggered = true;
                move_cancel();
                sleep_ms(5); // Wait for a short duration before starting the backward move
                move_linear(-0.05); // Move backward slightly when a line sensor is triggered
                while (!is_move_complete())
                {
                    // Wait until the backward move is complete
                }
                sleep_ms(10);
                move_rotational_degrees(100); // Turn around after moving backward slightly
                while (!is_move_complete())
                {
                    // Wait until the turn is complete
                }
                on_cooldown = false;
                last_move_time = utils::now();
                combat_state = STATE_SEARCH;
            }
            else if (!(line_sensor_left_gpio_state || line_sensor_right_gpio_state))
            {
                line_sensor_triggered = false;
            }

            if (on_cooldown)
            {
                if (utils::hasElapsed_ms(last_move_time, MOVE_COOLDOWN_MS))
                {
                    on_cooldown = false;
                }
                else
                {
                    // Still on cooldown
                    continue; //skip the rest of the loop while on cooldown
                }
            }

            if (!is_move_complete())
            {
                if (utils::hasElapsed_ms(last_move_time, MOVE_TIMEOUT))
                {
                    move_cancel(); // Cancel the move if it has timed out
                    last_move_time = utils::now(); // Update the last move time
                    on_cooldown = true; // Set the cooldown flag after moving
                    continue; // Skip the rest of the loop after handling the timeout
                }
            }

            // Calculate the remaining distance to the target based on the detected distance sensors
            int avg_distance = MAX_DISTANCE;

            if (sensorData.distanceSensorDetected[SI_FRONT] && sensorData.distanceSensorDetected[SI_LEFT2p5] && sensorData.distanceSensorDetected[SI_RIGHT2p5])
            {
                avg_distance = (sensorData.distanceSensorValues[SI_FRONT] + sensorData.distanceSensorValues[SI_LEFT2p5] + sensorData.distanceSensorValues[SI_RIGHT2p5]) / 3;
            }
            else if (sensorData.distanceSensorDetected[SI_FRONT] && sensorData.distanceSensorDetected[SI_LEFT2p5])
            {
                avg_distance = (sensorData.distanceSensorValues[SI_FRONT] + sensorData.distanceSensorValues[SI_LEFT2p5]) / 2;
            }
            else if (sensorData.distanceSensorDetected[SI_FRONT] && sensorData.distanceSensorDetected[SI_RIGHT2p5])
            {
                avg_distance = (sensorData.distanceSensorValues[SI_FRONT] + sensorData.distanceSensorValues[SI_RIGHT2p5]) / 2;
            }
            else if (sensorData.distanceSensorDetected[SI_FRONT])
            {
                avg_distance = sensorData.distanceSensorValues[SI_FRONT];
            }
            else if (sensorData.distanceSensorDetected[SI_LEFT2p5] && sensorData.distanceSensorDetected[SI_RIGHT2p5])
            {
                avg_distance = (sensorData.distanceSensorValues[SI_LEFT2p5] + sensorData.distanceSensorValues[SI_RIGHT2p5]) / 2;
            }

            //utils::debug_printf("TOTAL %d\n",
            //                   sensorData.targetDetected);

            switch (combat_state)
            {
            case STATE_SEARCH:
                {
                    // Handle search state
                    if (sensorData.targetDetected)
                    {
                        move_cancel();
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        combat_state = STATE_APPROACH; // If a target is detected, switch to approach state
                    }
                    // Move at a constant velocity, the robot will turn around when it hits the edge of the arena using the line avoidance sistem.
                    move_linear(0.2);
                    last_move_time = utils::now(); // Update the last move time
                    on_cooldown = true; // Set the cooldown flag after moving

                    break;
                }
            case STATE_APPROACH:
                {
                    //utils::debug_printf("Approach state: avg_distance = %d, targetDetected = %d\n", avg_distance, sensorData.targetDetected);

                    // Handle approach state
                    if (!sensorData.targetDetected)
                    {
                        combat_state = STATE_SEARCH; // If no target is detected, go back to search state
                        break;
                    }
                    

                    int remaining_distance = avg_distance - FRONT_SENSOR_ATTACK_THRESHOLD;
                    //utils::debug_printf("Remaining distance to target: %d\n", remaining_distance);
                    if (avg_distance < MAX_DISTANCE)
                    {
                        //Target is detected in front, move towards it
                        if (remaining_distance <= 0)
                        {
                            // Already at or past the attack threshold
                            combat_state = STATE_PREPARE_ATTACK;
                            attack_prepare_start_time = utils::now(); // Record the start time of the attack preparation
                            break;
                        }
                        // Move towards the target by half the remaining distance
                        move_linear(remaining_distance / 2000.0); // Convert mm to meters and move forward
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        break; // Exit the switch after moving
                    }

                    if (sensorData.distanceSensorDetected[SI_LEFT2p5])
                    {
                        //if only one side sensor detects the target, turn slightly.
                        move_rotational_degrees(-5);
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        break; // Exit the switch after moving
                    }

                    if (sensorData.distanceSensorDetected[SI_RIGHT2p5])
                    {
                        //if only one side sensor detects the target, turn slightly.
                        move_rotational_degrees(5);
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        break; // Exit the switch after moving
                    }

                    if (sensorData.distanceSensorDetected[SI_LEFT35])
                    {
                        move_rotational_degrees(-25);
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        break; // Exit the switch after moving
                    }

                    if (sensorData.distanceSensorDetected[SI_RIGHT35])
                    {
                        utils::debug_printf("Right 35 sensor detected, rotating 35 degrees...\n");
                        move_rotational_degrees(25);
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        break; // Exit the switch after moving
                    }

                    if (sensorData.distanceSensorDetected[SI_LEFT45])
                    {
                        //utils::debug_printf("Left 45 sensor detected, rotating -45 degrees...\n");
                        move_rotational_degrees(-35);
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        break; // Exit the switch after moving
                    }

                    if (sensorData.distanceSensorDetected[SI_RIGHT45])
                    {
                        //utils::debug_printf("Right 45 sensor detected, rotating 45 degrees...\n");
                        move_rotational_degrees(35);
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        break; // Exit the switch after moving
                    }

                    break;
                }
            case STATE_PREPARE_ATTACK:
                {
                    if (!sensorData.targetDetected)
                    {
                        //utils::debug_printf("No target detected, switching to search state...\n");
                        combat_state = STATE_SEARCH; // If no target is detected, go back to search state
                        break;
                    }

                    if (avg_distance > FRONT_SENSOR_ATTACK_THRESHOLD)
                    {
                        //utils::debug_printf("Average distance greater than threshold, switching to approach state...\n");
                        combat_state = STATE_APPROACH; // If the average distance is greater than the threshold, switch to approach state
                        break;
                    }

                    if (utils::hasElapsed_ms(attack_prepare_start_time, ATTACK_PREPARE_TIME))
                    {
                        //utils::debug_printf("Attack preparation time elapsed, switching to attack state...\n");
                        combat_state = STATE_ATTACK; // Switch to attack state after the preparation time has elapsed
                        break;
                    }

                    // Handle prepare attack state
                    break;
                }
            case STATE_ATTACK:
                {
                    // Handle attack state
                    if (!sensorData.targetDetected || avg_distance > FRONT_SENSOR_ATTACK_THRESHOLD)
                    {
                        //utils::debug_printf("Target lost or out of range, checking attack timeout...\n");
                        if (utils::hasElapsed_ms(attack_target_last_detected_time, ATTACK_TIMEOUT))
                        {
                            //utils::debug_printf("Attack timed out, canceling move and switching state...\n");
                            move_cancel();
                            last_move_time = utils::now(); // Update the last move time
                            on_cooldown = true; // Set the cooldown flag after moving
                            combat_state = sensorData.targetDetected ? STATE_APPROACH : STATE_SEARCH; // If the target is lost and attack times out, go back to approach or search state
                        }
                    }
                    else
                    {
                        //utils::debug_printf("Target detected, moving forward...\n");
                        attack_target_last_detected_time = utils::now(); // Update the last detected time of the attack target
                        move_constant_velocity(0.6, 0.0);
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                    }

                    break;
                }
            }
            //utils::debug_printf("Combat state: %d\n", combat_state);
        }
        else
        {
            combat_state = STATE_SEARCH;
        }
*/



