
//#include "pico_utils.h"
#include "app/hardware.h"


#include <stdio.h>
#include <stdarg.h>
#include "pico/stdlib.h"

#include "display.h"
#include "app/remote.h"
#include "app/motion.h"
#include "app/sensors.h"



//#include "config.h"

bool display_enabled = true;
bool start_state = false;
constexpr int MOVE_COOLDOWN_MS = 100; // Move cooldown in milliseconds, because sensors internaly are read at this interval
constexpr int MOVE_TIMEOUT = 200; // Move timeout in milliseconds


// Sensor index definitions for easier reference in the code
constexpr int SI_LEFT45 = 5;
constexpr int SI_RIGHT45 = 1;
constexpr int SI_FRONT = 3;
constexpr int SI_LEFT2p5 = 6;
constexpr int SI_RIGHT2p5 = 0;
constexpr int SI_LEFT35 = 2;
constexpr int SI_RIGHT35 = 4;

constexpr int FRONT_SENSOR_ATTACK_THRESHOLD = 100; // Distance threshold for preparing attack in mm
constexpr int ATTACK_PREPARE_TIME = 300;
constexpr int ATTACK_TIMEOUT = 100; // Attack timeout in milliseconds
constexpr double ATTACK_VELOCITY = 0.1;
constexpr double SEARCH_VELOCITY = 0.2;
CombatState combat_state = STATE_SEARCH;

int main()
{
    hardware_init();
    sensors_init();

    //left_motor.setPower(0.4);
     //   right_motor.setPower(0.4);

    bool line_sensor_triggered = false;

    

    utils::time_t last_move_time = 0;
    bool on_cooldown = false;

    utils::time_t attack_prepare_start_time = 0;
    utils::time_t attack_target_last_detected_time = 0;
    //sleep_ms(1000);
    //stdio_init_all();
    //display_init();

    while (true)
    {
        handle_remote();
        handle_sensors();
        STATUS_Led.update();

        bool st = gpio_get(START_PIN);
        if (st != start_state)
        {
            start_state = st;
            if (start_state)
            {
                motion_set_motors_enabled(true);
                display_enabled = false;
            }
            else
            {
                motion_set_motors_enabled(false);
                display_enabled = true;
            }
        }

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
                move_rotational_degrees(120); // Turn around after moving backward slightly
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
                        move_rotational_degrees(-2.5);
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        break; // Exit the switch after moving
                    }

                    if (sensorData.distanceSensorDetected[SI_RIGHT2p5])
                    {
                        //if only one side sensor detects the target, turn slightly.
                        move_rotational_degrees(2.5);
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        break; // Exit the switch after moving
                    }

                    if (sensorData.distanceSensorDetected[SI_LEFT35])
                    {
                        move_rotational_degrees(-35);
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        break; // Exit the switch after moving
                    }

                    if (sensorData.distanceSensorDetected[SI_RIGHT35])
                    {
                        utils::debug_printf("Right 35 sensor detected, rotating 35 degrees...\n");
                        move_rotational_degrees(35);
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        break; // Exit the switch after moving
                    }

                    if (sensorData.distanceSensorDetected[SI_LEFT45])
                    {
                        //utils::debug_printf("Left 45 sensor detected, rotating -45 degrees...\n");
                        move_rotational_degrees(-45);
                        last_move_time = utils::now(); // Update the last move time
                        on_cooldown = true; // Set the cooldown flag after moving
                        break; // Exit the switch after moving
                    }

                    if (sensorData.distanceSensorDetected[SI_RIGHT45])
                    {
                        //utils::debug_printf("Right 45 sensor detected, rotating 45 degrees...\n");
                        move_rotational_degrees(45);
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
                        move_linear(0.1);
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

        

        //int driver_enable = gpio_get(MOTOR_DRIVER_ENABLE);
        //int start = gpio_get(START_PIN);
        //utils::debug_printf("Driver enable: %d, Start: %d\n", driver_enable, start);
    }
}

void second_core_main(void)
{
    motion_init();

    while (true)
    {
        motion_update();
        //if (display_enabled)
            handle_display();
    }
}






