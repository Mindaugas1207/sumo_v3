
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

bool display_enabled = false;
bool start_state = false;

int main()
{
    hardware_init();
    sensors_init();

    //left_motor.setPower(0.4);
     //   right_motor.setPower(0.4);

    bool line_sensor_triggered = false;

    //sleep_ms(1000);
    //stdio_init_all();
    //display_init();

    while (true)
    {
        handle_remote();
        handle_sensors();

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
                sleep_ms(10);
                set_velocity(-0.3, 0.0);
                sleep_ms(10); // Wait for a short duration before starting the backward move
                move_linear(-0.2); // Move backward slightly when a line sensor is triggered
                while (!is_move_complete())
                {
                    // Wait until the backward move is complete
                }
                sleep_ms(10);
                move_rotational_degrees(180); // Turn around after moving backward slightly
                while (!is_move_complete())
                {
                    // Wait until the turn is complete
                }
            }
            else if (!(line_sensor_left_gpio_state || line_sensor_right_gpio_state))
            {
                line_sensor_triggered = false;
            }
        }

        STATUS_Led.update();

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
        handle_display();
    }
}






