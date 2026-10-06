
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






