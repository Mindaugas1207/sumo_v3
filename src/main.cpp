
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
        STATUS_Led.update();

        //int driver_enable = gpio_get(MOTOR_DRIVER_ENABLE);
        //int start = gpio_get(START_PIN);
        //utils::debug_printf("Driver enable: %d, Start: %d\n", driver_enable, start);
    }
}

void second_core_main(void)
{
    while (true)
    {
        motion_update();
        handle_display();
    }
}






