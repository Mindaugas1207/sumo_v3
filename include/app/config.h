
#ifndef CONFIG_H
#define CONFIG_H

#define ROBOT_VERSION 2

//MOTOR DRIVER
#define MOTOR_DRIVER_FREQUENCY 10000 //20kHz
#define MOTOR_DRIVER_PWMA 16
#define MOTOR_DRIVER_DIRA 18
#define MOTOR_DRIVER_INVA 0
#define MOTOR_DRIVER_PWMB 17
#define MOTOR_DRIVER_DIRB 19
#define MOTOR_DRIVER_INVB 0

#define MOTOR_DRIVER_ENABLE 25
#define START_PIN 26

//SENSORS
#define SENSORS_ENABLE 20
//V1 Robot config
#if ROBOT_VERSION == 1
#define DISTANCE_SENSOR_RIGHT45_PIN 0 //45 right
#define DISTANCE_SENSOR_RIGHT90_PIN 1 //90 right
#define DISTANCE_SENSOR_RIGHT0_PIN 2 //0 right
#define DISTANCE_SENSOR_LEFT18_PIN 3 //18 left
#define DISTANCE_SENSOR_LEFT27_PIN 4 //27 left
#define DISTANCE_SENSOR_CENTER_PIN 8 //center
#define DISTANCE_SENSOR_RIGHT27_PIN 10 //27 right
#define DISTANCE_SENSOR_RIGHT18_PIN 13 //18 right
#define DISTANCE_SENSOR_LEFT0_PIN 14 //0 left
#define DISTANCE_SENSOR_LEFT90_PIN 12 //90 left
#define DISTANCE_SENSOR_LEFT45_PIN 11 //45 left
#define DISTANCE_SENSOR_COUNT 11
#endif
//V2 Robot config
#if ROBOT_VERSION == 2
#define DISTANCE_SENSOR_RIGHT0_PIN 6 //0 right
#define DISTANCE_SENSOR_RIGHT45_PIN 7 //45 right
#define DISTANCE_SENSOR_LEFT35_PIN 8 //35 left
#define DISTANCE_SENSOR_CENTER_PIN 9 //center
#define DISTANCE_SENSOR_RIGHT35_PIN 10 //35 right
#define DISTANCE_SENSOR_LEFT45_PIN 11 //45 left
#define DISTANCE_SENSOR_LEFT0_PIN 12 //0 left
#define DISTANCE_SENSOR_COUNT 7
#endif

#if ROBOT_VERSION == 1
#define LINE_SENSOR_RIGHT_PIN 5 //right
#define LINE_SENSOR_LEFT_PIN 15 //left
#define LINE_SENSOR_BACK_PIN 6 //back
#define LINE_SENSOR_COUNT 3
#endif
#if ROBOT_VERSION == 2
#define LINE_SENSOR_RIGHT_PIN 14 //right
#define LINE_SENSOR_LEFT_PIN 4 //left
#define LINE_SENSOR_COUNT 2
#endif

#if ROBOT_VERSION == 1
#define RECEIVER_PIN 7
#endif
#if ROBOT_VERSION == 2
#define RECEIVER_PIN 1
#endif

#define FLAG_MODULE_PIN 9

//LED
#define LED_PIN 24

//IMU
#define IMU_INT_PIN 21

//I2C
#define I2C_PORT i2c1
#define I2C_SDA 22
#define I2C_SCL 23
#define I2C_SPEED 400000 //400kHz

#define I2C_PORT2 i2c0
#define I2C_SDA2 28
#define I2C_SCL2 29
#define I2C_SPEED2 400000 //400kHz

#define LSM6DSR_I2C I2C_PORT
#define MT6701_L_I2C I2C_PORT
#define MT6701_R_I2C I2C_PORT2



//DEBUG
#define DEBUG 1
#define DEBUG_UART uart0
#define DEBUG_UART_BAUD_RATE 115200
#define DEBUG_UART_TX_PIN 28
#define DEBUG_UART_RX_PIN 29
#define DEBUG_VBUS_PIN 27

#define USE_IR_RECEIVER 1
#define USE_DISTANCE_SENSORS 1
#define USE_LINE_SENSORS 1

#define PRINT_IMU_DATA 0
#define PRINT_SENSOR_DATA 0
#define PRINT_RECEIVER_DATA 1
#define PRINT_ENCODER_DATA 0
#define PRINT_COLLISION_DATA 0
#define PRINT_POSE_DATA 0
#define PRINT_RATE 10 //Hz

#define DEFAULT_SERIAL_BAUD_RATE 9600
#define SERIAL_BAUD_RATE 115200





/*


inline bool is_start_signal(void)
{
    return gpio_get(START_PIN) && gpio_get(MOTOR_DRIVER_ENABLE);
}

inline bool is_stop_signal(void)
{
    return !gpio_get(START_PIN) && !gpio_get(MOTOR_DRIVER_ENABLE);
}

inline void drop_flag(void)
{
    gpio_put(FLAG_MODULE_PIN, 0);
}

inline void release_flag(void)
{
    gpio_put(FLAG_MODULE_PIN, 1);
}





*/

#endif // CONFIG_H
