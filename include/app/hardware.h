#ifndef APP_HARDWARE_H
#define APP_HARDWARE_H

#include "config.h"
#include "pico_events.h"
#include "pio_ws2812.h"
#include "lsm6dsr.h"
#include "ir_receiver.h"
#include "distance_sensor.h"
#include "line_sensor.h"
#include "graphics.h"
#include "pico_motor_driver.h"
#include "mt6701.h"
#include <array>

struct nvm_config
{
    vmath::vector3d<double> accelBias = {0.0};
    vmath::vector3d<double> gyroBias = {0.0};
    bool calibrated = false;
    bool configured = false;
    //-----------------------------------//
    uint64_t LockCode = 0;
};

inline constexpr auto imu_data_rate = LSM6DSR::DATA_RATE_833HZ;
inline constexpr unsigned int imu_sample_rate_hz = LSM6DSR::DataRateToHz(imu_data_rate);
inline constexpr unsigned int imu_sample_period_us = 1000000 / imu_sample_rate_hz;
inline constexpr unsigned int front_left_line_sensor_index = 0;
inline constexpr unsigned int front_right_line_sensor_index = 1;
inline constexpr unsigned int num_distance_sensors = DISTANCE_SENSOR_COUNT;
inline constexpr unsigned int num_line_sensors = LINE_SENSOR_COUNT;

extern IoInterruptEvent imu_data_ready_event;
extern IoInterruptEvent line_sensor_events[num_line_sensors];


extern PioWS2812 STATUS_Led;

extern LSM6DSR imu;
extern IrReceiver irReceiver;
extern std::array<DistanceSensor, num_distance_sensors> distanceSensors;
extern std::array<LineSensor, num_line_sensors> lineSensors;
extern Graphics graphics;
extern MotorDriver left_motor;
extern MotorDriver right_motor;
extern MT6701 left_encoder;
extern MT6701 right_encoder;

extern nvm_config active_config;



void second_core_main(void);

void hardware_init(void);
/*!
 * @brief Handles critical non-recoverable hardware errors, the program will stay in an error state with some indication of the error.
 * @param code The error code indicating the type of hardware error.
 * @param format A printf-style format string describing the error.
 * @param ... Additional arguments for the format string.
 * @note This function does not return; it will keep the program in an error state.
 */

void calibrate_imu(void);
void calibrate_line_sensor(unsigned int n);
void save_config(bool lockout = true);
void load_config(void);

[[noreturn]] void error_handler(int code, const char *format, ...);

#endif // APP_HARDWARE_H