
#include "config.h"
#include "pico_utils.h"
#include "pio_ws2812.h"
#include "pico_i2c_bus.h"
#include "pio_one_wire_serial.h"
#include "hardware.h"
#include "lsm6dsr.h"
#include "ssd1306.h"
#include "pico_nvm.h"
#include "pico/multicore.h"

template <size_t n, size_t... Is>
auto make_serial_pins_impl(PioOneWireSerial &serial, const std::array<unsigned int, n> &pins, std::index_sequence<Is...>)
{
    return std::array{PioOneWireSerialMuxedPin(serial, pins[Is])...};
}

template <size_t n>
auto make_serial_pins(PioOneWireSerial &serial, const std::array<unsigned int, n> &pins)
{
    return make_serial_pins_impl(serial, pins, std::make_index_sequence<n>{});
}

template <typename SensorT, size_t... Is>
auto make_sensors_impl(std::array<PioOneWireSerialMuxedPin, sizeof...(Is)> &pins, std::index_sequence<Is...>)
{
    return std::array{SensorT(pins[Is])...};
}

template <typename SensorT, size_t n>
auto make_sensors(std::array<PioOneWireSerialMuxedPin, n> &pins)
{
    return make_sensors_impl<SensorT>(pins, std::make_index_sequence<n>{});
}

constexpr auto NVM_CONFG_LOCK_CODE = 0xACE9FBD132E3B907;

constexpr LSM6DSR::config imu_config = {
    .accel_range = LSM6DSR::ACCEL_RANGE_16G,
    .gyro_range = LSM6DSR::GYRO_RANGE_2000DPS,
    .data_rate = imu_data_rate
};
constexpr unsigned int imu_calibration_time_s = 10;

constexpr IrReceiver::config ir_receiver_config = {
    .io_mode = OneWireDevice::DeviceIOMode::IOMODE_SERIAL,
    .serial_baud_rate = OneWireDevice::baudRateToCode(SERIAL_BAUD_RATE)
};

constexpr DistanceSensor::config distance_sensor_config = {
    .io_mode = OneWireDevice::DeviceIOMode::IOMODE_SERIAL,
    .serial_baud_rate = OneWireDevice::baudRateToCode(SERIAL_BAUD_RATE),
    .range_timing_ms = 33,
    .max_range_mm = 400,
    .min_range_mm = 1,
    .detection_mode = DistanceSensor::DetectionMode::DETECTION_MODE_ANY_VALID_RANGE,
    .detection_threshold_low_mm = 100,
    .detection_threshold_high_mm = 200
};

constexpr LineSensor::config line_sensor_config = {
    .io_mode = OneWireDevice::DeviceIOMode::IOMODE_DO,
    .avg_count = 5
};
constexpr unsigned int line_sensor_calibration_time_s = 5;
constexpr double line_sensor_threshold_coeff = 0.4; // Threshold coefficient for line detection, between 0 and 1

constexpr unsigned int ir_receiver_pin = RECEIVER_PIN;

constexpr std::array<unsigned int, num_distance_sensors> distance_sensor_pins = {
    #if ROBOT_VERSION == 1
    DISTANCE_SENSOR_LEFT90_PIN,
    DISTANCE_SENSOR_LEFT45_PIN,
    DISTANCE_SENSOR_LEFT27_PIN,
    DISTANCE_SENSOR_LEFT18_PIN,
    DISTANCE_SENSOR_LEFT0_PIN,
    DISTANCE_SENSOR_CENTER_PIN,
    DISTANCE_SENSOR_RIGHT0_PIN,
    DISTANCE_SENSOR_RIGHT18_PIN,
    DISTANCE_SENSOR_RIGHT27_PIN,
    DISTANCE_SENSOR_RIGHT45_PIN,
    DISTANCE_SENSOR_RIGHT90_PIN,
    #endif
    #if ROBOT_VERSION == 2
    DISTANCE_SENSOR_RIGHT0_PIN,
    DISTANCE_SENSOR_RIGHT45_PIN,
    DISTANCE_SENSOR_LEFT35_PIN,
    DISTANCE_SENSOR_CENTER_PIN,
    DISTANCE_SENSOR_RIGHT35_PIN,
    DISTANCE_SENSOR_LEFT45_PIN,
    DISTANCE_SENSOR_LEFT0_PIN,
    #endif
};

constexpr std::array<unsigned int, num_line_sensors> line_sensor_pins = {
    #if ROBOT_VERSION == 1
    LINE_SENSOR_LEFT_PIN, //left
    LINE_SENSOR_RIGHT_PIN, //right
    //LINE_SENSOR_BACK_PIN //back
    #endif
    #if ROBOT_VERSION == 2
    LINE_SENSOR_LEFT_PIN, //left
    LINE_SENSOR_RIGHT_PIN, //right
    #endif
};

constexpr DistanceSensor::config distance_sensor_configs[num_distance_sensors] = {
    #if ROBOT_VERSION == 1
    distance_sensor_config, //90 left
    distance_sensor_config, //45 left
    distance_sensor_config, //27 left
    distance_sensor_config, //18 left
    distance_sensor_config, //0 left
    distance_sensor_config, //center
    distance_sensor_config, //0 right
    distance_sensor_config, //18 right
    distance_sensor_config, //27 right
    distance_sensor_config, //45 right
    distance_sensor_config, //90 right
    #endif
    #if ROBOT_VERSION == 2
    distance_sensor_config, //0 right
    distance_sensor_config, //45 right
    distance_sensor_config, //35 left
    distance_sensor_config, //center
    distance_sensor_config, //35 right
    distance_sensor_config, //45 left
    distance_sensor_config, //0 left
    #endif
};

constexpr LineSensor::config line_sensor_configs[num_line_sensors] = {
    line_sensor_config, //front left
    line_sensor_config, //front right
    //line_sensor_config, //back
};

PicoI2CBus i2c_bus(I2C_PORT, I2C_SPEED, I2C_SCL, I2C_SDA, true);
PicoI2CBus i2c_bus2(I2C_PORT2, I2C_SPEED, I2C_SCL2, I2C_SDA2, true);
PioOneWireSerial pio_serial(pio0, 0, 1);

PioWS2812 STATUS_Led(pio2, 0, LED_PIN);

auto receiver_serial_pin = PioOneWireSerialMuxedPin(pio_serial, ir_receiver_pin);
auto distance_sensor_serial_pins = make_serial_pins(pio_serial, distance_sensor_pins);
auto line_sensor_serial_pins = make_serial_pins(pio_serial, line_sensor_pins);

IoInterruptEvent imu_data_ready_event(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE);
IoInterruptEvent line_sensor_events[num_line_sensors];

LSM6DSR imu(i2c_bus);
IrReceiver irReceiver = IrReceiver(receiver_serial_pin);
std::array<DistanceSensor, num_distance_sensors> distanceSensors = make_sensors<DistanceSensor>(distance_sensor_serial_pins);
std::array<LineSensor, num_line_sensors> lineSensors = make_sensors<LineSensor>(line_sensor_serial_pins);
SSD1306 lcd(i2c_bus, SSD1306::Resolution::Pixels128x64, SSD1306::Orientation::Inverted);
Graphics graphics = Graphics(lcd);

nvm_config active_config;
NVM nvm_manager(sizeof(nvm_config), &active_config, sizeof(nvm_config));

MotorDriver left_motor(MOTOR_DRIVER_PWMA, MOTOR_DRIVER_DIRA, MOTOR_DRIVER_FREQUENCY, true);
MotorDriver right_motor(MOTOR_DRIVER_PWMB, MOTOR_DRIVER_DIRB, MOTOR_DRIVER_FREQUENCY, false);
MT6701 left_encoder(i2c_bus);
MT6701 right_encoder(i2c_bus2);

bool second_core_launched = false;
bool init_error = false;

void core1_entry(void);
void io_init(void);
void encoders_init(void);
void imu_init(void);
void serial_devices_init(void);
void serial_devices_configure(void);
const char* configure_ir_receiver(IrReceiver &receiver, const IrReceiver::config &config);
const char* configure_distance_sensor(DistanceSensor &sensor, const DistanceSensor::config &config);
const char* configure_line_sensor(LineSensor &sensor, const LineSensor::config &config);
long find_baud_rate(OneWireDevice &device);

inline void sensor_power_enable(bool enable)
{
    gpio_put(SENSORS_ENABLE, enable ? 1 : 0);
}

void hardware_init(void)
{
    io_init();

    utils::init_critical_section();

    load_config();

    STATUS_Led.begin();
    STATUS_Led.setBlocking(Color::None());

    if (!active_config.configured)
        serial_devices_configure();
    serial_devices_init();

    i2c_bus.clearBus(I2C_SCL, I2C_SDA, I2C_SPEED);
    i2c_bus2.clearBus(I2C_SCL2, I2C_SDA2, I2C_SPEED);
    sleep_ms(100);
    
    imu_init();
    lcd.init();
    encoders_init();
    left_motor.begin();
    right_motor.begin();

    if (utils::isDebugEnabled)
    {
        lcd.selfTest();
        utils::sleep_ms(1000); // Wait for 2 seconds to observe the self-test results
    }

    lcd.clear();

    if (init_error)
    {
        error_handler(0, "Initialization error");
    }
    utils::info_printf("Hardware initialization successful\n");

    //Launch the second core.
    utils::info_printf("Launching second core...\n");
    multicore_launch_core1(core1_entry);
}

void core1_entry(void)
{
    utils::info_printf("Second core launched\n");
    multicore_lockout_victim_init();

    second_core_launched = true;
    
    second_core_main();
}

void io_init(void)
{
    gpio_init(DEBUG_VBUS_PIN);
    gpio_set_dir(DEBUG_VBUS_PIN, GPIO_IN);
    gpio_init(START_PIN);
    gpio_set_dir(START_PIN, GPIO_IN);
    gpio_init(MOTOR_DRIVER_ENABLE);
    gpio_set_dir(MOTOR_DRIVER_ENABLE, GPIO_IN);

    gpio_init(SENSORS_ENABLE);
    gpio_set_dir(SENSORS_ENABLE, GPIO_OUT);

    stdio_init_all();

    if (gpio_get(DEBUG_VBUS_PIN))
    {
        utils::debug_enable(true);
        utils::set_log_level(utils::LOG_LEVEL_DEBUG);
        while (!stdio_usb_connected());
        utils::info_printf("Debug VBUS detected, enabling debug messages.\n");
        utils::sleep_ms(1000);
    }
}

void encoders_init(void)
{
    if (!left_encoder.begin())
    {
        utils::error_printf("Failed to initialize MT6701 encoder (left side)\n");
        init_error = true;
    }
    if (!right_encoder.begin())
    {
        utils::error_printf("Failed to initialize MT6701 encoder (right side)\n");
        init_error = true;
    }
    utils::info_printf("Encoders initialization successful\n");
}

void imu_init(void)
{
    utils::info_printf("Initializing IMU...\n");
    if (!imu.begin(imu_config.accel_range, imu_config.gyro_range, imu_config.data_rate, imu_config.data_rate))
    {
        utils::error_printf("Failed to initialize IMU\n");
        init_error = true;
        return;
    }
    if (!imu.setGyroLPF1(true, 0x04)) // Set gyro LPF1 with a cutoff frequency of 96Hz
    {
        utils::error_printf("Failed to set IMU gyro LPF\n");
        init_error = true;
        return;
    }
    if (!imu.setAccelLPF2(true, 0x02)) // Set accel LPF2 with a cutoff frequency of 51Hz
    {
        utils::error_printf("Failed to set IMU accel LPF\n");
        init_error = true;
        return;
    };
    if (!imu.setInterrupts(true, false, &imu_data_ready_event)) // Enable data ready interrupts for accelerometer only
    {
        utils::error_printf("Failed to set IMU interrupts\n");
        init_error = true;
        return;
    }

    utils::info_printf("IMU initialization successful\n");
}

void calibrate_imu(void)
{
    utils::info_printf("Calibrating IMU...\n");
    if (second_core_launched)
        multicore_lockout_start_blocking(); // Prevent core1 from reading IMU data while we're calibrating to avoid using partially updated bias values.
    STATUS_Led.setBlocking(Color::Black());
    vmath::vector3d<double> accelBiasSum = {0.0};
    vmath::vector3d<double> gyroBiasSum = {0.0};
    unsigned int sampleCount = 0;

    STATUS_Led.blinkBlocking(Color(0x00,0xFF,0xFF), Color::Black(), 50, 100, 10);

    utils::time_t startTime = utils::now();
    utils::time_t last_read_time = utils::now();

    while (!utils::hasElapsed_s(startTime, line_sensor_calibration_time_s))
    {
        // If data is available or if it's been too long since the last read (indicating a possible missed data ready signal), attempt to read data
        if (imu.isDataReady() || utils::hasElapsed_us(last_read_time, imu_sample_period_us * 2))
        {
            last_read_time = utils::now();
            vmath::vector3d a_raw;
            vmath::vector3d w_raw;
            if (imu.readData(a_raw, w_raw))
            {
                accelBiasSum += a_raw;
                gyroBiasSum += w_raw;
                sampleCount++;
                STATUS_Led.blinkBlocking(Color::Blue(), Color::Black(), 50, 100, 1);
            }
        }
    }

    STATUS_Led.setBlocking(Color::Blue());

    if (sampleCount > 0)
    {
        active_config.accelBias = accelBiasSum / (double)sampleCount;
        active_config.gyroBias = gyroBiasSum / (double)sampleCount;
        if (active_config.accelBias.Z > 1.0 / 3.0) active_config.accelBias.Z -= 1.0; // Remove gravity from Z axis bias
        else active_config.accelBias.Z += 1.0;
        active_config.calibrated = true;
        utils::info_printf("IMU calibration complete. Samples: %d, Accel Bias: (%.2f, %.2f, %.2f), Gyro Bias: (%.2f, %.2f, %.2f)\n", 
            sampleCount, active_config.accelBias.X, active_config.accelBias.Y, active_config.accelBias.Z, active_config.gyroBias.X, active_config.gyroBias.Y, active_config.gyroBias.Z);
        save_config();
        STATUS_Led.setBlocking(Color::Green());
        utils::info_printf("IMU calibration successful\n");
    }
    else
    {
        utils::error_printf("IMU calibration failed: no data samples collected\n");
        STATUS_Led.setBlocking(Color::Red());
    }

    if (second_core_launched)
        multicore_lockout_end_blocking();
    utils::sleep_ms(500);
    STATUS_Led.setBlocking(Color::Black());
}

void serial_devices_init(void)
{
    utils::info_printf("Initializing sensors...\n");
    pio_serial.end(); // Ensure the serial interface is stopped before configuring pins
    pio_serial.begin(SERIAL_BAUD_RATE); // Start the serial interface at the configured baud rate
    // Ensure all pins are high to let the sensors start in normal mode.
    for (int i = 0; i < num_distance_sensors; i++)
        distance_sensor_serial_pins[i].assertPin(true);
    for (int i = 0; i < num_line_sensors; i++)
        line_sensor_serial_pins[i].assertPin(true);
    receiver_serial_pin.assertPin(true);
    sensor_power_enable(true); // Enable sensors power
    utils::sleep_ms(100); // Wait for sensors to power up
    // Configure line sensor pins with interrupts for line detection.
    for (int i = 0; i < num_line_sensors; i++)
    {
        line_sensor_events[i].configure(line_sensor_pins[i], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    }
    // For the IR receiver, we can wait for it to start responding.
    if (!irReceiver.waitForBoot(500))
    {
        utils::error_printf("IR receiver failed to boot or is not responding\n");
        init_error = true;
        return;
    }
    // Restart the IR receiver to clear any potential garbage data in its FIFO and ensure it's ready to receive commands.
    if (!irReceiver.restartDevice())
    {
        utils::error_printf("Failed to restart IR receiver\n");
        init_error = true;
        return;
    }
    utils::info_printf("Sensor initialization successful\n");
}

void serial_devices_configure(void)
{
    utils::info_printf("Configuring sensors...\n");
    pio_serial.end(); // Ensure the serial interface is stopped before configuring pins
    pio_serial.begin(DEFAULT_SERIAL_BAUD_RATE); // Start the serial interface at the default baud rate for configuration
    utils::info_printf("Entering configuration mode...\n");
    sensor_power_enable(false); // Power off sensors
    utils::sleep_ms(500); // Wait for sensors to power down
    //Assert all pins low to enter configuration mode.
    for (int i = 0; i < num_distance_sensors; i++)
        distance_sensor_serial_pins[i].assertPin(false);
    for (int i = 0; i < num_line_sensors; i++)
        line_sensor_serial_pins[i].assertPin(false);
    receiver_serial_pin.assertPin(false);
    utils::sleep_ms(100);
    sensor_power_enable(true);
    utils::sleep_ms(500); // Wait for sensors to power up
#if USE_DISTANCE_SENSORS
    utils::info_printf("Configuring distance sensors...\n");
    for (int i = 0; i < num_distance_sensors; i++)
    {
        if (const char* error_msg = configure_distance_sensor(distanceSensors[i], distance_sensor_configs[i]); error_msg != nullptr)
        {
            utils::error_printf("Distance sensor %d configuration failed: %s\n", i, error_msg);
            init_error = true;
        }
    }
#endif
#if USE_LINE_SENSORS
    utils::info_printf("Configuring line sensors...\n");
    for (int i = 0; i < num_line_sensors; i++)
    {
        if (const char* error_msg = configure_line_sensor(lineSensors[i], line_sensor_configs[i]); error_msg != nullptr)
        {
            utils::error_printf("Line sensor %d configuration failed: %s\n", i, error_msg);
            init_error = true;
        }
        // Configure line sensor pins as inputs with interrupts.
        line_sensor_serial_pins[i].assertPin(true); // Set the sensor pin high to exit configuration mode and release pin from serial control
    }
#endif
#if USE_IR_RECEIVER
    utils::info_printf("Configuring IR receiver...\n");
    const char* error = configure_ir_receiver(irReceiver, ir_receiver_config);
    if (error)
    {
        utils::error_printf("IR receiver configuration error: %s\n", error);
        init_error = true;
    }
#endif

    active_config.configured = true;
    save_config();
    utils::info_printf("Sensor configuration complete\n");
}

const char* configure_ir_receiver(IrReceiver &receiver, const IrReceiver::config &config)
{
    auto baud = find_baud_rate(receiver);
    if (baud == -1)
    {
        return "Failed to determine baud rate";
    }
    pio_serial.begin(baud);
    if (!receiver.setIOMode(config.io_mode))
    {
        return "Failed to set I/O mode";
    }
    if (!receiver.setSerialBaudRate(config.serial_baud_rate))
    {
        return "Failed to set serial baud rate";
    }
    if (!receiver.saveConfiguration())
    {
        return "Failed to save configuration";
    }
    utils::sleep_ms(500); //The IR receiver needs to be restarted to apply the new configuration, and it may take a short time to restart, so wait before trying to restart the device.
    if (!receiver.restartDevice())
    {
        return "Failed to restart device";
    }
    pio_serial.begin(DEFAULT_SERIAL_BAUD_RATE);
    return nullptr;
}

const char* configure_distance_sensor(DistanceSensor &sensor, const DistanceSensor::config &config)
{
    if (!sensor.setIOMode(config.io_mode))
    {
        return "Failed to set I/O mode";
    }
    if (!sensor.setSerialBaudRate(config.serial_baud_rate))
    {
        return "Failed to set serial baud rate";
    }
    if (!sensor.setRangeTiming(config.range_timing_ms))
    {
        return "Failed to set range timing";
    }
    if (!sensor.setRangeMin(config.min_range_mm))
    {
        return "Failed to set minimum range";
    }
    if (!sensor.setRangeMax(config.max_range_mm))
    {
        return "Failed to set maximum range";
    }
    if (!sensor.setDetectionMode(config.detection_mode))
    {
        return "Failed to set detection mode";
    }
    if (!sensor.setDetectionLowerThreshold(config.detection_threshold_low_mm))
    {
        return "Failed to set detection lower threshold";
    }
    if (!sensor.setDetectionUpperThreshold(config.detection_threshold_high_mm))
    {
        return "Failed to set detection upper threshold";
    }
    if (!sensor.saveConfiguration())
    {
        return "Failed to save configuration";
    }
    return nullptr;
}

const char* configure_line_sensor(LineSensor &sensor, const LineSensor::config &config)
{
    if (!sensor.setIOMode(config.io_mode))
    {
        return "Failed to set I/O mode";
    }
    if (!sensor.setAveragingCount(config.avg_count))
    {
        return "Failed to set average count";
    }
    if (!sensor.saveConfiguration())
    {
        return "Failed to save configuration";
    }
    utils::sleep_ms(500); //The line sensor needs to be restarted to apply the new configuration, and it may take a short time to restart, so wait before trying to restart the device.
    if (!sensor.restartDevice())
    {
        return "Failed to restart device";
    }
    return nullptr;
}

long find_baud_rate(OneWireDevice &device)
{
    int imin = static_cast<int>(OneWireDevice::DeviceSerialBaudRate::SERIAL_BAUD_115200);
    int imax = static_cast<int>(OneWireDevice::DeviceSerialBaudRate::SERIAL_BAUD_250000);
    for (int i = imin; i <= imax; i++)
    {
        auto code = static_cast<OneWireDevice::DeviceSerialBaudRate>(i);
        auto baud = OneWireDevice::baudRateFromCode(code);
        pio_serial.begin(baud);
        if (device.waitForBoot(500))
        {
            return baud;
        }
    }
    return -1;
}

void calibrate_line_sensor(unsigned int n)
{
    bool error = false;
    utils::info_printf("Calibrating line sensor %d...\n", n);
    STATUS_Led.setBlocking(Color::Black());
    if (n >= num_line_sensors)
    {
        utils::error_printf("Invalid line sensor index %d for calibration\n", n);
        return;
    }
    // Release the pin from the interrupt so it can be used by the serial interface
    line_sensor_events[n].deconfigure();
    line_sensor_serial_pins[n].getSerial().begin(DEFAULT_SERIAL_BAUD_RATE); // Ensure the serial interface is at the default baud rate for calibration
    sensor_power_enable(false);
    utils::sleep_ms(500);
    line_sensor_serial_pins[n].assertPin(false);
    utils::sleep_ms(100); // Wait for pins to be driven low
    sensor_power_enable(true);
    utils::sleep_ms(500); // Wait for sensors to power up
    line_sensor_serial_pins[n].assertPin(true);
    if (!lineSensors[n].waitForBoot(500))
    {
        utils::error_printf("Line sensor %d failed to boot for calibration\n", n);
        line_sensor_serial_pins[n].getSerial().begin(SERIAL_BAUD_RATE);
        line_sensor_events[n].reconfigure();
        return;
    }

    STATUS_Led.blinkBlocking(Color(0x00,0xFF,0xFF), Color::Black(), 50, 100, 10);

    unsigned int max = 0;
    unsigned int min = 0xFFFFFFFF;
    unsigned int sampleCount = 0;

    utils::time_t startTime = utils::now();
    while (!utils::hasElapsed_s(startTime, line_sensor_calibration_time_s))
    {
        unsigned int value;
        if (!lineSensors[n].getTotalAverage(value))
        {
            continue; // Skip if failed to read sensor data, don't count it as a sample
        }
        if (value > max) max = value;
        if (value < min) min = value;
        sampleCount++;
        STATUS_Led.blinkBlocking(Color::Blue(), Color::Black(), 50, 100, 1);
    }

    STATUS_Led.setBlocking(Color::Blue());

    if (sampleCount < line_sensor_calibration_time_s * 1000 / 150 / 2) // Require at least half the expected samples
    {
        utils::error_printf("Line sensor %d calibration failed: insufficient valid samples collected\n", n);
        error = true;
    }

    unsigned int threshold = (max - min) * line_sensor_threshold_coeff + min;

    utils::debug_printf("Line sensor %d calibration complete. Min: %d, Max: %d, Threshold: %d\n", n, min, max, threshold);
    if (!lineSensors[n].setDetectionUpperThreshold(threshold))
    {
        utils::error_printf("Failed to set line sensor %d upper detection threshold\n", n);
        error = true;
    }
    utils::sleep_ms(10);
    if (!lineSensors[n].setDetectionLowerThreshold(threshold))
    {
        utils::error_printf("Failed to set line sensor %d lower detection threshold\n", n);
        error = true;
    }
    utils::sleep_ms(10);
    if (!lineSensors[n].saveConfiguration())
    {
        utils::error_printf("Failed to save line sensor %d configuration after calibration\n", n);
        error = true;
    }
    utils::sleep_ms(50);
    lineSensors[n].restartDevice();
    utils::sleep_ms(500);
    line_sensor_serial_pins[n].getSerial().begin(SERIAL_BAUD_RATE); //return to normal baud rate
    line_sensor_serial_pins[n].assertPin(true); // Ensure the pin is released from serial control
    // Reinitialize the line sensor pin as an input with interrupts for normal operation
    line_sensor_events[n].reconfigure();

    if (error)
    {
        STATUS_Led.setBlocking(Color::Red());
        utils::error_printf("Line sensor %d calibration encountered errors\n", n);
    }
    else
    {
        STATUS_Led.setBlocking(Color::Green());
        utils::info_printf("Line sensor %d calibration successful\n", n);
    }
    utils::sleep_ms(500);
    STATUS_Led.setBlocking(Color::Black());
}

void save_config(bool lockout)
{
    if (lockout && second_core_launched) {
        multicore_lockout_start_blocking();
    }
    active_config.LockCode = NVM_CONFG_LOCK_CODE;
    nvm_manager.save();
    if (lockout && second_core_launched) {
        multicore_lockout_end_blocking();
    }
}

void load_config(void)
{
    nvm_manager.load();
    if (active_config.LockCode != NVM_CONFG_LOCK_CODE) {
        utils::info_printf("No valid config found in NVM, loading defaults\n");
        active_config = {};
    }
    utils::info_printf("Config loaded.\n");
    utils::debug_printf("Accel Bias: (%.2f, %.2f, %.2f), Gyro Bias: (%.2f, %.2f, %.2f)\n", active_config.accelBias.X, active_config.accelBias.Y, active_config.accelBias.Z, active_config.gyroBias.X, active_config.gyroBias.Y, active_config.gyroBias.Z);
    utils::debug_printf("Calibrated: %s, Configured: %s\n", active_config.calibrated ? "true" : "false", active_config.configured ? "true" : "false");
}

[[noreturn]] void error_handler(int code, const char *format, ...)
{
    char message[256];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    STATUS_Led.setBlocking(Color::Red());
    while (true)
    {
        utils::error_printf("ERROR[%d]: ", code);
        utils::error_printf("%s\n", message);
        utils::sleep_ms(1000);
    }
}
