
#ifndef LSM6DSR_H
#define LSM6DSR_H

#include <atomic>
#include "imu.h"
#include "event.h"
#include "i2c_bus.h"
#include "pico_utils.h"

/*! @brief LSM6DSR IMU sensor driver.
 * This class provides an interface to the LSM6DSR IMU sensor, allowing for initialization and data reading of accelerometer and gyroscope measurements. It uses an I2C bus for communication and implements the IMU interface defined in imu.h.
 */
class LSM6DSR : public IMU
{
    static constexpr uint8_t LSM6DSR_ADDRESS = 0x6A;
    static constexpr int LSM6DSR_INIT_DELAY_MS = 100;

    static constexpr uint8_t LSM6DSR_WHO_AM_I_REG = 0x0F;
    static constexpr uint8_t LSM6DSR_WHO_AM_I_RESPONSE = 0x6B;

    static constexpr uint8_t LSM6DSR_RESET_REG = 0x12;
    static constexpr uint8_t LSM6DSR_RESET_BIT = 0x01;

    static constexpr uint8_t LSM6DSR_INT1_DRDY_XL = 0x01;
    static constexpr uint8_t LSM6DSR_INT1_DRDY_G = 0x02;

    static constexpr uint8_t LSM6DSR_INT1_CTRL = 0x0D;
    static constexpr uint8_t LSM6DSR_CTRL1_XL = 0x10;
    static constexpr uint8_t LSM6DSR_CTRL2_G = 0x11;
    static constexpr uint8_t LSM6DSR_CTRL3_C = 0x12;
    static constexpr uint8_t LSM6DSR_CTRL4_C = 0x13;
    static constexpr uint8_t LSM6DSR_CTRL5_C = 0x14;
    static constexpr uint8_t LSM6DSR_CTRL6_C = 0x15;
    static constexpr uint8_t LSM6DSR_CTRL7_G = 0x16;
    static constexpr uint8_t LSM6DSR_CTRL8_XL = 0x17;
    static constexpr uint8_t LSM6DSR_STATUS_REG = 0x1E;
    static constexpr uint8_t LSM6DSR_OUT_TEMP_L = 0x20;
    static constexpr uint8_t LSM6DSR_OUT_TEMP_H = 0x21;
    static constexpr uint8_t LSM6DSR_OUTX_L_G = 0x22;
    static constexpr uint8_t LSM6DSR_OUTX_H_G = 0x23;
    static constexpr uint8_t LSM6DSR_OUTY_L_G = 0x24;
    static constexpr uint8_t LSM6DSR_OUTY_H_G = 0x25;
    static constexpr uint8_t LSM6DSR_OUTZ_L_G = 0x26;
    static constexpr uint8_t LSM6DSR_OUTZ_H_G = 0x27;
    static constexpr uint8_t LSM6DSR_OUTX_L_A = 0x28;
    static constexpr uint8_t LSM6DSR_OUTX_H_A = 0x29;
    static constexpr uint8_t LSM6DSR_OUTY_L_A = 0x2A;
    static constexpr uint8_t LSM6DSR_OUTY_H_A = 0x2B;
    static constexpr uint8_t LSM6DSR_OUTZ_L_A = 0x2C;
    static constexpr uint8_t LSM6DSR_OUTZ_H_A = 0x2D;

    static constexpr uint8_t LSM6DSR_ODR_OFF = 0x00;
    static constexpr uint8_t LSM6DSR_ODR_1P6HZ = 0xB0;
    static constexpr uint8_t LSM6DSR_ODR_12P5HZ = 0x10;
    static constexpr uint8_t LSM6DSR_ODR_26HZ = 0x20;
    static constexpr uint8_t LSM6DSR_ODR_52HZ = 0x30;
    static constexpr uint8_t LSM6DSR_ODR_104HZ = 0x40;
    static constexpr uint8_t LSM6DSR_ODR_208HZ = 0x50;
    static constexpr uint8_t LSM6DSR_ODR_416HZ = 0x60;
    static constexpr uint8_t LSM6DSR_ODR_833HZ = 0x70;
    static constexpr uint8_t LSM6DSR_ODR_1P66KHZ = 0x80;
    static constexpr uint8_t LSM6DSR_ODR_3P33KHZ = 0x90;
    static constexpr uint8_t LSM6DSR_ODR_6P66KHZ = 0xA0;

    static constexpr uint8_t LSM6DSR_ACCEL_FS_2G = 0x00;
    static constexpr uint8_t LSM6DSR_ACCEL_FS_4G = 0x08;
    static constexpr uint8_t LSM6DSR_ACCEL_FS_8G = 0x0C;
    static constexpr uint8_t LSM6DSR_ACCEL_FS_16G = 0x04;

    static constexpr uint8_t LSM6DSR_GYRO_FS_125DPS = 0x02;
    static constexpr uint8_t LSM6DSR_GYRO_FS_250DPS = 0x00;
    static constexpr uint8_t LSM6DSR_GYRO_FS_500DPS = 0x04;
    static constexpr uint8_t LSM6DSR_GYRO_FS_1000DPS = 0x08;
    static constexpr uint8_t LSM6DSR_GYRO_FS_2000DPS = 0x0C;
    static constexpr uint8_t LSM6DSR_GYRO_FS_4000DPS = 0x01;

    static constexpr double LSM6DSR_ACCEL_FS2G_MG_PER_LSB = 0.061;
    static constexpr double LSM6DSR_ACCEL_FS4G_MG_PER_LSB = 0.122;
    static constexpr double LSM6DSR_ACCEL_FS8G_MG_PER_LSB = 0.244;
    static constexpr double LSM6DSR_ACCEL_FS16G_MG_PER_LSB = 0.488;

    static constexpr double LSM6DSR_ACCEL_FS2G_G_PER_LSB = LSM6DSR_ACCEL_FS2G_MG_PER_LSB / 1000.0;
    static constexpr double LSM6DSR_ACCEL_FS4G_G_PER_LSB = LSM6DSR_ACCEL_FS4G_MG_PER_LSB / 1000.0;
    static constexpr double LSM6DSR_ACCEL_FS8G_G_PER_LSB = LSM6DSR_ACCEL_FS8G_MG_PER_LSB / 1000.0;
    static constexpr double LSM6DSR_ACCEL_FS16G_G_PER_LSB = LSM6DSR_ACCEL_FS16G_MG_PER_LSB / 1000.0;

    static constexpr double LSM6DSR_GYRO_FS125_MDPS_PER_LSB = 4.375;
    static constexpr double LSM6DSR_GYRO_FS250_MDPS_PER_LSB = 8.75;
    static constexpr double LSM6DSR_GYRO_FS500_MDPS_PER_LSB = 17.50;
    static constexpr double LSM6DSR_GYRO_FS1000_MDPS_PER_LSB = 35.0;
    static constexpr double LSM6DSR_GYRO_FS2000_MDPS_PER_LSB = 70.0;
    static constexpr double LSM6DSR_GYRO_FS4000_MDPS_PER_LSB = 140.0;

    static constexpr double LSM6DSR_GYRO_FS125_DPS_PER_LSB = LSM6DSR_GYRO_FS125_MDPS_PER_LSB / 1000.0;
    static constexpr double LSM6DSR_GYRO_FS250_DPS_PER_LSB = LSM6DSR_GYRO_FS250_MDPS_PER_LSB / 1000.0;
    static constexpr double LSM6DSR_GYRO_FS500_DPS_PER_LSB = LSM6DSR_GYRO_FS500_MDPS_PER_LSB / 1000.0;
    static constexpr double LSM6DSR_GYRO_FS1000_DPS_PER_LSB = LSM6DSR_GYRO_FS1000_MDPS_PER_LSB / 1000.0;
    static constexpr double LSM6DSR_GYRO_FS2000_DPS_PER_LSB = LSM6DSR_GYRO_FS2000_MDPS_PER_LSB / 1000.0;
    static constexpr double LSM6DSR_GYRO_FS4000_DPS_PER_LSB = LSM6DSR_GYRO_FS4000_MDPS_PER_LSB / 1000.0;

    static constexpr double LSM6DSR_TEMP_DEG_C_PER_LSB = 256.0;
    static constexpr double LSM6DSR_TEMP_OFFSET_DEG_C = 25.0;

    I2CBus &i2c;
    double accelScaleFactor = 0; //g per LSB
    double gyroScaleFactor = 0; //rad/s per LSB

    Event *dataReadyEvent;
    std::atomic<bool> dataReady = false;
public:
    static constexpr double LSM6DSR_GYRO_NOISE_MDPS_PER_ROOTHZ = 5.0;
    static constexpr double LSM6DSR_GYRO_NOISE_DPS_PER_ROOTHZ = LSM6DSR_GYRO_NOISE_MDPS_PER_ROOTHZ / 1000.0;

    enum AccelerometerRange
    {
        ACCEL_RANGE_2G = LSM6DSR_ACCEL_FS_2G,
        ACCEL_RANGE_4G = LSM6DSR_ACCEL_FS_4G,
        ACCEL_RANGE_8G = LSM6DSR_ACCEL_FS_8G,
        ACCEL_RANGE_16G = LSM6DSR_ACCEL_FS_16G
    };

    enum GyroscopeRange
    {
        GYRO_RANGE_125DPS = LSM6DSR_GYRO_FS_125DPS,
        GYRO_RANGE_250DPS = LSM6DSR_GYRO_FS_250DPS,
        GYRO_RANGE_500DPS = LSM6DSR_GYRO_FS_500DPS,
        GYRO_RANGE_1000DPS = LSM6DSR_GYRO_FS_1000DPS,
        GYRO_RANGE_2000DPS = LSM6DSR_GYRO_FS_2000DPS,
        GYRO_RANGE_4000DPS = LSM6DSR_GYRO_FS_4000DPS
    };

    enum DataRate
    {
        DATA_RATE_OFF = LSM6DSR_ODR_OFF,
        DATA_RATE_1P6HZ = LSM6DSR_ODR_1P6HZ,
        DATA_RATE_12P5HZ = LSM6DSR_ODR_12P5HZ,
        DATA_RATE_26HZ = LSM6DSR_ODR_26HZ,
        DATA_RATE_52HZ = LSM6DSR_ODR_52HZ,
        DATA_RATE_104HZ = LSM6DSR_ODR_104HZ,
        DATA_RATE_208HZ = LSM6DSR_ODR_208HZ,
        DATA_RATE_416HZ = LSM6DSR_ODR_416HZ,
        DATA_RATE_833HZ = LSM6DSR_ODR_833HZ,
        DATA_RATE_1P66KHZ = LSM6DSR_ODR_1P66KHZ,
        DATA_RATE_3P33KHZ = LSM6DSR_ODR_3P33KHZ,
        DATA_RATE_6P66KHZ = LSM6DSR_ODR_6P66KHZ
    };

    struct config
    {
        AccelerometerRange accel_range;
        GyroscopeRange gyro_range;
        DataRate data_rate;
    };

    static constexpr uint DataRateToHz(DataRate rate)
    {
        switch (rate)
        {
            case DATA_RATE_OFF: return 0;
            case DATA_RATE_1P6HZ: return 1;
            case DATA_RATE_12P5HZ: return 12;
            case DATA_RATE_26HZ: return 26;
            case DATA_RATE_52HZ: return 52;
            case DATA_RATE_104HZ: return 104;
            case DATA_RATE_208HZ: return 208;
            case DATA_RATE_416HZ: return 416;
            case DATA_RATE_833HZ: return 833;
            case DATA_RATE_1P66KHZ: return 1660;
            case DATA_RATE_3P33KHZ: return 3330;
            case DATA_RATE_6P66KHZ: return 6660;
            default: return 0; // Invalid data rate
        }
    }

    /*! @brief Constructs an LSM6DSR object with the given I2C bus.
     *  @param i2c The I2C bus to use for communication with the sensor.
     */
    LSM6DSR(I2CBus &i2c) : i2c(i2c), dataReadyEvent(nullptr)
    {
        
        
    }

    /*! @brief Initializes the LSM6DSR sensor with the specified ranges and data rates.
     *  @param accelRange The accelerometer range to use.
     *  @param gyroRange The gyroscope range to use.
     *  @param accelRate The accelerometer data rate to use.
     *  @param gyroRate The gyroscope data rate to use.
     *  @return True if the sensor was successfully initialized, false otherwise.
     */
    bool begin(AccelerometerRange accelRange = ACCEL_RANGE_2G, GyroscopeRange gyroRange = GYRO_RANGE_250DPS, DataRate accelRate = DATA_RATE_OFF, DataRate gyroRate = DATA_RATE_OFF)
    {
        uint8_t whoami = 0;
        utils::time_t start = utils::now();

        while (!whoAmI(whoami))
        {
            if (utils::hasElapsed_ms(start, LSM6DSR_INIT_DELAY_MS))
            {
                utils::error_printf("LSM6DSR: Failed to read WHO_AM_I register within the initialization delay %u ms.\n", LSM6DSR_INIT_DELAY_MS);
                return false;
            }
            utils::sleep_ms(10);
        }

        if (!reset())
        {
            utils::error_printf("LSM6DSR: Failed to reset.\n");
            return false;
        }

        start = utils::now();
        while (!whoAmI(whoami))
        {
            if (utils::hasElapsed_ms(start, LSM6DSR_INIT_DELAY_MS))
            {
                utils::error_printf("LSM6DSR: Failed to read WHO_AM_I register within the initialization delay after reset %u ms.\n", LSM6DSR_INIT_DELAY_MS);
                return false;
            }
            utils::sleep_ms(10);
        }

        if (whoami != LSM6DSR_WHO_AM_I_RESPONSE)
        {
            utils::error_printf("LSM6DSR: WHO_AM_I register returned unexpected value: 0x%02X, should be 0x%02X\n", whoami, LSM6DSR_WHO_AM_I_RESPONSE);
            return false;
        }

        if (!setAccelerometerRange(accelRange))
        {
            utils::error_printf("LSM6DSR: Failed to set accelerometer range.\n");
            return false;
        }

        if (!setGyroscopeRange(gyroRange))
        {
            utils::error_printf("LSM6DSR: Failed to set gyroscope range.\n");
            return false;
        }

        if (!setDataRate(accelRate, gyroRate))
        {
            utils::error_printf("LSM6DSR: Failed to set data rate.\n");
            return false;
        }

        return true;
    }

    /*! @brief Reads the accelerometer and gyroscope data from the LSM6DSR sensor.
     *  @param acceleration A reference to a vector3d object to store the accelerometer data. The values are in g (gravitational acceleration).
     *  @param angularRate A reference to a vector3d object to store the gyroscope data. The values are in radians per second (rad/s).
     *  @return True if the data was successfully read, false otherwise.
     */
    bool readData(vmath::vector3d<double>& acceleration, vmath::vector3d<double>& angularRate) override
    {
        constexpr int offset = LSM6DSR_STATUS_REG;
        uint8_t buffer[LSM6DSR_OUTZ_H_A - LSM6DSR_STATUS_REG + 1];
        if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_STATUS_REG, buffer, sizeof(buffer)))
        {
            utils::error_printf("LSM6DSR: Failed to read data from sensor.\n");
            return false;
        }

        if (dataReadyEvent != nullptr)
        {
            dataReady = false; // Clear the data ready flag, since we are now reading the new data. The next time new data is ready, the interrupt will set this flag again.
        }

        /* Uncomment if temperature data is needed
            uint8_t status = buffer[0];
            double temperature = LSM6DSR_TEMP_DEG_C_PER_LSB * ((int16_t)((buffer[LSM6DSR_OUT_TEMP_H - offset] << 8) | buffer[LSM6DSR_OUT_TEMP_L - offset])) + LSM6DSR_TEMP_OFFSET_DEG_C;
        */
        angularRate.X = gyroScaleFactor * ((int16_t)((buffer[LSM6DSR_OUTX_H_G - offset] << 8) | buffer[LSM6DSR_OUTX_L_G - offset]));
        angularRate.Y = gyroScaleFactor * ((int16_t)((buffer[LSM6DSR_OUTY_H_G - offset] << 8) | buffer[LSM6DSR_OUTY_L_G - offset]));
        angularRate.Z = gyroScaleFactor * ((int16_t)((buffer[LSM6DSR_OUTZ_H_G - offset] << 8) | buffer[LSM6DSR_OUTZ_L_G - offset]));

        acceleration.X = accelScaleFactor * ((int16_t)((buffer[LSM6DSR_OUTX_H_A - offset] << 8) | buffer[LSM6DSR_OUTX_L_A - offset]));
        acceleration.Y = accelScaleFactor * ((int16_t)((buffer[LSM6DSR_OUTY_H_A - offset] << 8) | buffer[LSM6DSR_OUTY_L_A - offset]));
        acceleration.Z = accelScaleFactor * ((int16_t)((buffer[LSM6DSR_OUTZ_H_A - offset] << 8) | buffer[LSM6DSR_OUTZ_L_A - offset]));

        return true;
    }

    /*! @brief Reads the WHO_AM_I register from the LSM6DSR sensor.
     *  @param whoami A reference to a variable to store the WHO_AM_I response.
     *  @return True if the WHO_AM_I register was successfully read, false otherwise.
     */
    bool whoAmI(uint8_t &whoami) const
    {
        if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_WHO_AM_I_REG, &whoami, 1))
        {
            utils::error_printf("LSM6DSR: Failed to read WHO_AM_I register.\n");
            return false;
        }
        return true;
    }

    /*! @brief Sets the accelerometer range of the LSM6DSR sensor.
     *  @param range The accelerometer range to set.
     *  @return True if the accelerometer range was successfully set, false otherwise.
     */
    bool setAccelerometerRange(AccelerometerRange range)
    {
        uint8_t ctrl1xl = 0;
        if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL1_XL, &ctrl1xl, 1))
        {
            utils::error_printf("LSM6DSR: Failed to read CTRL1_XL register.\n");
            return false;
        }

        ctrl1xl &= ~0x0C; // Clear FS bits
        ctrl1xl |= range;

        if (!i2c.writeRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL1_XL, &ctrl1xl, 1))
        {
            utils::error_printf("LSM6DSR: Failed to write CTRL1_XL register.\n");
            return false;
        }

        switch (range)
        {
        case ACCEL_RANGE_2G:
            accelScaleFactor = LSM6DSR_ACCEL_FS2G_G_PER_LSB;
            break;
        case ACCEL_RANGE_4G:
            accelScaleFactor = LSM6DSR_ACCEL_FS4G_G_PER_LSB;
            break;
        case ACCEL_RANGE_8G:
            accelScaleFactor = LSM6DSR_ACCEL_FS8G_G_PER_LSB;
            break;
        case ACCEL_RANGE_16G:
            accelScaleFactor = LSM6DSR_ACCEL_FS16G_G_PER_LSB;
            break;
        default:
            accelScaleFactor = 1.0;
            utils::error_printf("LSM6DSR: Unknown accelerometer range.\n");
            break;
        }

        return true;
    }

    /*! @brief Sets the gyroscope range of the LSM6DSR sensor.
     *  @param range The gyroscope range to set.
     *  @return True if the gyroscope range was successfully set, false otherwise.
     */
    bool setGyroscopeRange(GyroscopeRange range)
    {
        uint8_t ctrl2g = 0;
        if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL2_G, &ctrl2g, 1))
        {
            utils::error_printf("LSM6DSR: Failed to read CTRL2_G register.\n");
            return false;
        }

        ctrl2g &= ~0x0F; // Clear FS bits
        ctrl2g |= range;

        if (!i2c.writeRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL2_G, &ctrl2g, 1))
        {
            utils::error_printf("LSM6DSR: Failed to write CTRL2_G register.\n");
            return false;
        }

        switch (range)
        {
        case GYRO_RANGE_125DPS:
            gyroScaleFactor = LSM6DSR_GYRO_FS125_DPS_PER_LSB * (M_PI / 180.0);
            break;
        case GYRO_RANGE_250DPS:
            gyroScaleFactor = LSM6DSR_GYRO_FS250_DPS_PER_LSB * (M_PI / 180.0);
            break;
        case GYRO_RANGE_500DPS:
            gyroScaleFactor = LSM6DSR_GYRO_FS500_DPS_PER_LSB * (M_PI / 180.0);
            break;
        case GYRO_RANGE_1000DPS:
            gyroScaleFactor = LSM6DSR_GYRO_FS1000_DPS_PER_LSB * (M_PI / 180.0);
            break;
        case GYRO_RANGE_2000DPS:
            gyroScaleFactor = LSM6DSR_GYRO_FS2000_DPS_PER_LSB * (M_PI / 180.0);
            break;
        case GYRO_RANGE_4000DPS:
            gyroScaleFactor = LSM6DSR_GYRO_FS4000_DPS_PER_LSB * (M_PI / 180.0);
            break;
        default:
            gyroScaleFactor = 0.0;
            utils::error_printf("LSM6DSR: Unknown gyroscope range.\n");
            break;
        }

        return true;
    }

    /*! @brief Sets the data rate of the accelerometer and gyroscope.
     *  @param accelRate The accelerometer data rate to set.
     *  @param gyroRate The gyroscope data rate to set.
     *  @return True if the data rate was successfully set, false otherwise.
     */
    bool setDataRate(DataRate accelRate, DataRate gyroRate)
    {
        uint8_t ctrl1xl = 0;
        if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL1_XL, &ctrl1xl, 1))
        {
            utils::error_printf("LSM6DSR: Failed to read CTRL1_XL register.\n");
            return false;
        }

        ctrl1xl &= ~0xF0; // Clear ODR bits
        ctrl1xl |= accelRate;

        if (!i2c.writeRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL1_XL, &ctrl1xl, 1))
        {
            utils::error_printf("LSM6DSR: Failed to write CTRL1_XL register.\n");
            return false;
        }

        uint8_t ctrl2g = 0;
        if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL2_G, &ctrl2g, 1))
        {
            utils::error_printf("LSM6DSR: Failed to read CTRL2_G register.\n");
            return false;
        }

        ctrl2g &= ~0xF0; // Clear ODR bits
        ctrl2g |= gyroRate;

        if (!i2c.writeRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL2_G, &ctrl2g, 1))
        {
            utils::error_printf("LSM6DSR: Failed to write CTRL2_G register.\n");
            return false;
        }

        return true;
    }

    /*! @brief Sets the gyroscope low-pass filter 1 (LPF1) configuration.
     *  @param enabled True to enable LPF1, false to disable.
     *  @param FTYPE The filter type to set (0-7).
     *  @return True if the LPF1 configuration was successfully set, false otherwise.
     */
    bool setGyroLPF1(bool enabled, uint8_t FTYPE)
    {
        uint8_t ctrl4c = 0;
        if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL4_C, &ctrl4c, 1))
        {
            utils::error_printf("LSM6DSR: Failed to read CTRL4_C register.\n");
            return false;
        }

        if (enabled)
        {
            ctrl4c |= 0x02; // Set GYRO_LPF1_EN bit
        }
        else
        {
            ctrl4c &= ~0x02; // Clear GYRO_LPF1_EN bit
        }

        if (!i2c.writeRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL4_C, &ctrl4c, 1))
        {
            utils::error_printf("LSM6DSR: Failed to write CTRL4_C register.\n");
            return false;
        }

        uint8_t ctrl6c = 0;
        if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL6_C, &ctrl6c, 1))
        {
            utils::error_printf("LSM6DSR: Failed to read CTRL6_C register.\n");
            return false;
        }

        ctrl6c &= ~0x07; // Clear GYRO_LPF1_FTYPE bits
        ctrl6c |= (FTYPE & 0x07);

        if (!i2c.writeRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL6_C, &ctrl6c, 1))
        {
            utils::error_printf("LSM6DSR: Failed to write CTRL6_C register.\n");
            return false;
        }

        return true;
    }

    /*! @brief Sets the accelerometer low-pass filter 2 (LPF2) configuration.
     *  @param enabled True to enable LPF2, false to disable.
     *  @param FTYPE The filter type to set (0-7).
     *  @return True if the LPF2 configuration was successfully set, false otherwise.
     */
    bool setAccelLPF2(bool enabled, uint8_t FTYPE)
    {
        uint8_t ctrl1xl = 0;
        if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL1_XL, &ctrl1xl, 1))
        {
            utils::error_printf("LSM6DSR: Failed to read CTRL1_XL register.\n");
            return false;
        }

        if (enabled)
        {
            ctrl1xl |= 0x02; // Set ACCEL_LPF2_EN bit
        }
        else
        {
            ctrl1xl &= ~0x02; // Clear ACCEL_LPF2_EN bit
        }

        if (!i2c.writeRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL1_XL, &ctrl1xl, 1))
        {
            utils::error_printf("LSM6DSR: Failed to write CTRL1_XL register.\n");
            return false;
        }

        uint8_t ctrl8xl = 0;
        if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL8_XL, &ctrl8xl, 1))
        {
            utils::error_printf("LSM6DSR: Failed to read CTRL8_XL register.\n");
            return false;
        }

        ctrl8xl &= ~0xE0; // Clear XL_LPF2_FTYPE bits
        ctrl8xl |= (FTYPE & 0x07) << 5;

        if (!i2c.writeRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL8_XL, &ctrl8xl, 1))
        {
            utils::error_printf("LSM6DSR: Failed to write CTRL8_XL register.\n");
            return false;
        }

        return true;
    }

    /*! @brief Sets the interrupt configuration for the accelerometer and gyroscope data ready signals.
     *  @param accelDataReadyEnable True to enable accelerometer data ready interrupt, false to disable.
     *  @param gyroDataReadyEnable True to enable gyroscope data ready interrupt, false to disable.
     *  @param dataReadyEvent An optional pointer to an event object that will be signaled when new data is ready. If nullptr, no event will be used.
     *  @return True if the interrupt configuration was successfully set, false otherwise.
     */
    bool setInterrupts(bool accelDataReadyEnable, bool gyroDataReadyEnable, Event *dataReadyEvent = nullptr)
    {
        uint8_t int1Ctrl = 0;
        if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_INT1_CTRL, &int1Ctrl, 1))
        {
            utils::error_printf("LSM6DSR: Failed to read INT1_CTRL register.\n");
            return false;
        }

        int1Ctrl &= ~(LSM6DSR_INT1_DRDY_XL | LSM6DSR_INT1_DRDY_G); // Clear data ready bits

        if (accelDataReadyEnable)
        {
            int1Ctrl |= LSM6DSR_INT1_DRDY_XL;
        }

        if (gyroDataReadyEnable)
        {
            int1Ctrl |= LSM6DSR_INT1_DRDY_G;
        }

        if (!i2c.writeRegister(LSM6DSR_ADDRESS, LSM6DSR_INT1_CTRL, &int1Ctrl, 1))
        {
            utils::error_printf("LSM6DSR: Failed to write INT1_CTRL register.\n");
            return false;
        }
        
        if (dataReadyEvent != nullptr)
        {
            dataReady = false;
            utils::debug_printf("LSM6DSR: Data ready event configured.\n");
            this->dataReadyEvent = dataReadyEvent;
            dataReadyEvent->setCallback([](void* ctx, uint32_t events) {
                // This callback will be called in the context of the interrupt handler, so it should be as fast as possible and not perform any blocking operations.
                // We will simply release the semaphore to signal that new data is ready to be read.
                static_cast<LSM6DSR*>(ctx)->dataReady = true;
                (void)events; // Unused parameter
            }, this);
        }

        return true;
    }

    /*! @brief Checks if new data is ready to be read from the sensor.
     *  @return True if new data is ready, false otherwise.
     */
    bool isDataReady() const override
    {
        if (dataReadyEvent != nullptr)
        {
            // If an event is configured, we will check the semaphore to see if new data is ready. This allows us to avoid reading the status register if we already know that new data is ready based on the interrupt.
            if (dataReady)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            // If no event is configured, we will read the status register to check if new data is ready.
            uint8_t status = 0;
            if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_STATUS_REG, &status, 1))
            {
                utils::error_printf("LSM6DSR: Failed to read STATUS_REG register.\n");
                return false;
            }

            return (status & (LSM6DSR_INT1_DRDY_XL | LSM6DSR_INT1_DRDY_G)) != 0;
        }
    }

    /*! @brief Resets the LSM6DSR sensor.
     *  @param timeout_ms The timeout in milliseconds for the reset operation.
     *  @return True if the reset was successful, false otherwise.
     */
    bool reset(int timeout_ms = 5 * LSM6DSR_INIT_DELAY_MS)
    {
        uint8_t ctrl1xl = 0;
        uint8_t ctrl2g = 0;
        bool complete = false;
        utils::time_t start = utils::now();
        while (!complete)
        {
            if (utils::hasElapsed_ms(start, timeout_ms))
            {
                return false;
            }

            if (!i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL1_XL, &ctrl1xl, 1) || !i2c.readRegister(LSM6DSR_ADDRESS, LSM6DSR_CTRL2_G, &ctrl2g, 1))
            {
                continue;
            }

            if (((ctrl1xl & 0xF0) == 0) && ((ctrl2g & 0xF0) == 0))
            {
                complete = true;
            }
            else
            {
                if (!i2c.writeRegister(LSM6DSR_ADDRESS, LSM6DSR_RESET_REG, &LSM6DSR_RESET_BIT, 1))
                {
                    utils::error_printf("LSM6DSR: Failed to write RESET_REG register.\n");
                }
                utils::sleep_ms(LSM6DSR_INIT_DELAY_MS);
            }
        }

        return true;
    }
};

#endif // LSM6DSR_H
