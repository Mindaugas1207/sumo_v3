#ifndef MT6701_H
#define MT6701_H

#include "encoder.h"
#include "i2c_bus.h"
#include <cstdint>
#include <cmath>

/*! @brief Class representing the MT6701 encoder.
 */
class MT6701 : public Encoder
{
    static constexpr uint8_t MT6701_ADDRESS = 0x06;
    static constexpr uint8_t MT6701_ANGLE_H_REG = 0x03;
    static constexpr uint8_t MT6701_ANGLE_L_REG = 0x04;
    static constexpr uint8_t MT6701_TIMEOUT_MS = 10;
    static constexpr int MT6701_INIT_TIMEOUT_MS = 100;

    I2CBus& i2c;
public:
    /*! @brief Construct a new MT6701 object
     *  @param i2c Reference to the I2C bus object
     */
    MT6701(I2CBus& i2c) : i2c(i2c) {}

    bool begin() const
    {
        utils::time_t start = utils::now();
        uint8_t buffer[1];
        while (!i2c.readRegister(MT6701_ADDRESS, MT6701_ANGLE_H_REG, buffer, 1))
        {
            if (utils::hasElapsed_ms(start, MT6701_INIT_TIMEOUT_MS))
            {
                return false;
            }
            utils::sleep_ms(10);
        }

        return true;
    }

    /*! @brief Read the angle in radians
     *  @param angle Reference to store the angle value
     *  @return true if the angle was successfully read, false otherwise
     */
    bool readAngleRadians(double& angle) const override
    {
        uint8_t buffer[2];

        if (!i2c.readRegister(MT6701_ADDRESS, MT6701_ANGLE_H_REG, buffer, sizeof(buffer)))
        {
            utils::error_printf("MT6701: Failed to read angle register for device at address 0x%02X\n", MT6701_ADDRESS);
            return false;
        }

        angle = (((uint16_t)buffer[0] << 6) | (buffer[1] >> 2)) * 4.0 * M_PI / 32768.0;
        return true;
    }

    /*! @brief Read the angle in degrees
     *  @param angle Reference to store the angle value
     *  @return true if the angle was successfully read, false otherwise
     */
    bool readAngleDegrees(double& angle) const override
    {
        uint8_t buffer[2];

        if (!i2c.readRegister(MT6701_ADDRESS, MT6701_ANGLE_H_REG, buffer, sizeof(buffer)))
        {
            utils::error_printf("MT6701: Failed to read angle register for device at address 0x%02X\n", MT6701_ADDRESS);
            return false;
        }

        angle = (((uint16_t)buffer[0] << 6) | (buffer[1] >> 2)) * 360.0 / 32768.0;
        return true;
    }
};

#endif