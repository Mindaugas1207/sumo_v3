#ifndef PICO_I2C_BUS_H
#define PICO_I2C_BUS_H

#include "i2c_bus.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"

/*! @brief Pico I2C bus implementation 
 */
class PicoI2CBus : public I2CBus
{
    static constexpr uint timeout_ms = 1000; // Timeout for I2C operations in milliseconds
    i2c_inst_t* i2c;
public:
    /*! @brief Constructor for PicoI2CBus.
     * @param i2c_instance Pointer to the i2c instance (e.g., i2c0 or i2c1)
     * @param baud_rate Desired baud rate for I2C communication.
     * @param clock_pin GPIO pin number for I2C clock (SCL)
     * @param data_pin GPIO pin number for I2C data (SDA)
     * @param pull_up Whether to enable internal pull-up resistors on the SCL and SDA lines (default: true)
     */
    PicoI2CBus(i2c_inst_t* i2c_instance, uint baud_rate, uint clock_pin, uint data_pin, bool pull_up = true) : i2c(i2c_instance)
    {
        i2c_init(i2c, baud_rate);
        gpio_set_function(clock_pin, GPIO_FUNC_I2C);
        gpio_set_function(data_pin, GPIO_FUNC_I2C);
        if (pull_up)
        {
            gpio_pull_up(clock_pin);
            gpio_pull_up(data_pin);
        }
        clearBus(clock_pin, data_pin, baud_rate);
    }

    /*! @brief Write data to an I2C device.
     * @param deviceAddress I2C address of the target device.
     * @param data Pointer to the data to be written.
     * @param length Number of bytes to write.
     * @return true if the write operation was successful, false otherwise.
     */
    bool writeData(uint8_t deviceAddress, const uint8_t* data, size_t length) const override
    {
        int result = i2c_write_blocking_until(i2c, deviceAddress, data, length, false, make_timeout_time_ms(timeout_ms));
        if (result == PICO_ERROR_TIMEOUT)
        {
            utils::warn_printf("PicoI2CBus: I2C write operation timed out. Device address: 0x%02X\n", deviceAddress);
            return false;
        }
        if (result == PICO_ERROR_GENERIC)
        {
            utils::warn_printf("PicoI2CBus: I2C write operation failed with generic error. Device address: 0x%02X\n", deviceAddress);
            return false;
        }
        if (result != static_cast<int>(length))
        {
            utils::warn_printf("PicoI2CBus: I2C write operation incomplete. Device address: 0x%02X, Expected length: %zu, Actual length: %d\n", deviceAddress, length, result);
            return false;
        }
        return true;
    }

    /*! @brief Read data from an I2C device.
     * @param deviceAddress I2C address of the target device.
     * @param data Pointer to the buffer to store the read data.
     * @param length Number of bytes to read.
     * @return true if the read operation was successful, false otherwise.
     */
    bool readData(uint8_t deviceAddress, uint8_t* data, size_t length) const override
    {
        int result = i2c_read_blocking_until(i2c, deviceAddress, data, length, false, make_timeout_time_ms(timeout_ms));
        if (result == PICO_ERROR_TIMEOUT)
        {
            utils::warn_printf("PicoI2CBus: I2C read operation timed out. Device address: 0x%02X\n", deviceAddress);
            return false;
        }
        if (result == PICO_ERROR_GENERIC)
        {
            utils::warn_printf("PicoI2CBus: I2C read operation failed address not acknowledged or no device present. Device address: 0x%02X\n", deviceAddress);
            return false;
        }
        if (result != static_cast<int>(length))
        {
            utils::warn_printf("PicoI2CBus: I2C read operation incomplete. Device address: 0x%02X, Expected length: %zu, Actual length: %d\n", deviceAddress, length, result);
            return false;
        }
        return true;
    }

    /*! @brief Write data to a specific register of an I2C device.
     * @param deviceAddress I2C address of the target device.
     * @param registerAddress Register address to write to.
     * @param data Pointer to the data to be written.
     * @param length Number of bytes to write.
     * @return true if the write operation was successful, false otherwise.
     */
    bool writeRegister(uint8_t deviceAddress, uint8_t registerAddress, const uint8_t* data, size_t length) const override
    {
        uint8_t buffer[length + 1];
        buffer[0] = registerAddress;
        memcpy(buffer + 1, data, length); // Copy the data to the buffer after the register address
        return writeData(deviceAddress, buffer, length + 1);
    }

    /*! @brief Read data from a specific register of an I2C device.
     * @param deviceAddress I2C address of the target device.
     * @param registerAddress Register address to read from.
     * @param data Pointer to the buffer to store the read data.
     * @param length Number of bytes to read.
     * @return true if the read operation was successful, false otherwise.
     */
    bool readRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, size_t length) const override
    {
        int result = i2c_write_blocking_until(i2c, deviceAddress, &registerAddress, 1, true, make_timeout_time_ms(timeout_ms));
        //Errors during the write operation must continue to the read operation because the write operation is not sending stop condition, if the function leaves without sending stop, all subsequent I2C operations would be affected.
        if (result == PICO_ERROR_TIMEOUT) utils::warn_printf("PicoI2CBus: I2C readRegister operation timed out at write operation. Device address: 0x%02X, Register address: 0x%02X\n", deviceAddress, registerAddress);
        if (result == PICO_ERROR_GENERIC) utils::warn_printf("PicoI2CBus: I2C readRegister operation failed at write operation: address not acknowledged or no device present. Device address: 0x%02X, Register address: 0x%02X\n", deviceAddress, registerAddress);
        if (result != 1) utils::warn_printf("PicoI2CBus: I2C readRegister operation incomplete (write operation). Device address: 0x%02X, Register address: 0x%02X, Expected length: 1, Actual length: %d\n", deviceAddress, registerAddress, result);
        result = i2c_read_blocking_until(i2c, deviceAddress, data, length, false, make_timeout_time_ms(timeout_ms));
        if (result == PICO_ERROR_TIMEOUT)
        {
            utils::warn_printf("PicoI2CBus: I2C readRegister operation timed out at read operation. Device address: 0x%02X, Register address: 0x%02X\n", deviceAddress, registerAddress);
            return false;
        }
        if (result == PICO_ERROR_GENERIC)
        {
            utils::warn_printf("PicoI2CBus: I2C readRegister operation failed at read operation: address not acknowledged or no device present. Device address: 0x%02X, Register address: 0x%02X\n", deviceAddress, registerAddress);
            return false;
        }
        if (result != static_cast<int>(length))
        {
            utils::warn_printf("PicoI2CBus: I2C readRegister operation incomplete (read operation). Device address: 0x%02X, Register address: 0x%02X, Expected length: %d, Actual length: %d\n", deviceAddress, registerAddress, length, result);
            return false;
        }
        return true;
    }

    /*! @brief Static method to clear the I2C bus by toggling the SCL line.
     * @param scl_pin GPIO pin number for I2C clock (SCL)
     * @param sda_pin GPIO pin number for I2C data (SDA)
     * @param baud_rate I2C bus speed in Hz (used to calculate delay for toggling)
     */
    static void clearBus(uint scl_pin, uint sda_pin, uint baud_rate)
    {
        //5us LOW/HIGH -> 10us period -> 100kHz freq
        uint delay = 500000 / baud_rate; // Calculate delay based on baud rate

        gpio_set_dir(scl_pin, false);
        gpio_set_dir(sda_pin, false);
        gpio_put(scl_pin, false);
        gpio_put(sda_pin, false);
        gpio_set_function(scl_pin, GPIO_FUNC_SIO);
        gpio_set_function(sda_pin, GPIO_FUNC_SIO);

        if (!gpio_get(sda_pin)) {
            int sclPulseCount = 0;
            while (sclPulseCount < 9 && !gpio_get(sda_pin)) {
                sclPulseCount++;
                gpio_set_dir(scl_pin, true);
                sleep_us(delay);
                gpio_set_dir(scl_pin, false);
                sleep_us(delay);
            }

            if (gpio_get(sda_pin)) {
                // Bus recovered : send a STOP
                gpio_set_dir(sda_pin, true);
                sleep_us(delay);
                gpio_set_dir(sda_pin, false);
            }
        }

        gpio_set_dir(scl_pin, false);
        gpio_set_dir(sda_pin, false);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);
        gpio_set_function(sda_pin, GPIO_FUNC_I2C); 
    }
};

#endif