
#ifndef ONE_WIRE_DEVICE_H
#define ONE_WIRE_DEVICE_H

#include "one_wire_serial.h"
#include "pico_utils.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>

/*! @brief Class to represent a device on a OneWire bus.
 * This class defines a common interface for configuring and communicating with devices that use the OneWire protocol.
 * It provides methods to read and write device registers, set I/O modes, configure serial communication settings, and save or reset device configurations.
 * The actual communication is done through a OneWireSerial interface, which must be implemented separately to handle the low-level details of sending and receiving data on the OneWire bus.
 */
class OneWireDevice
{
    static constexpr unsigned int CONFIG_IO_MODE_REG = 0x80;
    static constexpr unsigned int CONFIG_SERIAL_ID_REG = 0x81;
    static constexpr unsigned int CONFIG_SERIAL_BAUD_REG = 0x82;
    static constexpr unsigned int CONFIG_X_INDEX_REG = 0x83;
    static constexpr unsigned int CONFIG_X_LAST_REG = 0x84;

    OneWireSerial &oneWire;
    unsigned int index;
public:
    enum DeviceIOMode
    {
        IOMODE_SERIAL = 0,
        IOMODE_DO = 1,
        IOMODE_PWM = 2
    };

    enum DeviceSerialBaudRate
    {
        SERIAL_BAUD_ERROR = -1,
        SERIAL_BAUD_9600 = 0,
        SERIAL_BAUD_19200 = 1,
        SERIAL_BAUD_38400 = 2,
        SERIAL_BAUD_57600 = 3,
        SERIAL_BAUD_74880 = 4,
        SERIAL_BAUD_115200 = 5,
        SERIAL_BAUD_230400 = 6,
        SERIAL_BAUD_250000 = 7
    };

    /*! \brief Converter from DeviceSerialBaudRate code to actual baud rate value
     * \param code DeviceSerialBaudRate code
     * \return Baud rate in bits per second or -1 if the code is invalid
     */
    constexpr static long baudRateFromCode(DeviceSerialBaudRate code)
    {
        return code == SERIAL_BAUD_9600     ? 9600
               : code == SERIAL_BAUD_19200  ? 19200
               : code == SERIAL_BAUD_38400  ? 38400
               : code == SERIAL_BAUD_57600  ? 57600
               : code == SERIAL_BAUD_74880  ? 74880
               : code == SERIAL_BAUD_115200 ? 115200
               : code == SERIAL_BAUD_230400 ? 230400
               : code == SERIAL_BAUD_250000 ? 250000
                                            : -1;
    }

    
    /*! \brief Converter from baud rate to DeviceSerialBaudRate code
     * \param baud Baud rate in bits per second
     * \return DeviceSerialBaudRate code or SERIAL_BAUD_ERROR if the baud rate is invalid
     */
    constexpr static DeviceSerialBaudRate baudRateToCode(long baud)
    {
        return baud <= 9600     ? SERIAL_BAUD_9600
             : baud <= 19200    ? SERIAL_BAUD_19200
             : baud <= 38400    ? SERIAL_BAUD_38400
             : baud <= 57600    ? SERIAL_BAUD_57600
             : baud <= 74880    ? SERIAL_BAUD_74880
             : baud <= 115200   ? SERIAL_BAUD_115200
             : baud <= 230400   ? SERIAL_BAUD_230400
             : baud <= 250000   ? SERIAL_BAUD_250000
                                : SERIAL_BAUD_ERROR;
    }

    /*! \brief Constructor for OneWireDevice
     * \param oneWire Reference to OneWireSerial interface for communication with the device
     * \param index Index of the device on the OneWire bus, used for addressing in commands
     */
    constexpr OneWireDevice(OneWireSerial &oneWire, unsigned int index = 0) : oneWire(oneWire), index(index)
    {
    }

    /*! \brief Try to detect if the device is present and responsive on the OneWire bus by reading its serial ID register until a valid response is received or a timeout occurs
     * \param timeout Maximum time to wait in milliseconds, default is 500 (0.5 seconds)
     * \return true if the device responded, false if the timeout was reached
     */
    bool waitForBoot(int timeout = 500)
    {
        utils::time_t start = utils::now();
        utils::debug_printf("OneWireDevice: Waiting for device %u to boot with timeout %d ms\n", index, timeout);
        do
        {
            if (readRegister(CONFIG_SERIAL_ID_REG, nullptr))
            {
                utils::debug_printf("OneWireDevice: Device %u responded successfully\n", index);
                return true;
            }
            utils::sleep_ms(5);
        } while (!utils::hasElapsed_ms(start, timeout));
        utils::error_printf("OneWireDevice: Timeout reached, device %u did not respond\n", index);
        return false;
    }

    /*! \brief Set the index of the device on the OneWire bus
     * \param index Index of the device
     */
    void setIndex(unsigned int index)
    {
        this->index = index;
    }

    /*! \brief Get the index of the device on the OneWire bus
     * \return Index of the device
     */
    unsigned int getIndex(void)
    {
        return index;
    }

    /*! \brief Get the I/O mode of the device
     * \param mode Reference to variable to receive the I/O mode
     * \return true if the command was acknowledged and the mode read, false on error
     */
    bool getIOMode(DeviceIOMode &mode)
    {
        unsigned int tmp;
        if (readRegister(CONFIG_IO_MODE_REG, &tmp))
        {
            mode = (DeviceIOMode)tmp;
            utils::debug_printf("OneWireDevice: Device %u I/O mode read as %u\n", index, tmp);
            return true;
        }
        utils::error_printf("OneWireDevice: Failed to read I/O mode for device %u\n", index);
        return false;
    }

    /*! \brief Set the I/O mode of the device
     * \param mode I/O mode to set
     * \return true if the command was acknowledged, false on error
     */
    bool setIOMode(DeviceIOMode mode)
    {
        if (!writeRegister(CONFIG_IO_MODE_REG, (unsigned int)mode))
        {
            utils::error_printf("OneWireDevice: Failed to set I/O mode for device %u\n", index);
            return false;
        }
        return true;
    }

    /*! \brief Get the serial ID of the device
     * \param id Reference to variable to receive the serial ID
     * \return true if the command was acknowledged and the ID read, false on error
     */
    bool getSerialId(unsigned int &id)
    {
        unsigned int tmp;
        if (readRegister(CONFIG_SERIAL_ID_REG, &tmp))
        {
            id = tmp;
            utils::debug_printf("OneWireDevice: Device %u serial ID read as %u\n", index, tmp);
            return true;
        }
        utils::error_printf("OneWireDevice: Failed to read serial ID for device %u\n", index);
        return false;
    }

    /*! \brief Set the serial ID of the device
     * \param id Serial ID to set (0-255)
     * \return true if the command was acknowledged, false on error
     */
    bool setSerialId(unsigned int id)
    {
        if (!writeRegister(CONFIG_SERIAL_ID_REG, id))
        {
            utils::error_printf("OneWireDevice: Failed to set serial ID for device %u\n", index);
            return false;
        }
        return true;
    }

    /*! \brief Get the serial baud rate of the device
     * \param baud Reference to variable to receive the baud rate
     * \return true if the command was acknowledged and the baud rate read, false on error
     */
    bool getSerialBaudRate(DeviceSerialBaudRate &baud)
    {
        unsigned int tmp;
        if (readRegister(CONFIG_SERIAL_BAUD_REG, &tmp))
        {
            baud = (DeviceSerialBaudRate)tmp;
            utils::debug_printf("OneWireDevice: Device %u serial baud rate read as %u\n", index, tmp);
            return true;
        }
        utils::error_printf("OneWireDevice: Failed to read serial baud rate for device %u\n", index);
        return false;
    }

    /*! \brief Get the serial baud rate of the device
     * \param baud Reference to variable to receive the baud rate
     * \return true if the command was acknowledged and the baud rate read, false on error
     */
    bool getSerialBaudRate(long &baud)
    {
        DeviceSerialBaudRate baud_code;
        if (getSerialBaudRate(baud_code))
        {
            baud = baudRateFromCode(baud_code);
            utils::debug_printf("OneWireDevice: Device %u serial baud rate read as %ld\n", index, baud);
            return true;
        }
        utils::error_printf("OneWireDevice: Failed to read serial baud rate for device %u\n", index);
        return false;
    }

    /*! \brief Set the serial baud rate of the device
     * \param baud Baud rate to set
     * \return true if the command was acknowledged, false on error
     */
    bool setSerialBaudRate(DeviceSerialBaudRate baud)
    {
        if (!writeRegister(CONFIG_SERIAL_BAUD_REG, (unsigned int)baud))
        {
            utils::error_printf("OneWireDevice: Failed to set serial baud rate for device %u\n", index);
            return false;
        }
        utils::debug_printf("OneWireDevice: Device %u serial baud rate set to %u\n", index, (unsigned int)baud);
        return true;
    }

    /*! \brief Set the serial baud rate of the device
     * \param baud Baud rate to set
     * \return true if the command was acknowledged, false on error
     */
    bool setSerialBaudRate(long baud)
    {
        DeviceSerialBaudRate baud_code = baudRateToCode(baud);
        if (baud_code == SERIAL_BAUD_ERROR)
        {
            utils::error_printf("OneWireDevice: Invalid serial baud rate %ld for device %u\n", baud, index);
            return false;
        }
        utils::debug_printf("OneWireDevice: Setting serial baud rate to %ld for device %u\n", baud, index);
        return setSerialBaudRate(baud_code);
    }

    /*! \brief Read a value from a device register
     * \param reg Register address
     * \param value Pointer to variable to receive the value, or nullptr if the value is not required
     * \return true if the command was acknowledged and a value read, false on error
     */
    bool readRegister(unsigned int reg, unsigned int *value)
    {
        char buffer[7]; // "R" + 2*index + 2*reg + "\n" + "\0"
        snprintf(buffer, sizeof(buffer), "R%02X%02X\n", index, reg);
        utils::enter_critical_section();
        oneWire.write(buffer);
        size_t n = oneWire.read(buffer, sizeof(buffer));
        utils::exit_critical_section();
        if (n <= 0 || n > 4)
        { 
            utils::error_printf("OneWireDevice: Failed to read register %02X for device %u\n", reg, index);
            return false;
        }
        if (value == nullptr)
        {
            return true;
        }
        *value = (unsigned int)std::strtol(buffer, NULL, 16);
        return true;
    }

    /*! \brief Write a value to a device register
     * \param reg Register address
     * \param value Value to write
     * \return true if the command was acknowledged, false on error
     */
    bool writeRegister(unsigned int reg, unsigned int value)
    {
        char buffer[11]; // "W" + 2*index + 2*reg + 4*value + "\n" + "\0"
        snprintf(buffer, sizeof(buffer), "W%02X%02X%03X\n", index, reg, value);
        oneWire.write(buffer);
        size_t n = oneWire.read(buffer, sizeof(buffer));
        return n == 1 && buffer[0] == 'A';
    }

    /*! \brief Save the current device settings to non-volatile memory
     * \return true if the command was acknowledged, false on error
     */
    bool saveConfiguration(void)
    {
        char buffer[5]; // "S" + 2*index + "\n" + "\0"
        snprintf(buffer, sizeof(buffer), "S%02X\n", index);
        oneWire.write(buffer);
        size_t n = oneWire.read(buffer, sizeof(buffer));
        return n == 1 && buffer[0] == 'A';
    }

    /*! \brief Reset the device to factory default settings
     * \return true if the command was acknowledged, false on error
     */
    bool resetConfiguration(void)
    {
        char buffer[5]; // "Z" + 2*index + "\n" + "\0"
        snprintf(buffer, sizeof(buffer), "Z%02X\n", index);
        oneWire.write(buffer);
        size_t n = oneWire.read(buffer, sizeof(buffer));
        return n == 1 && buffer[0] == 'A';
    }

    /*! \brief Restart the device
     * \return true if the command was acknowledged, false on error
     */
    bool restartDevice(void)
    {
        char buffer[5]; // "S" + 2*index + "\n" + "\0"
        snprintf(buffer, sizeof(buffer), "U%02X\n", index);
        oneWire.write(buffer);
        size_t n = oneWire.read(buffer, sizeof(buffer));
        return n == 1 && buffer[0] == 'A';
    }
};

#endif // ONE_WIRE_DEVICE_H
