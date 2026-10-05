#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <cstdint>
#include <cstddef>

/*! @brief Abstract interface for I2C bus communication 
 */
class I2CBus
{
public:
    /*! @brief Write data to an I2C device.
     * @param deviceAddress I2C address of the target device.
     * @param data Pointer to the data to be written.
     * @param length Number of bytes to write.
     * @return true if the write operation was successful, false otherwise.
     */
    virtual bool writeData(uint8_t deviceAddress, const uint8_t* data, size_t length) const = 0;

    /*! @brief Read data from an I2C device.
     * @param deviceAddress I2C address of the target device.
     * @param data Pointer to the buffer to store the read data.
     * @param length Number of bytes to read.
     * @return true if the read operation was successful, false otherwise.
     */
    virtual bool readData(uint8_t deviceAddress, uint8_t* data, size_t length) const = 0;

    /*! @brief Writes data to a register of a device on the I2C bus
    *  @param deviceAddress The address of the I2C device
    *  @param registerAddress The address of the register to write to
    *  @param data Pointer to the data to write
    *  @param length Number of bytes to write
    *  @return true if the write operation was successful, false otherwise
    */
    virtual bool writeRegister(uint8_t deviceAddress, uint8_t registerAddress, const uint8_t* data, size_t length) const = 0;

    /*! @brief Reads data from a register of a device on the I2C bus
    *  @param deviceAddress The address of the I2C device
    *  @param registerAddress The address of the register to read from
    *  @param data Pointer to the buffer to store the read data
    *  @param length Number of bytes to read
    *  @return true if the read operation was successful, false otherwise
    */
    virtual bool readRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, size_t length) const = 0;
};

#endif // I2C_BUS_H