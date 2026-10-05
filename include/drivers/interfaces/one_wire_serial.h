#ifndef ONE_WIRE_SERIAL_H
#define ONE_WIRE_SERIAL_H

#include <cstddef>

/*! @brief Abstract base class for OneWire serial communication.
 */
class OneWireSerial
{
public:

    /*! @brief Write data to the serial interface.
     * @param buffer Buffer containing data to write.
     * @param len Number of bytes to write.
     */
    virtual void write(const char *buffer, size_t len) const = 0;

    /*! @brief Write a null-terminated string to the serial interface.
     * @param string Null-terminated string to write.
     */
    virtual void write(const char *string) const = 0;

    /*! @brief Read data from the serial interface.
     * @param buffer Buffer to store read data.
     * @param maxLength Maximum number of bytes to read.
     * @return Number of bytes read.
     */
    virtual size_t read(char *buffer, size_t maxLength) const = 0;

    /*! @brief Destructor.
     */
    virtual ~OneWireSerial() {};
};

#endif // ONE_WIRE_SERIAL_H
