
#ifndef IR_RECEIVER_H
#define IR_RECEIVER_H

#include "one_wire_device.h"

/*! @brief Class to represent an IR receiver using the OneWire protocol.
 * This class provides methods to read commands from the IR receiver's FIFO buffer and clear the buffer.
 * It inherits from OneWireDevice, so it uses the same underlying communication mechanism and register access methods.
 */
class IrReceiver : public OneWireDevice
{
    static constexpr unsigned int RCV_NFIFO_REG = 0x00;
    static constexpr unsigned int RCV_FIFO_DATA_REG = 0x01;
public:
    struct config
    {
        DeviceIOMode io_mode;
        DeviceSerialBaudRate serial_baud_rate;
    };
    
    // Inherit constructors
    using OneWireDevice::OneWireDevice;

    /*! \brief Get the number of entries in the FIFO
     * \param count Reference to variable to receive the FIFO count
     * \return true if the count was successfully read, false on error
     */
    bool getFIFOCount(unsigned int &count)
    {
        unsigned int tmp;
        if (readRegister(RCV_NFIFO_REG, &tmp))
        {
            count = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the next command from the FIFO
     * \param command Reference to variable to receive the command
     * \return true if a command was successfully read, false on error
     */
    bool readFIFO(unsigned int &command)
    {
        unsigned int tmp;
        if (readRegister(RCV_FIFO_DATA_REG, &tmp))
        {
            command = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Clear all entries in the FIFO
     * \return true if the FIFO was successfully cleared, false on error
     */
    bool clearFIFO(void)
    {
        // Read and discard all data in the FIFO
        while (true)
        {
            unsigned int count;
            if (!getFIFOCount(count) || count == 0)
                break; // No more data available or error reading FIFO count

            unsigned int tmp;
            if (!readFIFO(tmp))
                return false; // Error reading FIFO
        }
        return true;
    }

    /*! \brief Get the next command from the FIFO
     * \param command Reference to variable to receive the command
     * \return true if a command was successfully read, false on error
     */
    bool getCommand(unsigned int &command)
    {
        unsigned int count;
        if (!getFIFOCount(count) || count == 0)
            return false; // No data available or error reading FIFO count

        return readFIFO(command); // Attempt to read command from FIFO
    }

    /*! \brief Get the next command from the FIFO, blocking until a command is available
     * \param command Reference to variable to receive the command
     * \return true if a command was successfully read, false on error
     */
    bool getCommandBlocking(unsigned int &command)
    {
        bool ok = false;
        do
        {
            ok = getCommand(command);
        }
        while (!ok);

        return true;
    }
};

#endif // IR_RECEIVER_H
