
#ifndef PIO_ONE_WIRE_SERIAL_H
#define PIO_ONE_WIRE_SERIAL_H

#include "one_wire_serial.h"
#include "one_wire.pio.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

/*! @brief A class that implements a one-wire serial communication interface using the Raspberry Pi Pico's PIO subsystem.
 * This class manages two PIO state machines, one for transmitting data and one for receiving data, on a single GPIO pin.
 * It allows for configuring the baud rate, reading and writing data, and setting the GPIO pin used for communication.
 * The used pin can be changed on the fly, allowing for multiple devices to share the same serial interface by using different pins and asserting them as needed.
 */
class PioOneWireSerial : public OneWireSerial
{
    PIO pio;
    uint sm_tx;
    uint sm_rx;
    uint pin;
    bool started;
    long rx_timeout;
    long baudrate;
    char terminator;

    /*! @brief Listen for incoming data on the RX state machine.
     * @param enable If true, enable listening; if false, disable listening.
     */
    void listen(bool enable) const
    {
        if (enable)
        {
            uint32_t SM_STALL_MASK = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm_tx);
            pio->fdebug |= SM_STALL_MASK;   
            while (!(pio->fdebug & SM_STALL_MASK)); {} //wait for tx to stall
            pio_sm_set_enabled(pio, sm_rx, true);
        }
        else
        {
            pio_sm_set_enabled(pio, sm_rx, false);
        }
    }

    /*! @brief Get the number of bytes available in the RX FIFO.
     * @return Number of bytes available.
     */
    size_t available(void) const
    {
        return pio_sm_get_rx_fifo_level(pio, sm_rx);
    }

    /*! @brief Get a character from the RX FIFO.
     * @return Character from the RX FIFO.
     */
    char getc(void) const
    {
        io_rw_8 *rxfifo_shift = (io_rw_8 *)&pio->rxf[sm_rx] + 3;
        return (char)*rxfifo_shift;
    }

    /*! @brief Put a character into the TX FIFO.
     * @param c Character to put into the TX FIFO.
     */
    void putc(char c) const
    {
        pio_sm_put_blocking(pio, sm_tx, (uint32_t)c);
    }

    /*! @brief Read data from the RX FIFO until a specified stop character or timeout is reached.
     * @param buf Buffer to store the read data.
     * @param len Maximum number of bytes to read.
     * @param stopChar Character to stop reading at.
     * @param timeout Timeout in microseconds.
     * @return Number of bytes read.
     */
    size_t readUntil(char *buf, size_t len, char stopChar, long timeout) const
    {
        size_t count = 0;
        while (count < len)
        {
            absolute_time_t start = get_absolute_time();
            while (available() == 0)
            {
                if (absolute_time_diff_us(start, get_absolute_time()) > timeout)
                {
                    utils::warn_printf("PioOneWireSerial: Read timeout after %ld microseconds, source pio: %p, sm_rx: %u, len: %zu\n", timeout, pio, sm_rx, len);
                    return count;
                }
                tight_loop_contents();
            }
            char c = getc();
            buf[count++] = c;
            if (c == stopChar)
            {
                break;
            }
        }
        return count;
    }

public:
    static constexpr long DEFAULT_BAUDRATE = 9600;
    static constexpr uint PIN_NONE = 0xFFFFFFFF;

    /*! @brief Construct a PioOneWireSerial object with the specified PIO, state machines, timeout, and terminator.
     * @param pio PIO instance to use.
     * @param sm_tx State machine number for TX.
     * @param sm_rx State machine number for RX.
     * @param timeout Timeout for RX operations in microseconds. Defaults to 50000.
     * @param terminator Character to use as the terminator for RX operations. Defaults to '\n'.
     */
    PioOneWireSerial(PIO pio, uint sm_tx, uint sm_rx, long timeout = 50000, char terminator = '\n')
    {
        if (sm_tx > 3 || sm_rx > 3 || sm_tx == sm_rx)
        {
            panic("PIO state machine numbers must be between 0 and 3 and cannot be the same");
        }

        this->pio = pio;
        this->sm_tx = sm_tx;
        this->sm_rx = sm_rx;
        this->pin = PIN_NONE;
        baudrate = 0;
        rx_timeout = timeout;
        this->terminator = terminator;
        started = false;

        int offset_tx = pio_add_program(pio, &PioOneWireSerial_tx_program);
        if (offset_tx < 0)
        {
            utils::error_printf("PioOneWireSerial: Failed to add TX program to PIO. Source pio: %p\n", pio);
        }
        int offset_rx = pio_add_program(pio, &PioOneWireSerial_rx_program);
        if (offset_rx < 0)
        {
            utils::error_printf("PioOneWireSerial: Failed to add RX program to PIO. Source pio: %p\n", pio);
        }
        pio_sm_config c_tx = PioOneWireSerial_tx_program_get_default_config(offset_tx);
        pio_sm_config c_rx = PioOneWireSerial_rx_program_get_default_config(offset_rx);

        sm_config_set_out_shift(&c_tx, true, false, 32);
        sm_config_set_fifo_join(&c_tx, PIO_FIFO_JOIN_TX);

        sm_config_set_in_shift(&c_rx, true, false, 32);
        sm_config_set_fifo_join(&c_rx, PIO_FIFO_JOIN_RX);

        if (pio_sm_init(pio, sm_tx, offset_tx, &c_tx) != PICO_OK)
        {
            utils::error_printf("PioOneWireSerial: Failed to initialize TX state machine. Source pio: %p, sm_tx: %u, offset_tx: %d\n", pio, sm_tx, offset_tx);
        }
        if (pio_sm_init(pio, sm_rx, offset_rx, &c_rx) != PICO_OK)
        {
            utils::error_printf("PioOneWireSerial: Failed to initialize RX state machine. Source pio: %p, sm_rx: %u, offset_rx: %d\n", pio, sm_rx, offset_rx);
        }
    }

    /*! @brief Destroy the PioOneWireSerial object and release resources.
     */
    ~PioOneWireSerial()
    {
        end();
    }

    /*! @brief Begin serial communication with the specified baud rate and set the GPIO pin.
     * @param baud Baud rate for the serial communication. Defaults to DEFAULT_BAUDRATE.
     * @param pin GPIO pin number to set. Use PIN_NONE to remove the pin.
     */
    void begin(long baud, int pin)
    {
        begin(baud);
        setPin(pin);
    }

    /*! @brief Begin serial communication with the specified baud rate.
     * @param baud Baud rate for the serial communication. Defaults to DEFAULT_BAUDRATE.
     */
    void begin(long baud = DEFAULT_BAUDRATE)
    {
        if (started)
        {
            if (baudrate == baud)
            {
                return; // Already started with the same baud rate
            }
            pio_sm_set_enabled(pio, sm_tx, false);
            pio_sm_set_enabled(pio, sm_rx, false);
        }
        float div = (float)clock_get_hz(clk_sys) / (8 * baud);
        pio_sm_set_clkdiv(pio, sm_tx, div);
        pio_sm_set_clkdiv(pio, sm_rx, div);
        pio_sm_restart(pio, sm_tx);
        pio_sm_restart(pio, sm_rx);
        pio_sm_set_enabled(pio, sm_tx, true);
        pio_sm_set_enabled(pio, sm_rx, false);
        baudrate = baud;
        started = true;
    }

    /*! @brief End the serial communication and release resources.
     */
    void end(void)
    {
        pio_sm_set_enabled(pio, sm_tx, false);
        pio_sm_set_enabled(pio, sm_rx, false);
        setPin(PIN_NONE, true);
        started = false;
    }

    /*! @brief Write a null-terminated string to the serial interface.
     * @param string Null-terminated string to write. If the string is null or empty, no data will be written.
     */
    void write(const char *string) const override
    {
        if (!started || this->pin == PIN_NONE || string == nullptr || *string == '\0')
        {
            return;
        }
        listen(false);
        while (*string)
        {
            putc(*string++);
        }
    }

    /*! @brief Write data to the serial interface from a buffer.
     * @param buffer Buffer containing the data to write.
     * @param length Number of bytes to write from the buffer.
     */
    void write(const char *buffer, size_t length) const override
    {
        if (!started || this->pin == PIN_NONE || buffer == nullptr || length == 0)
        {
            return;
        }
        listen(false);
        for (size_t i = 0; i < length; i++)
        {
            putc(buffer[i]);
        }
    }

    /*! @brief Read data from the serial interface into a buffer until the terminator character is encountered or the maximum length is reached.
     * @param buffer Buffer to store the read data. Must be large enough to hold the expected data plus a null terminator (the received terminator character is replaced with a null terminator).
     * @param maxLength Maximum number of bytes to read. The function will read until the terminator character is received, or the timeout is reached, or the maxLength is exceeded.
     * @return Number of bytes read, excluding the terminator character. If the terminator character is not received before the timeout or maxLength is exceeded, the function returns 0 and the buffer contents are undefined.
     */
    size_t read(char *buffer, size_t maxLength) const override
    {
        if (!started || this->pin == PIN_NONE || buffer == nullptr || maxLength == 0)
        {
            return 0;
        }
        listen(true);
        size_t n = readUntil(buffer, maxLength, terminator, rx_timeout);
        listen(false);
        if (n == 0 || buffer[n - 1] != terminator)
        {
            return 0;
        }
        buffer[n - 1] = '\0';
        return n - 1;
    }

    /*! @brief Set the GPIO pin for this PioOneWireSerial instance.
     * @param pin GPIO pin number to set. Use PIN_NONE to release the pin.
     * @param force If true, force the pin change even if the current pin is the same as the new pin.
     */
    void setPin(uint pin, bool force = false)
    {
        if (this->pin == pin && !force)
        {
            return;
        }

        if (this->pin != PIN_NONE)
        {
            gpio_deinit(this->pin);
        }

        if (pin == PIN_NONE)
        {
            this->pin = PIN_NONE;
            return;
        }
        gpio_deinit(pin); // Ensure pin is not configured as GPIO before configuring for PIO
        pio_gpio_init(pio, pin);
        pio_sm_set_out_pins(pio, sm_tx, pin, 1);
        pio_sm_set_sideset_pins(pio, sm_tx, pin);
        pio_sm_set_in_pins(pio, sm_rx, pin);
        pio_sm_set_jmp_pin(pio, sm_rx, pin);
        this->pin = pin;
    }
};


/*! @brief A wrapper class for OneWireSerial that represents a single GPIO pin on a multiplexed serial interface, allowing the pin to be asserted or released for sensor configuration and normal operation while still using the same underlying PioOneWireSerial instance for communication.
 */
class PioOneWireSerialMuxedPin : public OneWireSerial
{
private:
    PioOneWireSerial &serial;
    uint pin;

public:
    /*! @brief Construct a new PioOneWireSerialMuxedPin object.
     * @param serial Reference to the PioOneWireSerial instance that manages the serial communication for this pin.
     * @param pin GPIO pin number that this muxed pin represents. This pin will be controlled by the PioOneWireSerial instance for serial communication when using the write and read methods, and can be asserted or released using the assertPin method.
     */
    constexpr PioOneWireSerialMuxedPin(PioOneWireSerial &serial, uint pin) : serial(serial), pin(pin)
    {
    }

    /*! @brief Get the underlying PioOneWireSerial instance associated with this muxed pin.
     * @return Reference to the PioOneWireSerial instance.
     */
    PioOneWireSerial& getSerial()
    {
        return serial;
    }

    /*! @brief Write a buffer of data to the serial interface for this pin. The pin will be configured for serial control during the write operation and will remain in serial control.
     * @param buffer Buffer containing the data to write. If the buffer is null or length is 0, the pin will still be configured for serial control but no data will be written.
     * @param len Length of the data in bytes. If len is 0, the pin will still be configured for serial control but no data will be written.
     */
    void write(const char *buffer, size_t len) const override
    {
        serial.setPin(pin); // Ensure the correct pin is selected and configured in serial control for this write operation
        serial.write(buffer, len);
    }

    /*! @brief Write a null-terminated string to the serial interface for this pin. The pin will be configured for serial control during the write operation and will remain in serial control.
     * @param string Null-terminated string to write. If the string is null or empty, the pin will still be configured for serial control but no data will be written.
     */
    void write(const char *string) const override
    {
        serial.setPin(pin); // Ensure the correct pin is selected and configured in serial control for this write operation
        serial.write(string);
    }

    /*! @brief Read data from the serial interface for this pin until the terminator character is received or the timeout is reached. The pin will be configured for serial control during the read operation and will remain in serial control.
     * @param buffer Buffer to store the read data. Must be large enough to hold the expected data plus a null terminator (the received terminator character is replaced with a null terminator).
     * @param maxLength Maximum length of data to read, including the null terminator. The function will read until the terminator character is received, or the timeout is reached, or the maxLength is exceeded.
     * @return The number of bytes read, not including the null terminator. If the terminator character is not received before the timeout or maxLength is exceeded, the function returns 0 and the buffer contents are undefined.
     */
    size_t read(char *buffer, size_t maxLength) const override
    {
        serial.setPin(pin); // Ensure the correct pin is selected and configured in serial control for this read operation
        return serial.read(buffer, maxLength);
    }

    /*! @brief Assert or release the pin by driving it low or by letting external pull-up resistors pull it high, respectively.
     * In any case the pin will be released from serial control, when driving low the pin is configured as GPIO output, while releasing it deinitializes the GPIO.
     * Any call to write or read will return the pin to serial control.
     * @param level If true, the pin will be released to let it be pulled high by external pull-up resistors. If false, the pin will be driven low by the microcontroller's GPIO.
     */
    void assertPin(bool level) const
    {
        serial.setPin(PioOneWireSerial::PIN_NONE); // Atempt to release pin from serial control before driving it
        if (level)
        {
            gpio_deinit(pin); // Release control of the pin to let it be pulled high by the sensor's pull-up resistor
        }
        else
        {
            gpio_init(pin); // Drive the pin low to assert it using the microcontroller's GPIO instead of the serial interface
            gpio_put(pin, 0);
            gpio_set_dir(pin, GPIO_OUT);
        }
    }
};

#endif // PIO_ONE_WIRE_SERIAL_H
