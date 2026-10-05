#ifndef SSD1306_H
#define SSD1306_H

#include <cstdint>
#include "lcd.h"
#include "i2c_bus.h"
#include "pico_utils.h"

class SSD1306 : public LCD
{
public:
    enum class Resolution
    {
        Pixels128x32,
        Pixels128x64
    };

    enum class Orientation
    {
        Normal,
        Inverted
    };

private:
    static constexpr uint8_t SSD1306_I2C_ADDRESS = 0x3C;
    static constexpr uint8_t SSD1306_WIDTH = 128;
    static constexpr uint8_t SSD1306_MAX_HEIGHT = 64;
    static constexpr size_t PIXEL_BUFFER_SIZE = SSD1306_WIDTH * SSD1306_MAX_HEIGHT / 8;
    static constexpr size_t TRANSFER_BUFFER_SIZE = PIXEL_BUFFER_SIZE + 1;
    static constexpr uint8_t SSD1306_COMMAND_MODE = 0x00;
    static constexpr uint8_t SSD1306_DATA_MODE = 0x40;
    static constexpr uint8_t SSD1306_SET_CONTRAST = 0x81;
    static constexpr uint8_t SSD1306_COLUMN_ADDR = 0x21;
    static constexpr uint8_t SSD1306_PAGE_ADDR = 0x22;
    static constexpr uint8_t SSD1306_SET_DISPLAY_ON = 0xAF;
    static constexpr uint8_t SSD1306_SET_DISPLAY_OFF = 0xAE;
    static constexpr uint8_t SSD1306_MAX_PAGE_COUNT = SSD1306_MAX_HEIGHT / 8;
    uint8_t transferBuffer[TRANSFER_BUFFER_SIZE];
    uint8_t lineTransferBuffer[SSD1306_WIDTH + 1];
    I2CBus& i2c;
    uint8_t height;
    uint8_t pageCount;
    Orientation orientation;
    // Per-page dirty column tracking. A page is clean when minCol[p] > maxCol[p].
    uint8_t dirtyMinCol[SSD1306_MAX_PAGE_COUNT];
    uint8_t dirtyMaxCol[SSD1306_MAX_PAGE_COUNT];

    void sendCommand(uint8_t command) const
    {
        uint8_t data[2] = {SSD1306_COMMAND_MODE, command};
        if (!i2c.writeData(SSD1306_I2C_ADDRESS, data, sizeof(data)))
        {
            utils::error_printf("SSD1306: Failed to send command 0x%02X\n", command);
        }
    }

    void markDirtyPixel(int x, int y)
    {
        uint8_t page = static_cast<uint8_t>(y / 8);
        if (x < dirtyMinCol[page]) dirtyMinCol[page] = static_cast<uint8_t>(x);
        if (x > dirtyMaxCol[page]) dirtyMaxCol[page] = static_cast<uint8_t>(x);
    }

    void markFullDirty()
    {
        for (uint8_t p = 0; p < pageCount; p++)
        {
            dirtyMinCol[p] = 0;
            dirtyMaxCol[p] = SSD1306_WIDTH - 1;
        }
    }

    void clearDirty()
    {
        for (uint8_t p = 0; p < pageCount; p++)
        {
            dirtyMinCol[p] = SSD1306_WIDTH;
            dirtyMaxCol[p] = 0;
        }
    }

public:
        SSD1306(I2CBus& i2c, Resolution resolution, Orientation orientation = Orientation::Normal)
        : i2c(i2c),
          height(resolution == Resolution::Pixels128x32 ? 32 : 64),
                    pageCount(height / 8),
                    orientation(orientation)
    {
        transferBuffer[0] = SSD1306_DATA_MODE;
        lineTransferBuffer[0] = SSD1306_DATA_MODE;
        clearDirty();
        clear();
    }

    void init()
    {
        uint8_t init1[] = {
            SSD1306_COMMAND_MODE,
            0xAE, // Display OFF
            0xD5, 0x80, // Set display clock divide ratio/oscillator frequency
            0xA8, static_cast<uint8_t>(height - 1) // Set multiplex ratio
        };
        if (!i2c.writeData(SSD1306_I2C_ADDRESS, init1, sizeof(init1)))
        {
            utils::error_printf("SSD1306: Failed to write init1 data\n");
        }
        uint8_t init2[] = {
            SSD1306_COMMAND_MODE,
            0xD3, 0x00, // Set display offset
            0x40, // Set start line address
            0x8D, 0x14 // Charge pump setting (enable)
        };
        if (!i2c.writeData(SSD1306_I2C_ADDRESS, init2, sizeof(init2)))
        {
            utils::error_printf("SSD1306: Failed to write init2 data\n");
        }

        uint8_t init3[] = {
            SSD1306_COMMAND_MODE,
            0x20, 0x00, // Set memory addressing mode (horizontal)
            static_cast<uint8_t>(orientation == Orientation::Inverted ? 0xA0 : 0xA1), // Segment re-map
            static_cast<uint8_t>(orientation == Orientation::Inverted ? 0xC0 : 0xC8), // COM scan direction
        };
        if (!i2c.writeData(SSD1306_I2C_ADDRESS, init3, sizeof(init3)))
        {
            utils::error_printf("SSD1306: Failed to write init3 data\n");
        }
        sendCommand(0xDA); // Set COM pins hardware configuration
        sendCommand(height == 32 ? 0x02 : 0x12); // COM pin configuration for panel height
        sendCommand(0x81); // Set contrast control
        sendCommand(0x8F); // Set contrast to a reasonable value
        sendCommand(0xD9); // Set pre-charge period
        sendCommand(0xF1); // Set pre-charge to a reasonable value
        uint8_t init4[] = {
            SSD1306_COMMAND_MODE,
            0xDB, 0x40, // Set VCOMH deselect level
            0xA4, // Entire display ON (resume to RAM content display)
            0xA6, // Set normal display (not inverted)
            0x2E, // Deactivate scroll
            0xAF // Display ON
        };
        if (!i2c.writeData(SSD1306_I2C_ADDRESS, init4, sizeof(init4)))
        {
            utils::error_printf("SSD1306: Failed to write init4 data\n");
        }
    }

    // LCD interface implementation
    int getWidth() const override { return SSD1306_WIDTH; }
    int getHeight() const override { return height; }

    void on() override
    {
        sendCommand(SSD1306_SET_DISPLAY_ON); // Display ON
    }

    void off() override
    {
        sendCommand(SSD1306_SET_DISPLAY_OFF); // Display OFF
    }

    void clear() override
    {
        for (size_t i = 1; i < sizeof(transferBuffer); i++)
        {
            transferBuffer[i] = 0x00;
        }
        markFullDirty();
    }

    void setPixel(int x, int y, bool on) override
    {
        if (x < 0 || x >= getWidth() || y < 0 || y >= getHeight())
        {
            return; // Out of bounds
        }

        int byteIndex = 1 + (y / 8) * getWidth() + x;
        uint8_t bitMask = 1 << (y & 7);
        uint8_t oldValue = transferBuffer[byteIndex];

        if (on)
        {
            transferBuffer[byteIndex] |= bitMask; // Set pixel
        }
        else
        {
            transferBuffer[byteIndex] &= ~bitMask; // Clear pixel
        }

        if (transferBuffer[byteIndex] != oldValue)
        {
            markDirtyPixel(x, y);
        }
    }

    void setPixel(int x, int y, Color color) override
    {
        setPixel(x, y, color != Color::Black());
    }

    void display() override
    {
        for (uint8_t page = 0; page < pageCount; page++)
        {
            if (dirtyMinCol[page] > dirtyMaxCol[page])
            {
                continue; // page is clean
            }

            uint8_t colStart = dirtyMinCol[page];
            uint8_t colEnd = dirtyMaxCol[page];
            size_t colCount = static_cast<size_t>(colEnd - colStart + 1);

            uint8_t addrCmds[] = {
                SSD1306_COMMAND_MODE,
                SSD1306_COLUMN_ADDR, colStart, colEnd,
                SSD1306_PAGE_ADDR, page, page
            };
            if (!i2c.writeData(SSD1306_I2C_ADDRESS, addrCmds, sizeof(addrCmds)))
            {
                utils::error_printf("SSD1306: Failed to write address commands\n");
            }

            size_t pageStart = 1 + static_cast<size_t>(page) * SSD1306_WIDTH;
            for (size_t x = 0; x < colCount; x++)
            {
                lineTransferBuffer[1 + x] = transferBuffer[pageStart + colStart + x];
            }
            if (!i2c.writeData(SSD1306_I2C_ADDRESS, lineTransferBuffer, colCount + 1))
            {
                utils::error_printf("SSD1306: Failed to write line data\n");
            }
        }

        clearDirty();
    }

    void selfTest()
    {
        utils::info_printf("SSD1306: Starting self-test\n");
        utils::info_printf("SSD1306: Clearing display buffer for self-test\n");
        clear();

        utils::info_printf("SSD1306: Drawing test pattern for self-test\n");
        utils::info_printf("SSD1306: Drawing border for self-test\n");
        // Border
        for (int x = 0; x < getWidth(); x++)
        {
            setPixel(x, 0, true);
            setPixel(x, getHeight() - 1, true);
        }
        for (int y = 0; y < getHeight(); y++)
        {
            setPixel(0, y, true);
            setPixel(getWidth() - 1, y, true);
        }

        utils::info_printf("SSD1306: Drawing vertical major grid for self-test\n");
        // Vertical major grid every 8 px
        for (int x = 0; x < getWidth(); x += 8)
        {
            for (int y = 0; y < getHeight(); y++)
            {
                setPixel(x, y, true);
            }
        }

        utils::info_printf("SSD1306: Drawing horizontal major grid for self-test\n");
        // Horizontal major grid every 8 px
        for (int y = 0; y < getHeight(); y += 8)
        {
            for (int x = 0; x < getWidth(); x++)
            {
                setPixel(x, y, true);
            }
        }

        utils::info_printf("SSD1306: Drawing top ruler ticks for self-test\n");
        // Top ruler ticks every 4 px
        for (int x = 0; x < getWidth(); x += 4)
        {
            setPixel(x, 1, true);
            setPixel(x, 2, true);
        }

        utils::info_printf("SSD1306: Drawing left ruler ticks for self-test\n");
        // Left ruler ticks every 4 px
        for (int y = 0; y < getHeight(); y += 4)
        {
            setPixel(1, y, true);
            setPixel(2, y, true);
        }
        utils::info_printf("SSD1306: Self-test pattern drawn, updating display\n");
        display();
    }

    void setContrast(int contrast) override
    {
        uint8_t commands[] = {SSD1306_COMMAND_MODE, SSD1306_SET_CONTRAST, static_cast<uint8_t>(contrast)};
        if (!i2c.writeData(SSD1306_I2C_ADDRESS, commands, sizeof(commands)))
        {
            utils::error_printf("SSD1306: Failed to set contrast\n");
        }
    }
};


#endif // SSD1306_H