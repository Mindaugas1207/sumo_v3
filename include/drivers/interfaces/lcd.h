#ifndef LCD_H
#define LCD_H

#include <cstdint>
#include "image.h"
#include "color.h"

/*! @brief Interface for LCD displays.
 */
class LCD
{
public:
    /*! @brief Get the width of the LCD in pixels.
     * @return Width of the LCD in pixels.
     */
    virtual int getWidth() const = 0;

    /*! @brief Get the height of the LCD in pixels.
     * @return Height of the LCD in pixels.
     */
    virtual int getHeight() const = 0;

    /*! @brief Turn on the LCD.
     */
    virtual void on() = 0;

    /*! @brief Turn off the LCD.
     */
    virtual void off() = 0;

    /*! @brief Clear the LCD display.
     */
    virtual void clear() = 0;

    /*! @brief Set the state of a specific pixel on the LCD.
     * @param x X-coordinate of the pixel.
     * @param y Y-coordinate of the pixel.
     * @param on Whether the pixel should be on or off.
     */
    virtual void setPixel(int x, int y, bool on) = 0;

    /*! @brief Set the color of a specific pixel on the LCD.
     * @param x X-coordinate of the pixel.
     * @param y Y-coordinate of the pixel.
     * @param color Color to set the pixel to.
     */
    virtual void setPixel(int x, int y, Color color) = 0;

    /*! @brief Update the LCD display with any changes made.
     */
    virtual void display() = 0;

    /*! @brief Set the contrast of the LCD display.
     * @param contrast Contrast level to set.
     */
    virtual void setContrast(int contrast) = 0;
};

#endif // LCD_H