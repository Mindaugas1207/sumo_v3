
#ifndef LED_H
#define LED_H

#include <cstdint>
#include "color.h"

/*! @brief Abstract base class for RGB LED control.
 * This class defines the interface for controlling an RGB LED, including setting colors and blinking patterns. It can be subclassed to implement specific LED hardware control.
 * The non blocking functions are intended to be used in conjunction with a periodic update() call in the main loop, while the blocking functions can be used for simple one-off color changes or blinking without needing to manage timing in the main loop.
 */
class Led
{
public:
    /*! @brief Set the LED color. Non-blocking.
     *  @param c The color to set the LED to.
     */
    virtual void set(Color c){}

    /*! @brief Set the LED color. Blocking.
     *  @param c The color to set the LED to.
     */
    virtual void setBlocking(Color c){}

    /*! @brief Update the LED state. Should be called periodically in the main loop.
     */
    virtual void update(void){}

    /*! @brief Blink the LED with specified colors and periods. Non-blocking.
     *  @param color_on The color to display when the LED is on.
     *  @param color_off The color to display when the LED is off.
     *  @param period_on The duration in milliseconds for which the LED stays on.
     *  @param period_off The duration in milliseconds for which the LED stays off.
     *  @param count The number of times to blink the LED.
     */
    virtual void blink(Color color_on, Color color_off, unsigned int period_on, unsigned int period_off, int count){}

    /*! @brief Blink the LED with specified colors and periods. Blocking.
     *  @param color_on The color to display when the LED is on.
     *  @param color_off The color to display when the LED is off.
     *  @param period_on The duration in milliseconds for which the LED stays on.
     *  @param period_off The duration in milliseconds for which the LED stays off.
     *  @param count The number of times to blink the LED.
     */
    virtual void blinkBlocking(Color color_on, Color color_off, unsigned int period_on, unsigned int period_off, int count){}
};

#endif // LED_H
