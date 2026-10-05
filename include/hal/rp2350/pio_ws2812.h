
#ifndef PIO_WS2812_H
#define PIO_WS2812_H

#include "led.h"
#include "ws2812.pio.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

/*! @brief Class to control WS2812 LEDs using PIO.
 */
class PioWS2812 : public Led
{
private:
    PIO pio;
    uint sm;

    uint32_t color;

    Color color_on;
    Color color_off;

    bool blinking;
    bool state;
    int count;
    absolute_time_t timestamp;
    uint period_on;
    uint period_off;

    /*! @brief Internal function to set the LED using the PIO.
     * @param grbw The color to set in GRBW format (8 bits per channel, with the order of Green, Red, Blue, White).
     */
    void _set(uint32_t grbw)
    {
        if (color == grbw) return;
        color = grbw;
        pio_sm_put_blocking(pio, sm, grbw << 8u);
    }

public:

    /*! @brief Construct a new PioWS2812 object.
     * @param pio The PIO instance to use.
     * @param sm The state machine within the PIO to use.
     * @param pin The GPIO pin connected to the WS2812 data line.
     * @param frequency The signal frequency for the WS2812 (default 800kHz).
     * @param rgbw Set to true if using RGBW LEDs, false for RGB (default false).
     */
    PioWS2812(PIO pio, uint sm, uint pin, float frequency = 800000, bool rgbw = false) : pio(pio), sm(sm)
    {
        color = 0;
        blinking = false;

        pio_gpio_init(pio, pin);
        if (pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true) != PICO_OK)
        {
            utils::error_printf("PIO_WS2812: Failed to set consecutive pin directions for PIO state machine. Source pio: %p, sm: %u, pin: %u\n", pio, sm, pin);
        }

        int offset = pio_add_program(pio, &ws2812_program);
        if (offset < 0)
        {
            utils::error_printf("PIO_WS2812: Failed to add WS2812 program to PIO. Source pio: %p\n", pio);
        }
        pio_sm_config c = ws2812_program_get_default_config(offset);
        sm_config_set_sideset_pins(&c, pin);
        sm_config_set_out_shift(&c, false, true, rgbw ? 32 : 24);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

        int cycles_per_bit = ws2812_T1 + ws2812_T2 + ws2812_T3;
        float div = clock_get_hz(clk_sys) / (frequency * cycles_per_bit);
        sm_config_set_clkdiv(&c, div);

        if (pio_sm_init(pio, sm, offset, &c) != PICO_OK)
        {
            utils::error_printf("PIO_WS2812: Failed to initialize PIO state machine. Source pio: %p, sm: %u, offset: %d\n", pio, sm, offset);
        }
    }

    /*! @brief Begin using the LED. This will enable the PIO state machine and set the initial color.
     */
    void begin(void)
    {
        color = 0;
        pio_sm_set_enabled(pio, sm, true);
        pio_sm_put_blocking(pio, sm, color);
    }

    /*! @brief Stop using the LED. This will disable the PIO state machine, leaving the LED in its last state.
    */
    void end(void)
    {
        pio_sm_set_enabled(pio, sm, false);
    }

    /*! @brief Set the LED color. This will stop any ongoing blinking. The color will be applied on the next call to update().
     * @param c The color to set.
     */
    void set(Color c) override
    {
        this->color_off = c;
        blinking = false;
    }

    /*! @brief Set the LED color immediately, blocking until the color has been sent to the LED. This will stop any ongoing blinking.
     * @param c The color to set.
     */
    void setBlocking(Color c) override
    {
        set(c);
        _set(c.toGRBW_u32());
    }

    /*! @brief Update the LED state. This should be called periodically to handle blinking.
     */
    void update(void) override
    {
        if (blinking)
        {
            auto t = get_absolute_time();
            if (state)
            {
                if (absolute_time_diff_us(timestamp, t) > period_on)
                {
                    timestamp = t;
                    if (count == 0)
                    {
                        blinking = false;
                    }
                    else
                    {
                        state = false;
                        _set(color_off.toGRBW_u32());
                    }
                }
            }
            else if (absolute_time_diff_us(timestamp, t) > period_off)
            {
                timestamp = t;

                if (count > 0)
                {
                    count--;
                }

                if (count == 0)
                {
                    blinking = false;
                }
                else
                {
                    state = true;
                    _set(color_on.toGRBW_u32());
                }
            }
        }
        else
        {
            _set(color_off.toGRBW_u32());
        }
    }

    /*! @brief Start blinking the LED with the specified colors and periods.
     * @param color_on The color to display when the LED is on.
     * @param color_off The color to display when the LED is off.
     * @param period_on The duration in milliseconds for which the LED stays on.
     * @param period_off The duration in milliseconds for which the LED stays off.
     * @param count The number of times to blink. Set to -1 for infinite blinking.
     */
    void blink(Color color_on, Color color_off, unsigned int period_on, unsigned int period_off, int count = -1) override
    {
        this->color_on = color_on;
        this->color_off = color_off;
        this->period_on = period_on * 1000;
        this->period_off = period_off * 1000;
        this->count = count;
        _set(color_on.toGRBW_u32());
        state = true;
        timestamp = get_absolute_time();
        blinking = true;
    }

    /*! @brief Start blinking the LED with the specified colors and periods, blocking until the blinking is complete.
     * @param color_on The color to display when the LED is on.
     * @param color_off The color to display when the LED is off.
     * @param period_on The duration in milliseconds for which the LED stays on.
     * @param period_off The duration in milliseconds for which the LED stays off.
     * @param count The number of times to blink. -1 will just return immediately since we can't block indefinitely.
     */
    void blinkBlocking(Color color_on, Color color_off, unsigned int period_on, unsigned int period_off, int count) override
    {
        if (count < 0) return;
        blink(color_on, color_off, period_on, period_off, count);

        while (this->blinking)
        {
            update();
        }
    }
};

#endif // PIO_WS2812_H
