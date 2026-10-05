
#ifndef PICO_MOTOR_DRIVER_H
#define PICO_MOTOR_DRIVER_H

#include <algorithm>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>

/*! @brief Class representing a motor driver.
 *  This class provides an interface for controlling a motor driver using PWM signals. It allows setting the motor power as a percentage, configuring the PWM frequency, and inverting the motor direction if needed.
 */
class MotorDriver
{
private:
    uint pinPWM;
    uint pinDIR;
    uint slice_num;
    uint chan;
    uint frequency;
    uint pwmPeriod;

    bool invert;
    
public:

    /*! @brief Construct a new Motor Driver object.
     *  @param pinPWM The PWM pin connected to the motor driver.
     *  @param pinDIR The direction pin connected to the motor driver.
     *  @param frequency The PWM frequency in Hz (default: 20000).
     *  @param invert Whether to invert the motor direction (default: false).
     */
    MotorDriver(uint pinPWM, uint pinDIR, uint frequency = 20000, bool invert = false) : pinPWM(pinPWM), pinDIR(pinDIR), frequency(frequency), invert(invert)
    {
        slice_num = pwm_gpio_to_slice_num(pinPWM);
        chan = pwm_gpio_to_channel(pinPWM);
        gpio_set_function(pinPWM, GPIO_FUNC_PWM);
        gpio_set_function(pinDIR, GPIO_FUNC_SIO);
        gpio_set_dir(pinDIR, GPIO_OUT);
        setFrequency(frequency);
        stop();
    }

    /*! @brief Initialize the motor driver, enabling the PWM output.
     */
    void begin(void)
    {
        pwm_set_enabled(slice_num, true);
    }

    /*! @brief End the motor driver, disabling the PWM output.
     */
    void end(void)
    {
        stop();
        pwm_set_enabled(slice_num, false);
    }

    /*! @brief Stop the motor.
     */
    void stop(void)
    {
        setPower(0);
    }

    /*! @brief Set the motor power % as an integer value between -100 and 100.
     *  @param power The desired motor power as an integer value.
     */
    void setPower(int power)
    {
        power = std::clamp(power, -100, 100);
        uint pwm = std::abs(power) * pwmPeriod / 100;

        gpio_put(pinDIR, power >= 0 ? invert : !invert);
        pwm_set_chan_level(slice_num, chan, pwm);
    }

    /*! @brief Set the motor power % as a double value between -1.0 and 1.0.
     *  @param power The desired motor power as a double value.
     */
    void setPower(double power)
    {
        setPower((int)(power * 100));
    }

    /*! @brief Set the PWM frequency for the motor driver.
     *  @param frequency The desired PWM frequency in Hz.
     */
    void setFrequency(uint frequency)
    {
        this->frequency = frequency;
        auto fclock = clock_get_hz(clk_sys);
        float div = (float)fclock / frequency;
        if (div <= 0xFFFF)
        {
            pwmPeriod = (uint)div;
            div = 1.0f;
        }
        else
        {
            pwmPeriod = 0xFFFF;
            div /= 0xFFFF;
        }

        pwm_set_clkdiv(slice_num, div);
        pwm_set_wrap(slice_num, pwmPeriod);
    }
};

#endif // PICO_MOTOR_DRIVER_H
