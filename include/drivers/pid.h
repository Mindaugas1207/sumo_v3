
#ifndef INC_PID_H_
#define INC_PID_H_

#include <math.h>
#include <algorithm>
#include <limits>
#include <vmath.h>

/*! @brief PID controller class template, default type is double
 * @tparam T Type for the PID calculations, default is double. Can be set to float for lower precision and faster calculations if needed.
 */
template <typename T = double>
class PID
{
    T lastError = 0;
    T integral = 0;
public:
    T kp;
    T ki;
    T kd;
    T integralLimit;
    T outputLimit;

    /*! @brief Constructor for PID controller.
     *  @param kp Proportional gain
     *  @param ki Integral gain
     *  @param kd Derivative gain
     *  @param integralLimit Maximum absolute value for the integral term
     *  @param outputLimit Maximum absolute value for the output
     */
    PID(T kp, T ki, T kd, T integralLimit = std::numeric_limits<T>::infinity(), T outputLimit = 1.0) : kp(kp), ki(ki), kd(kd), integralLimit(integralLimit), outputLimit(outputLimit)
    {
    }

    /*! @brief Compute the PID output.
     *  @param sp Setpoint
     *  @param pv Process variable
     *  @param dt Time step
     *  @return PID output
     */
    T compute(T sp, T pv, T dt)
    {
        T error = sp - pv;

        T proportional = kp * error;
        integral = std::clamp(integral + error * ki * dt, -integralLimit, integralLimit);
        T derivative = (error - lastError) * kd / dt;

        T output = std::clamp(proportional + integral + derivative, -outputLimit, outputLimit);
        lastError = error;
        return output;
    }

    /*! @brief Compute the PID output for angles in radians. Unwraps the error to be continuous over the -pi to pi range.
     *  @param sp Setpoint
     *  @param pv Process variable
     *  @param dt Time step
     *  @return PID output
     */
    T compute_radians(T sp, T pv, T dt)
    {
        T error = vmath::unwrapAngle(sp - pv);

        T proportional = kp * error;
        integral = std::clamp(integral + error * ki * dt, -integralLimit, integralLimit);
        T derivative = vmath::unwrapAngle(error - lastError) * kd / dt;
        
        T output = std::clamp(proportional + integral + derivative, -outputLimit, outputLimit);
        lastError = error;
        return output;
    }

    /*! @brief Compute the PID output for angles in degrees. Unwraps the error to be continuous over the -180 to 180 range.
     *  @param sp Setpoint
     *  @param pv Process variable
     *  @param dt Time step
     *  @return PID output
     */
    T compute_degrees(T sp, T pv, T dt)
    {
        T error = vmath::unwrapAngleDegrees(sp - pv);

        T proportional = kp * error;
        integral = std::clamp(integral + error * ki * dt, -integralLimit, integralLimit);
        T derivative = vmath::unwrapAngleDegrees(error - lastError) * kd / dt;
        
        T output = std::clamp(proportional + integral + derivative, -outputLimit, outputLimit);
        lastError = error;
        return output;
    }

    /*! @brief Reset the PID controller.
     *  This will reset the integral and last error terms to zero.
     */
    void reset(void)
    {
        lastError = 0;
        integral = 0;
    }
};

#endif // INC_PID_H_