
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
    bool hasLast = false;

    T update(T error, T errorChange, T dt)
    {
        T proportional = kp * error;
        T derivative = hasLast ? errorChange * kd / dt : 0; // No derivative on the first sample after a reset
        T candidate = std::clamp(integral + error * ki * dt, -integralLimit, integralLimit);

        T unsaturated = proportional + candidate + derivative;
        T output = std::clamp(unsaturated, -outputLimit, outputLimit);

        // Hold the integral while the output is saturated and the error would push it further in
        bool windingUp = (unsaturated > outputLimit && error > 0) || (unsaturated < -outputLimit && error < 0);
        if (!windingUp) integral = candidate;

        lastError = error;
        hasLast = true;
        return output;
    }
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
        return update(error, error - lastError, dt);
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
        return update(error, vmath::unwrapAngle(error - lastError), dt);
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
        return update(error, vmath::unwrapAngleDegrees(error - lastError), dt);
    }

    /*! @brief Reset the PID controller.
     *  This will reset the integral and last error terms to zero.
     */
    void reset(void)
    {
        lastError = 0;
        integral = 0;
        hasLast = false;
    }
};

#endif // INC_PID_H_