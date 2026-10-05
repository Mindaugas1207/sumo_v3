
#ifndef INC_VMATH_H_
#define INC_VMATH_H_

#include <math.h>
#include <array>
#include <tuple>
#include <algorithm>
#include <bit>
#include <cstdint>
#include <limits>
#include <type_traits>

/*! @brief Vector and matrix math utilities.
 */
namespace vmath
{
    /*! @brief Inverse square root function using the fast inverse square root algorithm.
     *  @tparam T Type of the input value, must be a floating point type. Default is double.
     *  @tparam iterations Number of iterations for the Newton-Raphson method, default is 2.
     *  @tparam constexpr_iterations Number of iterations for the Newton-Raphson method when evaluated in a constant expression context, default is 8 for improved accuracy.
     *  @param x Input value.
     *  @return Inverse square root of the input value.
     */
    template <typename T = double, char iterations = 2, char constexpr_iterations = 8>
    inline constexpr T invsqrt(const T x)
    {
        static_assert(std::is_floating_point<T>::value, "T must be floating point");
        static_assert(sizeof(T) == 4 || sizeof(T) == 8, "T must be float or double");
        static_assert(iterations > 0, "iterations must be more than 0");
        static_assert(constexpr_iterations > 0, "constexpr_iterations must be more than 0");
        if (x <= (T)0)
            return std::numeric_limits<T>::quiet_NaN();
        const T x2 = x * (T)0.5;
        T y;

        if constexpr (sizeof(T) == 8)
        {
            std::uint64_t i = std::bit_cast<std::uint64_t>(x);
            i = 0x5fe6eb50c7b537a9ULL - (i >> 1);
            y = std::bit_cast<T>(i);
        }
        else
        {
            std::uint32_t i = std::bit_cast<std::uint32_t>(x);
            i = 0x5f3759dfU - (i >> 1);
            y = std::bit_cast<T>(i);
        }

        const char iters = std::is_constant_evaluated() ? constexpr_iterations : iterations;
        for (char it = 0; it < iters; ++it)
        {
            y = y * ((T)1.5 - (x2 * y * y));
        }

        return y;
    }

    /*! @brief Square root wrapper with constexpr support.
     *  @tparam T Type of the input value, must be a floating point type. Default is double.
     *  @tparam iterations Number of Newton-Raphson iterations for constant evaluation, default is 8.
     *  @param x Input value.
     *  @return Square root of the input value.
     */
    template <typename T = double, char iterations = 8>
    inline constexpr T sqrt(const T x)
    {
        static_assert(std::is_floating_point<T>::value, "T must be floating point");
        static_assert(iterations > 0, "iterations must be more than 0");

        if (x < (T)0)
            return std::numeric_limits<T>::quiet_NaN();
        if (x == (T)0)
            return (T)0;

        if (std::is_constant_evaluated())
        {
            T y = (x > (T)1) ? x : (T)1;
            for (char it = 0; it < iterations; ++it)
            {
                y = (y + x / y) * (T)0.5;
            }
            return y;
        }

        return std::sqrt(x);
    }

    /*! @brief Floating-point modulo wrapper with constexpr support.
     *  @tparam T Type of the input values, must be a floating point type. Default is double.
     *  @param x Dividend.
     *  @param y Divisor.
     *  @return Remainder of x / y with the sign of x.
     */
    template <typename T = double>
    inline constexpr T fmod(const T x, const T y)
    {
        static_assert(std::is_floating_point<T>::value, "T must be floating point");

        if (y == (T)0)
            return std::numeric_limits<T>::quiet_NaN();

        if (std::is_constant_evaluated())
        {
            const T q = x / y;
            const T i64_max_t = (T)std::numeric_limits<std::int64_t>::max();
            const T i64_min_t = (T)std::numeric_limits<std::int64_t>::min();

            if (q >= i64_max_t || q <= i64_min_t)
            {
                return x;
            }

            const std::int64_t qi = (std::int64_t)q;
            return x - (T)qi * y;
        }

        return std::fmod(x, y);
    }

    /*! @brief Sine function wrapper with constexpr support.
     *  @tparam T Type of the input value, must be a floating point type. Default is double.
     *  @tparam terms Number of Taylor series terms for constant evaluation, default is 8.
     *  @param x Input angle in radians.
     *  @return Sine of the input angle.
     *  @note The constexpr path uses Horner's method on a range-reduced angle for accuracy.
     */
    template <typename T = double, char terms = 8>
    inline constexpr T sin(const T x)
    {
        static_assert(std::is_floating_point<T>::value, "T must be floating point");
        static_assert(terms > 0, "terms must be more than 0");

        if (std::is_constant_evaluated())
        {
            // Range reduce to [-pi, pi]
            const T two_pi = (T)(2 * M_PI);
            T rx = fmod(x, two_pi);
            if (rx > (T)M_PI)       rx -= two_pi;
            else if (rx < (T)(-M_PI)) rx += two_pi;

            // Horner's method: sin(x) = x * (1 - x²/(2·3) * (1 - x²/(4·5) * (...)))
            const T x2 = rx * rx;
            T result = (T)1;
            for (char i = terms; i >= 1; --i)
                result = (T)1 - x2 / (T)((2 * i) * (2 * i + 1)) * result;
            return rx * result;
        }

        return std::sin(x);
    }

    /*! @brief Cosine function wrapper with constexpr support.
     *  @tparam T Type of the input value, must be a floating point type. Default is double.
     *  @tparam terms Number of Taylor series terms for constant evaluation, default is 8.
     *  @param x Input angle in radians.
     *  @return Cosine of the input angle.
     *  @note The constexpr path uses Horner's method on a range-reduced angle for accuracy.
     */
    template <typename T = double, char terms = 8>
    inline constexpr T cos(const T x)
    {
        static_assert(std::is_floating_point<T>::value, "T must be floating point");
        static_assert(terms > 0, "terms must be more than 0");

        if (std::is_constant_evaluated())
        {
            // Range reduce to [-pi, pi]
            const T two_pi = (T)(2 * M_PI);
            T rx = fmod(x, two_pi);
            if (rx > (T)M_PI)       rx -= two_pi;
            else if (rx < (T)(-M_PI)) rx += two_pi;

            // Horner's method: cos(x) = 1 - x²/(1·2) * (1 - x²/(3·4) * (...))
            const T x2 = rx * rx;
            T result = (T)1;
            for (char i = terms; i >= 1; --i)
                result = (T)1 - x2 / (T)((2 * i - 1) * (2 * i)) * result;
            return result;
        }

        return std::cos(x);
    }

    /*! @brief Two-argument arctangent wrapper with constexpr support.
     *  @tparam T Type of the input values, must be a floating point type. Default is double.
     *  @tparam terms Number of Taylor series terms for constant evaluation, default is 12.
     *  @param y Numerator (sine) component.
     *  @param x Denominator (cosine) component.
     *  @return Angle in radians in the range (-pi, pi].
     *  @note The constexpr path uses Horner's method on atan(t) for |t|<=1, with range-reduction
     *        and quadrant correction to handle all quadrants.
     */
    template <typename T = double, char terms = 12>
    inline constexpr T atan2(const T y, const T x)
    {
        static_assert(std::is_floating_point<T>::value, "T must be floating point");
        static_assert(terms > 0, "terms must be more than 0");

        if (std::is_constant_evaluated())
        {
            if (x == (T)0 && y == (T)0)
                return (T)0;
            if (x == (T)0)
                return (y > (T)0) ? (T)(M_PI / 2) : (T)(-M_PI / 2);

            const T t = y / x;

            // atan(t) via Horner's method:
            // atan(t) = t * (1 - t²/3 + t⁴/5 - ...) = t * (1/(1) - t²*(1/(3) - t²*(1/(5) - ...)))
            auto atan_unit = [](const T u) -> T {
                const T u2 = u * u;
                T result = (T)1 / (T)(2 * terms + 1);
                for (char i = terms - 1; i >= 0; --i)
                    result = (T)1 / (T)(2 * i + 1) - u2 * result;
                return u * result;
            };

            T angle;
            if (t > (T)1 || t < (T)-1)
            {
                // atan(t) = sign(t)*pi/2 - atan(1/t)  for |t| > 1
                angle = ((t > (T)0) ? (T)(M_PI / 2) : (T)(-M_PI / 2)) - atan_unit((T)1 / t);
            }
            else
            {
                angle = atan_unit(t);
            }

            // Quadrant correction for x < 0
            if (x < (T)0)
                angle += (y >= (T)0) ? (T)M_PI : (T)(-M_PI);

            return angle;
        }

        return std::atan2(y, x);
    }

    /*! @brief Constrain an angle in radians to the range (-pi, pi].
     *  @tparam T Type of the input angle, must be a floating point type. Default is double.
     *  @param x Input angle in radians.
     *  @return Constrained angle in radians.
     */
    template <typename T = double>
    inline constexpr T constrainAngle(const T x)
    {
        T y = fmod(x + (T)M_PI, (T)(2 * M_PI));

        if (y < 0)
            return y + (T)(M_PI);
        return y - (T)M_PI;
    }

    /*! @brief Unwrap angle in radians to be continuous over time.
     *  @tparam T Type of the input angle, must be a floating point type. Default is double.
     *  @param angle Input angle in radians.
     *  @return Unwrapped angle in radians.
     *  @note Assumes the change between consecutive angles will not exceed 180 degrees in either direction.
     */
    template <typename T = double>
    inline constexpr T unwrapAngle(const T angle)
    {
        if (angle > (T)M_PI)
            return angle - (T)(M_PI * 2);
        else if (angle < (T)(-M_PI))
            return angle + (T)(M_PI * 2);
        else
            return angle;
    }
    /*! @brief Unwrap angle in degrees to be continuous over time.
     *  @tparam T Type of the input angle, must be a floating point type. Default is double.
     *  @param angle Input angle in degrees.
     *  @return Unwrapped angle in degrees.
     *  @note Assumes the change between consecutive angles will not exceed 180 degrees in either direction.
     */
    template <typename T = double>
    inline constexpr T unwrapAngleDegrees(const T angle)
    {
        if (angle > (T)180)
            return angle - (T)(180 * 2);
        else if (angle < (T)(-180))
            return angle + (T)(180 * 2);
        else
            return angle;
    }

    /*! @brief Calculate the difference between two angles in radians.
     *  @tparam T Type of the input angles, must be a floating point type. Default is double.
     *  @param from Starting angle in radians.
     *  @param to Ending angle in radians.
     *  @return Difference between the two angles in radians.
     */
    template <typename T = double>
    inline constexpr T angleDifference(const T from, const T to)
    {
        return unwrapAngle(to - from);
    }

    /*! @brief 2D vector structure. The class overloads basic arithmetic operators for component-wise operations, allowing the user to write code that closely resembles the mathematical notation for vectors.
     *  @tparam T Type of the vector components, must be a floating point type. Default is double.
     */
    template <typename T = double>
    struct vector2d
    {
        /*! @brief X and Y components of the vector. */
        T X, Y;

        /*! @brief Default constructor initializes the vector to (0, 0). */
        constexpr vector2d() : X(0), Y(0) {}
        /*! @brief Constructor initializes the vector with given x and y values. */
        constexpr vector2d(T x, T y) : X(x), Y(y) {}
        /*! @brief Constructor initializes the vector with a single value for both components. */
        constexpr vector2d(T val) : X(val), Y(val) {}

        /*! @brief Assigns the values from another vector to this vector. 
         *  @param rhs The vector to copy values from.
         *  @return Reference to this vector.
         */
        constexpr vector2d &operator=(const vector2d &rhs)
        {
            this->X = rhs.X;
            this->Y = rhs.Y;
            return *this;
        }

        /*! @brief Adds another vector to this vector component-wise.
         *  @param rhs The vector to add.
         *  @return Reference to this vector.
         */
        constexpr vector2d &operator+=(const vector2d &rhs)
        {
            this->X += rhs.X;
            this->Y += rhs.Y;
            return *this;
        }

        /*! @brief Subtracts another vector from this vector component-wise.
         *  @param rhs The vector to subtract.
         *  @return Reference to this vector.
         */
        constexpr vector2d &operator-=(const vector2d &rhs)
        {
            this->X -= rhs.X;
            this->Y -= rhs.Y;
            return *this;
        }

        /*! @brief Multiplies this vector by another vector component-wise.
         *  @param rhs The vector to multiply by.
         *  @return Reference to this vector.
         */
        constexpr vector2d &operator*=(const vector2d &rhs)
        {
            this->X *= rhs.X;
            this->Y *= rhs.Y;
            return *this;
        }

        /*! @brief Divides this vector by another vector component-wise.
         *  @param rhs The vector to divide by.
         *  @return Reference to this vector.
         */
        constexpr vector2d &operator/=(const vector2d &rhs)
        {
            this->X /= rhs.X;
            this->Y /= rhs.Y;
            return *this;
        }

        /*! @brief Assigns the values from a scalar to this vector.
         *  @param rhs The scalar value to assign.
         *  @return Reference to this vector.
         */
        template <typename Tr>
        constexpr vector2d &operator=(const Tr &rhs)
        {
            this->X = rhs;
            this->Y = rhs;
            return *this;
        }

        /*! @brief Adds a scalar to this vector component-wise.
         *  @param rhs The scalar to add.
         *  @return Reference to this vector.
         */
        template <typename Tr>
        constexpr vector2d &operator+=(const Tr &rhs)
        {
            this->X += rhs;
            this->Y += rhs;
            return *this;
        }

        /*! @brief Subtracts a scalar from this vector component-wise.
         *  @param rhs The scalar to subtract.
         *  @return Reference to this vector.
         */
        template <typename Tr>
        constexpr vector2d &operator-=(const Tr &rhs)
        {
            this->X -= rhs;
            this->Y -= rhs;
            return *this;
        }

        /*! @brief Multiplies this vector by a scalar component-wise.
         *  @param rhs The scalar to multiply by.
         *  @return Reference to this vector.
         */
        template <typename Tr>
        constexpr vector2d &operator*=(const Tr &rhs)
        {
            this->X *= rhs;
            this->Y *= rhs;
            return *this;
        }

        /*! @brief Divides this vector by a scalar component-wise.
         *  @param rhs The scalar to divide by.
         *  @return Reference to this vector.
         */
        template <typename Tr>
        constexpr vector2d &operator/=(const Tr &rhs)
        {
            this->X /= rhs;
            this->Y /= rhs;
            return *this;
        }

        /*! @brief Adds two vectors component-wise.
         *  @param lhs The left-hand side vector.
         *  @param rhs The right-hand side vector.
         *  @return A new vector representing the sum of the two vectors.
         */
        friend constexpr vector2d operator+(vector2d lhs, const vector2d &rhs)
        {
            lhs += rhs;
            return lhs;
        }

        /*! @brief Subtracts two vectors component-wise.
         *  @param lhs The left-hand side vector.
         *  @param rhs The right-hand side vector.
         *  @return A new vector representing the difference of the two vectors.
         */
        friend constexpr vector2d operator-(vector2d lhs, const vector2d &rhs)
        {
            lhs -= rhs;
            return lhs;
        }

        /*! @brief Multiplies two vectors component-wise.
         *  @param lhs The left-hand side vector.
         *  @param rhs The right-hand side vector.
         *  @return A new vector representing the product of the two vectors.
         */
        friend constexpr vector2d operator*(vector2d lhs, const vector2d &rhs)
        {
            lhs *= rhs;
            return lhs;
        }

        /*! @brief Divides two vectors component-wise.
         *  @param lhs The left-hand side vector.
         *  @param rhs The right-hand side vector.
         *  @return A new vector representing the quotient of the two vectors.
         */
        friend constexpr vector2d operator/(vector2d lhs, const vector2d &rhs)
        {
            lhs /= rhs;
            return lhs;
        }

        /*! @brief Adds a scalar to a vector component-wise.
         *  @param lhs The vector.
         *  @param rhs The scalar to add.
         *  @return A new vector representing the sum of the vector and the scalar.
         */
        template <typename Tr>
        friend constexpr vector2d operator+(vector2d lhs, const Tr &rhs)
        {
            lhs += rhs;
            return lhs;
        }

        /*! @brief Subtracts a scalar from a vector component-wise.
         *  @param lhs The vector.
         *  @param rhs The scalar to subtract.
         *  @return A new vector representing the difference of the vector and the scalar.
         */
        template <typename Tr>
        friend constexpr vector2d operator-(vector2d lhs, const Tr &rhs)
        {
            lhs -= rhs;
            return lhs;
        }

        /*! @brief Multiplies a vector by a scalar component-wise.
         *  @param lhs The vector.
         *  @param rhs The scalar to multiply by.
         *  @return A new vector representing the product of the vector and the scalar.
         */
        template <typename Tr>
        friend constexpr vector2d operator*(vector2d lhs, const Tr &rhs)
        {
            lhs *= rhs;
            return lhs;
        }

        /*! @brief Divides a vector by a scalar component-wise.
         *  @param lhs The vector.
         *  @param rhs The scalar to divide by.
         *  @return A new vector representing the quotient of the vector and the scalar.
         */
        template <typename Tr>
        friend constexpr vector2d operator/(vector2d lhs, const Tr &rhs)
        {
            lhs /= rhs;
            return lhs;
        }

        /*! @brief Calculates the length (magnitude) of the vector.
         *  @return The length of the vector.
         */
        constexpr T length() const { return sqrt(this->X * this->X + this->Y * this->Y); }

        /*! @brief Calculates the angle of the vector relative to the positive X-axis.
         *  @return The angle of the vector in radians.
         */
        constexpr T angle() const { return atan2(this->Y, this->X); }

        /*! @brief Calculates the angle between two vectors.
         *  @param from The starting vector.
         *  @param to The ending vector.
         *  @return The angle between the two vectors in radians.
         */
        static constexpr T angleBetween(const vector2d &from, const vector2d &to)
        {
            return atan2(from.X * to.Y - from.Y * to.X, from.X * to.X + from.Y * to.Y);
        }

        /*! @brief Calculates the angle from this vector to another vector.
         *  @param to The target vector.
         *  @return The angle to the target vector in radians.
         */
        constexpr T angleTo(const vector2d &to) const { return angleBetween(*this, to); }

        /*! @brief Rotates the vector by a given angle.
         *  @param angle The angle to rotate the vector by, in radians.
         *  @return A new vector representing the rotated vector.
         */
        constexpr vector2d rotate(const T angle) const
        {
            T cos_a = cos(angle);
            T sin_a = sin(angle);
            return {
                this->X * cos_a - this->Y * sin_a,
                this->X * sin_a + this->Y * cos_a};
        }

        /*! @brief Creates a vector from an angle and magnitude.
         *  @param angle The angle of the vector in radians.
         *  @param magnitude The magnitude (length) of the vector. Default is 1.
         *  @return A new vector representing the specified angle and magnitude.
         */
        static constexpr vector2d fromAngle(const T angle, const T magnitude = 1)
        {
            return {cos(angle) * magnitude,
                    sin(angle) * magnitude};
        }
    };

    /*! @brief 3D vector structure. The class overloads basic arithmetic operators for component-wise operations, as well as cross product and rotation by a matrix, allowing the user to write code that closely resembles the mathematical notation for vectors.
     * @tparam T Type of the vector components, must be a floating point type. Default is double.
     */
    template <typename T = double>
    struct vector3d
    {
        /*! @brief 3D vector components. */
        T X, Y, Z;

        /*! @brief Accessor for the roll component. */
        constexpr const T &roll() const { return this->X; }
        /*! @brief Accessor for the pitch component. */
        constexpr const T &pitch() const { return this->Y; }
        /*! @brief Accessor for the yaw component. */
        constexpr const T &yaw() const { return this->Z; }

        /*! @brief Setter for the roll component. */
        constexpr void roll(const T &val) { this->X = val; }
        /*! @brief Setter for the pitch component. */
        constexpr void pitch(const T &val) { this->Y = val; }
        /*! @brief Setter for the yaw component. */
        constexpr void yaw(const T &val) { this->Z = val; }

        /*! @brief Setter for all components. */
        constexpr void set(const T &x, const T &y, const T &z) { this->X = x; this->Y = y; this->Z = z; }

        /*! @brief Default constructor initializes the vector to (0, 0, 0). */
        constexpr vector3d() : X(0), Y(0), Z(0) {}
        /*! @brief Constructor initializes the vector with given x, y, and z values. */
        constexpr vector3d(T x, T y, T z) : X(x), Y(y), Z(z) {}
        /*! @brief Constructor initializes the vector with a single value for all components. */
        constexpr vector3d(T val) : X(val), Y(val), Z(val) {}

        /*! @brief Assigns the values from another vector to this vector. 
         *  @param rhs The vector to copy values from.
         *  @return Reference to this vector.
         */
        constexpr vector3d &operator=(const vector3d &rhs)
        {
            this->X = rhs.X;
            this->Y = rhs.Y;
            this->Z = rhs.Z;
            return *this;
        }

        /*! @brief Adds the values from another vector to this vector.
         *  @param rhs The vector to add.
         *  @return Reference to this vector.
         *  @note This operation adds the corresponding components of the two vectors together.
         */
        constexpr vector3d &operator+=(const vector3d &rhs)
        {
            this->X += rhs.X;
            this->Y += rhs.Y;
            this->Z += rhs.Z;
            return *this;
        }

        /*! @brief Subtracts the values from another vector from this vector.
         *  @param rhs The vector to subtract.
         *  @return Reference to this vector.
         *  @note This operation subtracts the corresponding components of the two vectors.
         */
        constexpr vector3d &operator-=(const vector3d &rhs)
        {
            this->X -= rhs.X;
            this->Y -= rhs.Y;
            this->Z -= rhs.Z;
            return *this;
        }

        /*! @brief Multiplies the values of this vector by the corresponding values of another vector.
         *  @param rhs The vector to multiply by.
         *  @return Reference to this vector.
         *  @note This operation multiplies the corresponding components of the two vectors together. It does not represent a dot product or cross product, but rather a simple component-wise multiplication.
         */
        constexpr vector3d &operator*=(const vector3d &rhs)
        {
            this->X *= rhs.X;
            this->Y *= rhs.Y;
            this->Z *= rhs.Z;
            return *this;
        }

        /*! @brief Divides the values of this vector by the corresponding values of another vector.
         *  @param rhs The vector to divide by.
         *  @return Reference to this vector.
         *  @note This operation divides the corresponding components of this vector by the corresponding components of another vector. It does not represent a dot product or cross product, but rather a simple component-wise division.
         */
        constexpr vector3d &operator/=(const vector3d &rhs)
        {
            this->X /= rhs.X;
            this->Y /= rhs.Y;
            this->Z /= rhs.Z;
            return *this;
        }

        /*! @brief Assigns the same value to all components of this vector.
         *  @param rhs The value to assign to all components.
         *  @return Reference to this vector.
         */
        template <typename Tr>
        constexpr vector3d &operator=(const Tr &rhs)
        {
            this->X = rhs;
            this->Y = rhs;
            this->Z = rhs;
            return *this;
        }

        /*! @brief Adds the same value to all components of this vector.
         *  @param rhs The value to add to all components.
         *  @return Reference to this vector.
         */
        template <typename Tr>
        constexpr vector3d &operator+=(const Tr &rhs)
        {
            this->X += rhs;
            this->Y += rhs;
            this->Z += rhs;
            return *this;
        }

        /*! @brief Subtracts the same value from all components of this vector.
         *  @param rhs The value to subtract from all components.
         *  @return Reference to this vector.
         */
        template <typename Tr>
        constexpr vector3d &operator-=(const Tr &rhs)
        {
            this->X -= rhs;
            this->Y -= rhs;
            this->Z -= rhs;
            return *this;
        }

        /*! @brief Multiplies the same value with all components of this vector.
         *  @param rhs The value to multiply with all components.
         *  @return Reference to this vector.
         */
        template <typename Tr>
        constexpr vector3d &operator*=(const Tr &rhs)
        {
            this->X *= rhs;
            this->Y *= rhs;
            this->Z *= rhs;
            return *this;
        }

        /*! @brief Divides the same value from all components of this vector.
         *  @param rhs The value to divide from all components.
         *  @return Reference to this vector.
         */
        template <typename Tr>
        constexpr vector3d &operator/=(const Tr &rhs)
        {
            this->X /= rhs;
            this->Y /= rhs;
            this->Z /= rhs;
            return *this;
        }

        /*! @brief Adds two vectors.
         *  @param lhs The left-hand side vector.
         *  @param rhs The right-hand side vector.
         *  @return A new vector that is the sum of the two input vectors.
         */
        friend constexpr vector3d operator+(vector3d lhs, const vector3d &rhs)
        {
            lhs += rhs;
            return lhs;
        }

        /*! @brief Subtracts two vectors.
         *  @param lhs The left-hand side vector.
         *  @param rhs The right-hand side vector.
         *  @return A new vector that is the difference of the two input vectors.
         */
        friend constexpr vector3d operator-(vector3d lhs, const vector3d &rhs)
        {
            lhs -= rhs;
            return lhs;
        }

        /*! @brief Multiplies two vectors component-wise.
        *  @param lhs The left-hand side vector.
        *  @param rhs The right-hand side vector.
        *  @return A new vector that is the component-wise product of the two input vectors.
        *  @note This operation multiplies the corresponding components of the two vectors together. It does not represent a dot product or cross product, but rather a simple component-wise multiplication.
        */
        friend constexpr vector3d operator*(vector3d lhs, const vector3d &rhs)
        {
            lhs *= rhs;
            return lhs;
        }

        /*! @brief Divides two vectors component-wise.
         *  @param lhs The left-hand side vector.
         *  @param rhs The right-hand side vector.
         *  @return A new vector that is the component-wise quotient of the two input vectors
         * @note This operation divides the corresponding components of the first vector by the corresponding components of the second vector. It does not represent a dot product or cross product, but rather a simple component-wise division.
         */
        friend constexpr vector3d operator/(vector3d lhs, const vector3d &rhs)
        {
            lhs /= rhs;
            return lhs;
        }

        /*! @brief Adds a scalar to all components of a vector.
         *  @param lhs The vector.
         *  @param rhs The scalar value.
         *  @return A new vector with the scalar added to each component.
         */
        template <typename Tr>
        friend constexpr vector3d operator+(vector3d lhs, const Tr &rhs)
        {
            lhs += rhs;
            return lhs;
        }

        /*! @brief Subtracts a scalar from all components of a vector.
         *  @param lhs The vector.
         *  @param rhs The scalar value.
         *  @return A new vector with the scalar subtracted from each component.
         */
        template <typename Tr>
        friend constexpr vector3d operator-(vector3d lhs, const Tr &rhs)
        {
            lhs -= rhs;
            return lhs;
        }

        /*! @brief Multiplies all components of a vector by a scalar.
         *  @param lhs The vector.
         *  @param rhs The scalar value.
         *  @return A new vector with each component multiplied by the scalar.
         */
        template <typename Tr>
        friend constexpr vector3d operator*(vector3d lhs, const Tr &rhs)
        {
            lhs *= rhs;
            return lhs;
        }

        /*! @brief Divides all components of a vector by a scalar.
         *  @param lhs The vector.
         *  @param rhs The scalar value.
         *  @return A new vector with each component divided by the scalar.
         */
        template <typename Tr>
        friend constexpr vector3d operator/(vector3d lhs, const Tr &rhs)
        {
            lhs /= rhs;
            return lhs;
        }

        /*! @brief Calculates the length (magnitude) of the vector.
         *  @return The length of the vector.
         */
        constexpr T length() const { return sqrt(this->X * this->X + this->Y * this->Y + this->Z * this->Z); }

        /*! @brief Calculates the cross product of two vectors.
         *  @param lhs The left-hand side vector.
         *  @param rhs The right-hand side vector.
         *  @return A new vector that is the cross product of the two input vectors.
         */
        static constexpr vector3d cross(const vector3d &lhs, const vector3d &rhs)
        {
            return {
                lhs.Y * rhs.Z - lhs.Z * rhs.Y,
                lhs.Z * rhs.X - lhs.X * rhs.Z,
                lhs.X * rhs.Y - lhs.Y * rhs.X};
        }

        /*! @brief Calculates the cross product of this vector with another vector and updates this vector with the result.
         *  @param rhs The right-hand side vector.
         *  @return Reference to this vector after being updated with the cross product result.
         */
        constexpr vector3d &cross(const vector3d &rhs)
        {
            vector3d temp = *this;
            this->X = temp.Y * rhs.Z - temp.Z * rhs.Y;
            this->Y = temp.Z * rhs.X - temp.X * rhs.Z;
            this->Z = temp.X * rhs.Y - temp.Y * rhs.X;
            return *this;
        }

        /*! @brief Overloads the ^= operator to perform a cross product with another vector and update this vector with the result.
         *  @param rhs The right-hand side vector.
         *  @return Reference to this vector after being updated with the cross product result.
         */
        constexpr vector3d &operator^=(const vector3d &rhs) { return cross(rhs); }

        /*! @brief Overloads the ^ operator to perform a cross product between two vectors.
         *  @param lhs The left-hand side vector.
         *  @param rhs The right-hand side vector.
         *  @return A new vector that is the cross product of the two input vectors.
         */
        friend constexpr vector3d operator^(vector3d lhs, const vector3d &rhs)
        {
            lhs ^= rhs;
            return lhs;
        }

        /*! @brief Normalizes the vector, making its length equal to 1.
         *  @return A new vector that is the normalized version of this vector.
         */
        constexpr vector3d normalize(void) { return *this * invsqrt(X * X + Y * Y + Z * Z); }

        /*! @brief Normalizes a given vector, making its length equal to 1.
         *  @param vec The vector to normalize.
         *  @return A new vector that is the normalized version of the input vector.
         */
        static constexpr vector3d normalize(const vector3d &vec) { return vec * invsqrt(vec.X * vec.X + vec.Y * vec.Y + vec.Z * vec.Z); }

        /*! @brief Rotates the vector using a rotation matrix.
         *  @param _Rotation The rotation matrix.
         *  @return Reference to this vector after being rotated.
         */
        /*vector3d &Rotate(const matrix_t<T, 3, 3> &_Rotation)
        {
            this->X = this->X * _Rotation[0][0] + this->Y * _Rotation[0][1] + this->Z * _Rotation[0][2];
            this->Y = this->X * _Rotation[1][0] + this->Y * _Rotation[1][1] + this->Z * _Rotation[1][2];
            this->Z = this->X * _Rotation[2][0] + this->Y * _Rotation[2][1] + this->Z * _Rotation[2][2];
            return *this *= (T)2.0;
        }*/

        /*! @brief Calculates the angle difference between this vector and another vector.
         *  @param from The vector to calculate the angle difference from.
         *  @return A new vector representing the angle difference.
         *  @note The resulting vector contains the angle differences for each corresponding component, calculated using the angleDifference function which constrains the result to the range (-pi, pi].
         */
        constexpr vector3d<T> angleDifferenceFrom(const vector3d &from) const
        {
            return {
                angleDifference(from.X, this->X),
                angleDifference(from.Y, this->Y),
                angleDifference(from.Z, this->Z)};
        }
    };

    /*! @brief Unwraps the angles of a vector, ensuring they are within the range (-pi, pi].
     *  @param angle The vector containing the angles to unwrap.
     *  @return A new vector with the unwrapped angles.
     */
    template <typename T = double>
    inline constexpr vector3d<T> unwrapAngle(const vector3d<T> angle)
    {
        return { unwrapAngle(angle.X), unwrapAngle(angle.Y), unwrapAngle(angle.Z) };
    }
/*
    template <typename T = double>
    struct rotMatrix_t : matrix_t<T, 3, 3>
    {
        rotMatrix_t Transpose(void)
        {
            rotMatrix_t _Result;

            _Result[0][0] = (*this)[0][0];
            _Result[0][1] = (*this)[1][0];
            _Result[0][2] = (*this)[2][0];

            _Result[1][0] = (*this)[0][1];
            _Result[1][1] = (*this)[1][1];
            _Result[1][2] = (*this)[2][1];

            _Result[2][0] = (*this)[0][2];
            _Result[2][1] = (*this)[1][2];
            _Result[2][2] = (*this)[2][2];

            return _Result;
        }

        double Determinant(void)
        {
            return ((*this)[0][0] * ((*this)[1][1] * (*this)[2][2] - (*this)[1][2] * (*this)[2][1])) - ((*this)[0][1] * ((*this)[1][0] * (*this)[2][2] - (*this)[1][2] * (*this)[2][0])) + ((*this)[0][2] * ((*this)[1][0] * (*this)[2][1] - (*this)[1][1] * (*this)[2][0]));
        }

        vector3d<T> RotateVector(const vector3d<T> &_Vector)
        {
            vector3d<T> _Result;

            _Result.X = _Vector.X * (*this)[0][0] + _Vector.Y * (*this)[0][1] + _Vector.Z * (*this)[0][2];
            _Result.Y = _Vector.X * (*this)[1][0] + _Vector.Y * (*this)[1][1] + _Vector.Z * (*this)[1][2];
            _Result.Z = _Vector.X * (*this)[2][0] + _Vector.Y * (*this)[2][1] + _Vector.Z * (*this)[2][2];

            return _Result *= (T)2.0;
        }
    };
*/
    /*! @brief Quaternion structure. The class overloads basic arithmetic operators for component-wise operations, as well as quaternion multiplication and vector rotation alowing the user to write code that closely resembles the mathematical notation for quaternions.
     *  @tparam T Type of the quaternion components, must be a floating point type. Default is double.
     */
    template <typename T = double>
    struct quaternion
    {
        /*! @brief Quaternion components. */
        T W, X, Y, Z;

        /*! @brief Default constructor initializes the quaternion to (1, 0, 0, 0). */
        constexpr quaternion() : W(1), X(0), Y(0), Z(0) {}
        /*! @brief Constructor initializes the quaternion with given w, x, y, and z values. */
        constexpr quaternion(T w, T x, T y, T z) : W(w), X(x), Y(y), Z(z) {}
        /*! @brief Constructor initializes the quaternion with a single value for all components. */
        constexpr quaternion(T val) : W(val), X(val), Y(val), Z(val) {}

        /* Operator overloads */

        /*! @brief Assigns the values of another quaternion to this quaternion.
         *  @param rhs The quaternion to assign from.
         *  @return A reference to this quaternion.
         */
        constexpr quaternion &operator=(const quaternion &rhs)
        {
            this->W = rhs.W;
            this->X = rhs.X;
            this->Y = rhs.Y;
            this->Z = rhs.Z;
            return *this;
        }

        /*! @brief Adds another quaternion to this quaternion.
         *  @param rhs The quaternion to add.
         *  @return A reference to this quaternion.
         *  @note This operation adds the corresponding components of the two quaternions together. It does not represent quaternion addition in the sense of combining rotations, but rather a simple component-wise addition.
         */
        constexpr quaternion &operator+=(const quaternion &rhs)
        {
            this->W += rhs.W;
            this->X += rhs.X;
            this->Y += rhs.Y;
            this->Z += rhs.Z;
            return *this;
        }

        /*! @brief Subtracts another quaternion from this quaternion.
         *  @param rhs The quaternion to subtract.
         *  @return A reference to this quaternion.
         *  @note This operation subtracts the corresponding components of the two quaternions. It does not represent quaternion subtraction in the sense of combining rotations, but rather a simple component-wise subtraction.
         */
        constexpr quaternion &operator-=(const quaternion &rhs)
        {
            this->W -= rhs.W;
            this->X -= rhs.X;
            this->Y -= rhs.Y;
            this->Z -= rhs.Z;
            return *this;
        }

        /*! @brief Subtracts a vector from this quaternion.
         *  @param rhs The vector to subtract.
         *  @return A reference to this quaternion.
         *  @note This operation subtracts the components of a vector from the corresponding components of the quaternion. Assuming the vector represents a pure quaternion (with a scalar part of zero).
         */
        constexpr quaternion &operator-=(const vector3d<T> &rhs)
        {
            this->X -= rhs.X;
            this->Y -= rhs.Y;
            this->Z -= rhs.Z;
            return *this;
        }

        /*! @brief Multiplies this quaternion by another quaternion.
         *  @param rhs The quaternion to multiply by.
         *  @return A reference to this quaternion.
         *  @note This operation combines the rotations represented by the two quaternions.
         *  @note Quaternion multiplication is not commutative, so the order of multiplication matters.
         */
        constexpr quaternion &operator*=(const quaternion &rhs)
        {
            quaternion lhs = *this;
            this->W = (lhs.W * rhs.W) - (lhs.X * rhs.X) - (lhs.Y * rhs.Y) - (lhs.Z * rhs.Z);
            this->X = (lhs.W * rhs.X) + (lhs.X * rhs.W) + (lhs.Y * rhs.Z) - (lhs.Z * rhs.Y);
            this->Y = (lhs.W * rhs.Y) - (lhs.X * rhs.Z) + (lhs.Y * rhs.W) + (lhs.Z * rhs.X);
            this->Z = (lhs.W * rhs.Z) + (lhs.X * rhs.Y) - (lhs.Y * rhs.X) + (lhs.Z * rhs.W);
            return *this;
        }

        /*! @brief Multiplies this quaternion by a vector.
         *  @param rhs The vector to multiply by.
         *  @return A reference to this quaternion.
         *  @note This operation applies the rotation represented by the quaternion to the vector.
         *  @note The vector is treated as a quaternion with a scalar part of zero for the purpose of multiplication.
         *  @note Quaternion multiplication is not commutative, so the order of multiplication matters. This operation applies the rotation to the vector, while multiplying the vector by the quaternion would not yield a meaningful result.
         */
        constexpr quaternion &operator*=(const vector3d<T> &rhs)
        {
            quaternion lhs = *this;
            this->W = -(lhs.X * rhs.X) - (lhs.Y * rhs.Y) - (lhs.Z * rhs.Z);
            this->X = (lhs.W * rhs.X) + (lhs.Y * rhs.Z) - (lhs.Z * rhs.Y);
            this->Y = (lhs.W * rhs.Y) - (lhs.X * rhs.Z) + (lhs.Z * rhs.X);
            this->Z = (lhs.W * rhs.Z) + (lhs.X * rhs.Y) - (lhs.Y * rhs.X);
            return *this;
        }

        /*! @brief Divides this quaternion by another quaternion.
         *  @param rhs The quaternion to divide by.
         *  @return A reference to this quaternion.
         *  @note This operation is equivalent to multiplying this quaternion by the inverse of the other quaternion.
         */
        constexpr quaternion &operator/=(const quaternion &rhs)
        {
            this->W /= rhs.W;
            this->X /= rhs.X;
            this->Y /= rhs.Y;
            this->Z /= rhs.Z;
            return *this;
        }

        /*! @brief Divides this quaternion by a scalar.
         *  @param rhs The scalar to divide by.
         *  @return A reference to this quaternion.
         *  @note This operation divides each component of the quaternion by the scalar.
         */
        template <typename Tr>
        constexpr quaternion &operator=(const Tr &rhs)
        {
            this->W = rhs;
            this->X = rhs;
            this->Y = rhs;
            this->Z = rhs;
            return *this;
        }

        /*! @brief Adds a scalar to this quaternion.
         *  @param rhs The scalar to add.
         *  @return A reference to this quaternion.
         *  @note This operation adds the scalar to each component of the quaternion.
         */
        template <typename Tr>
        constexpr quaternion &operator+=(const Tr &rhs)
        {
            this->W += rhs;
            this->X += rhs;
            this->Y += rhs;
            this->Z += rhs;
            return *this;
        }

        /*! @brief Subtracts a scalar from this quaternion.
         *  @param rhs The scalar to subtract.
         *  @return A reference to this quaternion.
         *  @note This operation subtracts the scalar from each component of the quaternion.
         */
        template <typename Tr>
        constexpr quaternion &operator-=(const Tr &rhs)
        {
            this->W -= rhs;
            this->X -= rhs;
            this->Y -= rhs;
            this->Z -= rhs;
            return *this;
        }

        /*! @brief Multiplies this quaternion by a scalar.
         *  @param rhs The scalar to multiply by.
         *  @return A reference to this quaternion.
         *  @note This operation multiplies each component of the quaternion by the scalar.
         */
        template <typename Tr>
        constexpr quaternion &operator*=(const Tr &rhs)
        {
            this->W *= rhs;
            this->X *= rhs;
            this->Y *= rhs;
            this->Z *= rhs;
            return *this;
        }

        /*! @brief Divides this quaternion by a scalar.
         *  @param rhs The scalar to divide by.
         *  @return A reference to this quaternion.
         *  @note This operation divides each component of the quaternion by the scalar.
         */
        template <typename Tr>
        constexpr quaternion &operator/=(const Tr &rhs)
        {
            this->W /= rhs;
            this->X /= rhs;
            this->Y /= rhs;
            this->Z /= rhs;
            return *this;
        }

        /* Friend operators */

        /*! @brief Adds two quaternions.
         *  @param lhs The first quaternion.
         *  @param rhs The second quaternion.
         *  @return The result of adding the two quaternions.
         *  @note This operation adds the corresponding components of the two quaternions together. It does not represent quaternion addition in the sense of combining rotations, but rather a simple component-wise addition.
         */
        friend constexpr quaternion operator+(quaternion lhs, const quaternion &rhs)
        {
            lhs += rhs;
            return lhs;
        }

        /*! @brief Subtracts two quaternions.
         *  @param lhs The first quaternion.
         *  @param rhs The second quaternion.
         *  @return The result of subtracting the second quaternion from the first.
         *  @note This operation subtracts the corresponding components of the two quaternions. It does not represent quaternion subtraction in the sense of combining rotations, but rather a simple component-wise subtraction.
         */
        friend constexpr quaternion operator-(quaternion lhs, const quaternion &rhs)
        {
            lhs -= rhs;
            return lhs;
        }

        /*! @brief Subtracts a vector from a quaternion.
         *  @param lhs The quaternion.
         *  @param rhs The vector.
         *  @return The result of subtracting the vector from the quaternion.
         *  @note This operation subtracts the components of a vector from the corresponding components of the quaternion. Assuming the vector represents a pure quaternion (with a scalar part of zero).
         */
        friend constexpr quaternion operator-(quaternion lhs, const vector3d<T> &rhs)
        {
            lhs -= rhs;
            return lhs;
        }

        /*! @brief Multiplies two quaternions.
         *  @param lhs The first quaternion.
         *  @param rhs The second quaternion.
         *  @return The result of multiplying the two quaternions.
         *  @note This operation combines the rotations represented by the two quaternions.
         *  @note Quaternion multiplication is not commutative, so the order of multiplication matters
         */
        friend constexpr quaternion operator*(quaternion lhs, const quaternion &rhs)
        {
            lhs *= rhs;
            return lhs;
        }

        /*! @brief Multiplies a quaternion by a vector.
         *  @param lhs The quaternion.
         *  @param rhs The vector.
         *  @return The result of multiplying the quaternion by the vector.
         *  @note This operation applies the rotation represented by the quaternion to the vector.
         *  @note The vector is treated as a quaternion with a scalar part of zero for the purpose of multiplication.
         *  @note Quaternion multiplication is not commutative, so the order of multiplication matters. This operation applies the rotation to the vector, while multiplying the vector by the quaternion would not yield a meaningful result.
         */
        friend constexpr quaternion operator*(quaternion lhs, const vector3d<T> &rhs)
        {
            lhs *= rhs;
            return lhs;
        }

        /*! @brief Divides two quaternions.
         *  @param lhs The first quaternion.
         *  @param rhs The second quaternion.
         *  @return The result of dividing the first quaternion by the second.
         *  @note This operation is equivalent to multiplying the first quaternion by the inverse of the second quaternion.
         */
        friend constexpr quaternion operator/(quaternion lhs, const quaternion &rhs)
        {
            lhs /= rhs;
            return lhs;
        }

        /*! @brief Adds a scalar to a quaternion.
         *  @param lhs The quaternion.
         *  @param rhs The scalar.
         *  @return The result of adding the scalar to the quaternion.
         *  @note This operation adds the scalar to each component of the quaternion.
         */
        template <typename Tr>
        friend constexpr quaternion operator+(quaternion lhs, const Tr &rhs)
        {
            lhs += rhs;
            return lhs;
        }

        /*! @brief Subtracts a scalar from a quaternion.
         *  @param lhs The quaternion.
         *  @param rhs The scalar.
         *  @return The result of subtracting the scalar from the quaternion.
         *  @note This operation subtracts the scalar from each component of the quaternion.
         */
        template <typename Tr>
        friend constexpr quaternion operator-(quaternion lhs, const Tr &rhs)
        {
            lhs -= rhs;
            return lhs;
        }

        /*! @brief Multiplies a quaternion by a scalar.
         *  @param lhs The quaternion.
         *  @param rhs The scalar.
         *  @return The result of multiplying the quaternion by the scalar.
         *  @note This operation multiplies each component of the quaternion by the scalar.
         */
        template <typename Tr>
        friend constexpr quaternion operator*(quaternion lhs, const Tr &rhs)
        {
            lhs *= rhs;
            return lhs;
        }

        /*! @brief Divides a quaternion by a scalar.
         *  @param lhs The quaternion.
         *  @param rhs The scalar.
         *  @return The result of dividing the quaternion by the scalar.
         *  @note This operation divides each component of the quaternion by the scalar.
         */
        template <typename Tr>
        friend constexpr quaternion operator/(quaternion lhs, const Tr &rhs)
        {
            lhs /= rhs;
            return lhs;
        }

        /* Quaternion operations */

        /*! @brief Normalizes the quaternion.
         *  @return The normalized quaternion.
         *  @note This operation scales the quaternion to have a magnitude of 1.
         */
        constexpr quaternion normalize(void) { return *this * invsqrt(this->W * this->W + this->X * this->X + this->Y * this->Y + this->Z * this->Z); }

        /*! @brief Normalizes a given quaternion.
         *  @param q The quaternion to normalize.
         *  @return The normalized quaternion.
         *  @note This operation scales the input quaternion to have a magnitude of 1.
         */
        static constexpr quaternion normalize(const quaternion &q) { return q * invsqrt(q.W * q.W + q.X * q.X + q.Y * q.Y + q.Z * q.Z); }

        /*! @brief Returns the conjugate of the quaternion.
         *  @return The conjugate of the quaternion.
         *  @note The conjugate of a quaternion negates the vector part (X, Y, Z) while keeping the scalar part (W) unchanged.
         */
        constexpr quaternion conjugate(void) { return {this->W, -this->X, -this->Y, -this->Z}; }

        /*! @brief Converts the quaternion to a vector.
         *  @return A vector representing the quaternion's vector part.
         *  @note This operation extracts the X, Y, and Z components of the quaternion.
         */
        constexpr vector3d<T> toVector(void) { return {this->X, this->Y, this->Z}; }

        /*! @brief Returns the Euler angles (roll, pitch, yaw) of the quaternion in radians.
         *  @return A vector representing the Euler angles.
         *  @note The Euler angles are calculated using the Tait-Bryan angles (roll, pitch, yaw) convention.
         */
        constexpr vector3d<T> eulerAngles(void) const
        {
            return {
                atan2(2 * (W * X + Y * Z), 1.0 - 2 * (X * X + Y * Y)),                                     // Roll
                -(M_PI / 2) + 2 * atan2(sqrt(1.0 + 2 * (W * Y - X * Z)), sqrt(1.0 - 2 * (W * Y - X * Z))), // Pitch
                atan2(2 * (W * Z + X * Y), 1.0 - 2 * (Y * Y + Z * Z))                                      // Yaw
            };
        }

        /*! @brief Returns the Euler angles (roll, pitch, yaw) of the quaternion in degrees.
         *  @return A vector representing the Euler angles in degrees.
         *  @note The Euler angles are calculated using the Tait-Bryan angles (roll, pitch, yaw) convention.
         */
        constexpr vector3d<T> eulerAnglesDegrees(void) const
        {
            return eulerAngles() * (180.0 / M_PI);
        }

        /*! @brief Creates a quaternion from Euler angles (roll, pitch, yaw) in radians.
         *  @param angles A vector representing the Euler angles.
         *  @return A quaternion representing the rotation.
         *  @note The Euler angles are interpreted using the Tait-Bryan angles (roll, pitch, yaw) convention.
         */
        static constexpr quaternion<T> fromEulerAngles(const vector3d<T> &angles)
        {
            double cy = cos(angles.Z * 0.5);
            double sy = sin(angles.Z * 0.5);
            double cp = cos(angles.Y * 0.5);
            double sp = sin(angles.Y * 0.5);
            double cr = cos(angles.X * 0.5);
            double sr = sin(angles.X * 0.5);

            return {
                cy * cp * cr + sy * sp * sr, // W
                cy * cp * sr - sy * sp * cr, // X
                sy * cp * sr + cy * sp * cr, // Y
                sy * cp * cr - cy * sp * sr  // Z
            };
        }

        /*! @brief Creates a quaternion from Euler angles (roll, pitch, yaw) in degrees.
         *  @param angles A vector representing the Euler angles in degrees.
         *  @return A quaternion representing the rotation.
         *  @note The Euler angles are interpreted using the Tait-Bryan angles (roll, pitch, yaw) convention.
         */
        static constexpr quaternion<T> fromEulerAnglesDegrees(const vector3d<T> &angles)
        {
            return fromEulerAngles(angles * (M_PI / 180.0));
        }

        /*! @brief Creates a quaternion representing a rotation around the X axis.
         *  @param angle The rotation angle in radians.
         *  @return A quaternion representing the rotation.
         */
        static constexpr quaternion<T> fromRotationX(const T angle)
        {
            const double half_angle = angle * 0.5;
            return { cos(half_angle), sin(half_angle), 0.0, 0.0 };
        }

        /*! @brief Creates a quaternion representing a rotation around the Y axis.
         *  @param angle The rotation angle in radians.
         *  @return A quaternion representing the rotation.
         */
        static constexpr quaternion<T> fromRotationY(const T angle)
        {
            const double half_angle = angle * 0.5;
            return { cos(half_angle), 0.0, sin(half_angle), 0.0 };
        }

        /*! @brief Creates a quaternion representing a rotation around the Z axis.
         *  @param angle The rotation angle in radians.
         *  @return A quaternion representing the rotation.
         */
        static constexpr quaternion<T> fromRotationZ(const T angle)
        {
            const double half_angle = angle * 0.5;
            return { cos(half_angle), 0.0, 0.0, sin(half_angle) };
        }

        /*! @brief Creates a quaternion representing a rotation around an arbitrary axis.
         *  @param angle The rotation angle in radians.
         *  @param axis The axis of rotation.
         *  @return A quaternion representing the rotation.
         */
        static constexpr quaternion<T> fromRotation(const T angle, const vector3d<T> &axis)
        {
            const double half_angle = angle * 0.5;
            const double s = sin(half_angle);
            return { cos(half_angle), axis.X * s, axis.Y * s, axis.Z * s };
        }
    };

    /*! @brief Madgwick filter for sensor fusion.
     *  @tparam T Type to use for calculations, must be a floating point type. Default is double.
     */
    template <typename T = double>
    class MadgwickFilter
    {
        quaternion<T> q;
        T beta;
    public:
        /*! @brief Default constructor.
         *  Initializes the filter with a default beta value of 0.1 and a unit quaternion.
         */
        MadgwickFilter() : beta(0.1), q({(T)1.0, (T)0.0, (T)0.0, (T)0.0})
        {
        }

        /*! @brief Constructor with custom beta value.
         *  @param beta The beta value to use for the filter.
         */
        MadgwickFilter(T beta) : beta(beta), q({(T)1.0, (T)0.0, (T)0.0, (T)0.0})
        {
        }

        /*! @brief Set the beta value for the filter.
         *  @param new_beta The new beta value to use for the filter.
         */
        void setBeta(T new_beta) { beta = new_beta; }

        /*! @brief Reset the filter to its initial state.
         *  Sets the quaternion to the unit quaternion.
         */
        void reset(void)
        {
            q = {(T)1.0, (T)0.0, (T)0.0, (T)0.0};
        }

        /*! @brief Compute the new orientation quaternion.
         *  @param gyro The measured gyroscope angular rates in radians per second.
         *  @param accel The measured accelerometer values in g.
         *  @param dt The time step in seconds.
         *  @return The updated orientation quaternion.
         */
        quaternion<T> compute(vector3d<T> gyro, vector3d<T> accel, T dt)
        {
            quaternion<T> qDot;
            quaternion<T> grad;
            quaternion<T> q_w = {0, gyro.X, gyro.Y, gyro.Z}; // Place gyroscope rates into quaternion form
            T F_g[3] = {0};
            T J_g[3][4] = {0};

            accel.normalize();

            // Objective function for gravity
            F_g[0] = 2 * (q.X * q.Z - q.W * q.Y) - accel.X;
            F_g[1] = 2 * (q.W * q.X + q.Y * q.Z) - accel.Y;
            F_g[2] = 2 * ((T)0.5 - q.X * q.X - q.Y * q.Y) - accel.Z;

            // Jacobian matrix for gravity
            J_g[0][0] = -2 * q.Y;
            J_g[0][1] = 2 * q.Z;
            J_g[0][2] = -2 * q.W;
            J_g[0][3] = 2 * q.X;

            J_g[1][0] = 2 * q.X;
            J_g[1][1] = 2 * q.W;
            J_g[1][2] = 2 * q.Z;
            J_g[1][3] = 2 * q.Y;

            // J_g[2][0] = 0; //We can skip this term since it's always 0 
            J_g[2][1] = -4 * q.X;
            J_g[2][2] = -4 * q.Y;
            // J_g[2][3] = 0; //We can skip this term since it's always 0

            // Compute gradient
            grad.W = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1]; // + J_g[2][0] * F_g[2]; //Thrid term always 0, so we can skip it
            grad.X = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
            grad.Y = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
            grad.Z = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1]; // + J_g[2][3] * F_g[2]; //Third term always 0, so we can skip it

            grad.normalize();
            grad *= beta;

            // Integrate angular rate
            q_w = q * (q_w * (T)0.5);
            // Apply gradient descent correction
            q_w -= grad;
            // Integrate to yield orientation
            q += q_w * dt;

            q.normalize();

            return q;
        }
    };
};

#endif // INC_VMATH_H_
