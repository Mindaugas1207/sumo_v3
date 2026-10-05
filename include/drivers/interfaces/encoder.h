#ifndef ENCODER_H
#define ENCODER_H

/*! @brief Abstract base class for encoders. Defines the interface for reading the angle in radians.
 */
class Encoder
{
public:
    /*! @brief Reads the angle from the encoder in radians.
     *  @param angle A reference to a double to store the angle in radians.
     *  @return True if the angle was successfully read, false otherwise.
     */
    virtual bool readAngleRadians(double& angle) const = 0;
    /*! @brief Reads the angle from the encoder in degrees.
     *  @param angle A reference to a double to store the angle in degrees.
     *  @return True if the angle was successfully read, false otherwise.
     */
    virtual bool readAngleDegrees(double& angle) const = 0;
};

#endif