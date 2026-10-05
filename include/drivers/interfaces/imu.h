#ifndef IMU_H
#define IMU_H

#include "vmath.h"

/*! @brief Abstract base class for Inertial Measurement Units (IMUs). 
 */
class IMU
{
public:
    /*! @brief Reads the accelerometer and gyroscope data from the IMU.
     *  @param acceleration A reference to a vector3d object to store the accelerometer data. The values are in g (gravitational acceleration).
     *  @param angularRate A reference to a vector3d object to store the gyroscope data. The values are in radians per second (rad/s).
     *  @return True if the data was successfully read, false otherwise.
     */
    virtual bool readData(vmath::vector3d<double>& acceleration, vmath::vector3d<double>& angularRate) = 0;

    /*! @brief Checks if new data is available from the IMU.
     *  @return True if new data is available, false otherwise.
     */
    virtual bool isDataReady() const = 0;
};

#endif // IMU_H

