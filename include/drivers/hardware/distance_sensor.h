
#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include "one_wire_device.h"

/*! @brief Class to represent a distance sensor using the OneWire protocol.
 * This class provides methods to read distance measurements and related data from the sensor, as well as configure various settings such as timing, offsets, crosstalk compensation, and detection modes.
 * It inherits from OneWireDevice, so it uses the same underlying communication mechanism and register access methods.
 */
class DistanceSensor : public OneWireDevice
{
    static constexpr unsigned int RANGE_FLAG_REG = (0x00);
    static constexpr unsigned int RANGE_DIST_MM_REG = (0x01);
    static constexpr unsigned int RANGE_SIG_RATE_REG = (0x02);
    static constexpr unsigned int RANGE_AMB_RATE_REG = (0x03);
    static constexpr unsigned int RANGE_SIGMA_REG = (0x04);
    static constexpr unsigned int RANGE_SPADS_REG = (0x05);
    static constexpr unsigned int RANGE_DIST_CM_REG = (0x06);
    static constexpr unsigned int RANGE_DET_OUT_REG = (0x07);

    static constexpr unsigned int RANGE_CFG_TIMING_REG = (0xB0);
    static constexpr unsigned int RANGE_CFG_OFFSET_REG = (0xB1);
    static constexpr unsigned int RANGE_CFG_XTALK_REG = (0xB2);
    static constexpr unsigned int RANGE_CFG_CHECKS_REG = (0xB4);
    static constexpr unsigned int RANGE_CFG_SIGNAL_REG = (0xB5);
    static constexpr unsigned int RANGE_CFG_SIGMA_REG = (0xB6);
    static constexpr unsigned int RANGE_CFG_MIN_REG = (0xB7);
    static constexpr unsigned int RANGE_CFG_MAX_REG = (0xB8);
    static constexpr unsigned int RANGE_CFG_DET_MODE_REG = (0xBA);
    static constexpr unsigned int RANGE_CFG_DET_INV_REG = (0xBB);
    static constexpr unsigned int RANGE_CFG_DET_LOW_REG = (0xBC);
    static constexpr unsigned int RANGE_CFG_DET_HIGH_REG = (0xBD);
public:
    enum RangeFlags
    {
        RANGE_FLAG_VALID = 0x01, // measurement valid
        RANGE_FLAG_WRAP = 0x02, // range wrapped around
        RANGE_FLAG_PHASE = 0x04, // out of phase
        RANGE_FLAG_SIGNAL_LOW = 0x08, // signal too low
        RANGE_FLAG_SIGMA_LOW = 0x10, // sigma too low
        RANGE_FLAG_SIGMA_HIGH = 0x20, // sigma too high
        RANGE_FLAG_MIN = 0x40, // below minimum range
        RANGE_FLAG_MAX = 0x80 // above maximum range
    };

    enum DetectionMode
    {
        DETECTION_MODE_ANY_VALID_RANGE = 0,
        DETECTION_MODE_THRESHOLD = 1,
        DETECTION_MODE_WINDOW = 2
    };

    struct config
    {
        DeviceIOMode io_mode;
        DeviceSerialBaudRate serial_baud_rate;
        unsigned int range_timing_ms;
        unsigned int max_range_mm;
        unsigned int min_range_mm;
        DetectionMode detection_mode;
        unsigned int detection_threshold_low_mm;
        unsigned int detection_threshold_high_mm;
    };

    // Inherit constructors
    using OneWireDevice::OneWireDevice;

    /*! \brief Get the range status flags from the last measurement
     * \param flags Reference to variable to receive the flags as a bitmask of RangeFlags values
     * \return true if the flags were successfully read, false on error.
     */
    bool getRangeFlags(RangeFlags &flags)
    {
        unsigned int tmp;
        if (readRegister(RANGE_FLAG_REG, &tmp))
        {
            flags = (RangeFlags)tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the last measured distance
     * \param distance Reference to variable to receive the distance in mm
     * \return true if the distance was successfully read, false on error.
     */
    bool getDistance(unsigned int &distance)
    {
        unsigned int tmp;
        if (readRegister(RANGE_DIST_MM_REG, &tmp))
        {
            distance = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the signal rate from the last measurement
     * \param signalRate Reference to variable to receive the signal rate in MCPS (Mega Counts Per Second)
     * \return true if the signal rate was successfully read, false on error
     */
    bool getSignalRate(unsigned int &signalRate)
    {
        unsigned int tmp;
        if (readRegister(RANGE_SIG_RATE_REG, &tmp))
        {
            signalRate = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the ambient rate from the last measurement
     * \param ambientRate Reference to variable to receive the ambient rate in MCPS (Mega Counts Per Second)
     * \return true if the ambient rate was successfully read, false on error
     */
    bool getAmbientRate(unsigned int &ambientRate)
    {
        unsigned int tmp;
        if (readRegister(RANGE_AMB_RATE_REG, &tmp))
        {
            ambientRate = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the signal sigma from the last measurement
     * \param sigma Reference to variable to receive the signal sigma in mm
     * \return true if the signal sigma was successfully read, false on error
     */
    bool getSignalSigma(unsigned int &sigma)
    {
        unsigned int tmp;
        if (readRegister(RANGE_SIGMA_REG, &tmp))
        {
            sigma = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the number of SPADs used in the last measurement
     * \param count Reference to variable to receive the SPAD count
     * \return true if the SPAD count was successfully read, false on error
    */
    bool getSPADCount(unsigned int &count)
    {
        unsigned int tmp;
        if (readRegister(RANGE_SPADS_REG, &tmp))
        {
            count = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the last measured distance
     * \param distance Reference to variable to receive the distance in cm
     * \return true if the distance was successfully read, false on error.
     */
    bool getDistance_cm(unsigned int &distance)
    {
        unsigned int tmp;
        if (readRegister(RANGE_DIST_CM_REG, &tmp))
        {
            distance = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the detection output state
     * \param detected Reference to variable to receive the result
     * \return true if the command was acknowledged, false on error
    */
    bool getDetectionState(bool &detected)
    {
        unsigned int tmp;
        if (readRegister(RANGE_DET_OUT_REG, &tmp))
        {
            detected = tmp != 0;
            return true;
        }
        return false;
    }

    /*! \brief Get the range timing budget
     * \param timing Reference to variable to receive the timing budget in ms
     * \return true if the timing budget was successfully read, false on error
     */
    bool getRangeTiming(unsigned int &timing)
    {
        unsigned int tmp;
        if (readRegister(RANGE_CFG_TIMING_REG, &tmp))
        {
            timing = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the range timing budget
     * \param timing Timing budget in ms (10 to 200)
     * \return true if the command was acknowledged, false on error
     */
    bool setRangeTiming(unsigned int timing)
    {
        return writeRegister(RANGE_CFG_TIMING_REG, timing);
    }

    /*! \brief Get the range offset
     * \param offset Reference to variable to receive the offset in mm
     * \return true if the offset was successfully read, false on error
     */
    bool getRangeOffset(unsigned int &offset)
    {
        unsigned int tmp;
        if (readRegister(RANGE_CFG_OFFSET_REG, &tmp))
        {
            offset = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the range offset
     * \param offset Offset in mm (-32768 to 32767)
     * \return true if the command was acknowledged, false on error
     */
    bool setRangeOffset(unsigned int offset)
    {
        return writeRegister(RANGE_CFG_OFFSET_REG, offset);
    }

    /*! \brief Get the crosstalk compensation value
     * \param xtalk Reference to variable to receive the crosstalk in counts
     * \return true if the crosstalk was successfully read, false on error
     */
    bool getRangeXTALK(unsigned int &xtalk)
    {
        unsigned int tmp;
        if (readRegister(RANGE_CFG_XTALK_REG, &tmp))
        {
            xtalk = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the crosstalk compensation value
     * \param xtalk Crosstalk in counts (0 to 32767)
     * \return true if the command was acknowledged, false on error
     */
    bool setRangeXTALK(unsigned int xtalk)
    {
        return writeRegister(RANGE_CFG_XTALK_REG, xtalk);
    }

    /*! \brief Get the range check flags
     * \param flags Reference to variable to receive the flags as a bitmask of RangeFlags values
     * \return true if the flags were successfully read, false on error.
     */
    bool getRangeChecks(RangeFlags &flags)
    {
        unsigned int tmp;
        if (readRegister(RANGE_CFG_CHECKS_REG, &tmp))
        {
            flags = (RangeFlags)tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the range check flags
     * \param flags RangeFlags bitmask
     * \return true if the command was acknowledged, false on error
     */
    bool setRangeChecks(RangeFlags flags)
    {
        return writeRegister(RANGE_CFG_CHECKS_REG, flags);
    }

    /*! \brief Get the signal limit used in the last measurement
     * \param limit Reference to variable to receive the signal limit in MCPS (Mega Counts Per Second)
     * \return true if the signal limit was successfully read, false on error
     */
    bool getRangeSignalLimit(unsigned int &limit)
    {
        unsigned int tmp;
        if (readRegister(RANGE_CFG_SIGNAL_REG, &tmp))
        {
            limit = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the signal limit used in measurements
     * \param limit Signal limit in MCPS (Mega Counts Per Second)
     * \return true if the command was acknowledged, false on error
     */
    bool setRangeSignalLimit(unsigned int limit)
    {
        return writeRegister(RANGE_CFG_SIGNAL_REG, limit);
    }

    /*! \brief Get the sigma limit used in the last measurement
     * \param limit Reference to variable to receive the sigma limit in mm
     * \return true if the sigma limit was successfully read, false on error
     */
    bool getRangeSigmaLimit(unsigned int &limit)
    {
        unsigned int tmp;
        if (readRegister(RANGE_CFG_SIGMA_REG, &tmp))
        {
            limit = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the sigma limit used in measurements
     * \param limit Sigma limit in mm
     * \return true if the command was acknowledged, false on error
     */
    bool setRangeSigmaLimit(unsigned int limit)
    {
        return writeRegister(RANGE_CFG_SIGMA_REG, limit);
    }

    /*! \brief Get the minimum range distance
     * \param min Reference to variable to receive the minimum distance in mm
     * \return true if the minimum distance was successfully read, false on error
     */
    bool getRangeMin(unsigned int &min)
    {
        unsigned int tmp;
        if (readRegister(RANGE_CFG_MIN_REG, &tmp))
        {
            min = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the minimum range distance
     * \param min Minimum distance in mm
     * \return true if the command was acknowledged, false on error
     */
    bool setRangeMin(unsigned int min)
    {
        return writeRegister(RANGE_CFG_MIN_REG, min);
    }

    /*! \brief Get the maximum range distance
     * \param max Reference to variable to receive the maximum distance in mm
     * \return true if the maximum distance was successfully read, false on error
     */
    bool getRangeMax(unsigned int &max)
    {
        unsigned int tmp;
        if (readRegister(RANGE_CFG_MAX_REG, &tmp))
        {
            max = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the maximum range distance
     * \param max Maximum distance in mm
     * \return true if the command was acknowledged, false on error
     */
    bool setRangeMax(unsigned int max)
    {
        return writeRegister(RANGE_CFG_MAX_REG, max);
    }

    /*! \brief Get the detection mode
     * \param mode Reference to variable to receive the DetectionMode value
     * \return true if the detection mode was successfully read, false on error
     */
    bool getDetectionMode(DetectionMode &mode)
    {
        unsigned int tmp;
        if (readRegister(RANGE_CFG_DET_MODE_REG, &tmp))
        {
            mode = (DetectionMode)tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the detection mode
     * \param mode DetectionMode value to set
     * \return true if the command was acknowledged, false on error
     */
    bool setDetectionMode(DetectionMode mode)
    {
        return writeRegister(RANGE_CFG_DET_MODE_REG, mode);
    }

    /*! \brief Get whether the detection output is inverted
     * \param inverted Reference to variable to receive the result
     * \return true if the command was acknowledged, false on error
     */
    bool getDetectionInverted(bool &inverted)
    {
        unsigned int tmp;
        if (readRegister(RANGE_CFG_DET_INV_REG, &tmp))
        {
            inverted = tmp != 0;
            return true;
        }
        return false;
    }

    /*! \brief Set whether the detection output is inverted
     * \param inverted true to invert the output, false for normal operation
     * \return true if the command was acknowledged, false on error
     */
    bool setDetectionInverted(bool inverted)
    {
        return writeRegister(RANGE_CFG_DET_INV_REG, inverted ? 1 : 0);
    }

    /*! \brief Get the lower threshold for detection mode
     * \param threshold Reference to variable to receive the lower threshold in mm
     * \return true if the lower threshold was successfully read, false on error
     */
    bool getDetectionLowerThreshold(unsigned int &threshold)
    {
        unsigned int tmp;
        if (readRegister(RANGE_CFG_DET_LOW_REG, &tmp))
        {
            threshold = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the lower threshold for detection mode
     * \param threshold Lower threshold in mm
     * \return true if the command was acknowledged, false on error
     */
    bool setDetectionLowerThreshold(unsigned int threshold)
    {
        return writeRegister(RANGE_CFG_DET_LOW_REG, threshold);
    }

    /*! \brief Get the upper threshold for detection mode
     * \param threshold Reference to variable to receive the upper threshold in mm
     * \return true if the upper threshold was successfully read, false on error
     */
    bool getDetectionUpperThreshold(unsigned int &threshold)
    {
        unsigned int tmp;
        if (readRegister(RANGE_CFG_DET_HIGH_REG, &tmp))
        {
            threshold = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the upper threshold for detection mode
     * \param threshold Upper threshold in mm
     * \return true if the command was acknowledged, false on error
     */
    bool setDetectionUpperThreshold(unsigned int threshold)
    {
        return writeRegister(RANGE_CFG_DET_HIGH_REG, threshold);
    }
};

#endif // DISTANCE_SENSOR_H
