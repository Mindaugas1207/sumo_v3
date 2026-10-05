
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "one_wire_device.h"

/*! @brief Class to represent a line sensor using the OneWire protocol.
 * This class provides methods to read line sensor values and configure various settings such as detection modes and thresholds.
 * It inherits from OneWireDevice, so it uses the same underlying communication mechanism and register access methods.
 */
class LineSensor : public OneWireDevice
{
    static constexpr unsigned int LINE_VALUE_AVG_TOTAL_REG = 0x01;
    static constexpr unsigned int LINE_VALUE_AVG0_REG = 0x02;
    static constexpr unsigned int LINE_VALUE_AVG1_REG = 0x03;
    static constexpr unsigned int LINE_VALUE_RAW0_REG = 0x04;
    static constexpr unsigned int LINE_VALUE_RAW1_REG = 0x05;
    static constexpr unsigned int LINE_DET_OUT_REG = 0x07;

    static constexpr unsigned int LINE_CFG_FSMP_REG = 0xB0;
    static constexpr unsigned int LINE_CFG_NAVG_REG = 0xB1;

    static constexpr unsigned int LINE_CFG_MAX_REG = 0xB7;
    static constexpr unsigned int LINE_CFG_MIN_REG = 0xB8;

    static constexpr unsigned int LINE_CFG_DET_MODE_REG = 0xBA;
    static constexpr unsigned int LINE_CFG_DET_INV_REG = 0xBB;
    static constexpr unsigned int LINE_CFG_DET_LOW_REG = 0xBC;
    static constexpr unsigned int LINE_CFG_DET_HIGH_REG = 0xBD;
public:
    enum DetectionMode
    {
        DETECTION_MODE_THRESHOLD = 0,
        DETECTION_MODE_WINDOW = 1
    };

    struct config
    {
        DeviceIOMode io_mode;
        uint avg_count;
    };

    // Inherit constructors
    using OneWireDevice::OneWireDevice;

    /*! \brief Get the average value of all sensors (S0 and S1 combined)
     * \param average Reference to variable to receive the average value
     * \return true if the value was read successfully, false on error
     */
    bool getTotalAverage(unsigned int &average)
    {
        unsigned int tmp;
        if (readRegister(LINE_VALUE_AVG_TOTAL_REG, &tmp))
        {
            average = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the average value of sensor S0
     * \param average Reference to variable to receive the average value
     * \return true if the value was read successfully, false on error
     */
    bool getS0Average(unsigned int &average)
    {
        unsigned int tmp;
        if (readRegister(LINE_VALUE_AVG0_REG, &tmp))
        {
            average = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the average value of sensor S1
     * \param average Reference to variable to receive the average value
     * \return true if the value was read successfully, false on error
     */
    bool getS1Average(unsigned int &average)
    {
        unsigned int tmp;
        if (readRegister(LINE_VALUE_AVG1_REG, &tmp))
        {
            average = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the raw value of sensor S0
     * \param value Reference to variable to receive the raw value
     * \return true if the value was read successfully, false on error
     */
    bool getS0Raw(unsigned int &value)
    {
        unsigned int tmp;
        if (readRegister(LINE_VALUE_RAW0_REG, &tmp))
        {
            value = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the raw value of sensor S1
     * \param value Reference to variable to receive the raw value
     * \return true if the value was read successfully, false on error
     */
    bool getS1Raw(unsigned int &value)
    {
        unsigned int tmp;
        if (readRegister(LINE_VALUE_RAW1_REG, &tmp))
        {
            value = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Get the detection state
     * \param detected Reference to variable to receive the detection state
     * \return true if the value was read successfully, false on error
     */
    bool getDetectionState(bool &detected)
    {
        unsigned int tmp;
        if (readRegister(LINE_DET_OUT_REG, &tmp))
        {
            detected = tmp != 0;
            return true;
        }
        return false;
    }

    /*! \brief Get the sampling rate
     * \param fsmp Reference to variable to receive the sampling rate
     * \return true if the value was read successfully, false on error
     */
    bool getSamplingRate(unsigned int &fsmp)
    {
        unsigned int tmp;
        if (readRegister(LINE_CFG_FSMP_REG, &tmp))
        {
            fsmp = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the sampling rate
     * \param fsmp Sampling rate to set
     * \return true if the value was written successfully, false on error
     */
    bool setSamplingRate(unsigned int fsmp)
    {
        return writeRegister(LINE_CFG_FSMP_REG, fsmp);
    }

    /*! \brief Get the averaging count (NAVG)
     * \param navg Reference to variable to receive the averaging count
     * \return true if the value was read successfully, false on error
     */
    bool getAveragingCount(unsigned int &navg)
    {
        unsigned int tmp;
        if (readRegister(LINE_CFG_NAVG_REG, &tmp))
        {
            navg = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the averaging count (NAVG)
     * \param navg Averaging count to set
     * \return true if the value was written successfully, false on error
     */
    bool setAveragingCount(unsigned int navg)
    {
        return writeRegister(LINE_CFG_NAVG_REG, navg);
    }

    /*! \brief Get the maximum value
     * \param max Reference to variable to receive the maximum value
     * \return true if the value was read successfully, false on error
     */
    bool getMax(unsigned int &max)
    {
        unsigned int tmp;
        if (readRegister(LINE_CFG_MAX_REG, &tmp))
        {
            max = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the maximum value
     * \param max Maximum value to set
     * \return true if the value was written successfully, false on error
     */
    bool setMax(unsigned int max)
    {
        return writeRegister(LINE_CFG_MAX_REG, max);
    }

    /*! \brief Get the minimum value
     * \param min Reference to variable to receive the minimum value
     * \return true if the value was read successfully, false on error
     */
    bool getMin(unsigned int &min)
    {
        unsigned int tmp;
        if (readRegister(LINE_CFG_MIN_REG, &tmp))
        {
            min = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the minimum value
     * \param min Minimum value to set
     * \return true if the value was written successfully, false on error
     */
    bool setMin(unsigned int min)
    {
        return writeRegister(LINE_CFG_MIN_REG, min);
    }

    /*! \brief Get the detection mode
     * \param mode Reference to variable to receive the detection mode
     * \return true if the value was read successfully, false on error
     */
    bool getDetectionMode(DetectionMode &mode)
    {
        unsigned int tmp;
        if (readRegister(LINE_CFG_DET_MODE_REG, &tmp))
        {
            mode = (DetectionMode)tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the detection mode
     * \param mode Detection mode to set
     * \return true if the value was written successfully, false on error
     */
    bool setDetectionMode(DetectionMode mode)
    {
        return writeRegister(LINE_CFG_DET_MODE_REG, mode);
    }
    
    /*! \brief Get the detection inverted state
     * \param inverted Reference to variable to receive the detection inverted state
     * \return true if the value was read successfully, false on error
     */
    bool getDetectionInverted(bool &inverted)
    {
        unsigned int tmp;
        if (readRegister(LINE_CFG_DET_INV_REG, &tmp))
        {
            inverted = tmp != 0;
            return true;
        }
        return false;
    }

    /*! \brief Set the detection inverted state
     * \param inverted Whether to invert the detection output
     * \return true if the value was written successfully, false on error
     */
    bool setDetectionInverted(bool inverted)
    {
        return writeRegister(LINE_CFG_DET_INV_REG, inverted ? 1 : 0);
    }

    /*! \brief Get the detection lower threshold
     * \param threshold Reference to variable to receive the detection lower threshold
     * \return true if the value was read successfully, false on error
     */
    bool getDetectionLowerThreshold(unsigned int &threshold)
    {
        unsigned int tmp;
        if (readRegister(LINE_CFG_DET_LOW_REG, &tmp))
        {
            threshold = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the detection lower threshold
     * \param threshold Detection lower threshold to set
     * \return true if the value was written successfully, false on error
     */
    bool setDetectionLowerThreshold(unsigned int threshold)
    {
        return writeRegister(LINE_CFG_DET_LOW_REG, threshold);
    }

    /*! \brief Get the detection upper threshold
     * \param threshold Reference to variable to receive the detection upper threshold
     * \return true if the value was read successfully, false on error
     */
    bool getDetectionUpperThreshold(unsigned int &threshold)
    {
        unsigned int tmp;
        if (readRegister(LINE_CFG_DET_HIGH_REG, &tmp))
        {
            threshold = tmp;
            return true;
        }
        return false;
    }

    /*! \brief Set the detection upper threshold
     * \param threshold Detection upper threshold to set
     * \return true if the value was written successfully, false on error
     */
    bool setDetectionUpperThreshold(unsigned int threshold)
    {
        return writeRegister(LINE_CFG_DET_HIGH_REG, threshold);
    }
};

#endif // LINE_SENSOR_H
