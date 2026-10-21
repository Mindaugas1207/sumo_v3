
#include "config.h"
#include "hardware.h"
#include "sensors.h"
#include <array>

constexpr double distanceFilter = 0.8;

std::array<int, num_distance_sensors> distanceSensorValues;


void sensors_init(void)
{
    // Initialize distance sensor values to MAX_DISTANCE
    for (int i = 0; i < num_distance_sensors; i++)
    {
        distanceSensorValues[i] = MAX_DISTANCE;
    }
    
}

void handle_sensors(void)
{
    // Read distance sensor values
    for (int i = 0; i < num_distance_sensors; i++)
    {
        unsigned int distance;
        if (!distanceSensors[i].getDistance(distance))
        {
            distance = MAX_DISTANCE;
            utils::error_printf("Failed to read distance sensor %d\n", i);
        }
        if (distance <= 0 || distance >= MAX_DISTANCE)
        {
            distance = MAX_DISTANCE;
        }
        
        distanceSensorValues[i] = static_cast<int>(distanceFilter * distanceSensorValues[i] + (1.0 - distanceFilter) * distance);
    }
}

SensorData get_sensor_data(void)
{
    SensorData data;
    data.targetDetected = false;
    for (int i = 0; i < num_distance_sensors; i++)
    {
        data.distanceSensorValues.push_back(distanceSensorValues[i]);
        bool detected = distanceSensorValues[i] < DETECTION_THRESHOLD;
        data.distanceSensorDetected[i] = detected;
        data.targetDetected = data.targetDetected || detected;
    }
    return data;
}


