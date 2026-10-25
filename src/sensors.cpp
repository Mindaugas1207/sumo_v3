
#include "config.h"
#include "hardware.h"
#include "sensors.h"
#include <array>
#include <atomic>
constexpr double distanceFilter = 0.0;

using SensorArray = std::array<int, num_distance_sensors>;

SensorArray filteredBuffer;
SensorArray sensorBufferA;
SensorArray sensorBufferB;
std::atomic<SensorArray*> activeBuffer{&sensorBufferA};

void sensors_init(void)
{
    // Initialize distance sensor values to MAX_DISTANCE
    for (int i = 0; i < num_distance_sensors; i++)
    {
        filteredBuffer[i] = MAX_DISTANCE;
        sensorBufferA[i] = MAX_DISTANCE;
        sensorBufferB[i] = MAX_DISTANCE;
    }
}

void handle_sensors(void)
{
    // Read distance sensor values
    static SensorArray* writeBuffer = &sensorBufferB;

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

        filteredBuffer[i] = static_cast<int>(distanceFilter * filteredBuffer[i] + (1.0 - distanceFilter) * distance);
        (*writeBuffer)[i] = filteredBuffer[i];
    }
    activeBuffer.store(writeBuffer, std::memory_order_release);
    writeBuffer = (writeBuffer == &sensorBufferA) ? &sensorBufferB : &sensorBufferA;
}

SensorData get_sensor_data(void)
{
    SensorArray* sensors = activeBuffer.load(std::memory_order_acquire);

    SensorData data;
    data.targetDetected = false;
    for (int i = 0; i < num_distance_sensors; i++)
    {
        int value = (*sensors)[i];
        data.distanceSensorValues[i] = value;
        bool detected = value < DETECTION_THRESHOLD;
        data.distanceSensorDetected[i] = detected;
        data.targetDetected = data.targetDetected || detected;
    }
    return data;
}


