#ifndef SENSORS_H
#define SENSORS_H

#include "distance_sensor.h"

#include <array>

struct SensorData
{
    std::array<int, 7> distanceSensorValues;
    bool distanceSensorDetected[7];
    bool targetDetected;

    int nearestTargetDistance;
    double nearestTargetAngle;
};

void sensors_init(void);
void handle_sensors(void);

SensorData get_sensor_data(void);

inline constexpr int MAX_DISTANCE = 1000;
inline constexpr int DETECTION_THRESHOLD = 400;

#endif // SENSORS_H