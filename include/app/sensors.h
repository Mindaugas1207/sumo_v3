#ifndef SENSORS_H
#define SENSORS_H

#include "distance_sensor.h"

#include <vector>

struct SensorData
{
    std::vector<int> distanceSensorValues;
    bool distanceSensorDetected[7];
    bool lineSensorLeft;
    bool lineSensorRight;
    bool targetDetected;
};

void sensors_init(void);
void handle_sensors(void);

SensorData get_sensor_data(void);

inline constexpr int MAX_DISTANCE = 1000;
inline constexpr int DETECTION_THRESHOLD = 400;

#endif // SENSORS_H