#ifndef SENSORS_H
#define SENSORS_H

#include "distance_sensor.h"

#include <vector>

struct SensorData
{
    std::vector<int> distanceSensorValues;
    bool lineSensorLeft;
    bool lineSensorRight;
};

void sensors_init(void);
void handle_sensors(void);

SensorData get_sensor_data(void);


#endif // SENSORS_H