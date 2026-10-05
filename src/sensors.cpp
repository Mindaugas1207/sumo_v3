
#include "config.h"
#include "hardware.h"
#include "sensors.h"
#include <array>

constexpr int MAX_DISTANCE = 1000;

std::array<int, num_distance_sensors> distanceSensorValues;
bool line_sensor_left;
bool line_sensor_right;

void line_sensor_callback(void* context, uint32_t events);

void sensors_init(void)
{
    // Initialize distance sensor values to MAX_DISTANCE
    for (int i = 0; i < num_distance_sensors; i++)
    {
        distanceSensorValues[i] = MAX_DISTANCE;
    }
    // Initialize line sensors to false
    line_sensor_left = false;
    line_sensor_right = false;
    line_sensor_events[front_left_line_sensor_index].setCallback(line_sensor_callback, reinterpret_cast<void*>(front_left_line_sensor_index));
    line_sensor_events[front_right_line_sensor_index].setCallback(line_sensor_callback, reinterpret_cast<void*>(front_right_line_sensor_index));
}

void handle_sensors(void)
{
    // Read distance sensor values
    for (int i = 0; i < num_distance_sensors; i++)
    {
        unsigned int distance;
        if (!distanceSensors[i].getDistance(distance))
        {
            distanceSensorValues[i] = MAX_DISTANCE;
            utils::error_printf("Failed to read distance sensor %d\n", i);
        }
        if (distance > 0 && distance < MAX_DISTANCE)
        {
            distanceSensorValues[i] = distance;
        }
        else
        {
            distanceSensorValues[i] = MAX_DISTANCE;
        }
    }
}

SensorData get_sensor_data(void)
{
    SensorData data;
    for (int i = 0; i < num_distance_sensors; i++)
    {
        data.distanceSensorValues.push_back(distanceSensorValues[i]);
    }
    data.lineSensorLeft = line_sensor_left;
    data.lineSensorRight = line_sensor_right;
    return data;
}

void line_sensor_callback(void* context, uint32_t events)
{
    uint index = reinterpret_cast<uintptr_t>(context);

    utils::debug_printf("Line sensor %d event: 0x%x\n", index, events);

    bool state;
    if (events & GPIO_IRQ_EDGE_RISE)
    {
        state = true;
    }
    else if (events & GPIO_IRQ_EDGE_FALL)
    {
        state = false;
    }
    else
    {
        return; // Unhandled event type
    }
    
    if (index == front_left_line_sensor_index)
    {
        line_sensor_left = state;
    }
    else if (index == front_right_line_sensor_index)
    {
        line_sensor_right = state;
    }
}
