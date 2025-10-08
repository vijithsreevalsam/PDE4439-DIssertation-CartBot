// Ultrasonic sensor library 
// author: Viju.sreevalsam
// this is an addional utility library for cartBot robot 
// to read ultrasonic sensor and publish the data as ROS message
// The sensor used is HC-SR04 ultrasonic sensor
// Trigger pin and Echo pin are to be provided during object creation
// The getData() function returns a sensor_msgs/msg/Range message
// The range is calculated in meters
// The frame_id in the header is set to "ultrasonic_link"   
// not yet tested commpletely with micro-ROS
// inital work is promising.



#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>
#include <sensor_msgs/msg/range.h>

class UltrasonicSensor
{
public:
    UltrasonicSensor(int trigger_pin, int echo_pin) : 
        trigger_pin_(trigger_pin), 
        echo_pin_(echo_pin)
    {}

    // Initialize the sensor pins
    void init()
    {
        pinMode(trigger_pin_, OUTPUT);
        pinMode(echo_pin_, INPUT);
    }

    // Read the sensor and return the data in a ROS message
    sensor_msgs__msg__Range getData()
    {
        sensor_msgs__msg__Range range_msg;

        // --- Populate the message's static fields ---
        range_msg.header.frame_id.data = (char *) "ultrasonic_link";
        range_msg.header.frame_id.size = strlen(range_msg.header.frame_id.data);
        range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
        range_msg.field_of_view = 0.26; // ~15 degrees in radians
        range_msg.min_range = 0.02;     // in meters
        range_msg.max_range = 4.0;      // in meters

        // --- Perform the measurement ---
        // Clears the trigger_pin
        digitalWrite(trigger_pin_, LOW);
        delayMicroseconds(2);

        // Sets the trigger_pin on HIGH state for 10 micro seconds
        digitalWrite(trigger_pin_, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger_pin_, LOW);

        // Reads the echo_pin, returns the sound wave travel time in microseconds
        long duration = pulseIn(echo_pin_, HIGH);

        // Calculating the distance in meters
        // Speed of sound wave divided by 2 (go and back)
        float distance_in_meters = (duration * 0.0343) / 200.0;

        // Ensure the range is within the defined bounds
        if (distance_in_meters < range_msg.min_range) {
            distance_in_meters = range_msg.min_range;
        } else if (distance_in_meters > range_msg.max_range) {
            distance_in_meters = range_msg.max_range;
        }

        range_msg.range = distance_in_meters;

        return range_msg;
    }

private:
    int trigger_pin_;
    int echo_pin_;
};

#endif // ULTRASONIC_SENSOR_H
