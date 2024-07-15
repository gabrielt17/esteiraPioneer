#ifndef CONVERTER_H
#define CONVERTER_H

#include <Arduino.h>
#include <geometry_msgs/Twist.h>
#include <ros.h>

/// @brief Stores two members: Left and right motor RPM values.
struct MotorVel {
    float lrpm;
    float rrpm;
};
/// @brief Static class that converts ROS messages to RPM values.
class Converter {

    private:

        static float convertAngularToRPM(float VALUE);
        static float convertLinearToRPM(float VALUE);

    public:

        static MotorVel convertMessage(geometry_msgs::Twist MSG);
};

#endif // CONVERTER_H