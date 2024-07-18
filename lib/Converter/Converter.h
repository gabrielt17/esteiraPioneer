/*
*    ASTROS, a Pioneer P3-DX inspired robot.
*    Copyright (C) <2024>  <Gabriel Víctor and Hiago Batista>
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


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