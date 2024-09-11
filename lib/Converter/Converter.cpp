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

#include "Converter.h"

/// @brief Converts linear speed to RPM. The literal 0.0276 value used
/// at the formula is the wheel radius in meters.
/// @param VALUE Linear speed you desire to convert.
/// @return Converted value in RPM.
float Converter::convertLinearToRPM(float VALUE)
{
    return (30*VALUE)/(M_PI*0.0275);
}

/// @brief Converts angular speed to RPM.
/// @param VALUE Angular speed you desire to convert.
/// @return Converted value in RPM.
float Converter::convertAngularToRPM(float VALUE) {
	return (30*VALUE)/(M_PI);
}

/// @brief Convert ROS geometry_msgs::Twist to motorVel struct.
/// @param MSG ROS geometry_msgs::Twist you desire to convert.
/// @return RPM value of each motor in a motorVel struct.
MotorVel Converter::convertMessage(geometry_msgs::Twist MSG) {

	float rRADS = 36.363*MSG.linear.x+2.244*MSG.angular.z;
	float lRADS = 36.363*MSG.linear.x-2.244*MSG.angular.z;
	float rRPM = Converter::convertAngularToRPM(-rRADS);
	float lRPM = Converter::convertAngularToRPM(-lRADS);;
	return {lRPM, rRPM};
}
