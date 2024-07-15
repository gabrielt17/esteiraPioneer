#include "Converter.h"

/// @brief Converts linear speed to RPM. The literal 0.0276 value used
/// at the formula is the wheel radius in meters.
/// @param VALUE Linear speed you desire to convert.
/// @return Converted value in RPM.
float Converter::convertLinearToRPM(float VALUE)
{
    return (30*VALUE)/(M_PI*0.0276);
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

	float rRADS = 36.232*MSG.linear.x+2.246*MSG.angular.z;
	float lRADS = 36.232*MSG.linear.x-2.246*MSG.angular.z;
	float rRPM = Converter::convertAngularToRPM(-rRADS);
	float lRPM = Converter::convertAngularToRPM(-lRADS);;
	return {lRPM, rRPM};
}
