#include "Converter.h"

float Converter::convertLinearToRPM(float VALUE)
{
    return (30*VALUE)/(M_PI*0.0276);
}

float Converter::convertAngularToRPM(float VALUE) {
	return (30*VALUE)/(M_PI);
}

MotorVel Converter::convertMessage(geometry_msgs::Twist MSG) {

	float rRADS = 36.232*MSG.linear.x+2.246*MSG.angular.z;
	float lRADS = 36.232*MSG.linear.x-2.246*MSG.angular.z;
	float rRPM = Converter::convertAngularToRPM(-rRADS);
	float lRPM = Converter::convertAngularToRPM(-lRADS);;
	return {lRPM, rRPM};
}
