#ifndef CONVERTER
#define CONVERTER

#include "Arduino.h"
#include <geometry_msgs/Twist.h>
#include <ros.h>

struct motorRPM {
	float lrpm;
	float rrpm;
};

float convertAngularToRPM(float VALUE);
float convertLinearToRPM(float VALUE);
motorRPM convertMessage(geometry_msgs::Twist MSG);

float convertLinearToRPM(float VALUE)
{
    return (30*VALUE)/(M_PI*0.0276);
}

float convertAngularToRPM(float VALUE) {
	return (30*VALUE)/(M_PI);
}

motorRPM convertMessage(geometry_msgs::Twist MSG) {
	float u = MSG.linear.x;
	float w = MSG.angular.z;
	float rRADS = 36.232*u+2.246*w;
	float lRADS = 36.232*u-2.246*w;
	float rRPM = convertAngularToRPM(-rRADS);
	float lRPM = convertAngularToRPM(lRADS);
	return {lRPM, rRPM};
}

#endif // CONVERTER