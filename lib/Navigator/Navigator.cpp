#include "Navigator.h"
#include "Arduino.h"

/** @brief Class contructor.
* Creates the Navigator object
*/
Navigator::Navigator(Controller& LCONTROLLER, Controller& RCONTROLLER,\
Motor &LMOTOR, Motor &RMOTOR, Encoder &LENCODER, Encoder &RENCODER, float RADIUS, float WHEELDIST) 
: lController(LCONTROLLER), rController(RCONTROLLER), lmotor(LMOTOR),\
rmotor(RMOTOR), lencoder(LENCODER), rencoder(RENCODER), radius(RADIUS) {
}

/** @brief Drives the robot forward.
* Negative PWM values will drive the robot in the opposite direction.
* @param MSG PWM that will be sent to the PWM channel.
*/
void Navigator::move(geometry_msgs::Twist MSG) {
	if ((MSG.linear.x != 0) && (MSG.angular.z == 0)) {
		float RPMlinear = Navigator::linearToRPM(MSG);
		// Serial.printf("RPMlinear: %3.3f\n", RPMlinear);
		float lpwm = lController.controlMotor(RPMlinear, lencoder.getRPM(lmotor.isClockwise));
		float rpwm = rController.controlMotor(-RPMlinear, rencoder.getRPM(rmotor.isClockwise));
		lmotor.setSpeed(lpwm);
		rmotor.setSpeed(rpwm);
		

	} else if ((MSG.linear.x == 0 ) && (MSG.angular.z != 0)) {
		float RPMangular = Navigator::angularToRPM(MSG);
		// Serial.printf("RPMangular %3.3f\n", RPMangular);
		float lpwm = lController.controlMotor(-RPMangular/2, lencoder.getRPM(lmotor.isClockwise));
		float rpwm = rController.controlMotor(-RPMangular/2, rencoder.getRPM(rmotor.isClockwise));
		lmotor.setSpeed(lpwm);
		rmotor.setSpeed(rpwm);
		Serial.printf(">lpwm:%3.3f\n>rpwm:%3.3f\n",lpwm, rpwm);
	} else {
		Serial.println("ENTREI NO ZERO");
		lmotor.setSpeed(0);
		rmotor.setSpeed(0);
	}
}

float Navigator::linearToRPM(geometry_msgs::Twist MSG) {
	return (30*MSG.linear.x)/(M_PI*Navigator::radius);
}

float Navigator::angularToRPM(geometry_msgs::Twist MSG) {
	return (30*MSG.angular.z)/(M_PI);
}