#include "Navigator.h"
#include "Arduino.h"

/** @brief Class contructor.
* Creates the Navigator object
*/
Navigator::Navigator(Motor& LMOTOR, Motor&  RMOTOR) 
: lMotor(LMOTOR), rMotor(RMOTOR) {
}

/** @brief Drives the robot foward.
* Negative PWM values will drive the robot in the opposite direction.
* @param PWM PWM that will be sent to the PWM channel.
*/
void Navigator::moveAhead(const int16_t PWM) {

	if (PWM < 0) {
		Navigator::moveBackwards(-PWM);
	} else {
		Navigator::changeSpeed(PWM, -PWM);
	}

}

/** @brief Drives the robot backwards.
* Negative PWM values will drive the robot in the opposite direction.
* @param PWM PWM that will be sent to the PWM channel.
*/
void Navigator::moveBackwards(const int16_t PWM) {

	if (PWM < 0) {Navigator::moveAhead(-PWM);}
	else {
		Navigator::changeSpeed(-PWM, PWM);
	}
}

void Navigator::halt() {

	Navigator::changeSpeed(0, 0);
}

void Navigator::turnLeft(const int16_t lPWM, const int16_t rPWM) {

	uint16_t lpwm = fabs(lPWM);
	uint16_t rpwm = fabs(rPWM);
	if ((lPWM && rPWM) < 0) {
		Navigator::turnRight(-lPWM, -rPWM);
	} else {
		Navigator::changeSpeed(-lpwm, -rpwm);
	}
}

void Navigator::turnRight(const int16_t lPWM, const int16_t rPWM) {

	uint16_t lpwm = fabs(lPWM);
	uint16_t rpwm = fabs(rPWM);
	Serial.printf("%d; %d\n", lpwm, rpwm);
	if ((lPWM && rPWM) < 0) {
		Navigator::turnRight(-lPWM, -rPWM);
	} else {
		Navigator::changeSpeed(lpwm, rpwm);
	}
}

void Navigator::changeSpeed(const int lPWM, const int rPWM) {

	lMotor.setSpeed(lPWM);
	rMotor.setSpeed(rPWM);
}