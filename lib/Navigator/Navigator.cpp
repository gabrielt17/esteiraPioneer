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
void Navigator::moveAhead(const int PWM) {

	if (PWM < 0) {Navigator::moveBackwards(-PWM);}
	else {
		lMotor.isNegative = false;
		rMotor.isNegative = false;
		lMotor.setAntiClockwise();
		rMotor.setClockwise();
		Navigator::changeSpeed(PWM, PWM);
	}

}

/** @brief Drives the robot backwards.
* Negative PWM values will drive the robot in the opposite direction.
* @param PWM PWM that will be sent to the PWM channel.
*/
void Navigator::moveBackwards(const int PWM) {

	if (PWM < 0) {Navigator::moveAhead(-PWM);}
	else {
		lMotor.isNegative = true;
		rMotor.isNegative = true;
		lMotor.setClockwise();
		rMotor.setAntiClockwise();
		Navigator::changeSpeed(PWM, PWM);
	}
}

void Navigator::halt() {

	Navigator::changeSpeed(0, 0);
}

void Navigator::turnLeft(const int lPWM, const int rPWM) {

	if ((lPWM && rPWM) < 0) {Navigator::turnRight(-lPWM, -rPWM);}
	else {
		lMotor.isNegative = true;
		rMotor.isNegative = false;
		lMotor.setClockwise();
		rMotor.setClockwise();
		Navigator::changeSpeed(lPWM, rPWM);
	}
}

void Navigator::turnRight(const int lPWM, const int rPWM) {

	if ((lPWM && rPWM) < 0) {Navigator::turnRight(-lPWM, -rPWM);}
	else {
		lMotor.isNegative = false;
		rMotor.isNegative = true;
		lMotor.setAntiClockwise();
		rMotor.setAntiClockwise();
		Navigator::changeSpeed(lPWM, rPWM);
	}
}

void Navigator::changeSpeed(const int lPWM, const int rPWM) {

	lMotor.setSpeed(lPWM);
	rMotor.setSpeed(rPWM);
}