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

	if (PWM < 0) {Navigator::moveBackwards(-PWM);}
	else {
		lMotor.setAntiClockwise();
		rMotor.setClockwise();
		Navigator::changeSpeed(PWM);
	}

}

/** @brief Drives the robot backwards.
* Negative PWM values will drive the robot in the opposite direction.
* @param PWM PWM that will be sent to the PWM channel.
*/
void Navigator::moveBackwards(const int16_t PWM) {

	if (PWM < 0) {Navigator::moveAhead(-PWM);}
	else {
		lMotor.setClockwise();
		rMotor.setAntiClockwise();
		Navigator::changeSpeed(PWM);
	}
}

void Navigator::halt() {

	Navigator::changeSpeed(0);
}

void Navigator::turnLeft(const int16_t PWM) {

	if (PWM < 0) {Navigator::turnRight(-PWM);}
	else {
		lMotor.setClockwise();
		rMotor.setClockwise();
		Navigator::changeSpeed(PWM);
	}
}

void Navigator::turnRight(const int16_t PWM) {

	if (PWM < 0) {Navigator::turnRight(-PWM);}
	else {
		lMotor.setAntiClockwise();
		rMotor.setAntiClockwise();
		Navigator::changeSpeed(PWM);
	}
}

void Navigator::changeSpeed(const int16_t PWM) {

	lMotor.setSpeed(PWM);
	rMotor.setSpeed(PWM);
}