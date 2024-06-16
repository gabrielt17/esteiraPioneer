#include "Navigator.h"
#include "Arduino.h"

/** @brief Class contructor.
* This version of the constructor requires the user to specify the operation channel
*/
Motor::Motor(const int In1, const int In2, const int Pwm, const int Channel) {
	
	Motor::in1 = In1;
	Motor::in2 = In2;
	Motor::pwmpin = Pwm;
	Motor::channel = Channel;

	pinMode(Motor::in1, OUTPUT);
	pinMode(Motor::in2, OUTPUT);

	ledcSetup(Motor::channel, 5000, 10);
	ledcAttachPin(Motor::pwmpin, Motor::channel);
	
}

/** @brief Class contructor.
* This version of the constructor uses the channel 0 as default
*/
Motor::Motor(const int In1, const int In2, const int Pwm) {
	
	Motor::in1 = In1;
	Motor::in2 = In2;
	Motor::pwmpin = Pwm;
	Motor::channel = 0;

	pinMode(Motor::in1, OUTPUT);
	pinMode(Motor::in2, OUTPUT);

	ledcSetup(Motor::channel, 5000, 10);
	ledcAttachPin(Motor::pwmpin, 0);

}

/** @brief Drives the motor foward.
 * Negative PWM values will drive the motor in the opposite direction.
* @param PWM PWM that will be sent to the motor channel.
*/
void Motor::goAhead(int PWM) {

	if (PWM < 0) {Motor::reverse(-PWM);}
	else {
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	ledcWrite(Motor::channel, PWM);
	}

}

/** @brief Drives the motor backwards.
* Negative PWM values will drive the motor in the opposite direction.
* @param PWM PWM that will be sent to the motor channel.
*/
void Motor::reverse(int PWM) {

	if (PWM < 0) {Motor::goAhead(-PWM);}
	else {
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	ledcWrite(Motor::channel, PWM);
	}
}

void Motor::halt() {

	ledcWrite(Motor::channel, 0);
}

void Motor::turnLeft(int PWM, Motor SecMotor) {

	digitalWrite(Motor::in1, HIGH);
	digitalWrite(Motor::in2, LOW);
	digitalWrite(SecMotor.in1, LOW);
	digitalWrite(SecMotor.in2, HIGH);
	ledcWrite(Motor::channel, PWM);
	ledcWrite(SecMotor.channel, PWM);

}

void Motor::turnRight(int PWM, Motor SecMotor) {

	digitalWrite(Motor::in1, LOW);
	digitalWrite(Motor::in2, HIGH);
	digitalWrite(SecMotor.in1, HIGH);
	digitalWrite(SecMotor.in2, LOW);
	ledcWrite(Motor::channel, PWM);
	ledcWrite(SecMotor.channel, PWM);

}

/** @brief
* 
*/



void Motor::switchInput() {

	int temp = Motor::in1;
	Motor::in1 = Motor::in2;
	Motor::in2 = temp;

}