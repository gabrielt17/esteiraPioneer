#include "Motor.h"
#include "Arduino.h"

// Class constructor

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

void Motor::goAhead(int PWM) {

	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	ledcWrite(Motor::channel, PWM);

}

void Motor::reverse(int PWM) {

	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	ledcWrite(Motor::channel, PWM);

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

// Switches h-bridge input pins in case they're inverted
void Motor::switchInput() {

	int temp = Motor::in1;
	Motor::in1 = Motor::in2;
	Motor::in2 = temp;

}