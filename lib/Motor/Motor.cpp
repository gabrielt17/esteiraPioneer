#include "Motor.h"
#include "Arduino.h"

/** @brief Class contructor.
* This version of the constructor requires the user to specify the operation channel
*/
Motor::Motor(const uint8_t IN1, const uint8_t IN2, const uint8_t PWM, const uint8_t CHANNEL)
: in1(IN1), in2(IN2), pwmpin(PWM), channel(CHANNEL) {
	
    Motor::setupArduino();
    Motor::setupLedc();
}


/** @brief Class contructor.
* This version of the constructor uses the channel 0 as default
*/
Motor::Motor(const uint8_t IN1, const uint8_t IN2, const uint8_t PWM) 
: in1(IN1), in2(IN2), pwmpin(PWM) {

	Motor::setupArduino();
    Motor::setupLedc();
}

void Motor::setClockwise() {

    digitalWrite(Motor::in1, HIGH);
    digitalWrite(Motor::in2, LOW);
}

void Motor::setAntiClockwise() {

    digitalWrite(Motor::in1, LOW);
    digitalWrite(Motor::in2, HIGH);
}

void Motor::switchInput() {

	int temp = Motor::in1;
	Motor::in1 = Motor::in2;
	Motor::in2 = temp;
}

void Motor::setupArduino() {

    pinMode(Motor::in1, OUTPUT);
    pinMode(Motor::in2, OUTPUT);
}

void Motor::setupLedc() {

    ledcSetup(Motor::channel, 5000, 10);
    ledcAttachPin(Motor::pwmpin, Motor::channel);
}