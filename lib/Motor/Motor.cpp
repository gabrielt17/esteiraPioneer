#include "Motor.h"

/** @brief Class contructor.
* This version of the constructor requires the user to specify the operation channel
*/
Motor::Motor(const uint8_t IN1, const uint8_t IN2, const uint16_t PWM, const uint8_t CHANNEL)
: in1(IN1), in2(IN2), pwmpin(PWM), channel(CHANNEL) {
	
    Motor::setupArduino();
    Motor::setupLedc();
}

/** @brief Class contructor.
* This version of the constructor uses the channel 0 as default
*/
Motor::Motor(const uint8_t IN1, const uint8_t IN2, const uint16_t PWM) 
: in1(IN1), in2(IN2), pwmpin(PWM), channel(0) {

	Motor::setupArduino();
    Motor::setupLedc();
}

/// @brief Sets the motor rotation to clockwise
void Motor::setClockwise() {

    digitalWrite(Motor::in1, HIGH);
    digitalWrite(Motor::in2, LOW);
}

/// @brief Sets the motor rotation to anti-clockwise
void Motor::setAntiClockwise() {

    digitalWrite(Motor::in1, LOW);
    digitalWrite(Motor::in2, HIGH);
}

/// @brief Switches the IN1 and IN2 pins between themselves
void Motor::switchInput() {

	int temp = Motor::in1;
	Motor::in1 = Motor::in2;
	Motor::in2 = temp;
}

/// @brief Sets the arduino library up
void Motor::setupArduino() {

    pinMode(Motor::in1, OUTPUT);
    pinMode(Motor::in2, OUTPUT);
}

/// @brief Sets the ledc library up
void Motor::setupLedc() {

    ledcSetup(Motor::channel, 5000, 10);
    ledcAttachPin(Motor::pwmpin, Motor::channel);
}

void Motor::setSpeed(const uint16_t PWM) {

    ledcWrite(Motor::channel, PWM);
}