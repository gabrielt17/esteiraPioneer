/*
*    ASTROS, a Pioneer P3-DX inspired robot.
*    Copyright (C) <2024>  <Gabriel Víctor and Hiago Batista>
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "Motor.h"

/** @brief Class contructor.
* This version of the constructor requires the user to specify the operation channel
*/
Motor::Motor(const uint8_t IN1, const uint8_t IN2, const uint16_t PWMPIN, const uint8_t CHANNEL)
: in1(IN1), in2(IN2), pwmpin(PWMPIN), channel(CHANNEL) {
	
    this->setupArduino();
    this->setupLedc();
}

/** @brief Class contructor.
* This version of the constructor uses the channel 0 as default
*/
Motor::Motor(const uint8_t IN1, const uint8_t IN2, const uint16_t PWMPIN) 
: in1(IN1), in2(IN2), pwmpin(PWMPIN), channel(0) {

	this->setupArduino();
    this->setupLedc();
}

/// @brief Sets the motor rotation to clockwise.
void Motor::setClockwise() {
    this->isClockwise = true;
    digitalWrite(Motor::in1, HIGH);
    digitalWrite(Motor::in2, LOW);
}

/// @brief Sets the motor rotation to anti-clockwise.
void Motor::setAntiClockwise() {
    this->isClockwise = false;
    digitalWrite(Motor::in1, LOW);
    digitalWrite(Motor::in2, HIGH);
}

/// @brief Switches the IN1 and IN2 pins between themselves.
void Motor::switchInput() {

	int temp = Motor::in1;
	this->in1 = Motor::in2;
	this->in2 = temp;
}

/// @brief Sets the arduino library up.
void Motor::setupArduino() {

    pinMode(Motor::in1, OUTPUT);
    pinMode(Motor::in2, OUTPUT);
}

/// @brief Sets the ledc library up.
void Motor::setupLedc() {

    ledcSetup(this->channel, 10000, 10);
    ledcAttachPin(this->pwmpin, this->channel);
}
/// @brief Sets the current motor PWM
/// @param PWM Sets the desired PWM value. Uses 10-bit resolution.
void Motor::setSpeed(const int16_t PWM) {
    if (PWM < 0) {
        this->setClockwise();
        ledcWrite(Motor::channel, -PWM);
    } else {
        this->setAntiClockwise();
        ledcWrite(Motor::channel, PWM);
    }
}