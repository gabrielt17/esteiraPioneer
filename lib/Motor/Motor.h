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

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

/** @brief H-bridge motor driver library for ESP32.
* It uses a 10-bit resolution PWM signal to control the motors.
*
* You can especify the ESP32 PWM channel you desire or omit it when 
*creating an instance.
* @param IN1 INI1 H-bridge pin.
* @param IN2 INI2 H-bridge pin.
* @param PWM PWM Input H-bridge pin.
* @param CHANNEL Custom channel value, defaults to 0.
 */
class Motor {

    private:

        uint8_t in1 = 0;
        uint8_t in2 = 0;
        const uint16_t pwmpin;
        const uint8_t channel;

        void setupArduino();
        void setupLedc();

    public:

        Motor(const uint8_t IN1, const uint8_t IN2, const uint16_t PWMPIN, const uint8_t CHANNEL);
        Motor(const uint8_t IN1, const uint8_t IN2, const uint16_t PWMPIN);
        
        void switchInput();
        void setClockwise();
        void setAntiClockwise();
        void setSpeed(const int16_t PWM);

        bool isClockwise;

};

#endif // MOTOR_H