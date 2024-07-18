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

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

class Controller {

    private:

        float kp;
        float ki;
        float kd; 
        float i = 0; // Stores the integrative calculations
        float pwm = 0; // Stores the lastest calculated PWM conversion
        unsigned long previousMicros = 0 ;
        float previousError = 0;
        float accumulated = 0;
        
    public:

        Controller(float KP, float KD, float KI);
        float getControlSignal(float TARGET, float RADSMEASUREMENT);
        float convertToPWM(float VALUE);
        float controlMotor(float TARGET, float RPMMEASUREMENT);
};


#endif 
