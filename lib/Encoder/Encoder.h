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

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

extern portMUX_TYPE mux;

class Encoder
{
public:
    Encoder(uint8_t ENCODERPIN, uint PULSESPEROTATION, uint8_t POWERPIN);
    Encoder(uint8_t ENCODERPIN, uint PULSESPEROTATION);

    unsigned long previousMicros;
    bool isCalculated;
    volatile uint pulses;
    void calculateRPM();
    float getRPM(bool ISCLOCKWISE) const;
    void resetCounter();

private:
    void setupArduino(bool POWER, uint8_t POWERPIN);

    uint8_t encoderPin;
    uint pulsesPerRotation;
    uint8_t powerPin;
    float rpm;
    
};

#endif // ENCODER_H
