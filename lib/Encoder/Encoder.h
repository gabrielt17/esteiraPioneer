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

/**
 * @param ENCODERPIN Encoder signal pin
 * @param PULSESPEROTATION Number of pulses on a single spin
 * @param POWERPIN Encoder power pin, in case there is one. Leave blank if none
 * @brief Disclaimer: You have to manually create the ISR functions and attach/detach them
 */
class Encoder {
    private:

        const uint8_t powerPin;
        uint pulsesPerRotation;
        float rpm;
        
        const uint8_t encoderPin;
        void setupArduino(bool POWER, uint8_t POWERPIN);       

    public:

        unsigned long previousMicros = 0;
        bool isCalculated;
        
        Encoder(const uint8_t& ENCODERPIN, const uint8_t& PULSESPEROTATION, const uint8_t& POWERPIN);
        Encoder(const uint8_t& ENCODERPIN, const uint8_t& PULSESPEROTATION);
        void calculateRPM();
        float getRPM(bool ISCLOCKWISE);
        void resetCounter();
        volatile uint pulses;
};

#endif // ENCODER_H
