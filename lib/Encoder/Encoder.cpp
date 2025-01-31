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

#include "Encoder.h"

/** @brief Construtor que leva em consideração a existência de um pino
 *de alimentação.
 */
Encoder::Encoder(const uint8_t &ENCODERPIN, const uint8_t &PULSESPEROTATION, const uint8_t &POWERPIN)
    : encoderPin(ENCODERPIN), pulsesPerRotation(PULSESPEROTATION), powerPin(POWERPIN), pulses(0), rpm(0), previousMicros(0)
{
    Encoder::setupArduino(true, Encoder::powerPin);
}

// Construtor sem pino de alimentação
Encoder::Encoder(const uint8_t &ENCODERPIN, const uint8_t &PULSESPEROTATION)
    : encoderPin(ENCODERPIN), powerPin(0), pulses(0), rpm(0), previousMicros(0), pulsesPerRotation(PULSESPEROTATION)
{
    Encoder::setupArduino(false, 0);
}

// Configuração do Arduino
void Encoder::setupArduino(bool POWER, uint8_t POWERPIN)
{
    pinMode(Encoder::encoderPin, INPUT);
    if (POWER)
    {
        pinMode(POWERPIN, OUTPUT);
        digitalWrite(POWERPIN, HIGH);
    }
}

// Calcula o RPM
void Encoder::calculateRPM()
{

    unsigned long currentMicros = micros();
    unsigned long deltaMicros = currentMicros - Encoder::previousMicros;


    Encoder::rpm = 60 * (static_cast<double>(this->pulses) / Encoder::pulsesPerRotation) / (static_cast<double>(deltaMicros) / 1e6);
    this->resetCounter();
    Encoder::previousMicros = currentMicros;
    Encoder::isCalculated = true;
}

// Retorna o RPM
float Encoder::getRPM(bool ISCLOCKWISE)
{
    Encoder::isCalculated = false;
    if (ISCLOCKWISE)
    {
        return -Encoder::rpm;
    } else {
        return Encoder::rpm;
    }
    
}

void Encoder::resetCounter()
{
    this->pulses = 0;
}