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
 * de alimentação.
 */
Encoder::Encoder(uint8_t ENCODERPIN, uint PULSESPEROTATION, uint8_t POWERPIN)
    : encoderPin(ENCODERPIN), pulsesPerRotation(PULSESPEROTATION), powerPin(POWERPIN), pulses(0), rpm(0), previousMicros(0), isCalculated(false)
{
    setupArduino(true, powerPin);
}

// Construtor sem pino de alimentação
Encoder::Encoder(uint8_t ENCODERPIN, uint PULSESPEROTATION)
    : encoderPin(ENCODERPIN), pulsesPerRotation(PULSESPEROTATION), powerPin(0), pulses(0), rpm(0), previousMicros(0), isCalculated(false)
{
    setupArduino(false, 0);
}

// Configuração do Arduino
void Encoder::setupArduino(bool POWER, uint8_t POWERPIN)
{
    pinMode(encoderPin, INPUT);
    if (POWER)
    {
        pinMode(POWERPIN, OUTPUT);
        digitalWrite(POWERPIN, HIGH);
    }
}

// Calcula o RPM
void Encoder::calculateRPM()
{   
    portENTER_CRITICAL_ISR(&mux);
    unsigned long currentMicros = micros();
    unsigned long deltaMicros = currentMicros - previousMicros;
    
    if (deltaMicros > 50e3)
    {
        rpm = 60.0 * (static_cast<double>(pulses) / pulsesPerRotation) / (static_cast<double>(deltaMicros) / 1e6);
    }
    // else
    // {
    //     rpm = 0; // Evita divisão por zero
    // }
    
    resetCounter();
    previousMicros = currentMicros;
    isCalculated = true;
    portEXIT_CRITICAL_ISR(&mux);
}

// Retorna o RPM
float Encoder::getRPM(bool ISCLOCKWISE) const
{
    return ISCLOCKWISE ? -rpm : rpm;
}

// Reseta o contador de pulsos
void Encoder::resetCounter()
{
    portENTER_CRITICAL_ISR(&mux);
    pulses = 0;
    portEXIT_CRITICAL_ISR(&mux);
}
