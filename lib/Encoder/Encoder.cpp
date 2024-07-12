#include "Encoder.h"
#include <Arduino.h>

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

    if (ISCLOCKWISE)
    {
        return -Encoder::rpm;
    } else {
        return Encoder::rpm;
    }
    Encoder::isCalculated = false;
}

void Encoder::resetCounter()
{
    this->pulses = 0;
}