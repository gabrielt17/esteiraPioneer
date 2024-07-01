#include "Encoder.h"
#include <Arduino.h>

/** @brief Construtor que leva em consideração a existência de um pino
 *de alimentação.
 */
Encoder::Encoder(const uint8_t& ENCODERPIN, const uint8_t& PULSESPEROTATION, const uint8_t& POWERPIN) 
: encoderPin(ENCODERPIN), pulsesPerRotation(PULSESPEROTATION), powerPin(POWERPIN), pulses(0), rpm(0), previousMicros(0) {
    Encoder::setupArduino(true, Encoder::powerPin);
}

// Construtor sem pino de alimentação
Encoder::Encoder(const uint8_t& ENCODERPIN, const uint8_t& PULSESPEROTATION)
: encoderPin(ENCODERPIN), powerPin(0), pulses(0), rpm(0), previousMicros(0), pulsesPerRotation(PULSESPEROTATION) {
    Encoder::setupArduino(false, 0);
}

// Configuração do Arduino
void Encoder::setupArduino(bool POWER, uint8_t POWERPIN) {
    pinMode(Encoder::encoderPin, INPUT);
    if (POWER) {
        pinMode(POWERPIN, OUTPUT);
        digitalWrite(POWERPIN, HIGH);
    }
}

// Calcula o RPM
void Encoder::calculateRPM() {

    unsigned long currentMicros = micros();
    unsigned long deltaMicros = currentMicros - Encoder::previousMicros;
    
    if (pulsesPerRotation == 0) {
        Serial.println("Erro: pulsesPerRotation é zero!");
        Encoder::rpm = 0;
        return;
    }

    if (deltaMicros > 0) {
        Encoder::rpm = 60.0 * (static_cast<double>(Encoder::pulses)/Encoder::pulsesPerRotation) / (static_cast<double>(deltaMicros)/1e6);
    } else {
        Encoder::rpm = 0;
    }

    Encoder::resetCounter();
    Encoder::previousMicros = currentMicros;
}

// Retorna o RPM
float Encoder::getRPM() {
    return Encoder::rpm;
}

void Encoder::resetCounter() {
    Encoder::pulses = 0;
}