#include <Encoder.h>
#include <Arduino.h>

Encoder* Encoder::sEncoder = 0;

Encoder::Encoder(const uint8_t& ENCODERPIN, const uint8_t& PULSESPEROTATION, const uint8_t& POWERPIN) 
: encoderPin(ENCODERPIN), pulsesPerRotation(PULSESPEROTATION), powerPin(POWERPIN) {

    sEncoder = this;
    Encoder::setupArduino(true, Encoder::powerPin);
}


Encoder::Encoder(const uint8_t& ENCODERPIN, const uint8_t& PULSESPEROTATION)
: encoderPin(ENCODERPIN) {

    sEncoder = this;
    Encoder::setupArduino(false, 0);
}

void Encoder::counterISR() {

    if (sEncoder != 0) {
        sEncoder->addToCounter();
    }
}

void Encoder::addToCounter() {

    Encoder::pulses++;
}

void Encoder::setupArduino(bool POWER, uint8_t POWERPIN) {

    pinMode(Encoder::encoderPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(Encoder::encoderPin), Encoder::counterISR, RISING);
    if (POWER) {
        pinMode(POWERPIN, OUTPUT);
        digitalWrite(POWERPIN, HIGH);
    }
}

void Encoder::resetCounter() {

    Encoder::pulses = 0;
}

void Encoder::calculateRPM() {

  unsigned long deltaMicros = micros() - Encoder::previousMicros;
  detachInterrupt(digitalPinToInterrupt(Encoder::encoderPin));
  Encoder::rpm = 60.0 * ((static_cast<float>(Encoder::pulses)/Encoder::pulsesPerRotation)/static_cast<double>(deltaMicros)) * 1e6;
  attachInterrupt(digitalPinToInterrupt(Encoder::encoderPin), counterISR, RISING);
  Encoder::resetCounter();
  Encoder::previousMicros = micros();
}

float Encoder::getRPM() {

    return Encoder::rpm;
}