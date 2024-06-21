#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
    private:
        const uint8_t encoderPin;
        const uint8_t powerPin;
        uint pulses;
        uint pulsesPerRotation;
        float rpm;

        void setupArduino(bool POWER, uint8_t POWERPIN);
        void resetCounter();

    public:

        long int previousMicros;
        static Encoder* sEncoder;
        
        Encoder(const uint8_t& ENCODERPIN, const uint8_t& PULSESPEROTATION, const uint8_t& POWERPIN);
        Encoder(const uint8_t& ENCODERPIN, const uint8_t& PULSESPEROTATION);
        void calculateRPM();
        float getRPM();
        void addToCounter();
};

#endif // ENCODER_H