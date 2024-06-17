#ifndef ENCODER_H
#define ENCONDER_H
#include <Arduino.h>

class Encoder {

    private:

        const uint8_t encoderPin = 0;
        const uint8_t powerPin = 0;
        uint pulses = 0;
        uint pulsesPerRotation = 0;
        float rpm = 0;
        long int previousMicros = 0;

        static Encoder* sEncoder;
        static void counterISR();

        void setupArduino(const bool POWER, const uint8_t POWERPIN);
        void resetCounter();
        void addToCounter();

    public:

        Encoder(const uint8_t& ENCODERPIN, const uint8_t& PULSESPEROTATION, const uint8_t& POWERPIN);
        Encoder(const uint8_t& ENCODERPIN, const uint8_t& PULSESPEROTATION);
        void calculateRPM();
        float getRPM();

};

#endif // ENCODER_H