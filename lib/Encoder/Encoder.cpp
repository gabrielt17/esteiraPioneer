#include "Encoder.h"
#include <Arduino.h>

Encoder* Encoder::sEncoder = nullptr;

// Função ISR
void IRAM_ATTR counterISR() {
    if (Encoder::sEncoder != nullptr) {
        Encoder::sEncoder->addToCounter();
    }
}

// Construtor com pino de alimentação
Encoder::Encoder(const uint8_t& ENCODERPIN, const uint8_t& PULSESPEROTATION, const uint8_t& POWERPIN) 
: encoderPin(ENCODERPIN), pulsesPerRotation(PULSESPEROTATION), powerPin(POWERPIN), pulses(0), rpm(0), previousMicros(0) {
    sEncoder = this;
    Encoder::setupArduino(true, Encoder::powerPin);
}

// Construtor sem pino de alimentação
Encoder::Encoder(const uint8_t& ENCODERPIN, const uint8_t& PULSESPEROTATION)
: encoderPin(ENCODERPIN), powerPin(0), pulses(0), rpm(0), previousMicros(0) {
    sEncoder = this;
    Encoder::setupArduino(false, 0);
}

// Incrementa o contador de pulsos
void Encoder::addToCounter() {
    Encoder::pulses++;
}

// Configuração do Arduino
void Encoder::setupArduino(bool POWER, uint8_t POWERPIN) {
    // Instala o serviço ISR apenas uma vez
    static bool isrServiceInstalled = false;
    if (!isrServiceInstalled) {
        if (gpio_install_isr_service(ESP_INTR_FLAG_IRAM) == ESP_OK) {
            isrServiceInstalled = true;
        } else {
            Serial.println("Erro ao inicializar o serviço ISR");
            return; // Sai da função se não puder instalar o serviço ISR
        }
    }

    pinMode(Encoder::encoderPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(Encoder::encoderPin), counterISR, RISING);

    if (POWER) {
        pinMode(POWERPIN, OUTPUT);
        digitalWrite(POWERPIN, HIGH);
    }
}

// Reseta o contador de pulsos
void Encoder::resetCounter() {
    Encoder::pulses = 0;
}

// Calcula o RPM
void Encoder::calculateRPM() {
    unsigned long currentMicros = micros();
    unsigned long deltaMicros = currentMicros - previousMicros;

    // Desanexa a interrupção
    detachInterrupt(digitalPinToInterrupt(encoderPin));

    // Calcula o RPM
    if (deltaMicros > 0) { // Evitar divisão por zero
        Encoder::rpm = 60.0 * (static_cast<float>(Encoder::pulses) / Encoder::pulsesPerRotation) / (static_cast<double>(deltaMicros) / 1e6);
    } else {
        Encoder::rpm = 0;
    }

    // Reseta o contador e atualiza o tempo anterior
    Encoder::resetCounter();
    previousMicros = currentMicros;

    // Reanexa a interrupção
    attachInterrupt(digitalPinToInterrupt(encoderPin), counterISR, RISING);
}

// Retorna o RPM
float Encoder::getRPM() {
    return Encoder::rpm;
}