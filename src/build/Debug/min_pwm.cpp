#include "Pins.h"
#include <Arduino.h>
#include <Motor.h>
#include <Encoder.h>
#include <ArduinoOTA.h>

// Encoder pulse count by rotation
const uint8_t PULSESPERROTATION = 183; 
const uint CALCULATERPM_INTERVAL = 3000000; 

// ISR variables
volatile bool lencodertrigger = false;
volatile int numberOfEncoderInterrupts = 0;

// Motor A (LEFT)
Motor lmotor(AIN1, AIN2, PWMA);

// Encoder A (LEFT)
Encoder lencoder(encoderAChannel2, PULSESPERROTATION);

// Critical session initializer
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/**********************************************************************
* Function prototypes
**********************************************************************/ 
void wait(int Time);
void IRAM_ATTR lencoderCounter();

void setup() {
    Serial.begin(115200);

    // Attach encoder ISR
    attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
}

void loop() {
    int pwmValue = 0;

    while (pwmValue <= 1023) { 
        // Aplica o PWM ao motor
        lmotor.setSpeed(pwmValue);

        // Espera o motor estabilizar antes da leitura
        wait(2000); 

        // Calcula o RPM
        lencoder.calculateRPM();
        float rpm = lencoder.getRPM(lmotor.isClockwise);

        // Exibe os resultados no Serial Monitor
        Serial.printf(">PWM:%d>\n>RPM:%.1f\n", pwmValue, rpm);

        // Aumenta o PWM gradualmente
        pwmValue += 10; // Ajuste o incremento conforme necessário
    }

    // Para o motor após o teste
    lmotor.setSpeed(0);
    while (true); // Impede que o loop continue rodando indefinidamente
}

/**********************************************************************
* Function definitions
**********************************************************************/ 

// Encoder ISR function
void IRAM_ATTR lencoderCounter() {
    portENTER_CRITICAL_ISR(&mux);
    lencoder.pulses++;
    portEXIT_CRITICAL_ISR(&mux);
}

// Delay sem modo de sleep
void wait(int time) {
    int start = millis();
    while ((millis() - start) < time) {
    }
}
