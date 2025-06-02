/*

Relevant info:

DISCLAIMER: Tests made using 10-bit resolution PWM

*/

#include <Arduino.h>
#include <Motor.h>
#include <Controller.h>
#include <Encoder.h>
#include "Pins.h"
#include <Kinematics.h>
#include <SimpleKalmanFilter.h>

// Encoder pulse count by rotation
const int PULSERPERROTATION = 8;

// PID Controller constants
const double kp = 0.2;
const double kd = 0;
const double ki = 0;

// Global variables

// Time
const int CALCULATERPM_INTERVAL = 50000;
const long SAMPLE_TIME_MICROS = 20000;
unsigned long LAST_SAMPLE_TIME = 0;
const int CALLBACK_INTERVAL = 1000000;
unsigned long currentCb_timer = 0;
unsigned long currentMicrosA, currentMicrosB = 0;

// Controller
Controller lcontroller(kp, kd, ki);
Controller rcontroller(kp, kd, ki);

// Motor A (LEFT)
Motor lmotor(AIN1, AIN2, PWMA);

// Motor B (RIGHT)
Motor rmotor(BIN1, BIN2, PWMB, 1);

// Encoder A (LEFT)
Encoder lencoder(encoderAChannel2, PULSERPERROTATION);

// Encoder B (RIGHT)
Encoder rencoder(encoderBChannel1, PULSERPERROTATION);

// Kalman filter
SimpleKalmanFilter lkf = SimpleKalmanFilter(5, 5, 0.01);
SimpleKalmanFilter rkf = SimpleKalmanFilter(5, 5, 0.01);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Function prototypes (declarations)
void wait(int Time);
void IRAM_ATTR lencoderCounter();
void IRAM_ATTR rencoderCounter();

motorRPM rpm;

bool rodar = true;

void setup()
{

    // Initalizing serial
    Serial.begin(115200);

    // Attach encoder ISR's
    attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
    attachInterrupt(encoderBChannel1, rencoderCounter, RISING);

    rmotor.setSpeed(1023);
    lmotor.setSpeed(1023);

}

void loop()
{
    wait(10000);
    rmotor.setSpeed(0);
    lmotor.setSpeed(0);
    if (rodar) {
        rodar = false;
        Serial.printf("R encoder pulses: %d\n", rencoder.pulses);
        Serial.printf("L encoder pulses: %d\n", lencoder.pulses);
    }
    
}
/* Function definitions*/

void IRAM_ATTR lencoderCounter()
{
    portENTER_CRITICAL_ISR(&mux);
    lencoder.pulses++;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR rencoderCounter()
{
    portENTER_CRITICAL_ISR(&mux);
    rencoder.pulses++;
    portEXIT_CRITICAL_ISR(&mux);
}

// Delay function that doesn't engage sleep mode
void wait(int time)
{
    int lasttime = millis();
    while ((millis() - lasttime) <= time)
    {
    }
}