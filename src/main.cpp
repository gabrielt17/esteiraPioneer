/*

Relevant info:

DISCLAIMER: Tests made using 10-bit resolution PWM

*/



#include <Arduino.h>
#include <Motor.h>
#include <Navigator.h>
#include <Controller.h>
#include <Encoder.h>
#include "Pins.h"

// PWM related values
const int freq = 5000;
const byte controllerA_channel = 0;
const byte resolution = 10;

// Encoder pulse count by rotation
const int PULSERPERROTATION = 38;

// PID Controller constants
const double kp = 0.746/2;
const double kd = 0.746/40;
const double ki = 0;

// Global variables
  
  // Time
  unsigned long previousMicros = 0;
  const int calculate_interval = 150000;

  unsigned long currentMicrosA, currentMicrosB = 0;

  // Controller
  Controller lcontroller(kp, kd, ki);
  Controller rcontroller(kp, kd, ki);

  // Motor A (LEFT)
  Motor lmotor(AIN1, AIN2, PWMA);

  // Motor B (RIGHT)
  Motor rmotor(BIN1, BIN2, PWMB, 1);

  // Robot movement
  Navigator trackbot(lmotor, rmotor);

  // Encoder A (LEFT)
  Encoder lencoder(encoderAChannel2, PULSERPERROTATION, lmotor);
  
  // Encoder B (RIGHT)
  Encoder rencoder(encoderBChannel1, PULSERPERROTATION, rmotor);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Function prototypes (declarations)
void wait(int Time);
void IRAM_ATTR lencoderCounter();
void IRAM_ATTR rencoderCounter();


void setup() {

  // Initalizing serial
  Serial.begin(115200);

  // Attach encoder ISR's
  attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
  attachInterrupt(encoderBChannel1, rencoderCounter, RISING);
}

void loop() {

  // float lTarget = 400;
  float rTarget = 400;

  if ((micros() - lencoder.previousMicros) > calculate_interval) {
    currentMicrosA = micros();
    detachInterrupt(encoderAChannel2);
    detachInterrupt(encoderBChannel1);
    lencoder.calculateRPM();
    attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
    attachInterrupt(encoderBChannel1, rencoderCounter, RISING);
  }

  if ((micros() - rencoder.previousMicros) > calculate_interval) {
    currentMicrosB = micros();
    detachInterrupt(encoderBChannel1);
    detachInterrupt(encoderAChannel2);
    rencoder.calculateRPM();
    attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
    attachInterrupt(encoderBChannel1, rencoderCounter, RISING);
  }

  int currentlRPM = lencoder.getRPM();
  int currentrRPM = rencoder.getRPM();

  // int lpwm = lcontroller.controlMotor(lTarget, currentlRPM);
  // int rpwm = rcontroller.controlMotor(rTarget, currentrRPM);

  // Serial.printf("%d; %d; %3.3f; %d; %d\n", 1500, 30, rTarget, currentrRPM, rpwm);
  Serial.printf("LRPM: %d; RRPM: %d\n", currentlRPM, currentrRPM);

  trackbot.moveAhead(400);
  
}

/* Function definitions*/

void IRAM_ATTR lencoderCounter() {
  portENTER_CRITICAL_ISR(&mux);
  lencoder.pulses++;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR rencoderCounter() {
  portENTER_CRITICAL_ISR(&mux);
  rencoder.pulses++;
  portEXIT_CRITICAL_ISR(&mux);
}

// Delay function that doesn't engage sleep mode
void wait(int time) {
  int lasttime = millis();
  while ((millis() - lasttime) <= time) {
  }
}