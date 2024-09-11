/*

Relevant info:

DISCLAIMER: Tests made using 10-bit resolution PWM

*/



#include <Arduino.h>
#include <Motor.h>
#include <Controller.h>
#include <Encoder.h>
#include "Pins.h"

// Encoder pulse count by rotation
const int PULSERPERROTATION = 38;

// PID Controller constants
const double kp = 0.746/50;
const double kd = 0.746/300;
const double ki = 0;

// Global variables
  
  // Time
  unsigned long testtime = 0;
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

  // Encoder A (LEFT)
  Encoder lencoder(encoderAChannel2, PULSERPERROTATION);
  
  // Encoder B (RIGHT)
  Encoder rencoder(encoderBChannel1, PULSERPERROTATION);

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

  lmotor.setSpeed(700);
  rmotor.setSpeed(700);
  wait(2000);
  lmotor.setSpeed(-700);
  rmotor.setSpeed(-700);
  wait(2000);
  lmotor.setSpeed(-700);
  rmotor.setSpeed(700);
  wait(2000);
  lmotor.setSpeed(700);
  rmotor.setSpeed(-700);
  wait(2000);
  
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