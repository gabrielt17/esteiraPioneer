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
  Controller controllerA(kp, kd, ki);
  Controller controllerB(kp, kd, ki);

  // Motor A (LEFT)
  Motor lmotor(AIN1, AIN2, PWMA);

  // Motor B (RIGHT)
  Motor rmotor(BIN1, BIN2, PWMB, 1);

  // Robot movement
  Navigator trackbot(lmotor, rmotor);

  // Encoder A (LEFT)
  Encoder lencoder(encoderAChannel1, PULSERPERROTATION, lmotor);
  
  // Encoder B (RIGHT)
  Encoder rencoder(encoderBChannel1, PULSERPERROTATION, rmotor);

// Function prototypes (declarations)
void wait(int Time);
void IRAM_ATTR lencoderCounter();
void IRAM_ATTR rencoderCounter();


void setup() {

  // Initalizing serial
  Serial.begin(115200);

  // Attach encoder ISR's
  attachInterrupt(digitalPinToInterrupt(encoderAChannel1), &lencoderCounter, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderBChannel2), &rencoderCounter, RISING);

}

void loop() {

  // float target = 400*cos(2*M_PI*4.5*micros()/10e6)+500;
  float target = 600;

  if ((micros() - lencoder.previousMicros) > calculate_interval) {
    currentMicrosA = micros();
    detachInterrupt(encoderAChannel1);
    detachInterrupt(encoderBChannel2);
    lencoder.calculateRPM();
    attachInterrupt(digitalPinToInterrupt(encoderAChannel1), &lencoderCounter, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderBChannel2), &rencoderCounter, RISING);
  }

  if ((micros() - rencoder.previousMicros) > calculate_interval) {
    currentMicrosB = micros();
    detachInterrupt(encoderBChannel2);
    detachInterrupt(encoderAChannel1);
    rencoder.calculateRPM();
    attachInterrupt(digitalPinToInterrupt(encoderBChannel2), &rencoderCounter, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderAChannel1), &lencoderCounter, RISING);
  }

  int currentlRPM = lencoder.getRPM();
  int currentrRPM = rencoder.getRPM();

  int pwm = controllerA.controlMotor(target, currentlRPM);

  Serial.printf("%d; %d; %3.3f; %d; %d\n", 1500, 30, target, currentlRPM, pwm);

  
  trackbot.moveAhead(960);

}

/* Function definitions*/

void lencoderCounter() {
  lencoder.pulses++;
}

void rencoderCounter() {
  rencoder.pulses++;
}

// Delay function that doesn't engage sleep mode
void wait(int time) {
  int lasttime = millis();
  while ((millis() - lasttime) <= time) {
  }
}