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
const double kp = 0.234/5;
const double kd = 0.234/40;
const double ki = 0;

// Global variables
  
  // Time
  unsigned long previousMicros = 0;
  const int micros_interval = 300000;

  // Controller
  Controller controllerA(kp, kd, ki);
  Controller controllerB(kp, kd, ki);

  // Motor A (LEFT)
  Motor lmotor(AIN1, AIN2, PWMA);

  // Motor B (RIGHT)
  Motor rmotor(BIN1, BIN2, PWMB, 1);

  // Robot movement
  Navigator trackbot(lmotor, rmotor);

  // Encoder
  Encoder encoderA(encoderApin, PULSERPERROTATION, encoderAPower);
  

// Function prototypes (declarations)
void wait(int Time);

void isr();


void setup() {

  // Initalizing serial
  Serial.begin(115200);

}

void loop() {

  if ((micros() - encoderA.previousMicros) > micros_interval) {
    encoderA.calculateRPM();
  } 
  float currentRPM = encoderA.getRPM();
  Serial.printf("\nRPM: %3.3f\n", currentRPM);
  trackbot.moveAhead(820);
  wait(50);
}

/* Function definitions*/

// Delay function that doesn't engage sleep mode
void wait(int time) {
  int lasttime = millis();
  while ((millis() - lasttime) <= time) {
  }
}