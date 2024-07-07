/*

Relevant info:

DISCLAIMER: Tests made using 10-bit resolution PWM

*/

int lastmicros = 0;

#include <Arduino.h>
#include <Motor.h>
#include <Navigator.h>
#include <Controller.h>
#include <Encoder.h>
#include <ros.h>
#include "Pins.h"

// PWM related values
const int freq = 5000;
const byte controllerA_channel = 0;
const byte resolution = 10;

// Encoder pulse count by rotation
const int PULSERPERROTATION = 38;

// PID Controller constants
const double kp = 0.792/38;
const double kd = 0;
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

  // Encoder A (LEFT)
  Encoder lencoder(encoderAChannel2, PULSERPERROTATION);
  
  // Encoder B (RIGHT)
  Encoder rencoder(encoderBChannel1, PULSERPERROTATION);

  // Robot movement
  Navigator trackbot(lcontroller, rcontroller, lmotor, rmotor, lencoder, rencoder, 0.025, 0.5);

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

  // float ltarget = 400*cos(TWO_PI*micros()*4.5/1e6);
  // float rtarget = -400*cos(TWO_PI*micros()*4.5/1e6);

  // float ltarget = 900;
  // float rtarget = -900;

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

  geometry_msgs::Twist msg;
  // msg.linear.x = 0.25*cos(M_TWOPI*micros()*4.5/10e6);
  msg.angular.z = 30*M_PI;
  trackbot.move(msg);
  
//   float currentlRPM = lencoder.getRPM(lmotor.isClockwise);
//   float currentrRPM = rencoder.getRPM(rmotor.isClockwise);

//   float rpwm = rcontroller.controlMotor(rtarget, currentrRPM);
//   float lpwm = lcontroller.controlMotor(ltarget, currentlRPM);

  
//   rmotor.setSpeed(rpwm);
//   lmotor.setSpeed(lpwm);
  
  
//   Serial.printf(">rtarget:%3.3f\n>currentrRPM:%3.3f\n>ltarget:%3.3f\n>currentlRPM:%3.3f\n", rtarget, currentrRPM, ltarget, currentlRPM);
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