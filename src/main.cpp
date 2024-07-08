/*

Relevant info:

DISCLAIMER: Tests made using 10-bit resolution PWM

*/

#include <Arduino.h>
#include <Motor.h>
#include <Converter.cpp>
#include <Controller.h>
#include <Encoder.h>
#include <ros.h>
#include "Pins.h"
#include <WiFi.h>

// PWM related values
const int freq = 5000;
const byte controllerA_channel = 0;
const byte resolution = 10;

// Encoder pulse count by rotation
const int PULSERPERROTATION = 38;

// PID Controller constants
const double kp = 0.792/50;
const double kd = 0;
const double ki = 0.792/150;

// Global variables

  // WiFi
  const char* ssid = "NERo-Arena";
  const char* pw = "BDPsystem10";
  
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

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Function prototypes (declarations)
void wait(int Time);
void IRAM_ATTR lencoderCounter();
void IRAM_ATTR rencoderCounter();

geometry_msgs::Twist msg;
motorRPM rpm;

void setup() {

  // Initalizing serial
  Serial.begin(115200);
//   Serial.printf(">rtarget:%3.3f\n>currentrRPM:%3.3f\n>ltarget:%3.3f\n>currentlRPM:%3.3f\n", rtarget, currentrRPM, ltarget, currentlRPM);

  // Attach encoder ISR's
  attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
  attachInterrupt(encoderBChannel1, rencoderCounter, RISING);

  msg.linear.x = 2;
  msg.angular.z = TWO_PI;
  rpm = convertMessage(msg);
}

void loop() {

  float ltarget = 400*cos(TWO_PI*micros()*0.5/1e6);
  float rtarget = -400*cos(TWO_PI*micros()*0.5/1e6);

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
  
  float currentlRPM = lencoder.getRPM(lmotor.isClockwise);
  float currentrRPM = rencoder.getRPM(rmotor.isClockwise);

  float rpwm = rcontroller.controlMotor(rtarget, currentrRPM);
  float lpwm = lcontroller.controlMotor(ltarget, currentlRPM);

  // Serial.printf("rrpm:%3.3f, lrpm:%3.3f\n",rpm.rrpm, rpm.lrpm);

  lmotor.setSpeed(lpwm);
  rmotor.setSpeed(rpwm);
  
  
  
  Serial.printf(">rtarget:%3.3f\n>currentrRPM:%3.3f\n>ltarget:%3.3f\n>currentlRPM:%3.3f\n", rtarget, currentrRPM, ltarget, currentlRPM);
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