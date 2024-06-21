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
  const int calculate_interval = 300000;

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
  Encoder lencoder(encoderAChannel1, PULSERPERROTATION);
  
  // Encoder B (RIGHT)
  Encoder rencoder(encoderBChannel2, PULSERPERROTATION);

// Function prototypes (declarations)
void wait(int Time);
void IRAM_ATTR lencoderCounter();
void IRAM_ATTR rencoderCounter();


void setup() {

  // Initalizing serial
  Serial.begin(115200);

  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);

  // Attach encoder ISR's
  attachInterrupt(digitalPinToInterrupt(encoderAChannel1), &lencoderCounter, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderBChannel2), &rencoderCounter, RISING);

}

void loop() {

  if ((micros() - lencoder.previousMicros) > calculate_interval) {
    detachInterrupt(encoderAChannel1);
    detachInterrupt(encoderBChannel2);
    lencoder.calculateRPM();
    attachInterrupt(digitalPinToInterrupt(encoderAChannel1), &lencoderCounter, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderBChannel2), &rencoderCounter, RISING);
  }
  if ((micros() - rencoder.previousMicros) > calculate_interval) {
    detachInterrupt(encoderBChannel2);
    detachInterrupt(encoderAChannel1);
    rencoder.calculateRPM();
    attachInterrupt(digitalPinToInterrupt(encoderBChannel2), &rencoderCounter, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderAChannel1), &lencoderCounter, RISING);
  }

  float currentlRPM = lencoder.getRPM();
  float currentrRPM = rencoder.getRPM();
  Serial.print("PULSOS L: ");
  Serial.println(lencoder.pulses);
  Serial.print("PULSOS R: ");
  Serial.println(rencoder.pulses);
  Serial.print("LEFT RPM: ");
  Serial.println(currentlRPM);
  Serial.print("RIGHT RPM: ");
  Serial.println(currentrRPM);
  Serial.print("PINO ENCODER A: ");
  Serial.println(lencoder.encoderPin);
  Serial.print("PINO ENCODER B: ");
  Serial.println(rencoder.encoderPin);
  trackbot.moveAhead(820);
  wait(50);
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