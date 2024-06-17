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
  
  // ISR 
  volatile int pulsesA = 0;
  volatile int pulsesB = 0;

  // Time
  unsigned long previousMicros = 0;
  const int micros_interval = 300000;

  // Controller
  Controller controllerA(kp, kd, ki);
  Controller controllerB(kp, kd, ki);
  int pwmA = 0;
  int pwmB = 0;
  float target = 0;
  float rpmA = 0;
  double rpmB = 0;

  // Motor A
  Motor motorA(AIN1, AIN2, PWMA);

  // Motor B
  Motor motorB(BIN1, BIN2, PWMB, 1);

  Navigator trackbot(motorA, motorB);

// Critical session class object
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Function prototypes (declarations)
void pulseCounterA();
void resetCounterA();
void pulseCounterB();
void resetCounterB();
float getRPMA(const uint8_t, const uint);
float getRPMB(const uint8_t, const uint);
void wait(int Time) ;


void setup() {
  // Initalizing serial
  Serial.begin(115200);

  // Defining pin types
  pinMode(encoderAPower, OUTPUT);
  pinMode(encoderA, INPUT);
  pinMode(encoderBpower, OUTPUT);
  pinMode(encoderB, INPUT);
  // pinMode(AIN1, OUTPUT);
  // pinMode(AIN2, OUTPUT);

  // PWM related setups
  // ledcSetup(controllerA_channel, freq, resolution);
  // ledcAttachPin(PWMA, controllerA_channel);

  // Initial code setup
  attachInterrupt(digitalPinToInterrupt(encoderA), &pulseCounterA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB), &pulseCounterB, RISING);

  // Turn on the encoder A and B
  digitalWrite(encoderAPower, HIGH);
  digitalWrite(encoderBpower, HIGH);

  // Sets in which orientation the motor will spin
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

}

void loop() {

  // User defined target
  target = 1000*cos(0.5*static_cast<float>(millis())/1.0e3)+2000;

  // Time interval needed to get consistent encoder readings
  if ((micros()-previousMicros) > micros_interval) {
    // rpmA = getRPMA(encoderA, PULSERPERROTATION);
    // rpmB = getRPMB(encoderB, PULSERPERROTATION);
  }

  // Calculates the new PWM signal to reach the target
  pwmA = controllerA.controlMotor(target, rpmA);
  pwmB = controllerB.controlMotor(target, rpmB);

  // Writes the new PWM value to motor A and gives 50 ms breathing room
  trackbot.moveBackwards(820);
  // Serial.printf("Motor A: %3.3f; %3.3f; %d; %d\n",rpmA ,target, 4000, 100);
  Serial.printf("Motor B: %3.3f; %3.3f; %d; %d\n", rpmB, target, 4000, 100);
  wait(50);
}

/* Function definitions*/

// ISR function. Counts the pulses in a given moment
void pulseCounterA() {
  portENTER_CRITICAL_ISR(&mux);
  pulsesA++;
  portEXIT_CRITICAL_ISR(&mux);
}

// ISR function. Counts the pulses in a given moment
void pulseCounterB() {
  portENTER_CRITICAL_ISR(&mux);
  pulsesB++;
  portEXIT_CRITICAL_ISR(&mux);
}

// Resets the ISR counter value
void resetCounterA() {
  portENTER_CRITICAL_ISR(&mux);
  pulsesA = 0;
  portEXIT_CRITICAL_ISR(&mux);
}

// Resets the ISR counter value
void resetCounterB() {
  portENTER_CRITICAL_ISR(&mux);
  pulsesB = 0;
  portEXIT_CRITICAL_ISR(&mux);
}

// Calculates de current RPM measurement
float getRPMA(const uint8_t Encoder, const uint PulsesPerRotation) {
  unsigned long deltaMicros = micros() - previousMicros;
  detachInterrupt(digitalPinToInterrupt(Encoder));
  // Serial.printf("\nPulsos antes: %d  ", pulsesA);
  float rpm = 60.0 * ((pulsesA/PulsesPerRotation)/static_cast<double>(deltaMicros)) * 1e6;
  attachInterrupt(digitalPinToInterrupt(Encoder), &pulseCounterA, RISING);
  resetCounterA();
  previousMicros = micros();
  return rpm;
}

float getRPMB(const uint8_t Encoder, const uint16_t PulsesPerRotation) {
  unsigned long deltaMicros = micros() - previousMicros;
  detachInterrupt(digitalPinToInterrupt(Encoder));
  Serial.printf("\nPulsos antes: %d  \n", pulsesB);
  float rpm = 60.0 * ((static_cast<float>(pulsesB)/PulsesPerRotation)/static_cast<double>(deltaMicros)) * 1e6;
  Serial.printf("\nRPMB calculado: %f\n", rpm);
  attachInterrupt(digitalPinToInterrupt(Encoder), &pulseCounterB, RISING);
  resetCounterB();
  previousMicros = micros();
  return rpm;
}

// Delay function that doesn't engage sleep mode
void wait(int time) {
  int lasttime = millis();
  while ((millis() - lasttime) <= time) {
  }
}
