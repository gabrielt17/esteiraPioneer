/*

Relevant info:

DISCLAIMER: Tests made using 10-bit resolution PWM

*/



#include <Arduino.h>
#include <Motor.h>
#include <Controller.h>
#include "Pins.h"

// Pin assignemnts

// PWM related values
const int freq = 5000;
const byte controllerA_channel = 0;
const byte resolution = 10;

// Encoder pulse count by rotation
const int PULSERPERROTATION = 7;

// PID Controller constants
const double kp = 0.234/5;
const double kd = 0.234/40;
const double ki = 0;

// Global variables
  
  // ISR 
  volatile int pulses = 0;

  // Time
  unsigned long previousMicros = 0;
  const int micros_interval = 300000;

  // Controller
  Controller controllerA(kp, kd, ki);
  int pwm = 0;
  float target = 0;
  double rpm = 0;

// Critical session class object
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Function prototypes (declarations)
void pulseCounter();
void resetCounter();
float getRPM();
void wait(int Time) ;


void setup() {
  // Initalizing serial
  Serial.begin(115200);

  // Defining pin types
  pinMode(encoderAPower, OUTPUT);
  pinMode(encoderA, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  // PWM related setups
  ledcSetup(controllerA_channel, freq, resolution);
  ledcAttachPin(PWMA, controllerA_channel);

  // Initial code setup
  attachInterrupt(digitalPinToInterrupt(encoderA), &pulseCounter, RISING);

  // Turn on the encoder A
  digitalWrite(encoderAPower, HIGH);

  // Sets in which orientation the motor will spin
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

}

void loop() {

  // User defined target
  target = 1000*cos(0.5*static_cast<float>(millis())/1.0e3)+2000;

  // Time interval needed to get consistent encoder readings
  if ((micros()-previousMicros) > micros_interval) {
    rpm = getRPM();
  }

  // Calculates the new PWM signal to reach the target
  pwm = controllerA.controlMotor(target, rpm);

  // Writes the new PWM value to motor A and gives 50 ms breathing room
  ledcWrite(controllerA_channel, pwm);
  Serial.printf("%3.3f; %3.3f; %d; %d\n",rpm ,target, 4000, 100);
  wait(50);
}

/* Function definitions*/

// ISR function. Counts the pulses in a given moment
void pulseCounter() {
  portENTER_CRITICAL_ISR(&mux);
  pulses++;
  portEXIT_CRITICAL_ISR(&mux);
}

// Resets the ISR counter value
void resetCounter() {
  portENTER_CRITICAL_ISR(&mux);
  pulses = 0;
  portEXIT_CRITICAL_ISR(&mux);
}

// Calculates de current RPM measurement
float getRPM(){
  unsigned long deltaMicros = micros() - previousMicros;
  detachInterrupt(digitalPinToInterrupt(encoderA));
  // Serial.printf("\nPulsos antes: %d  ", pulses);
  float rpm = 60.0 * ((pulses/PULSERPERROTATION)/(float)deltaMicros) * 1e6;
  attachInterrupt(digitalPinToInterrupt(encoderA), &pulseCounter, RISING);
  resetCounter();
  previousMicros = micros();
  return rpm;
}

// Delay function that doesn't engage sleep mode
void wait(int time) {
  int lasttime = millis();
  while ((millis() - lasttime) <= time) {
  }
}
