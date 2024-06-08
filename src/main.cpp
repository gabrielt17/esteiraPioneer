/*

Relevant info:

DISCLAIMER: Tests made using 10-bit resolution PWM

*/



#include <Arduino.h>
#include <Controller.h>
#include "Pins.h"

// Pin assignemnts

// PWM related values
const int freq = 5000;
const byte motorA_channel = 0;
const byte resolution = 10;

// Relevant constant values
const int PULSERPERROTATION = 7;

// Global variables
volatile int pulses = 0;
unsigned long previousMillis = 0;
Controller motorA(0.23435580855242363/5,0.23435580855242363/40,0);
int pwm = 0;
float target = 0;
const int interval = 300000;
long currentTime = 0;
double valor = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Function prototypes (declarations)
void pulseCounter();
void resetCounter();
double getRPM();
void wait(int Time) ;


void setup() {
  // Initalizing serial
  Serial.begin(115200);

  // Defining pin types
  pinMode(encoderA, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  // PWM related setups
  ledcSetup(motorA_channel, freq, resolution);
  ledcAttachPin(PWMA, motorA_channel);

  // Initial code setup
  attachInterrupt(digitalPinToInterrupt(encoderA), &pulseCounter, RISING);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

}

void loop() {
  target = 1000*cos(0.5*static_cast<float>(millis())/1.0e3)+2000;
  // target = 2500;
  if ((micros()-previousMillis) > interval) {
    valor = getRPM();
  }
  // Serial.printf("RPM: %3.f", valor);
  double u = motorA.getControlSignal(target, valor);
  Serial.printf("\nValor de u: %3.3f", u);   
  pwm = static_cast<int>(motorA.convertToPWM(target, u));
  currentTime = millis();
  Serial.printf("\nValor de PWM: %d\n", pwm);
  ledcWrite(motorA_channel, pwm);
  Serial.printf("%3.3f; %3.3f; %d; %d\n",valor ,target, 4000, 100);
  // Serial.printf("\n\n\n\n");
  wait(50);
}

void pulseCounter() {
  portENTER_CRITICAL_ISR(&mux);
  pulses++;
  portEXIT_CRITICAL_ISR(&mux);
}

void resetCounter() {
  portENTER_CRITICAL_ISR(&mux);
  pulses = 0;
  portEXIT_CRITICAL_ISR(&mux);
}

double getRPM(){
  unsigned long deltaMillis = micros() - previousMillis;
  detachInterrupt(digitalPinToInterrupt(encoderA));
  // Serial.printf("\nPulsos antes: %d  ", pulses);
  double rpm = 60.0 * ((pulses / PULSERPERROTATION )/(double)deltaMillis) * 1e6;
  attachInterrupt(digitalPinToInterrupt(encoderA), &pulseCounter, RISING);
  resetCounter();
  previousMillis = micros();
  return rpm;
}

void wait(int time) {
  int lasttime = millis();
  while ((millis() - lasttime) <= time) {
  }
}