#include <Arduino.h>
#include <Motor.h>
#include "Pins.h"
#include <ESP32Encoder.h>

// Encoder resolution: (1/28)*(60*1000/delta_t)

// --- CONFIGURAÇÕES ---
const int PWM_TEST = 512; 
const int PULSES_PER_REV = 28;
const uint16_t sample_time = 100;


// --- VARIÁVEIS GLOBAIS ---
TimerHandle_t encoderTimer = NULL;
volatile int32_t pulsesInWindow = 0;
volatile bool calculateRPM = false;

ESP32Encoder encoder;

Motor lmotor(AIN1, AIN2, PWMA);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void gatherEncoderData(TimerHandle_t xTimer);
void wait(int Time);

void setup() {
  // Aumentei a velocidade para não perder dados
  Serial.begin(115200);

  encoderTimer = xTimerCreate(
    "EncoderTimer",                   // Timer name
    sample_time / portTICK_PERIOD_MS,      // 500ms period
    pdTRUE,                         // Auto-reload (periodic timer)
    NULL,                           // Timer ID
    gatherEncoderData                  // Callback function
  );
  
  if (encoderTimer == NULL) {
    Serial.println("Failed to create timer!");
    while (1);
  }

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  
  encoder.attachFullQuad(encoderAChannel2, encoderAChannel1);
  
  encoder.clearCount();

  lmotor.setSpeed(PWM_TEST);

  wait(3000);

  xTimerStart(encoderTimer, 0);
}

void loop() {

  if (calculateRPM) {
    int32_t pulses = 0;
    bool doCalc = false;

    portENTER_CRITICAL(&mux);
    pulses = pulsesInWindow;
    doCalc = calculateRPM;
    calculateRPM = false;
    portEXIT_CRITICAL(&mux);

    if (doCalc) {
      float rpm = ((float)pulses / (float)PULSES_PER_REV) * (60000.0f/(float)sample_time);
      Serial.printf(">RPM: %.2f\n", rpm);
    }
  }

  if (millis() > 60000) {
    lmotor.setSpeed(0);
    while (1);
  }
}

void gatherEncoderData(TimerHandle_t xTimer) {
  portENTER_CRITICAL_ISR(&mux);
  pulsesInWindow = encoder.getCount();
  encoder.clearCount();
  calculateRPM = true;
  portEXIT_CRITICAL_ISR(&mux);
}

// Delay function that doesn't engage sleep mode
void wait(int time) {
  int lasttime = millis();
  vTaskDelay(time / portTICK_PERIOD_MS);
}