#include <Arduino.h>
#include <Motor.h>
#include "Pins.h"
#include <ESP32Encoder.h>

// Encoder resolution: (1/28)*(60*1000/delta_t)

// --- CONFIGURAÇÕES ---
const int PWM_TEST = 511; // PWM de teste (0-1023)
const int PULSES_PER_REV = 28;
const uint16_t sample_time = 100; // Amostragem em ms (influencia no cálculo do RPM e resolução)
const float alpha = 0.5; // Ajuste do filtro de memória
const float bias_correction = 1.008840; // Fator calculado via Mínimos Quadrados


// --- VARIÁVEIS GLOBAIS ---
TimerHandle_t encoderTimer = NULL;
volatile int32_t pulsesInWindow = 0; // Quantidade de pulsos contados por tempo de amostragem
volatile bool calculateRPM = false;
float last_rpm = 0.0; // Último valor de RPM calculado para filtro de memória
bool first_reading = true;

ESP32Encoder encoder;

Motor rmotor(BIN1, BIN2, PWMB, 1);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void gatherEncoderData(TimerHandle_t xTimer);
void wait(int Time);

void setup() {
  
  Serial.begin(115200);

  encoderTimer = xTimerCreate(
    "EncoderTimer",                   // Timer name
    pdMS_TO_TICKS(sample_time),      // Período em ticks
    pdTRUE,                         // Auto-reload (periodic timer)
    NULL,                           // Timer ID
    gatherEncoderData                  // Callback function
  );
  
  if (encoderTimer == NULL) {
    Serial.println("Failed to create timer!");
    while (1);
  }

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  
  encoder.attachFullQuad(encoderBChannel2, encoderBChannel1);
  
  encoder.clearCount();

  rmotor.setSpeed(PWM_TEST);

  wait(3000); // Espera motor entrar em regime permanente.

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

    if (first_reading) {
      last_rpm = ((float)pulses / (float)PULSES_PER_REV) * (60000.0f/(float)sample_time);
      last_rpm *= bias_correction;
      first_reading = false;
      doCalc = false;
    }

    if (doCalc) {
      float rpm_raw = ((float)pulses / (float)PULSES_PER_REV) * (60000.0f/(float)sample_time);
      rpm_raw *= bias_correction;
      float rpm_filtered = alpha*rpm_raw + (1.0f - alpha)*last_rpm;
      last_rpm = rpm_filtered;
      Serial.printf(">RPM: %.2f\n", rpm_filtered);
    }
  }

  if (millis() > 60000) {
    rmotor.setSpeed(0);
    while (1);
  }
}

void gatherEncoderData(TimerHandle_t xTimer) {
  portENTER_CRITICAL(&mux);
  pulsesInWindow = encoder.getCount();
  encoder.clearCount();
  calculateRPM = true;
  portEXIT_CRITICAL(&mux);
}

// Delay function that doesn't engage sleep mode
void wait(int time) {
  int lasttime = millis();
  vTaskDelay(time / portTICK_PERIOD_MS);
}