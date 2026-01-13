#include <Arduino.h>
#include <Motor.h>
#include "Pins.h"

// --- CONFIGURAÇÕES ---
const int PWM_TEST = 512; 
const int PULSES_PER_REV = 28;
const float ALPHA = 0.2; // Começando com seu valor

// --- VARIÁVEIS GLOBAIS ---
volatile uint32_t last_time = 0;
volatile uint32_t delta_t_current = 0;
volatile bool new_pulse = false;

float previous_rpm = 0.0;
bool is_first_reading = true; // Flag para corrigir a partida

Motor lmotor(AIN1, AIN2, PWMA); 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isr_coleta();

void setup() {
  // Aumentei a velocidade para não perder dados
  Serial.begin(115200); 
  
  lmotor.switchInput(); 
  pinMode(encoderAChannel1, INPUT);
  pinMode(encoderAChannel2, INPUT);
  
  Serial.println("--- INICIANDO DATA LOGGER ---");
  delay(1000);

  // Zera variáveis antes de começar
  last_time = 0;
  is_first_reading = true;

  // Liga Interrupções
  attachInterrupt(digitalPinToInterrupt(encoderAChannel1), isr_coleta, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderAChannel2), isr_coleta, CHANGE);

  // Liga Motor
  lmotor.setSpeed(PWM_TEST);
}

void loop() {
  // Variáveis locais para cópia segura
  uint32_t delta_t_copy = 0;
  bool pulse_detected = false;

  // --- SESSÃO CRÍTICA ---
  // Entra e sai o mais rápido possível
  if (new_pulse) {
    portENTER_CRITICAL(&mux);
    delta_t_copy = delta_t_current;
    pulse_detected = new_pulse;
    new_pulse = false;
    portEXIT_CRITICAL(&mux);
  }

  // --- PROCESSAMENTO (Fora da sessão crítica) ---
  if (pulse_detected && delta_t_copy > 0) {
    
    // 1. Calcula RPM Instantâneo (Bruto)
    float rpm_raw = (60.0 * 1000000.0) / (PULSES_PER_REV * delta_t_copy);

    // 2. Filtro EMA
    float rpm_filtered;
    
    if (is_first_reading) {
      // Se é a primeira vez, não filtra, assume o valor atual como o anterior
      rpm_filtered = rpm_raw;
      previous_rpm = rpm_raw;
      is_first_reading = false;
    } else {
      // Aplica o filtro normalmente
      rpm_filtered = (ALPHA * rpm_raw) + ((1.0 - ALPHA) * previous_rpm);
      previous_rpm = rpm_filtered;
    }

    // 3. Plotter (Formato Teleplot)
    // Plota as duas linhas para comparar
    Serial.printf(">rpm_raw:%.2f\n", rpm_raw);
    Serial.printf(">rpm_filter:%.2f\n", rpm_filtered);
  }

  // Timeout de segurança (10s)
  if (millis() > 10000) {
    lmotor.setSpeed(0);
    // Desliga interrupções para parar de processar
    detachInterrupt(digitalPinToInterrupt(encoderAChannel1));
    detachInterrupt(digitalPinToInterrupt(encoderAChannel2));
    Serial.println("Fim do teste.");
    while (1);
  }
}

// --- ISR OTIMIZADA ---
void IRAM_ATTR isr_coleta() {
  uint32_t currentTime = micros();
  
  // Só calcula se não for o primeiro pulso da história
  if (last_time != 0) {
    delta_t_current = currentTime - last_time;
    new_pulse = true;
  }
  
  last_time = currentTime;
}