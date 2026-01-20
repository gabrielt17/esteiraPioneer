#include <Arduino.h>
#include <Motor.h> 
#include "Pins.h" 

// --- CONFIGURAÇÕES ---
// Resolução do PWM (10 bits = 1023)
const int MAX_PWM = 1023; 

// Pino do LED (Geralmente GPIO 2 no ESP32)
const int PIN_LED = 2;    

// Tempo em cada degrau para estabilização (ms)
const int TEMPO_POR_DEGRAU = 10000; 

Motor lmotor(AIN1, AIN2, PWMA); 

// Função auxiliar para piscar o LED rápido
void piscarLed() {
  digitalWrite(PIN_LED, HIGH);
  delay(100); 
  digitalWrite(PIN_LED, LOW);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_LED, OUTPUT);
  lmotor.switchInput(); 
  
  Serial.println("--- INICIANDO RAMPA (5% em 5% até 50%) ---");
  delay(1000);
}

void loop() {
  
  // Loop de 0% até 50%, pulando de 5 em 5
  for (int pct = 0; pct <= 50; pct += 5) {
    
    // Cálculo exato da porcentagem para PWM
    // Ex: 50% de 1023 = 511
    int pwm_atual = (pct * MAX_PWM) / 100;
    
    // Atualiza Motor
    lmotor.setSpeed(pwm_atual);
    
    // Pisca LED indicando mudança
    piscarLed();
    
    // Log
    Serial.printf("Nível: %d%% | PWM: %d\n", pct, pwm_atual);
    
    // Aguarda
    delay(TEMPO_POR_DEGRAU);
  }

  Serial.println("Chegou em 50%. Parando e aguardando...");
  
  // Para o motor
  lmotor.setSpeed(0);
  
  // Espera 5 segundos antes de reiniciar o teste
  delay(5000);
}