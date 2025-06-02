#include <Arduino.h>
#include <ESP32Encoder.h>
#include "Pins.h"
#include <Motor.h>

// Crie um objeto encoder
ESP32Encoder encoder;

// Motor A (LEFT)
Motor lmotor(AIN1, AIN2, PWMA);

// Motor B (RIGHT)
Motor rmotor(BIN1, BIN2, PWMB, 1);


// Variável para armazenar a última contagem de pulsos lida
long lastCount = 0;

void setup() {
  Serial.begin(115200); // Inicia a comunicação serial
  delay(1000);         // Pequena pausa para garantir que o Monitor Serial esteja pronto

  Serial.println("Teste de Encoder ESP32");
  Serial.println("Gire o encoder manualmente.");

  // Habilita os resistores de pull-up internos do ESP32 (recomendado para encoders)
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  // Você também pode usar DOWN se o seu encoder precisar de pull-downs, ou NONE se já tiver resistores externos.

  // Anexa os pinos ao objeto encoder usando o modo de quadratura completa
  // Isso usa ambas as bordas de ambos os canais A e B para 4x a resolução
  encoder.attachFullQuad(encoderAChannel1, encoderAChannel2);

  // Zera a contagem inicial do encoder
  encoder.clearCount();
  Serial.print("Contagem inicial do encoder: ");
  Serial.println(encoder.getCount());
}

void loop() {
  // Lê a contagem atual de pulsos do encoder
  long currentCount = encoder.getCount();

  // Verifica se a contagem mudou desde a última leitura
  if (currentCount != lastCount) {
    Serial.print("Contagem de Pulsos: ");
    Serial.println(currentCount);
    lastCount = currentCount; // Atualiza a última contagem
  }

  
}
