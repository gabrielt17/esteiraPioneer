#include <Arduino.h>
#include <Pins.h>

// Counter variables
volatile int encoderAChannel1counter = 0;
volatile int encoderBChannel1counter = 0;

volatile bool encoderAChannel1trigger = false; // True if the encoderAChannel1 pin ISR was called at least one time
volatile bool encoderBChannel1trigger = false; // True if the encoderBChannel1 pin ISR was called at least one time
volatile int numberOfButtonInterrupts = 0;  // Set how many times the ISR was called

// Object that initiates a critical session
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Defining ISR functions
void IRAM_ATTR encoderAChannel1Count();
void IRAM_ATTR encoderBChannel1Count();

void setup() {


  attachInterrupt(digitalPinToInterrupt(encoderAChannel1), &encoderAChannel1Count, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderBChannel1), &encoderBChannel1Count, RISING);
  Serial.begin(115200);

  pinMode(encoderAChannel1, INPUT);
  pinMode(encoderBChannel1, INPUT);
  pinMode(encoderAChannel1, INPUT);
}

void loop() {
  
  // If one of the ISR's was called, then one of the counter should be iterated
  if (numberOfButtonInterrupts != 0) {
    
    // In case the encoderAChannel1 ISR was called, then +1 to the encoderAChannel1 counter
    if (encoderAChannel1trigger) {
      encoderAChannel1counter++;
      encoderAChannel1trigger = false;
      Serial.println("Ponto para o AZUL!");
    } 
    // Else, the encoderBChannel1 counter gets +1 in its counter
    else if (encoderBChannel1trigger) {
      encoderBChannel1counter++;
      encoderBChannel1trigger = false;
      Serial.println("Ponto para o ROXO!");
    }
    portENTER_CRITICAL_ISR(&mux);
    numberOfButtonInterrupts = 0;  // Resets the cycle
    portEXIT_CRITICAL_ISR(&mux);

    // Print out the counters
    Serial.printf("\nPontuação do azul: %d", encoderAChannel1counter);
    Serial.printf("\nPontuação do roxo: %d\n", encoderBChannel1counter);
  }
}

// encoderAChannel1 counter trigger
void IRAM_ATTR encoderAChannel1Count() {
  portENTER_CRITICAL_ISR(&mux);
  encoderAChannel1trigger = true;
  numberOfButtonInterrupts++;
  portEXIT_CRITICAL_ISR(&mux);
}

// 
void IRAM_ATTR encoderBChannel1Count() {
  portENTER_CRITICAL_ISR(&mux);
  encoderBChannel1trigger = true;
  numberOfButtonInterrupts++;
  portEXIT_CRITICAL_ISR(&mux);
}

void wait(int time) {
  int lasttime = millis();
  while (true) {
    if ((millis() - lasttime) >= time) {
      break;
    }
  }
}
