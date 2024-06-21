#include <Encoder.h>
#include <Pins.h>

const int interval = 300000;

Encoder encoder(encoderApin, 38, encoderAPower); // Exemplo de pinos

void test();

void setup() {
    Serial.begin(115200);
    attachInterrupt(digitalPinToInterrupt(encoderApin), test, RISING);
}

void loop() {
    if ((micros() - encoder.previousMicros) > interval) {
      detachInterrupt(digitalPinToInterrupt(encoderApin));
      encoder.calculateRPM();
      attachInterrupt(digitalPinToInterrupt(encoderApin), test, RISING);
    }
    
    Serial.println(encoder.getRPM());
    delay(1000);
}

void test() {
  encoder.addToCounter();
}