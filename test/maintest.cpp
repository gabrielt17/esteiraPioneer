#include <Pins.h>
#include <ISR.h>

ISR test;

void wait(int Time);

void setup() {
    Serial.begin(115200);

}

void loop() {

    Serial.println("Parce que deu certo");
    Serial.println("Valor de counter: " + test.counter);
    wait(500);
}

// Delay function that doesn't engage sleep mode
void wait(int time) {
  int lasttime = millis();
  while ((millis() - lasttime) <= time) {
  }
}