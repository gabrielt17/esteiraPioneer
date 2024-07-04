#include "Pins.h"

void setup() {

    pinMode(AIN1, OUTPUT);
    pinMode(BIN1, OUTPUT);
    digitalWrite(BIN1, HIGH);
    digitalWrite(AIN1, HIGH);
    ledcSetup(0, 5000, 10);
    ledcAttachPin(PWMA, 0);
    ledcSetup(1, 5000, 10);
    ledcAttachPin(PWMB, 1);

}
void loop() {
    ledcWrite(0, 600);
    ledcWrite(1, 600);
}