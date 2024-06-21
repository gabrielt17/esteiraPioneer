#include <ISR.h>

#include <Arduino.h>

ISR* ISR::sISR = nullptr;

ISR::ISR() {

    sISR = this;
    attachInterrupt(digitalPinToInterrupt(digitalPinToInterrupt(13)), ISR::ISRcaller, RISING);
    ISR::ISRcaller();
}

void ISR::ISRcaller() {

    if (sISR != nullptr) {

        sISR->ISRcounter();
    }
}

void ISR::ISRcounter() {

    ISR::counter = ISR::counter + 1;
}