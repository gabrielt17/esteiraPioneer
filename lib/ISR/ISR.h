#ifndef ISR_H
#define ISR_H

#include <Arduino.h>

class ISR {

    private:

        static ISR* sISR;
        static void ISRcaller();

        void ISRcounter();

    public:

        ISR();
        int counter = 0;
};

#endif // ISR_H