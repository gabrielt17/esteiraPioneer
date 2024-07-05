#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller {

    private:

        float kp;
        float ki;
        float kd; 
        float i = 0; // Stores the integrative calculations
        float pwm = 0; // Stores the lastest calculated PWM conversion
        unsigned long previousMicros = 0 ;
        float previousError = 0;
        int accumulated = 0;
        
    public:

        Controller(float Kp, float Kd, float Ki);
        float getControlSignal(int Target, float Measurement);
        int convertToPWM(float Value);
        int controlMotor(int Target, float Measurement);
};


#endif 
