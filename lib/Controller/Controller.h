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
        float accumulated = 0;
        
    public:

        Controller(float KP, float KD, float KI);
        float getControlSignal(float TARGET, float RADSMEASUREMENT);
        float convertToPWM(float VALUE);
        float controlMotor(int TARGET, float RPMMEASUREMENT);
};


#endif 
