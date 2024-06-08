#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller {
    private:
        double kp;
        double ki;
        double kd; 
        double i = 0; // Stores the integrative calculations
        double pwm = 0; // Stores the lastest calculated PWM conversion
        unsigned long previousMicros = 0 ;
        double previousError = 0;
        int accumulated = 0;
    public:
        Controller(double Kp, double Kd, double Ki);
        double getControlSignal(int Target, double Measurement);
        double convertToPWM(double setPoint, double Value);
        int controlMotor(int Target, int Measurement);
};


#endif 