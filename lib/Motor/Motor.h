#ifndef MOTOR_H
#define MOTOR_H

class Motor {
    
    private:

        unsigned int IN1 = 0;
        unsigned int IN2 = 0;
        int PWM = 0;

    public:

        void Motor::motorSetup(int In1, int In2, int Pwm);
};

#endif