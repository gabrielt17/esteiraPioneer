#ifndef MOTOR_H
#define MOTOR_H

class Motor {
    
    private:

        unsigned int in1 = 0;
        unsigned int in2 = 0;
        unsigned int pwmpin = 0;
        int channel = 0;

    public:

        Motor(const int In1, const int In2, const int Pwm, const int Channel);
        Motor(const int In1, const int In2, const int Pwm);
        void goAhead(int PWM);
        void reverse(int PWM);
        void halt();
        void turnLeft(int PWM, Motor SecMotor);
        void turnRight(int PWM, Motor SecMotor);
        void switchInput();

};

#endif
