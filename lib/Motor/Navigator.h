#ifndef NAVIGATOR_H
#define NAVIGATOR_H

/** @brief Motor driver library for ESP32.
* It uses a 10-bit resolution PWM signal to control the motors.
*
* You can especify the ESP32 PWM channel you desire or omit it when 
*creating a object, which in the last case will use the default 0 channel.
* @param In1 INI1 H-bridge pin.
* @param In2 INI2 H-bridge pin.
* @param Pwm PWM Input H-bridge pin .
 */
class Motor {
    
    private:

        unsigned int in1 = 0;
        unsigned int in2 = 0;
        unsigned int pwmpin = 0;
        int channel = 0;

        void rotateClockwise();
        void rotateCounterClockwise();

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
