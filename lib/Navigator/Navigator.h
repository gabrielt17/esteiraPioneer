#ifndef NAVIGATOR_H
#define NAVIGATOR_H

/** @brief Navigator driver library for ESP32.
* It uses a 10-bit resolution PWM signal to control the Navigators.
*
* You can especify the ESP32 PWM channel you desire or omit it when 
*creating a object, which in the last case will use the default 0 channel.
* @param In1 INI1 H-bridge pin.
* @param In2 INI2 H-bridge pin.
* @param Pwm PWM Input H-bridge pin .
 */
class Navigator {
    
    private:

        unsigned int in1 = 0;
        unsigned int in2 = 0;
        unsigned int pwmpin = 0;
        int channel = 0;

        void rotateClockwise();
        void rotateCounterClockwise();

    public:

        Navigator(const int In1, const int In2, const int Pwm, const int Channel);
        Navigator(const int In1, const int In2, const int Pwm);
        void goAhead(int PWM);
        void reverse(int PWM);
        void halt();
        void turnLeft(int PWM, Navigator SecNavigator);
        void turnRight(int PWM, Navigator SecNavigator);
        void switchInput();

};

#endif // NAVIGATOR_H