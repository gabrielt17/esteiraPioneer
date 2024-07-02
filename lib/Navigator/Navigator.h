#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "Motor.h"

/** @brief Controls the robot movement.
* @param lMotor Left motor object from Motor class.
* @param rMotor Right motor object from Motor class.
 */
class Navigator {
    
    private:

        Motor& lMotor;
        Motor& rMotor;

        void changeSpeed(const int lPWM, const int rPWM);

    public:

        Navigator(Motor& LMOTOR, Motor& RMOTOR);
        void moveAhead(const int PWM);
        void moveBackwards(const int PWM);
        void halt();
        void turnLeft(const int lPWM, const int rPWM);
        void turnRight(const int lPWM, const int rPWM);
};

#endif // NAVIGATOR_H