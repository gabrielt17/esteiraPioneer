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

        void changeSpeed(const int PWM);

    public:

        Navigator(Motor& LMOTOR, Motor& RMOTOR);
        void moveAhead(const int PWM);
        void moveBackwards(const int PWM);
        void halt();
        void turnLeft(const int PWM);
        void turnRight(const int PWM);
};

#endif // NAVIGATOR_H