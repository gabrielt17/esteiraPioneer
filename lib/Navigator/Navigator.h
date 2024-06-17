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

        void Navigator::changeSpeed(const int16_t PWM);

    public:

        Navigator(Motor& LMOTOR, Motor&  RMOTOR);
        void Navigator::moveAhead(const int16_t PWM);
        void Navigator::moveBackwards(const int16_t PWM);
        void Navigator::halt();
        void Navigator::turnLeft(const int16_t PWM);
        void Navigator::turnRight(const int16_t PWM);
};

#endif // NAVIGATOR_H