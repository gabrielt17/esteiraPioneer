#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "Controller.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

struct motorPWM {
    int lpwm;
    int rpwm;
};

/** @brief Controls the robot movement.
* @param lMotor Left motor object from Motor class.
* @param rMotor Right motor object from Motor class.
 */
class Navigator {
    
    private:

        Controller& lController;
        Controller& rController;

    public:

        Navigator(Controller& LCONTROLLER, Controller& RCONTROLLER);
        motorPWM move(geometry_msgs::Twist COMMAND, float LCURRENTMEASUREMENT, float RCURRENTMEASUREMENT);
};

#endif // NAVIGATOR_H