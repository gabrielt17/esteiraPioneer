#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "Controller.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "Motor.h"
#include "Encoder.h"

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

        Controller &lController;
        Controller &rController;
        Motor &lmotor;
        Motor &rmotor;
        Encoder &lencoder;
        Encoder &rencoder;
        float radius = 1;
        float wheeldist;

        float linearToRPM(geometry_msgs::Twist MSG);
        float angularToRPM(geometry_msgs::Twist MSG);

    public:

        Navigator(Controller& LCONTROLLER, Controller& RCONTROLLER,\
        Motor &LMOTOR, Motor &RMOTOR, Encoder &LENCODER,\
        Encoder &RENCODER, float RADIUS, float WHEELDIST);

        void move(geometry_msgs::Twist COMMAND);
};

#endif // NAVIGATOR_H