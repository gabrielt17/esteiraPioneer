#include "Navigator.h"
#include "Arduino.h"

/** @brief Class contructor.
* Creates the Navigator object
*/
Navigator::Navigator(Controller& LCONTROLLER, Controller& RCONTROLLER) 
: lController(LCONTROLLER), rController(RCONTROLLER) {
}

/** @brief Drives the robot foward.
* Negative PWM values will drive the robot in the opposite direction.
* @param COMMAND PWM that will be sent to the PWM channel.
*/
motorPWM Navigator::move(geometry_msgs::Twist COMMAND, float LCURRENTMEASUREMENT, float RCURRENTMEASUREMENT) {
	int wheelangularvel = static_cast<int>(COMMAND.linear.x/0.025);
	int lpwm = lController.controlMotor(wheelangularvel, LCURRENTMEASUREMENT);
	int rpwm = rController.controlMotor(-wheelangularvel, RCURRENTMEASUREMENT);
	return {lpwm, rpwm};
} 

