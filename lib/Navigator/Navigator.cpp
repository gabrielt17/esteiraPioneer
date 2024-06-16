#include "Navigator.h"
#include "Arduino.h"

/** @brief Class contructor.
* This version of the constructor requires the user to specify the operation channel
*/
Navigator::Navigator(const int In1, const int In2, const int Pwm, const int Channel) {
	
	Navigator::in1 = In1;
	Navigator::in2 = In2;
	Navigator::pwmpin = Pwm;
	Navigator::channel = Channel;

	pinMode(Navigator::in1, OUTPUT);
	pinMode(Navigator::in2, OUTPUT);

	ledcSetup(Navigator::channel, 5000, 10);
	ledcAttachPin(Navigator::pwmpin, Navigator::channel);
	
}

/** @brief Class contructor.
* This version of the constructor uses the channel 0 as default
*/
Navigator::Navigator(const int In1, const int In2, const int Pwm) {
	
	Navigator::in1 = In1;
	Navigator::in2 = In2;
	Navigator::pwmpin = Pwm;
	Navigator::channel = 0;

	pinMode(Navigator::in1, OUTPUT);
	pinMode(Navigator::in2, OUTPUT);

	ledcSetup(Navigator::channel, 5000, 10);
	ledcAttachPin(Navigator::pwmpin, 0);

}

/** @brief Drives the Navigator foward.
 * Negative PWM values will drive the Navigator in the opposite direction.
* @param PWM PWM that will be sent to the Navigator channel.
*/
void Navigator::goAhead(int PWM) {

	if (PWM < 0) {Navigator::reverse(-PWM);}
	else {
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	ledcWrite(Navigator::channel, PWM);
	}

}

/** @brief Drives the Navigator backwards.
* Negative PWM values will drive the Navigator in the opposite direction.
* @param PWM PWM that will be sent to the Navigator channel.
*/
void Navigator::reverse(int PWM) {

	if (PWM < 0) {Navigator::goAhead(-PWM);}
	else {
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	ledcWrite(Navigator::channel, PWM);
	}
}

void Navigator::halt() {

	ledcWrite(Navigator::channel, 0);
}

void Navigator::turnLeft(int PWM, Navigator SecNavigator) {

	digitalWrite(Navigator::in1, HIGH);
	digitalWrite(Navigator::in2, LOW);
	digitalWrite(SecNavigator.in1, LOW);
	digitalWrite(SecNavigator.in2, HIGH);
	ledcWrite(Navigator::channel, PWM);
	ledcWrite(SecNavigator.channel, PWM);

}

void Navigator::turnRight(int PWM, Navigator SecNavigator) {

	digitalWrite(Navigator::in1, LOW);
	digitalWrite(Navigator::in2, HIGH);
	digitalWrite(SecNavigator.in1, HIGH);
	digitalWrite(SecNavigator.in2, LOW);
	ledcWrite(Navigator::channel, PWM);
	ledcWrite(SecNavigator.channel, PWM);

}

