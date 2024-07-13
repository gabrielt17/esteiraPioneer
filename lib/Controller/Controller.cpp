#include "Controller.h"

/** @brief
 * Class constructor
 * @param KP Proporcional gain constant. Should be the biggest value of
 *all three.
 * @param KD Derivative gain constant. Should be smaller compared to KP.
 * @param KI Integrative gain constant. Should even smaller than KP and KD.
 */
Controller::Controller(float KP, float KD, float KI) 
: kp(KP), kd(KD), ki(KI) {
}

/** @brief
 * Calculates the control signal to the given parameters
 * @param 
 */ 
float Controller::getControlSignal(float TARGET, float RPMMEASUREMENT) {

    // Time in which the error was registered
    unsigned long currentMicros = micros();

    // Error and time intervals calculations
    float deltaMicros = static_cast<float>((currentMicros-previousMicros))/(1e6);
    float error = TARGET - RPMMEASUREMENT;
    float deltaError = error-previousError;
    if (fabs(error) < 3) {
        return 0;
    } 

    // Time the last error measurement was taken
    previousMicros = currentMicros;

    // Proporcional formula
    float r = error*this->kp;

    // Derivative formula
    float d = (deltaError/deltaMicros)*this->kd;

    // Integrative formula
    this->i += error*this->ki*deltaMicros;

    // Control signal equation
    float u = r + d + i;

    // Registers tjh
    this->previousError = error;
    
    return u;
}

// Converts the given control signal into a 10-bit PWM value
float Controller::convertToPWM(float VALUE) {
    
    float pwm = VALUE;
    if (pwm > 1023) {
        pwm = 1023;
    }
    else if (pwm < -1023)
    {
        pwm = -1023;
    }

    return pwm;
}

/** @brief
 * Combines the previous methods into one. Accumulates the control singal.
 *  
 */ 
float Controller::controlMotor(float TARGET, float RPMMEASUREMENT) {

    this->accumulated += this->convertToPWM(this->getControlSignal(TARGET, RPMMEASUREMENT));
    return this->accumulated;
}
