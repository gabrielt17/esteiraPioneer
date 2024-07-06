#include "Controller.h"
#include "Arduino.h"

// Class constructor
Controller::Controller(float KP, float KD, float KI) 
: kp(KP), kd(KD), ki(KI) {
}

// Calculates the control signal to the given parameters
float Controller::getControlSignal(float TARGET, float RPMMEASUREMENT) {

    // Time in which the error was registered
    unsigned long currentMicros = micros();

    // Error and time intervals calculations
    float deltaMicros = static_cast<float>((currentMicros-previousMicros))/(1e6);
    float error = TARGET - RPMMEASUREMENT;
    float deltaError = error-previousError;

    // Time in which the previous error was registered
    previousMicros = currentMicros;

    // Proporcional instance
    float r = error*Controller::kp;

    // Derivative instance
    float d = (deltaError/deltaMicros)*Controller::kd;

    // Integrative instance
    Controller::i += error*Controller::ki*deltaMicros;

    // Control signal equation
    float u = r + d + i;

    Controller::previousError = error;
    
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
float Controller::controlMotor(int TARGET, float RPMMEASUREMENT) {

    Controller::accumulated += Controller::convertToPWM(Controller::getControlSignal(TARGET, RPMMEASUREMENT));
    return Controller::accumulated;
}
