#include "Controller.h"
#include "Arduino.h"

// Class constructor
Controller::Controller(float Kp, float Kd, float Ki) {
    
    Controller::kp = Kp;
    Controller::kd = Kd;
    Controller::ki = Ki;
}

// Calculates the control signal to the given parameters
float Controller::getControlSignal(int Target, float RPMMeasurement) {

    // Time in which the error was registered
    unsigned long currentMicros = micros();

    // Error and time intervals calculations
    float deltaMicros = static_cast<float>((currentMicros-previousMicros))/(1e6);
    float error = static_cast<float>(Target - RPMMeasurement);
    float deltaError = static_cast<float>(error-previousError);

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
int Controller::convertToPWM(float Value) {
    
    int pwm = Value;
    
    if (pwm > 1023) {
        pwm = 1023;
    }
    else if (pwm < -1023)
    {
        pwm = -1023;
    }

    return static_cast<int>(pwm);
}

/** @brief
 * Combines the previous methods into one. Accumulates the control singal.
 *  
 */ 
int Controller::controlMotor(int Target, float Measurement) {

    Controller::accumulated += Controller::convertToPWM(Controller::getControlSignal(Target, Measurement));
    return Controller::accumulated;
}
