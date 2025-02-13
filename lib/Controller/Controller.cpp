#include "Controller.h"

/** @brief
 * Class constructor
 * @param KP Proporcional gain constant. Should be the biggest value of
 * all three.
 * @param KD Derivative gain constant. Should be smaller compared to KP.
 * @param KI Integrative gain constant. Should be even smaller than KP and KD.
 */
Controller::Controller(float KP, float KD, float KI) 
: kp(KP), kd(KD), ki(KI), previousMicros(0), previousError(0), i(0), accumulated(0) {
}

/** @brief
 * Calculates the control signal to the given parameters.
 */ 
float Controller::getControlSignal(float TARGET, float RPMMEASUREMENT) {

    // Time in which the error was registered
    unsigned long currentMicros = micros();
    float deltaMicros = static_cast<float>(currentMicros - previousMicros) / 1e6;
    
    // Calculate error
    float error = TARGET - RPMMEASUREMENT;
    float deltaError = error - previousError;
    const float errorThreshold = 1.0;

    // If error is small, reduce control effort smoothly
    if (fabs(error) < errorThreshold) {
        return accumulated * 0.9;
    } 

    // Update previous time
    previousMicros = currentMicros;

    // Proportional term
    float r = error * kp;

    // Derivative term (only if delta time is reasonable)
    float d = (deltaMicros > 1e-6) ? (deltaError / deltaMicros) * kd : 0;

    // Integrative term (prevent integral windup)
    i += error * ki * deltaMicros;
    const float iMax = 500.0;
    if (i > iMax) i = iMax;
    if (i < -iMax) i = -iMax;

    // Control signal
    float u = r + d + i;

    // Save previous error
    previousError = error;
    
    return u;
}

/** @brief
 * Combines the previous methods into one. Accumulates the control signal.
 */ 
float Controller::controlMotor(float TARGET, float RPMMEASUREMENT) {
    float u = getControlSignal(TARGET, RPMMEASUREMENT);
    u = constrain(u, -1023, 1023);


    return u;
}