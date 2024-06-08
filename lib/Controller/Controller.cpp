#include "Controller.h"
#include "Arduino.h"

Controller::Controller(double Kp, double Kd, double Ki) {
    Controller::kp = Kp;
    Controller::kd = Kd;
    Controller::ki = Ki;
}

double Controller::getControlSignal(int Target, double Measurement) {

    // Calculating parameters
    unsigned long currentMicros = micros();
    double deltaMicros = static_cast<double>((currentMicros-previousMicros))/(1.0e6);
    double error = static_cast<double>(Target - Measurement);
    double deltaError = static_cast<double>(error-previousError);
    previousMicros = currentMicros;

    // Serial.printf("\nValor de deltaMillis: %3.3f\n", deltaMicros);
    // Serial.printf("\nValor de error: %3.3f\n", error);
    // Serial.printf("\nValor de deltaError: %3.3f\n", deltaError);

    // Proporcional instance
    double r = error*Controller::kp;

    // Derivative instance
    double d = (deltaError/deltaMicros)*Controller::kd;
    // Serial.printf("\nValor do derivativo: %f\n", d);
    // Integrative instance
    Controller::i += error*Controller::ki*deltaMicros;
    // Serial.printf("\nValor do integrativo: %f", i);

    // Serial.printf("\nValor de kp: %f", kp);
    // Serial.printf("\nValor de kd: %f", kd);
    // Serial.printf("\nValor de ki: %f", ki);

    // Control signal equation
    double u = r + d + i;

    // Serial.printf("\nValor de u: %f", u);

    Controller::previousError = error;
    
    return u;
}

double Controller::convertToPWM(double setPoint, double Value) {
    
    // Controller::pwm = 0.05656472*setPoint+7.58660113+Value; // 8 bit resolution
    // Controller::pwm = 0.23435580855242363*setPoint+48.46517168830019+Value;
    Controller::pwm += Value;
    // Serial.printf("\nValor de **PWM**: %3.3f", pwm);
    
    if (pwm > 1023) {
        // Serial.printf("\nEntrei no if do 255");
        pwm = 1023;
    }
    else if (pwm < -1023)
    {
        pwm = -1023;
    }
    return pwm;
}

int Controller::controlMotor(int Target, int Measurement) {
    // float controle = Controller::getControlSignal(Target, Measurement);
    // Serial.printf("\nValor de controle: %.3f", controle);
    // int finalVal = Controller::convertToPWM(controle);
    // Serial.printf("\n Valor final: %d\n", finalVal);
    // return finalVal; 
    Controller::accumulated += Controller::convertToPWM(Target, Controller::getControlSignal(Target, Measurement));
    return Controller::accumulated;
}