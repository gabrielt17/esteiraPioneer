/*

Relevant info:

DISCLAIMER: Tests made using 10-bit resolution PWM

*/

#include <Arduino.h>
#include <Motor.h>
#include <Controller.h>
#include "Pins.h"
#include <Kinematics.h>
#include <ESP32Encoder.h>
#include <SimpleKalmanFilter.h>

// Encoder pulse count by rotation
// const int PULSERPERROTATION = 8;
const int PULSERPERROTATION_FULLQUADRATE = 28;
// PID Controller constants
const double kp = 0.2;
const double kd = 0;
const double ki = 0;

// Global variables

// Time
const long SAMPLE_TIME_MICROS = 500000;
unsigned long LAST_SAMPLE_TIME = 0;
const int CALLBACK_INTERVAL = 1000000;

// Controller
Controller lcontroller(kp, kd, ki);
Controller rcontroller(kp, kd, ki);

// Motor A (LEFT)
Motor lmotor(AIN1, AIN2, PWMA);

// Motor B (RIGHT)
Motor rmotor(BIN1, BIN2, PWMB, 1);

// Encoder object
ESP32Encoder encoderA;
ESP32Encoder encoderB;

// Kalman filter
SimpleKalmanFilter lkf = SimpleKalmanFilter(5, 5, 0.01);
SimpleKalmanFilter rkf = SimpleKalmanFilter(5, 5, 0.01);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Function prototypes (declarations)
void wait(int Time);

motorRPM rpm;

void setup()
{

    // Initalizing serial
    Serial.begin(115200);
    delay(1000);

    // Encoder as pull-up
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    // Attach to the pins as full quadrature encoders
    encoderA.attachFullQuad(encoderAChannel1, encoderAChannel2);
    encoderB.attachFullQuad(encoderBChannel1, encoderBChannel2);

    // Resets the initial count of the encoders
    encoderA.clearCount();
    encoderB.clearCount();
}



void loop()
{
    for (int pwm = 0; pwm < 1024; pwm=pwm+5)
    {
        LAST_SAMPLE_TIME = micros();
        lmotor.setSpeed(pwm);
        wait(2000);
        int CURRENT_SAMPLE_TIME = micros();
        // Calcula o RPM para cada motor com base nos pulsos coletados
        float lRPM = convertPulsesToRPM(encoderA.getCount(), PULSERPERROTATION_FULLQUADRATE, CURRENT_SAMPLE_TIME - LAST_SAMPLE_TIME);
        encoderA.clearCount();
        Serial.printf("%d,%2.1f\n", pwm, lRPM);
    }
    lmotor.setSpeed(0);
}
/* Function definitions */

// Delay function that doesn't engage sleep mode
void wait(int time)
{
    int lasttime = millis();
    while ((millis() - lasttime) <= time)
    {
    }
}