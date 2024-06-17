#ifndef MOTOR_H
#define MOTOR_H

/** @brief H-bridge motor driver library for ESP32.
* It uses a 10-bit resolution PWM signal to control the motors.
*
* You can especify the ESP32 PWM channel you desire or omit it when 
*creating a object.
* @param IN1 INI1 H-bridge pin.
* @param IN2 INI2 H-bridge pin.
* @param PWM PWM Input H-bridge pin.
* @param CHANNEL Custom channel value, defaults to 0.
 */
class Motor {

    private:

        uint8_t in1 = 0;
        uint8_t in2 = 0;
        uint8_t pwmpin = 0;
        uint8_t channel = 0;

        void setupArduino();
        void setupLedc();

    public:

        Motor(const uint8_t IN1, const uint8_t IN2, const uint8_t PWM, const uint8_t CHANNEL);
        Motor(const uint8_t IN1, const uint8_t IN2, const uint8_t PWM);
        
        void Motor::switchInput();
        void Motor::setClockwise();
        void Motor::setAntiClockwise();
        void Motor::setSpeed(const int PWM);

};

#endif // MOTOR_H