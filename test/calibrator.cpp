/*
*    ASTROS, a Pioneer P3-DX inspired robot.
*    Copyright (C) <2024>  <Gabriel Víctor and Hiago Batista>
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "Pins.h"

#include <Arduino.h>
#include <Motor.h>

#include <Converter.h>
#include <Controller.h>
#include <Encoder.h>
#include <ArduinoOTA.h>

#include <ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32MultiArray.h>
#include <custom_msgs/Encoder.h>
#include <custom_msgs/Motor_power.h>

#include <WiFi.h>
#include <SimpleKalmanFilter.h>

/**********************************************************************
* PID constants
**********************************************************************/
const double lKP = 2.0401663533082357/30;
const double lKD = 2.0401663533082357/2000;
const double lKI = 0;

const double rKP = 0.792/30;
const double rKD = 0.792/2000;
const double rKI = 0;

/**********************************************************************
* Global variables and defines
**********************************************************************/

// WiFi
const char* SSID = "NERo-Arena";
const char* PASSWD = "BDPsystem10";

// Encoder pulse count by rotation
const uint8_t PULSESPERROTATION = 91; // Encoder pulses in a single spin
const uint8_t TOLERANCE = 3;

// Time
const uint CALCULATERPM_INTERVAL = 50000; // Time to calculate RPM
const uint CBTIMEOUT_INTERVAL = 1000000; // Timeout time to callback
unsigned long cbTimeout = 0; // Callback timemout timer

// Controller
Controller lcontroller(lKP, lKD, lKI);
Controller rcontroller(rKP, rKD, rKI);

// Motor A (LEFT)
Motor lmotor(AIN1, AIN2, PWMA);

// Motor B (RIGHT)
Motor rmotor(BIN1, BIN2, PWMB, 1);

// Encoder A (LEFT)
Encoder lencoder(encoderAChannel1, PULSESPERROTATION);

// Encoder B (RIGHT)
Encoder rencoder(encoderBChannel1, PULSESPERROTATION);

// Kalman filter
SimpleKalmanFilter lkf = SimpleKalmanFilter(5, 5, 0.01);
SimpleKalmanFilter rkf = SimpleKalmanFilter(5, 5, 0.01);

// Critical session initializer
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// RPM target values

MotorVel vel; // Stores the callback in RPM values


// PWM target values

  // Both stores PWM from Controller or pwmCb
float lpwm;
float rpwm;

// If the user sets a manual PWM value, then the controller main loop will be turned off
bool overrideCb = false;

// ROS variables and functions
IPAddress server(192,168,0,112); // MASTER IP
const uint16_t serverPort = 11411; // TCP CONNECTION PORT
ros::NodeHandle nh; // Node handle object
geometry_msgs::Twist rosvel;
custom_msgs::Encoder encoderReadings;
void ROS_Setup();
void velCb(const geometry_msgs::Twist &MSG);
void pwmCb(const custom_msgs::Motor_power &MSG);
ros::Subscriber<custom_msgs::Motor_power> motor_pw("motor_pw", &pwmCb);
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &velCb);
ros::Publisher encoderFb("encoder_fb", &encoderReadings);

/**********************************************************************
* Function prototypes (declarations)
**********************************************************************/ 
void wait(int Time);
void IRAM_ATTR lencoderCounter();
void IRAM_ATTR rencoderCounter();
void lCalculate();
void rCalculate();

void setup() {

  // Initalizing serial
  Serial.begin(115200);

  // Attach encoder ISR's
  attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
  attachInterrupt(encoderBChannel1, rencoderCounter, RISING);

}

void loop() {

  int pwm = -300;
  while (pwm <= 1023) {

    if (pwm > 1023) {
      pwm = 1023;
    }

    lmotor.setSpeed(pwm);
    rmotor.setSpeed(pwm);

    wait(3000);

    Serial.println(lencoder.pulses);

    while (!lencoder.isCalculated) {
      lCalculate();
      wait(200);
    }

    float currentlRPM = lencoder.getRPM(lmotor.isClockwise);
    float lastReading = currentlRPM;

    wait(1000);

    while (!lencoder.isCalculated) {
      lCalculate();
      wait(200);
    }

    currentlRPM = lencoder.getRPM(lmotor.isClockwise);

    if (abs(currentlRPM - lastReading) <= TOLERANCE) {
      Serial.printf("%d;%3.3f\n", pwm, currentlRPM);
      lmotor.setSpeed(0);
      rmotor.setSpeed(0);
      wait(500);
      pwm += 10;
    }

    
  }

  // int pwm = -1023;
  // while (pwm <= 1023) {
  //   if (pwm > 1023) {
  //     pwm = 1023;
  //   }
  //   lmotor.setSpeed(pwm);
  //   rmotor.setSpeed(pwm);

  //   while (!rencoder.isCalculated) {
  //     rCalculate();
  //     wait(3000);
  //   }
  //   float currentrRPM = lkf.updateEstimate(rencoder.getRPM(lmotor.isClockwise));
  //   Serial.printf("%d;%3.3f", pwm, currentrRPM);
  // }
}

/**********************************************************************
* Function definitions
**********************************************************************/

// Encoder ISR functions
void IRAM_ATTR lencoderCounter() {

  portENTER_CRITICAL_ISR(&mux);
  lencoder.pulses++;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR rencoderCounter() {

  portENTER_CRITICAL_ISR(&mux);
  rencoder.pulses++;
  portEXIT_CRITICAL_ISR(&mux);
}

// ROS motor velocity callback function
void velCb(const geometry_msgs::Twist &MSG) {

  vel = Converter::convertMessage(MSG);
  cbTimeout = micros();
}

void pwmCb(const custom_msgs::Motor_power &MSG) {
  
  lpwm = static_cast<float>(MSG.leftPW*-10.23);
  rpwm = static_cast<float>(MSG.rightPW*-10.23);
  Serial.printf("lpwm: %3.3f, rpwm: %3.3f\n", lpwm, rpwm);
  if (rpwm > 1023) {
    rpwm = 1023;
  }
  else if (rpwm < -1023)
  {
    rpwm = -1023;
  }

  if (lpwm > 1023) {
    lpwm = 1023;
  }
  else if (lpwm < -1023)
  {
    lpwm = -1023;
  }

  overrideCb = true;
  cbTimeout = micros();
}

void lCalculate() {
  if ((micros() - lencoder.previousMicros) > CALCULATERPM_INTERVAL) {
    detachInterrupt(encoderAChannel2);
    detachInterrupt(encoderBChannel1);
    lencoder.calculateRPM();
    attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
    attachInterrupt(encoderBChannel1, rencoderCounter, RISING);
  }
}

void rCalculate() {
  if ((micros() - rencoder.previousMicros) > CALCULATERPM_INTERVAL) {
    detachInterrupt(encoderBChannel1);
    detachInterrupt(encoderAChannel2);
    rencoder.calculateRPM();
    attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
    attachInterrupt(encoderBChannel1, rencoderCounter, RISING);
  }
}

// Delay function that doesn't engage sleep mode
void wait(int time) {
  int lasttime = millis();
  while ((millis() - lasttime) <= time) {
    ArduinoOTA.handle();
  }
}