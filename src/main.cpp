/*
 * Tractor
 * 
 * Copyright 2024 Gabriel Víctor <gabriel.muniz@ufv.br>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

#include "Pins.h"

#include <Arduino.h>
#include <Motor.h>// Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

#include <Converter.h>
#include <Controller.h>
#include <Encoder.h>
#include <ArduinoOTA.h>

#include <ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32MultiArray.h>

#include <WiFi.h>
#include <SimpleKalmanFilter.h>

/**********************************************************************
* PID constants
**********************************************************************/
const double KP = 0.792/30;
const double KD = 0.792/2000;
const double KI = 0;

/**********************************************************************
* Global variables and defines
**********************************************************************/

// WiFi
const char* SSID = "NERo-Arena";
const char* PASSWD = "BDPsystem10";

// Encoder pulse count by rotation
const int PULSESPERROTATION = 38;

// Time
const int CALCULATERPM_INTERVAL = 50000;
const int CBTIMEOUT_INTERVAL = 1000000;
unsigned long cbTimeout = 0;
unsigned long currentMicrosA, currentMicrosB = 0;

// Controller
Controller lcontroller(KP, KD, KI);
Controller rcontroller(KP, KD, KI);

// Motor A (LEFT)
Motor lmotor(AIN1, AIN2, PWMA);

// Motor B (RIGHT)
Motor rmotor(BIN1, BIN2, PWMB, 1);

// Encoder A (LEFT)
Encoder lencoder(encoderAChannel2, PULSESPERROTATION);

// Encoder B (RIGHT)
Encoder rencoder(encoderBChannel1, PULSESPERROTATION);

// Kalman filter
SimpleKalmanFilter lkf = SimpleKalmanFilter(5, 5, 0.01);
SimpleKalmanFilter rkf = SimpleKalmanFilter(5, 5, 0.01);

// Critical session
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// OTA instance

// RPM target values
MotorVel vel;

// Function prototypes (declarations)
void wait(int Time);
void IRAM_ATTR lencoderCounter();
void IRAM_ATTR rencoderCounter();
void velCb(const geometry_msgs::Twist &MSG);

// ROS variables
IPAddress server(192,168,0,118); // MASTER IP
const uint16_t serverPort = 11411; // TCP CONNECTION
ros::NodeHandle nh;
geometry_msgs::Twist rosvel;
std_msgs::Float32MultiArray encoderReadings;
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &velCb);
ros::Publisher encoderFb("encoder_fb", &encoderReadings);

void setup() {

  // Initalizing serial
  Serial.begin(115200);

  encoderReadings.layout.dim = (std_msgs::MultiArrayDimension*)malloc(sizeof(std_msgs::MultiArrayDimension));
  encoderReadings.layout.dim[0].label = "encoderFb";
  encoderReadings.layout.dim[0].size = 2;
  encoderReadings.layout.dim[0].stride = 2;
  encoderReadings.layout.data_offset = 0;
  encoderReadings.data_length = 2;
  encoderReadings.data = (float*)malloc(sizeof(float) * 2);

  // Wifi setup
  WiFi.begin(SSID, PASSWD);
  WiFi.mode(WIFI_STA);
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    wait(800);
  }
  Serial.printf("\nConectado ao WI-FI!\n");

  // OTA setup
  otaSetup();

  // Attach encoder ISR's
  attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
  attachInterrupt(encoderBChannel1, rencoderCounter, RISING);

  // ROS setup
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(cmd_vel);
  nh.advertise(encoderFb);
  while (!nh.connected()) {
    nh.spinOnce();
    Serial.print(".");
    wait(800);
  }
  Serial.println("Conectado ao master!");

}

void loop() {

  if ((micros() - cbTimeout) >= CBTIMEOUT_INTERVAL) {
    vel.lrpm = 0;
    vel.rrpm = 0;
  }

  if ((micros() - lencoder.previousMicros) > CALCULATERPM_INTERVAL) {
    detachInterrupt(encoderAChannel2);
    detachInterrupt(encoderBChannel1);
    lencoder.calculateRPM();
    attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
    attachInterrupt(encoderBChannel1, rencoderCounter, RISING);
  }

  if ((micros() - rencoder.previousMicros) > CALCULATERPM_INTERVAL) {
    detachInterrupt(encoderBChannel1);
    detachInterrupt(encoderAChannel2);
    rencoder.calculateRPM();
    attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
    attachInterrupt(encoderBChannel1, rencoderCounter, RISING);
  }

  if (lencoder.isCalculated) {
    float currentlRPM = lkf.updateEstimate(lencoder.getRPM(lmotor.isClockwise));
    float lpwm = lcontroller.controlMotor(vel.lrpm, currentlRPM);
    Serial.printf("%3.3f", vel.lrpm);
    lmotor.setSpeed(lpwm);
    encoderReadings.data[0] = currentlRPM;
    encoderFb.publish(&encoderReadings);
  }
  if (rencoder.isCalculated) {
    float currentrRPM = rkf.updateEstimate(rencoder.getRPM(rmotor.isClockwise));
    float rpwm = rcontroller.controlMotor(vel.rrpm, currentrRPM);
    rmotor.setSpeed(rpwm);
    encoderReadings.data[1] = currentrRPM;
    encoderFb.publish(&encoderReadings);
  }
  Serial.printf("LRPM:%3.3f RRPM:%3.3f\n", vel.lrpm, vel.rrpm);
  ArduinoOTA.handle();
  nh.spinOnce();
  wait(10);
}

/**********************************************************************
* Function definitions
**********************************************************************/

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

void velCb(const geometry_msgs::Twist &MSG) {
  vel = Converter::convertMessage(MSG);
  cbTimeout = micros();
}

void otaSetup() {

  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("Tracker");

  // No authentication by default
  ArduinoOTA.setPassword((const char *)"temqueverissoai");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// Delay function that doesn't engage sleep mode
void wait(int time) {
  int lasttime = millis();
  while ((millis() - lasttime) <= time) {
    ArduinoOTA.handle();
  }
}