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
const uint8_t PULSESPERROTATION = 183; // Encoder pulses in a single spin

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
Encoder lencoder(encoderAChannel2, PULSESPERROTATION);

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
IPAddress server(192,168,0,111); // MASTER IP
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
void OTA_Setup();
void WIFI_Setup(const IPAddress IPV4, const IPAddress GATEWAY, const IPAddress SUBNET);

void setup() {

  // Initalizing serial
  Serial.begin(115200);
  wait(2000);
  Serial.printf("\n\n");
  Serial.print("<ASTROS>  Copyright (C) <2024>  <Gabriel Víctor");
  Serial.println(" and Hiago Batista>");
  Serial.print("This program comes with ABSOLUTELY NO WARRANTY;");
  Serial.print(" for details access: https://www.gnu.org/licenses/gpl-3.0.html.");
  Serial.print(" This is free software, and you are welcome to redistribute it");
  Serial.println(" under certain conditions; access https://www.gnu.org/licenses/gpl-3.0.html for details.");
  wait(1000);

  // WiFi Setup

  // Network variables
  IPAddress my_IP(192, 168, 0, 218);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  WIFI_Setup(my_IP, gateway, subnet);

  // OTA setup
  OTA_Setup();

  // Attach encoder ISR's
  attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
  attachInterrupt(encoderBChannel1, rencoderCounter, RISING);
  
 // ROS setup
  ROS_Setup();

}

void loop() {

  if ((micros() - cbTimeout) >= CBTIMEOUT_INTERVAL) {

    vel.lrpm = 0;
    vel.rrpm = 0;
    lpwm = 0;
    rpwm = 0;
    overrideCb = false;
  }

  if (!overrideCb) {

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
      Serial.println(lencoder.pulses);
      float currentlRPM = lkf.updateEstimate(lencoder.getRPM(lmotor.isClockwise));
      lpwm = lcontroller.controlMotor(vel.lrpm, currentlRPM);
      lmotor.setSpeed(lpwm);
      encoderReadings.leftRPM = currentlRPM;
      encoderFb.publish(&encoderReadings);
    }

    if (rencoder.isCalculated) {
      Serial.println(rencoder.pulses);
      float currentrRPM = rkf.updateEstimate(rencoder.getRPM(rmotor.isClockwise));
      rpwm = rcontroller.controlMotor(vel.rrpm, currentrRPM);
      rmotor.setSpeed(rpwm);
      encoderReadings.rightRPM = currentrRPM;
      encoderFb.publish(&encoderReadings);
    }
  } 
  else {
    lmotor.setSpeed(lpwm);
    rmotor.setSpeed(rpwm);
    float currentlRPM = lkf.updateEstimate(lencoder.getRPM(lmotor.isClockwise));
    float currentrRPM = rkf.updateEstimate(rencoder.getRPM(rmotor.isClockwise));
    encoderReadings.leftRPM = currentlRPM;
    encoderFb.publish(&encoderReadings);
    encoderReadings.rightRPM = currentrRPM;
    encoderFb.publish(&encoderReadings);
  }

  Serial.printf("Left: %d; Right: %d\n", lencoder.pulses, rencoder.pulses);

  ArduinoOTA.handle();
  nh.spinOnce();
  wait(10);
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

// OTA setup function
void OTA_Setup() {

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
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// ROS setup function
void ROS_Setup() {

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(cmd_vel);
  nh.subscribe(motor_pw);
  nh.advertise(encoderFb);
  while (!nh.connected()) {
    ArduinoOTA.handle();
    nh.spinOnce();
    wait(800);
  }
  Serial.println("");
  Serial.println(" ___                              _                                    ");
  Serial.println("|_ _|   ___ ___  _ __ ___   ___  | |_ ___    ___  ___ _ ____   _____   ");
  Serial.println(" | |   / __/ _ \\| '_ ` _ \\ / _ \\ | __/ _ \\  / __|/ _ \\ '__\\ \\ / / _ \\  ");
  Serial.println(" | |  | (_| (_) | | | | | |  __/ | || (_) | \\__ \\  __/ |   \\ V /  __/_ ");
  Serial.println("|___|  \\___\\___/|_| |_| |_|\\___|  \\__\\___/  |___/\\___|_|    \\_/ \\___( )");
  Serial.println("                                                                    |/ ");
  Serial.println("             _     _          _                                        _ ");
  Serial.println(" _ __   ___ | |_  | |_ ___   | |__   ___   ___  ___ _ ____   _____  __| |");
  Serial.println("| '_ \\ / _ \\| __| | __/ _ \\  | '_ \\ / _ \\ / __|/ _ \\ '__\\ \\ / / _ \\/ _` |");
  Serial.println("| | | | (_) | |_  | || (_) | | |_) |  __/ \\__ \\  __/ |   \\ V /  __/ (_| | _");
  Serial.println("|_| |_|\\___/ \\__|  \\__\\___/  |_.__/ \\___| |___/\\___|_|    \\_/ \\___|\\__,_|(_)");
  Serial.println();
}

void WIFI_Setup(const IPAddress IPV4, const IPAddress GATEWAY, const IPAddress SUBNET) {
   if (!WiFi.config(IPV4, GATEWAY, SUBNET)) {
    Serial.println("The static IP setup failed.");
  }
  WiFi.begin(SSID, PASSWD);
  WiFi.mode(WIFI_STA);
  Serial.println("Connecting to WIFI...");
  while (WiFi.status() != WL_CONNECTED) {
    ArduinoOTA.handle();
    Serial.print(".");
    wait(800);
  }
  Serial.printf("\nConnected to WIFI.\n");
}

// Delay function that doesn't engage sleep mode
void wait(int time) {
  int lasttime = millis();
  while ((millis() - lasttime) <= time) {
    ArduinoOTA.handle();
  }
}