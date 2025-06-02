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

#include <Arduino.h>
#include <Motor.h>
#include <Kinematics.h>
#include <Controller.h>
#include <Encoder.h>
#include <ros.h>
#include <geometry_msgs/Pose2D.h>
#include "Pins.h"
#include <WiFi.h>
#include <SimpleKalmanFilter.h>

// Encoder pulse count by rotation
const int PULSERPERROTATION = 8;

// PID Controller constants
const double kp = 0.792/30;
const double kd = 0;
const double ki = 0;

// Global variables

  // WiFi
  const char* ssid = "NERo-Arena";
  const char* pw = "BDPsystem10";
  
  // Time
  const int CALCULATERPM_INTERVAL = 50000;
  const long SAMPLE_TIME_MICROS = 20000;
  unsigned long LAST_SAMPLE_TIME = 0;
  const int CALLBACK_INTERVAL = 1000000;
  unsigned long currentCb_timer = 0;
  unsigned long currentMicrosA, currentMicrosB = 0;

  // Controller
  Controller lcontroller(kp, kd, ki);
  Controller rcontroller(kp, kd, ki);

  // Motor A (LEFT)
  Motor lmotor(AIN1, AIN2, PWMA);

  // Motor B (RIGHT)
  Motor rmotor(BIN1, BIN2, PWMB, 1);

  // Encoder A (LEFT)
  Encoder lencoder(encoderAChannel2, PULSERPERROTATION);
  
  // Encoder B (RIGHT)
  Encoder rencoder(encoderBChannel1, PULSERPERROTATION);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Kalman filter
SimpleKalmanFilter lkf = SimpleKalmanFilter(5, 5, 0.01);
SimpleKalmanFilter rkf = SimpleKalmanFilter(5, 5, 0.01);

// Function prototypes (declarations)
void wait(int Time);
void IRAM_ATTR lencoderCounter();
void IRAM_ATTR rencoderCounter();
void messageCb(const geometry_msgs::Twist &MSG);

// ROS variables
IPAddress server(192,168,0,115); // MASTER IP
const uint16_t serverPort = 11411; // TCP CONNECTION
ros::NodeHandle nh;
geometry_msgs::Twist msg;
geometry_msgs::Pose2D encodermsg;
ros::Publisher pub_encoder("encoder_fb", &encodermsg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &messageCb );

motorRPM rpm;

void setup() {

  // Initalizing serial
  Serial.begin(115200);

  // Wifi setup
  WiFi.begin(ssid, pw);
  while (WiFi.status() != WL_CONNECTED){
    wait(500);
    Serial.print(".");
    wait(800);
  }
  Serial.printf("\nConectado ao WiFi!\n");

  // Attach encoder ISR's
  attachInterrupt(encoderAChannel2, lencoderCounter, RISING);
  attachInterrupt(encoderBChannel1, rencoderCounter, RISING);

  // ROS setup
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(cmd_vel);
  nh.advertise(pub_encoder);
  while (!nh.connected()) {
    nh.spinOnce();
    Serial.print(".");
    wait(800);
  }
  Serial.println();

}

void loop() {

  if (micros() - LAST_SAMPLE_TIME >= SAMPLE_TIME_MICROS) {
    LAST_SAMPLE_TIME = micros();
    
    // Ler os encoders

    // Copias os pulsos para variáveis auxiliares de forma segura
    // e as reseta
    portENTER_CRITICAL(&mux);
    long lpulses = lencoder.pulses;
    lencoder.pulses = 0;
    long rpulses = rencoder.pulses;
    rencoder.pulses = 0;
    portEXIT_CRITICAL(&mux);

    // Calcula o RPM para cada motor com base nos pulsos coletados
    lencoder.calculateRPM(lpulses, SAMPLE_TIME_MICROS);
    rencoder.calculateRPM(rpulses, SAMPLE_TIME_MICROS);

    // Filtra os valores de RPM para suavizar a leitura
    float currentlRPM = lkf.updateEstimate(lencoder.getRPM(lmotor.isClockwise));
    float currentrRPM = rkf.updateEstimate(rencoder.getRPM(rmotor.isClockwise));

    // Lógica de controle PID

    if ((micros() - currentCb_timer) >= CALLBACK_INTERVAL) {
        rpm = {0, 0}; // Define o alvo como 0 se não receber comandos
    }

    // Calcula o PWM necessário para cada motor usando o controlador corrigido
    float lpwm = lcontroller.controlMotor(rpm.lrpm, currentlRPM);
    float rpwm = rcontroller.controlMotor(rpm.rrpm, currentrRPM);

    // **PASSO 3: ATUAR NOS MOTORES**
    lmotor.setSpeed(lpwm);
    rmotor.setSpeed(rpwm);
    
    // (Opcional) Publicar dados de feedback via ROS
    // É melhor publicar aqui para ter uma taxa consistente
    encodermsg.x = currentlRPM;
    encodermsg.y = currentrRPM;
    pub_encoder.publish(&encodermsg);
  }
 
  //Serial.printf("LRPM:%3.3f, RRPM:%3.3f, PWM:%3.3f\n", rpm.lrpm, rpm.rrpm, measpwm);
  nh.spinOnce();
  wait(10);
}

/* Function definitions*/

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

void messageCb(const geometry_msgs::Twist &MSG) {
  rpm = convertMessage(MSG);
  currentCb_timer = micros();
}

// Delay function that doesn't engage sleep mode
void wait(int time) {
  int lasttime = millis();
  while ((millis() - lasttime) <= time) {
  }
}