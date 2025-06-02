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

// --- Variáveis Globais para RPM (atualizadas pelo loop de leitura do encoder) ---
// Estas armazenarão o RPM mais recente calculado.
// O uso de 'volatile' é uma precaução se o compilador otimizar demais o acesso,
// mas com a estrutura de loop único e timers, pode não ser estritamente necessário.
// No entanto, não prejudica.
volatile float current_lRPM_from_encoder = 0.0;
volatile float current_rRPM_from_encoder = 0.0;

// Encoder pulse count by rotation
// const int PULSERPERROTATION = 8;
const int PULSES_PER_REVOLUTION_FULLQUAD = 28;
// PID Controller constants
const double kp = 1.8;
const double kd = kp/100;
const double ki = 0;

// Global variables

// --- Configurações de Tempo ---
// Taxa de atualização para LEITURA DO ENCODER e cálculo de RPM instantâneo
const unsigned long ENCODER_SAMPLE_TIME_MICROS = 500000; //
unsigned long last_encoder_sample_time = 0;
long previous_l_encoder_pulses = 0;
long previous_r_encoder_pulses = 0;

// Taxa de atualização para o LOOP DE CONTROLE PID e ENVIO AOS MOTORES
const unsigned long PID_CONTROL_SAMPLE_TIME_MICROS = 1000000; // 
unsigned long last_pid_control_time = 0;

// --- Setpoint ---
int targetRPM = 1300; // Seu RPM alvo

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
SimpleKalmanFilter lkf = SimpleKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter rkf = SimpleKalmanFilter(2, 2, 0.01);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Function prototypes (declarations)
void wait(int Time);

motorRPM rpm;

void setup()
{

  // Initalizing serial
  Serial.begin(115200);
  delay(1000);

  lmotor.switchInput(); // Inverte a direção do motor A para que fique no sentido horário
  rmotor.switchInput(); // Inverte a direção do motor B para que fique no sentido horário
  // Encoder as pull-up
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // Attach to the pins as full quadrature encoders
  encoderA.attachFullQuad(encoderAChannel1, encoderAChannel2);
  encoderB.attachFullQuad(encoderBChannel1, encoderBChannel2);

  // Resets the initial count of the encoders
  encoderA.clearCount();
  encoderB.clearCount();

  previous_l_encoder_pulses = encoderA.getCount();
  previous_r_encoder_pulses = encoderB.getCount();

  last_encoder_sample_time = micros();
  last_pid_control_time = micros();
}

void loop()
{
  unsigned long currentTime = micros();

  // --- Bloco de Leitura do Encoder e Cálculo de RPM (executa mais frequentemente) ---
  if (currentTime - last_encoder_sample_time >= ENCODER_SAMPLE_TIME_MICROS)
  {
    long current_l_pulses = encoderA.getCount();
    long current_r_pulses = encoderB.getCount();

    long delta_l_pulses = current_l_pulses - previous_l_encoder_pulses;
    long delta_r_pulses = current_r_pulses - previous_r_encoder_pulses;

    previous_l_encoder_pulses = current_l_pulses;
    previous_r_encoder_pulses = current_r_pulses;

    unsigned long actual_encoder_delta_time = currentTime - last_encoder_sample_time;
    last_encoder_sample_time = currentTime; // Atualiza o tempo da última leitura do encoder

    // Calcula o RPM bruto com base no tempo real decorrido para esta amostra do encoder
    float raw_lRPM = convertPulsesToRPM(delta_l_pulses, PULSES_PER_REVOLUTION_FULLQUAD, actual_encoder_delta_time);
    float raw_rRPM = convertPulsesToRPM(delta_r_pulses, PULSES_PER_REVOLUTION_FULLQUAD, actual_encoder_delta_time);

    // Atualiza as variáveis globais de RPM (protegendo o acesso se fosse multithread,
    // mas em loop único com timers, o risco de corrupção é baixo se bem gerenciado)
    // Para garantir atomicidade em arquiteturas de 8 bits ao ler/escrever floats,
    // ou se 'current_lRPM_from_encoder' fosse maior que o tamanho da palavra do processador,
    // noInterrupts()/interrupts() seria mais crítico. No ESP32 (32-bit),
    // a escrita de um float é geralmente atômica.
    noInterrupts(); // Garante que a escrita seja atômica e não interrompida
    current_lRPM_from_encoder = raw_lRPM;
    current_rRPM_from_encoder = raw_rRPM;
    interrupts();
  }

  // --- Bloco de Controle PID e Atuação nos Motores (executa menos frequentemente) ---
  if (currentTime - last_pid_control_time >= PID_CONTROL_SAMPLE_TIME_MICROS)
  {
    last_pid_control_time = currentTime; // Atualiza o tempo do último controle

    // Lê as últimas estimativas de RPM (que foram atualizadas pelo bloco do encoder)
    float lRPM_for_PID, rRPM_for_PID;

    noInterrupts(); // Garante que a leitura seja atômica
    lRPM_for_PID = current_lRPM_from_encoder;
    rRPM_for_PID = current_rRPM_from_encoder;
    interrupts();

    // Filtra os valores de RPM para suavizar a leitura ANTES de usar no PID
    float filtered_lRPM = lkf.updateEstimate(lRPM_for_PID);
    float filtered_rRPM = rkf.updateEstimate(rRPM_for_PID);

    // Lógica de controle PID
    float lpwm = lcontroller.controlMotor(targetRPM, filtered_lRPM);
    float rpwm = rcontroller.controlMotor(targetRPM, filtered_rRPM);

    Serial.printf(">SetpointL:%d\n", targetRPM); // Setpoint para o motor esquerdo
    Serial.printf(">ActualL_RPM:%.2f\n", filtered_lRPM);
    Serial.printf(">L_PWM:%.0f\n", lpwm);

    Serial.printf(">SetpointR:%d\n", targetRPM); // Setpoint para o motor direito
    Serial.printf(">ActualR_RPM:%.2f\n", filtered_rRPM);
    Serial.printf(">R_PWM:%.0f\n", rpwm);

    // ATUAR NOS MOTORES
    lmotor.setSpeed(lpwm);
    rmotor.setSpeed(rpwm);

    // Serial.printf("Target: %d, L_Enc: %.2f, R_Enc: %.2f, L_Filt: %.2f, R_Filt: %.2f, L_PWM: %.0f, R_PWM: %.0f\n",
    //               targetRPM, lRPM_for_PID, rRPM_for_PID, filtered_lRPM, filtered_rRPM, lpwm, rpwm);
  }
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