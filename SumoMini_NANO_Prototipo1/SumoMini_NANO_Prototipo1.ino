// ------------------------------------
// GNU GENERAL PUBLIC LICENSE
// Version 3, 29 June 2007
// ------------------------------------
// REDBONE - Protótipo Sumô Mini - V0.1
// ------------------------------------
// Pre-requisitos:
// Biblioteca - AsyncSonar by Luis Llamas -- removido
// Biblioteca - IRremote by shirriff, z3t0, ArminJo
// Biblioteca - NewPing by Tim Eckel
// Biblioteca - PID by Brett Beauregard
// Biblioteca - QTRSensors by Pololu
// ------------------------------------

// ###############################
// ####   Variáveis Globais   ####
// ###############################

int estado_robo = 0; // Por padrão, 0 será STOP. 1 - Pronto e 2 - Ativo
int distancia_maxima = 80; // Distancia maxima retornada pelos sensores ultrasonicos.
int distancia_segura = 30; // Distancia considerada segura.
bool flag_ativar = false;
// ##########################################
// #### Definição das portas utilizadas! ####
// ##########################################

// #### Motores ####
#define pin_mot_standby 9   // Sinal de Standby dos motores. (desativar) | Na placa: Standby
#define pin_dir_mot_esq_a 4 // Sinal de direção do motor esquerdo. | Na placa: AN1
#define pin_dir_mot_esq_b 3 // Sinal de direção do motor esquerdo. | Na placa: AN2
#define pin_pot_mot_esq 5   // Sinal de PWM do motor esquerdo. | Na placa: PWMA
#define pin_dir_mot_dir_a 7 // Sinal de direção do motor direito. | Na placa: BN1
#define pin_dir_mot_dir_b 8 // Sinal de ireção do motor direito. | Na placa: BN2
#define pin_pot_mot_dir 6   // Sinal de PWM do motor direito. | Na placa: PWMB

// #### Receptor de Controle Remoto ####
#define pin_receptor 2 // Sinal do receptor de controle remoto. | Na placa: OUT
#define pin_receptor_led 13 // Sinal do LED para confirmar recebimento de um comando. | Na placa: D13
// #### Sensores de linha ####
//#define pin_linha_esq A1 // Sinal do sensor de linha esquerdo. | Na placa: 6
//#define pin_linha_dir A0 // Sinal do sensor de linha direito. | Na placa: 1

// #### Sensor de distancia ####
#define pin_dist_esq_trig A2 // Sensor de distancia esquerdo. | Na placa: Trig 
#define pin_dist_esq_echo A3 // Sensor de distancia direito. | Na placa: Echo

#define pin_dist_cen_trig 10 // Sensor de distancia central. | Na placa: Trig
#define pin_dist_cen_echo 11 // Sensor de distancia central. | Na placa: Echo

#define pin_dist_dir_trig A4 // Sensor de distancia direito. | Na placa: Trig
#define pin_dist_dir_echo 12 // Sensor de distancia direito. | Na placa: Echo

// ###############################
// #### Definição do receptor ####
// ###############################

#include <IRremote.hpp> // include the library

#define DECODE_NEC // Padrão de comunicação do controle NEC
#define DECODE_SONY // Padrão de comunicação do controle SONY

// Controle ROBOCORE
// A = 0xC
// B = 0x18
// C = 0x5E
// D = 0x8
// E = 0x1C
// F = 0x5A
// G = 0x42
// H = 0x52
// I = 0x4A

// #############################################
// #### Definição dos sensores ultrasonicos ####
// #############################################

#include <NewPing.h>

NewPing sensor_ultrasonico_esq(pin_dist_esq_trig, pin_dist_esq_echo, distancia_maxima);
NewPing sensor_ultrasonico_cen(pin_dist_cen_trig, pin_dist_cen_echo, distancia_maxima);
NewPing sensor_ultrasonico_dir(pin_dist_dir_trig, pin_dist_dir_echo, distancia_maxima);

int val_sensor_distancia_esq = 0;
int val_sensor_distancia_cen = 0;
int val_sensor_distancia_dir = 0;

// ##########################
// #### Definição do PID ####
// ##########################

/*
#include <PID_v1.h>

// Parametros PID
double Kp = 2.0;
double Ki = 5.0;
double Kd = 1.0;

// Variaveis PID
double pid_setpoint = 0.0;
double pid_input = 0.0;
double pid_output = 0.0;

int minimo_pid_output = -100;
int maximo_pid_output = 100;

PID algoritmoPID(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT);
*/
void setup() {
  // #### SETUP - Comunicação Serial com o Computador
  Serial.begin(115200);

  // #### SETUP - Motores
  pinMode(pin_mot_standby, OUTPUT);   // Sinal de Standby dos motores. (desativar) | Na placa: Standby
  pinMode(pin_dir_mot_esq_a, OUTPUT); // Sinal de direção do motor esquerdo. | Na placa: AN1
  pinMode(pin_dir_mot_esq_b, OUTPUT); // Sinal de direção do motor esquerdo. | Na placa: AN2
  pinMode(pin_pot_mot_esq, OUTPUT);   // Sinal de PWM do motor esquerdo. | Na placa: PWMA
  pinMode(pin_dir_mot_dir_a, OUTPUT); // Sinal de direção do motor direito. | Na placa: BN1
  pinMode(pin_dir_mot_dir_b, OUTPUT); // Sinal de ireção do motor direito. | Na placa: BN2
  pinMode(pin_pot_mot_dir, OUTPUT);   // Sinal de PWM do motor direito. | Na placa: PWMB

  // #### SETUP - Receptor
  IrReceiver.begin(pin_receptor, pin_receptor_led);
  
  // #### SETUP - Sensores de linha
 // pinMode(pin_linha_esq, INPUT); // Sinal do sensor de linha esquerdo. | Na placa: 6
 // pinMode(pin_linha_dir, INPUT); // Sinal do sensor de linha direito. | Na placa: 1

  // #### SETUP - PID
  //algoritmoPID.SetMode(AUTOMATIC);
  //algoritmoPID.SetOutputLimits(minimo_pid_output, maximo_pid_output);
}

void loop() {

  // #### LOOP - Receptor ####
  // #### MAQUINA DE ESTADO DO ROBO DE ACORDO COM OS COMANDOS DO CONTROLE ####
  lerControle();
  lerSensoresUltrasonicos();
  //lerSensoresLinha();

  if(estado_robo == 2){
    flag_ativar = true;
    modoTornado();
  }else{
    flag_ativar = false;
  }
  //Serial.println("A: " + String(val_sensor_ultrasonico_esq) + "| B: " + String(val_sensor_ultrasonico_cen) + "| C:" + String(val_sensor_ultrasonico_dir));

}

void lerControle(){
  if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      } else {
          IrReceiver.resume();
      }

      if (IrReceiver.decodedIRData.command == 0xC) { // TECLA A do controle ROBOCORE
          estado_robo = 1; // Define o estado do robô como "PRONTO"
      } else if (IrReceiver.decodedIRData.command == 0x18) { // TECLA B do controle ROBOCORE
          if(estado_robo == 1){ // Caso o estado do robô esteja "PRONTO", definir o estado como "ATIVO"
            estado_robo = 2;
          } // Caso contrário, ignorar o comando de iniciar.
      } else if (IrReceiver.decodedIRData.command == 0x5E) { // TECLA C do controle ROBOCORE
          estado_robo = 0; // Mudar o robô para INATIVO, STOP.
      }
  }
}

void lerSensoresUltrasonicos(){
  int sensor_esq = sensor_ultrasonico_esq.ping_cm();
  int sensor_cen = sensor_ultrasonico_cen.ping_cm();
  int sensor_dir = sensor_ultrasonico_dir.ping_cm();

  if(sensor_esq == 0){
    val_sensor_distancia_esq = distancia_maxima;
  }else{
    val_sensor_distancia_esq = constrain(sensor_esq, 0, distancia_maxima);
  }
  delayMicroseconds(10);
  if(sensor_cen == 0){
    val_sensor_distancia_cen = distancia_maxima;
  }else{
    val_sensor_distancia_cen = constrain(sensor_cen, 0, distancia_maxima);
  }
  delayMicroseconds(10);
  if(sensor_dir == 0){
    val_sensor_distancia_dir = distancia_maxima;
  }else{
    val_sensor_distancia_dir = constrain(sensor_dir, 0, distancia_maxima);
  }
  delayMicroseconds(10);
}

void lerSensoresLinha(){

}

void controlarMotores(bool e_dir, bool d_dir,int pot, bool ativar){
  // e_dir = Direção do motor esquerdo. Valor 0 ou 1 (False, True)
  // d_dir = Direção do motor direito. Valor 0 ou 1 (False, True)
  // pot = Potencia dos motores. Valor de 0 a 100
  // ativar = Ativa ou desativa os motores. Ignora todos os outros valores. Valor 0 ou 1 (False, True)
  
  int potencia_maxima = 255; //Define o valor máximo a ser mandado para os motores.

  // Ativar ou desativar os motores.
  if(ativar){ // Se ativar for 1 (True) - Ativa os motores.
    digitalWrite(pin_mot_standby, 1);
  }else{ // Se ativar for 0 (True) - Desliga os motores.
    digitalWrite(pin_mot_standby, 0);
  }

  if(e_dir){ // Se a direção esquerda for 1 (True)
    digitalWrite(pin_dir_mot_esq_a, HIGH);
    digitalWrite(pin_dir_mot_esq_b, LOW);
  }else{ // Se a direção esquerda for 0 (False)
    digitalWrite(pin_dir_mot_esq_a, LOW);
    digitalWrite(pin_dir_mot_esq_b, HIGH);
  }
  
  // Define a potencia do motor esquerdo.
  analogWrite(pin_pot_mot_esq, map(constrain(abs(pot), 0, 100), 0, 100, 0, potencia_maxima));

  if(d_dir){ // Se a direção direita for 1 (True)
    digitalWrite(pin_dir_mot_dir_a, HIGH);
    digitalWrite(pin_dir_mot_dir_b, LOW);
  }else{ // Se a direção direita for 0 (False)
    digitalWrite(pin_dir_mot_dir_a, LOW);
    digitalWrite(pin_dir_mot_dir_b, HIGH); 
  }

  // Define a potencia do motor direito.
  analogWrite(pin_pot_mot_dir, map(constrain(abs(pot), 0, 100), 0, 100, 0, potencia_maxima));

}

void controlarMotores(int e_pot, int d_pot, bool ativar){
  // e_pot = Potencia do motor esquerdo. Valor de -100 a 100
  // d_pot = Potencia do motor direito. Valor de -100 a 100
  // ativar = Ativa ou desativa os motores. Ignora todos os outros valores. Valor 0 ou 1 (False, True)
  
  int potencia_maxima = 255; //Define o valor máximo a ser mandado para os motores.

  // Ativar ou desativar os motores.
  if(ativar){ // Se ativar for 1 (True) - Ativa os motores.
    digitalWrite(pin_mot_standby, 1);
  }else{ // Se ativar for 0 (True) - Desliga os motores.
    digitalWrite(pin_mot_standby, 0);
  }

  if(e_pot >= 0){ // Se a potencia do motor esquerdo for acima ou igual a zero
    digitalWrite(pin_dir_mot_esq_a, HIGH);
    digitalWrite(pin_dir_mot_esq_b, LOW);
  }else{ 
    digitalWrite(pin_dir_mot_esq_a, LOW);
    digitalWrite(pin_dir_mot_esq_b, HIGH);
  }
  
  // Define a potencia do motor esquerdo.
  analogWrite(pin_pot_mot_esq, map(constrain(abs(e_pot), 0, 100), 0, 100, 0, potencia_maxima));

  if(d_pot >= 0){ // Se a potencia do motor direito for acima ou igual a zero
    digitalWrite(pin_dir_mot_dir_a, HIGH);
    digitalWrite(pin_dir_mot_dir_b, LOW);
  }else{
    digitalWrite(pin_dir_mot_dir_a, LOW);
    digitalWrite(pin_dir_mot_dir_b, HIGH); 
  }

  // Define a potencia do motor direito.
  analogWrite(pin_pot_mot_dir, map(constrain(abs(d_pot), 0, 100), 0, 100, 0, potencia_maxima));

}

void modoTornado(){
  if(val_sensor_distancia_esq > distancia_segura && val_sensor_distancia_cen > distancia_segura && val_sensor_distancia_esq > distancia_segura){

  }else if(val_sensor_distancia_esq < distancia_segura && val_sensor_distancia_cen < distancia_segura && val_sensor_distancia_esq < distancia_segura){
    //controlarMotores(1,1,50,flag_ativar);
  }else if(val_sensor_distancia_esq < distancia_segura && val_sensor_distancia_cen > distancia_segura && val_sensor_distancia_esq > distancia_segura){
    controlarMotores(0,1,100,flag_ativar);
  }else if(val_sensor_distancia_esq > distancia_segura && val_sensor_distancia_cen > distancia_segura && val_sensor_distancia_esq < distancia_segura){
    controlarMotores(1,0,100,flag_ativar);
  }else if(val_sensor_distancia_esq > distancia_segura && val_sensor_distancia_cen < distancia_segura && val_sensor_distancia_esq < distancia_segura){
    controlarMotores(1,0,50,flag_ativar);
  }else if(val_sensor_distancia_esq < distancia_segura && val_sensor_distancia_cen < distancia_segura && val_sensor_distancia_esq > distancia_segura){
    controlarMotores(0,1,50,flag_ativar);
  }else if(val_sensor_distancia_esq > distancia_segura && val_sensor_distancia_cen < distancia_segura && val_sensor_distancia_esq > distancia_segura){
    //controlarMotores(1,1,50,flag_ativar);
  }
}