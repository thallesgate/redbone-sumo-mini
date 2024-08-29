// ------------------------------------
// GNU GENERAL PUBLIC LICENSE
// Version 3, 29 June 2007
// ------------------------------------
// REDBONE - Protótipo Sumô Mini - V0.1
// ------------------------------------
// Pre-requisitos:
// Biblioteca - AsyncSonar by Luis Llamas
// Biblioteca - IRremote by shirriff, z3t0, ArminJo
// Biblioteca - Ultrasonic by Erick Simões
// ------------------------------------

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
#define pin_linha_esq A1 // Sinal do sensor de linha esquerdo. | Na placa: 6
#define pin_linha_dir A0 // Sinal do sensor de linha direito. | Na placa: 1

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

#include <Ultrasonic.h>

Ultrasonic sensor_ultrasonico_esq(pin_dist_esq_trig, pin_dist_esq_echo);
Ultrasonic sensor_ultrasonico_cen(pin_dist_cen_trig, pin_dist_cen_echo);
Ultrasonic sensor_ultrasonico_dir(pin_dist_dir_trig, pin_dist_dir_echo);

int val_sensor_ultrasonico_esq = 0;
int val_sensor_ultrasonico_cen = 0;
int val_sensor_ultrasonico_dir = 0;

// ###############################
// ####   Variáveis Globais   ####
// ###############################

int estado_robo = 0; // Por padrão, 0 será STOP. 1 - Pronto e 2 - Ativo

void setup() {
  // #### SETUP - Comunicação Serial com o Computador
  Serial.begin(115200);

  // #### SETUP - Receptor
  IrReceiver.begin(pin_receptor, pin_receptor_led);
  

}

void loop() {

  // #### LOOP - Receptor ####
  // #### MAQUINA DE ESTADO DO ROBO DE ACORDO COM OS COMANDOS DO CONTROLE ####
  lerControle();
  lerSensoresUltrasonicos();

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
  val_sensor_ultrasonico_esq = sensor_ultrasonico_esq.read();
  val_sensor_ultrasonico_cen = sensor_ultrasonico_cen.read();
  val_sensor_ultrasonico_dir = sensor_ultrasonico_dir.read();
}

void lerSensoresLinha(){

}

void controlarMotores(bool e_dir, bool d_dir,int e_pot, int d_pot, bool ativar){
  // e_dir = Direção do motor esquerdo. Valor 0 ou 1 (False, True)
  // d_dir = Direção do motor direito. Valor 0 ou 1 (False, True)
  // e_pot = Potencia do motor esquerdo. Valor de 0 a 100
  // d_pot = Potencia do motor direito. Valor de 0 a 100
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
  analogWrite(pin_pot_mot_esq, map(constrain(abs(e_pot), 0, 100), 0, 100, 0, potencia_maxima));

  if(d_dir){ // Se a direção direita for 1 (True)
    digitalWrite(pin_dir_mot_dir_a, HIGH);
    digitalWrite(pin_dir_mot_dir_b, LOW);
  }else{ // Se a direção direita for 0 (False)
    digitalWrite(pin_dir_mot_dir_a, LOW);
    digitalWrite(pin_dir_mot_dir_b, HIGH); 
  }

  // Define a potencia do motor direito.
  analogWrite(pin_pot_mot_dir, map(constrain(abs(d_pot), 0, 100), 0, 100, 0, potencia_maxima));

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