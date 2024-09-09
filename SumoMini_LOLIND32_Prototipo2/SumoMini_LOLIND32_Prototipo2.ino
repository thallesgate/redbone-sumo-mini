// ------------------------------------
// GNU GENERAL PUBLIC LICENSE
// Version 3, 29 June 2007
// ------------------------------------
// REDBONE - Protótipo Sumô Mini 2 - V0.1
// ------------------------------------
// Pre-requisitos:
// Core - esp32 Core - Versao 2.0.17
// Biblioteca - IRremote by shirriff, z3t0, ArminJo
// Biblioteca - PID by Brett Beauregard
// Biblioteca - QTRSensors by Pololu
// ------------------------------------

// ##########################################
// #### Definição de Portas e Variaveis! ####
// ##########################################

// #### Motores
#define motStandbyPin 4 // Sinal de Standby dos motores. (desativar) | Na placa: Standby
#define lMotPinA 0 // Sinal de direção do motor esquerdo. | Na placa: AN1
#define lMotPinB 2 // Sinal de direção do motor esquerdo. | Na placa: AN2
#define lMotPinS 15 // Sinal de PWM do motor esquerdo. | Na placa: PWMA
#define rMotPinA 16 // Sinal de direção do motor direito. | Na placa: BN1
#define rMotPinB 17 // Sinal de direção do motor direito. | Na placa: BN2
#define rMotPinS 5 // Sinal de PWM do motor direito. | Na placa: PWMB

// #### Receptor de Controle Remoto
#include <IRremote.hpp>
#define receiverPin 13 // Sinal do receptor de controle remoto. | Na placa: OUT

int botaoPressionado = 0;

// #### Sensores de linha
#define lLinePin 25 // Sinal do sensor de linha esquerdo frontal. |
#define lLineBackPin 14// Sinal do sensor de linha esquerdo traseiro. | NAO CONECTADO
#define cLinePin 26 // Sinal do sensor de linha central traseiro. |
#define rLinePin 27// Sinal do sensor de linha direito frontal. |
#define rLineBackPin 12// Sinal do sensor de linha esquerdo traseiro. | NAO CONECTADO

int lLineVal = 0;
int cLineVal = 0;
int rLineVal = 0;

// #### Sensor de distancia
#define lDistancePin 34 // Sensor de distancia esquerdo. |
#define cDistancePin 32 // Sensor de distancia central. |
#define rDistancePin 33 // Sensor de distancia direito. |

int lDistanceVal = 0;
int cDistanceVal = 0;
int rDistanceVal = 0;

void setup() {
  IrReceiver.begin(receiverPin);
  pinMode(lLinePin, INPUT);
  pinMode(cLinePin, INPUT);
  pinMode(rLinePin, INPUT);
  pinMode(lDistancePin, INPUT);
  pinMode(cDistancePin, INPUT);
  pinMode(rDistancePin, INPUT);

  pinMode(motStandbyPin, OUTPUT);
  pinMode(lMotPinA, OUTPUT);
  pinMode(lMotPinB, OUTPUT);
  pinMode(lMotPinS, OUTPUT);
  pinMode(rMotPinA, OUTPUT);
  pinMode(rMotPinB, OUTPUT);
  pinMode(rMotPinS, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  lerControle();
  //botaoPressionado
  lerSensoresDistancia();
  lerSensoresLinha();

  Serial.print("Distancia L:");
  Serial.print(lDistanceVal);
  Serial.print(",");
  Serial.print("Distancia C:");
  Serial.print(cDistanceVal);
  Serial.print(",");
  Serial.print("Distancia D:");
  Serial.print(rDistanceVal);
  Serial.print(",");
  Serial.print("Linha L:");
  Serial.print(lLineVal);
  Serial.print(",");
  Serial.print("Linha C:");
  Serial.print(cLineVal);
  Serial.print(",");
  Serial.print("Linha D:");
  Serial.println(rLineVal);
  delay(100);
}
void lerSensoresDistancia(){

int maxDistanceVal = 80;
int minSafeDistance = 10;

lDistanceVal = constrain((6762/(analogRead(lDistancePin)-9))-4, 0, maxDistanceVal);
cDistanceVal = constrain((6762/(analogRead(cDistancePin)-9))-4, 0, maxDistanceVal);
rDistanceVal = constrain((6762/(analogRead(rDistancePin)-9))-4, 0, maxDistanceVal);
}
void lerSensoresLinha(){
lLineVal = analogRead(lLinePin);
cLineVal = analogRead(cLinePin);
rLineVal = analogRead(rLinePin);
}
void lerControle(){
  // CONTROLE ROBOCORE
  // 1 ON - 0x45
  // 2 ROBOCORE - 0x47
  // 3 FRENTE - 0x40
  // 4 ESQUERDA - 0x7
  // 5 OK - 0x15
  // 6 DIREITA - 0x9
  // 7 BAIXO - 0x19
  // 8 A - 0xC
  // 9 B - 0x18
  // 10 C - 0x5E
  // 11 D - 0x8
  // 12 E - 0x1C
  // 13 F - 0x5A
  // 14 G - 0x42
  // 15 H - 0x52
  // 16 I - 0x4A


  if(IrReceiver.decode()){
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
    } else {
        IrReceiver.resume(); // Early enable receiving of the next IR frame
    }

    if (IrReceiver.decodedIRData.command == 0x45) {
      botaoPressionado = 1;
    } else if (IrReceiver.decodedIRData.command == 0x47) {
      botaoPressionado = 2;
    } else if (IrReceiver.decodedIRData.command == 0x40) {
      botaoPressionado = 3;
    } else if (IrReceiver.decodedIRData.command == 0x7) {
      botaoPressionado = 4;
    } else if (IrReceiver.decodedIRData.command == 0x15) {
      botaoPressionado = 5;
    } else if (IrReceiver.decodedIRData.command == 0x9) {
      botaoPressionado = 6;
    } else if (IrReceiver.decodedIRData.command == 0x19) {
      botaoPressionado = 7;
    } else if (IrReceiver.decodedIRData.command == 0xC) {
      botaoPressionado = 8;
    } else if (IrReceiver.decodedIRData.command == 0x18) {
      botaoPressionado = 9;
    } else if (IrReceiver.decodedIRData.command == 0x5E) {
      botaoPressionado = 10;
    } else if (IrReceiver.decodedIRData.command == 0x8) {
      botaoPressionado = 11;
    } else if (IrReceiver.decodedIRData.command == 0x1C) {
      botaoPressionado = 12;
    } else if (IrReceiver.decodedIRData.command == 0x5A) {
      botaoPressionado = 13;
    } else if (IrReceiver.decodedIRData.command == 0x42) {
      botaoPressionado = 14;
    } else if (IrReceiver.decodedIRData.command == 0x52) {
      botaoPressionado = 15;
    } else if (IrReceiver.decodedIRData.command == 0x4A) {
      botaoPressionado = 16;
    }
  }

}
void controlarMotores(int lPot, int rPot, bool enable){
  // e_pot = Potencia do motor esquerdo. Valor de -100 a 100
  // d_pot = Potencia do motor direito. Valor de -100 a 100
  // ativar = Ativa ou desativa os motores. Ignora todos os outros valores. Valor 0 ou 1 (False, True)
  
  int maxPower = 200; //Define o valor máximo a ser mandado para os motores.

  // Ativar ou desativar os motores.
  if(enable){ // Se ativar for 1 (True) - Ativa os motores.
    digitalWrite(motStandbyPin, 1);
  }else{ // Se ativar for 0 (True) - Desliga os motores.
    digitalWrite(motStandbyPin, 0);
  }

  if(lPot >= 0){ // Se a potencia do motor esquerdo for acima ou igual a zero
    digitalWrite(lMotPinA, HIGH);
    digitalWrite(lMotPinB, LOW);
  }else{ 
    digitalWrite(lMotPinA, LOW);
    digitalWrite(lMotPinB, HIGH);
  }
  
  // Define a potencia do motor esquerdo.
  analogWrite(lMotPinS, map(constrain(abs(lPot), 0, 100), 0, 100, 0, maxPower));

  if(rPot >= 0){ // Se a potencia do motor direito for acima ou igual a zero
    digitalWrite(rMotPinA, HIGH);
    digitalWrite(rMotPinB, LOW);
  }else{
    digitalWrite(rMotPinA, LOW);
    digitalWrite(rMotPinB, HIGH); 
  }

  // Define a potencia do motor direito.
  analogWrite(rMotPinS, map(constrain(abs(rPot), 0, 100), 0, 100, 0, maxPower));

}