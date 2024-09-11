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

// #### LEDs Endereçáveis
#include <FastLED.h>

// #### Memoria Flash
#include <EEPROM.h> // MEMORIA FLASH PARA SALVAR DADOS DE CALIBRAGEM
#define EEPROM_SIZE 6

// #### Giroscópio


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

#define DECODE_NEC
#define DECODE_SONY

int remoteButton = 0;

// #### Sensores de linha
#define lLinePin 25 // Sinal do sensor de linha esquerdo frontal. |
#define lLineBackPin 14// Sinal do sensor de linha esquerdo traseiro. | NAO CONECTADO
#define cLinePin 27 // Sinal do sensor de linha central traseiro. |
#define rLinePin 26// Sinal do sensor de linha direito frontal. |
#define rLineBackPin 12// Sinal do sensor de linha esquerdo traseiro. | NAO CONECTADO

int lLineRawVal = 0;
int cLineRawVal = 0;
int rLineRawVal = 0;

bool lLineVal = 0;
bool cLineVal = 0;
bool rLineVal = 0;

int lLineWhiteCalibrationVal = 0;
int cLineWhiteCalibrationVal = 0;
int rLineWhiteCalibrationVal = 0;
int lLineBlackCalibrationVal = 0;
int cLineBlackCalibrationVal = 0;
int rLineBlackCalibrationVal = 0;
// #### Sensor de distancia
#define lDistancePin 33 // Sensor de distancia esquerdo. |
#define cDistancePin 32 // Sensor de distancia central. |
#define rDistancePin 34 // Sensor de distancia direito. |

int lDistanceRawVal = 0;
int cDistanceRawVal = 0;
int rDistanceRawVal = 0;

bool lDistanceVal = 0;
bool cDistanceVal = 0;
bool rDistanceVal = 0;

// #### Comportamento do Robô
String botMode = "STOP";
String botStrategy = "MODO_ESTRATEGIA_1"; //MODO_ESTRATEGIA_2 e MODO_ESTRATEGIA_3

void setup() {
  // Inicializa Comunicação SERIAL com o Computador
  Serial.begin(115200);

  // Inicializa Memória EEPROM para guardar variáveis
  EEPROM.begin(EEPROM_SIZE);

  lLineWhiteCalibrationVal = EEPROM.read(0);
  cLineWhiteCalibrationVal = EEPROM.read(1);
  rLineWhiteCalibrationVal = EEPROM.read(2);
  lLineBlackCalibrationVal = EEPROM.read(3);
  cLineBlackCalibrationVal = EEPROM.read(4);
  rLineBlackCalibrationVal = EEPROM.read(5);

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
  lerControle();
  definirModo();
  if(botMode == "MODO_STOP"){
    //Stop
  }else if(botMode == "MODO_PRONTO"){

  }else if(botMode == "MODO_LUTA"){
    
  }else if(botMode == "MODO_CALIBRAGEM_LINHA_FRONTAL"){
    
  }else if(botMode == "MODO_CALIBRAGEM_LINHA_MEIO"){
    
  }else if(botMode == "MODO_CALIBRAGEM_LINHA_TRASEIRO"){
    
  }else if(botMode == "MODO_ESTRATEGIA_1"){
    
  }else if(botMode == "MODO_ESTRATEGIA_2"){
    
  }else if(botMode == "MODO_ESTRATEGIA_3"){
    
  }else if(botMode == "MODO_DEBUG"){
    lerSensoresDistancia();
    lerSensoresLinha();
    debugSerial();
  }
  delay(33);
}

void lerSensoresDistancia(){

  int maxSensorVal = 80;
  int minSafeVal = 1500;

  //lDistanceVal = constrain((6762/(analogRead(lDistancePin)-9))-4, 0, maxDistanceVal);
  //cDistanceVal = constrain((6762/(analogRead(cDistancePin)-9))-4, 0, maxDistanceVal);
  //rDistanceVal = constrain((6762/(analogRead(rDistancePin)-9))-4, 0, maxDistanceVal);

  lDistanceRawVal = analogRead(lDistancePin);
  cDistanceRawVal = analogRead(cDistancePin);
  rDistanceRawVal = analogRead(rDistancePin);


  if(lDistanceRawVal > minSafeVal){
    lDistanceVal = 1;
  }else{
    lDistanceVal = 0;
  }
  if(cDistanceRawVal > minSafeVal){
    cDistanceVal = 1;
  }else{
    cDistanceVal = 0;
  }
  if(rDistanceRawVal > minSafeVal){
    rDistanceVal = 1;
  }else{
    rDistanceVal = 0;
  }
}
void lerSensoresLinha(){
  lLineRawVal = analogRead(lLinePin);
  cLineRawVal = analogRead(cLinePin);
  rLineRawVal = analogRead(rLinePin);

}

void calibrarSensoresLinha(){
  //EEPROM.write(0, ledState);
  //EEPROM.commit();
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

  remoteButton = 0;
  if(IrReceiver.decode()){
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      IrReceiver.resume();
    } else {
      IrReceiver.resume();
    }
    if (IrReceiver.decodedIRData.command == 0x45) {
      remoteButton = 1;
    } else if (IrReceiver.decodedIRData.command == 0x47) {
      remoteButton = 2;
    } else if (IrReceiver.decodedIRData.command == 0x40) {
      remoteButton = 3;
    } else if (IrReceiver.decodedIRData.command == 0x7) {
      remoteButton = 4;
    } else if (IrReceiver.decodedIRData.command == 0x15) {
      remoteButton = 5;
    } else if (IrReceiver.decodedIRData.command == 0x9) {
      remoteButton = 6;
    } else if (IrReceiver.decodedIRData.command == 0x19) {
      remoteButton = 7;
    } else if (IrReceiver.decodedIRData.command == 0xC) {
      remoteButton = 8;
    } else if (IrReceiver.decodedIRData.command == 0x18) {
      remoteButton = 9;
    } else if (IrReceiver.decodedIRData.command == 0x5E) {
      remoteButton = 10;
    } else if (IrReceiver.decodedIRData.command == 0x8) {
      remoteButton = 11;
    } else if (IrReceiver.decodedIRData.command == 0x1C) {
      remoteButton = 12;
    } else if (IrReceiver.decodedIRData.command == 0x5A) {
      remoteButton = 13;
    } else if (IrReceiver.decodedIRData.command == 0x42) {
      remoteButton = 14;
    } else if (IrReceiver.decodedIRData.command == 0x52) {
      remoteButton = 15;
    } else if (IrReceiver.decodedIRData.command == 0x4A) {
      remoteButton = 16;
    }
  }
}
void definirModo(){
  if(remoteButton == 8 && botMode == "MODO_STOP"){
    botMode = "MODO_PRONTO"; // MODO READY
  }else if(remoteButton == 9 && botMode == "MODO_PRONTO"){
    botMode = "MODO_LUTA"; // MODO LUTA
  }else if(remoteButton == 10){
    botMode = "MODO_STOP"; // MODO STOP
  }else if(remoteButton == 14 && botMode == "MODO_STOP"){ //BOTAO G
    botMode = "MODO_CALIBRAGEM_LINHA_FRONTAL"; // MODO CALIBRAGEM SENSORES LINHA FRONTAIS
  }else if(remoteButton == 15 && botMode == "MODO_STOP"){ //BOTAO H
    botMode = "MODO_CALIBRAGEM_LINHA_MEIO"; // MODO CALIBRAGEM SENSORES LINHA MEIO
  }else if(remoteButton == 16 && botMode == "MODO_STOP"){ //BOTAO I
    botMode = "MODO_CALIBRAGEM_LINHA_TRASEIRO"; // MODO CALIBRAGEM SENSORES LINHA TRASEIRO
  }else if(remoteButton == 11 && botMode == "MODO_STOP"){ //BOTAO D
    botMode = "MODO_ESTRATEGIA_1"; // ESTRATEGIA 1
  }else if(remoteButton == 13 && botMode == "MODO_STOP"){ //BOTAO E
    botMode = "MODO_ESTRATEGIA_2"; // ESTRATEGIA 2
  }else if(remoteButton == 14 && botMode == "MODO_STOP"){ //BOTAO F
    botMode = "MODO_ESTRATEGIA_3"; // ESTRATEGIA 3
  }else if(remoteButton == 2 && botMode == "MODO_STOP"){ //BOTAO RAIO
    botMode = "MODO_DEBUG"; // DEBUG MODE
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
void debugSerial(){
  Serial.print("DistanciaL:");
  Serial.print(lDistanceRawVal);
  Serial.print(",");
  Serial.print("DistanciaC:");
  Serial.print(cDistanceRawVal);
  Serial.print(",");
  Serial.print("DistanciaD:");
  Serial.print(rDistanceRawVal);
  Serial.print(",");
  Serial.print("LinhaL:");
  Serial.print(lLineRawVal);
  Serial.print(",");
  Serial.print("LinhaC:");
  Serial.print(cLineRawVal);
  Serial.print(",");
  Serial.print("LinhaD:");
  Serial.print(rLineRawVal);
  Serial.print(",");
  Serial.print("BotaoControle:");
  Serial.print(remoteButton);
  Serial.print(",");
  Serial.print("Modo:");
  Serial.println(botMode);
}