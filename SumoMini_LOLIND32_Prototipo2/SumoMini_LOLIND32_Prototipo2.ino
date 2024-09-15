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
#define NUM_LEDS 8
#define LED_PIN 19
#define LED_BRIGHTNESS 25 // Brilho dos LEDs - Até 255
CRGB leds[NUM_LEDS];
CRGB previousColor = CRGB::Black;
// #### Memoria Flash
#include <EEPROM.h> // MEMORIA FLASH PARA SALVAR DADOS DE CALIBRAGEM
#define EEPROM_SIZE 13
// #### Giroscópio
#include <MPU6050.h>


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

int lLineInnerCalibrationMinVal = 0;
int lLineInnerCalibrationMaxVal = 0;
int cLineInnerCalibrationMinVal = 0;
int cLineInnerCalibrationMaxVal = 0;
int rLineInnerCalibrationMinVal = 0;
int rLineInnerCalibrationMaxVal = 0;

int lLineOuterCalibrationMinVal = 0;
int lLineOuterCalibrationMaxVal = 0;
int cLineOuterCalibrationMinVal = 0;
int cLineOuterCalibrationMaxVal = 0;
int rLineOuterCalibrationMinVal = 0;
int rLineOuterCalibrationMaxVal = 0;

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

// #### Geral


// #### Comportamento do Robô
String botMode = "MODO_STOP";
bool motorEnable = 0;
int botStrategyNum = 1;

void setup() {
   // Inicializa os LEDs
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  flashLeds(CRGB::White, 100, 0);

  // Inicializa Comunicação SERIAL com o Computador
  delay(1000);
  Serial.begin(115200);
  flashLeds(CRGB::Cyan, 100, 0);

  // Inicializa Memória EEPROM para guardar variáveis
  EEPROM.begin(EEPROM_SIZE);
  botStrategyNum = EEPROM.read(0);

  lLineInnerCalibrationMinVal = EEPROM.read(1);
  lLineInnerCalibrationMaxVal = EEPROM.read(2);
  cLineInnerCalibrationMinVal = EEPROM.read(3);
  cLineInnerCalibrationMaxVal = EEPROM.read(4);
  rLineInnerCalibrationMinVal = EEPROM.read(5);
  rLineInnerCalibrationMaxVal = EEPROM.read(6);

  lLineOuterCalibrationMinVal = EEPROM.read(7);
  lLineOuterCalibrationMaxVal = EEPROM.read(8);
  cLineOuterCalibrationMinVal = EEPROM.read(9);
  cLineOuterCalibrationMaxVal = EEPROM.read(10);
  rLineOuterCalibrationMinVal = EEPROM.read(11);
  rLineOuterCalibrationMaxVal = EEPROM.read(12);

  flashLeds(CRGB::Cyan, 100, 0);

  // Inicializa o controle remoto
  IrReceiver.begin(receiverPin);
  pinMode(lLinePin, INPUT);
  pinMode(cLinePin, INPUT);
  pinMode(rLinePin, INPUT);
  pinMode(lDistancePin, INPUT);
  pinMode(cDistancePin, INPUT);
  pinMode(rDistancePin, INPUT);
  flashLeds(CRGB::Cyan, 100, 0);

  // Inicializa os motores
  pinMode(motStandbyPin, OUTPUT);
  pinMode(lMotPinA, OUTPUT);
  pinMode(lMotPinB, OUTPUT);
  pinMode(lMotPinS, OUTPUT);
  pinMode(rMotPinA, OUTPUT);
  pinMode(rMotPinB, OUTPUT);
  pinMode(rMotPinS, OUTPUT);
  flashLeds(CRGB::Cyan, 100, 0);

  // Setup Pronto!
  delay(200);
  flashLeds(CRGB::Green, 200, 0);
  delay(200);
  flashLeds(CRGB::Green, 200, 0);
  delay(200);
  flashLeds(CRGB::Green, 200, 0);
  delay(200);
  flashLeds(CRGB::Green, 200, 0);
  delay(200);
}

void loop() {
  lerControle();
  definirModo();
  if(botMode == "MODO_STOP"){
    fillLeds(CRGB::Red); 
    motorEnable = false;
    controlarMotores(0,0,motorEnable);
    
  }else if(botMode == "MODO_PRONTO"){
    fillLeds(CRGB::Orange);
    motorEnable = false;
    controlarMotores(0,0,motorEnable);

  }else if(botMode == "MODO_LUTA"){
    motorEnable = true;
    lerSensoresLinha();
    lerSensoresDistancia();
    //fillLeds(CRGB::Green);
    if(botStrategyNum == 1){
      flashLeds(CRGB::Cyan, 5, 0);
      //delay(1000);
      if(lLineVal == 0 and rLineVal == 0){
        controlarMotores(70,70,motorEnable);
      }else if (lLineVal == 1 and rLineVal == 1){
        controlarMotores(-50, -50, motorEnable);
        delay(300);
      }else if (lLineVal == 1){
        controlarMotores(230,40,motorEnable);
      }else if (rLineVal == 1){
        controlarMotores(40,230,motorEnable);
      }
      

    }else if(botStrategyNum == 2){
      flashLeds(CRGB::Purple, 5, 0);
      if(lDistanceVal == 1 && cDistanceVal == 1 && rDistanceVal == 1){
        controlarMotores(140,140,motorEnable);
      }else if (lDistanceVal == 1 && cDistanceVal == 1 && rDistanceVal == 0){
        controlarMotores(-70,70,motorEnable);
      }else if (lDistanceVal == 0 && cDistanceVal == 1 && rDistanceVal == 1){
        controlarMotores(70,-70,motorEnable);
      }else if (lDistanceVal == 0 && cDistanceVal == 0 && rDistanceVal == 1){
        controlarMotores(140,-140,motorEnable);
      }else if (lDistanceVal == 1 && cDistanceVal == 0 && rDistanceVal == 0){
        controlarMotores(-140,140,motorEnable);
      }else if (lDistanceVal == 0 && cDistanceVal == 0 && rDistanceVal == 0){
        controlarMotores(0,0,motorEnable);
      }else if (lDistanceVal == 0 && cDistanceVal == 1 && rDistanceVal == 0){
        controlarMotores(70,70,motorEnable);
      }

    }else if(botStrategyNum == 3){
      flashLeds(CRGB::Cyan, 100, 0);
      flashLeds(CRGB::Cyan, 100, 0);
      flashLeds(CRGB::Cyan, 100, 0);
      delay(1000);
    }

  }else if(botMode == "MODO_CALIBRAGEM_LINHA"){
    fillLeds(CRGB::Purple);
    flashLeds(CRGB::Purple, 1000, 0);
    calibrarSensoresLinha();
    botMode = "MODO_STOP";

  }else if(botMode == "MODO_DEBUG_SENSORES"){
    lerSensoresDistancia();
    lerSensoresLinha();
    //flashLeds(CRGB::Cyan, 100, 0);
    if(lLineVal){
      leds[0] = CRGB::White;
    }else{
      leds[0] = CRGB::Black;
    }
    if(cLineVal){
      leds[1] = CRGB::White;
      leds[2] = CRGB::White;
    }else{
      leds[1] = CRGB::Black;
      leds[2] = CRGB::Black;
    }
    if(rLineVal){
      leds[3] = CRGB::White;
    }else{
      leds[3] = CRGB::Black;
    }

    if(lDistanceVal){
      leds[4] = CRGB::Red;
    }else{
      leds[4] = CRGB::Black;
    }
    if(cDistanceVal){
      leds[5] = CRGB::Red;
      leds[6] = CRGB::Red;
    }else{
      leds[5] = CRGB::Black;
      leds[6] = CRGB::Black;
    }
    if(rDistanceVal){
      leds[7] = CRGB::Red;
    }else{
      leds[7] = CRGB::Black;
    }

    

    FastLED.show();
    delay(33);
    //flashLeds(CRGB::Cyan, 100, 0);
    //delay(1000);
  }else if(botMode == "MODO_DEBUG_MOTORES"){
    motorEnable = true;
    flashLeds(CRGB::Purple, 100, 0);

    leds[0] = CRGB::Red;
    controlarMotores(0,0,motorEnable);
    FastLED.show();
    delay(1000);
    leds[0] = CRGB::Green;
    FastLED.show();
    controlarMotores(0,0,motorEnable);
    
    leds[1] = CRGB::Red;
    controlarMotores(50,50,motorEnable);
    FastLED.show();
    delay(1000);
    leds[1] = CRGB::Green;
    FastLED.show();
    controlarMotores(0,0,motorEnable);
    
    leds[2] = CRGB::Red;
    controlarMotores(-50,-50,motorEnable);
    FastLED.show();
    delay(1000);
    leds[2] = CRGB::Green;
    FastLED.show();
    controlarMotores(0,0,motorEnable);
    
    leds[3] = CRGB::Red;
    controlarMotores(100,0,motorEnable);
    FastLED.show();
    delay(1000);
    leds[3] = CRGB::Green;
    FastLED.show();
    controlarMotores(0,0,motorEnable);
    
    leds[4] = CRGB::Red;
    controlarMotores(0,100,motorEnable);
    FastLED.show();
    delay(1000);
    leds[4] = CRGB::Green;
    FastLED.show();
    controlarMotores(0,0,motorEnable);
    
    leds[5] = CRGB::Red;
    controlarMotores(-100,100,motorEnable);
    FastLED.show();
    delay(1000);
    leds[5] = CRGB::Green;
    FastLED.show();
    controlarMotores(0,0,motorEnable);
    
    leds[6] = CRGB::Red;
    controlarMotores(100,-100,motorEnable);
    FastLED.show();
    delay(1000);
    leds[6] = CRGB::Green;
    FastLED.show();
    controlarMotores(0,0,motorEnable);
    
    leds[7] = CRGB::Red;
    controlarMotores(230,230,motorEnable);
    FastLED.show();
    delay(300);
    leds[7] = CRGB::Green;
    FastLED.show();
    controlarMotores(0,0,motorEnable);

    flashLeds(CRGB::Green, 500, 0);
    flashLeds(CRGB::Green, 500, 0);
    flashLeds(CRGB::Green, 500, 0);

    botMode = "MODO_STOP";

  }else if(botMode == "MODO_ESTRATEGIA_1"){
    fillLeds(CRGB::Cyan);
    flashLeds(CRGB::Cyan, 350, 0);
    botStrategyNum = 1;
    EEPROM.write(0, botStrategyNum);
    EEPROM.commit();
    delay(1000);
    botMode = "MODO_STOP";

  }else if(botMode == "MODO_ESTRATEGIA_2"){
    fillLeds(CRGB::Cyan);
    flashLeds(CRGB::Cyan, 350, 0);
    flashLeds(CRGB::Cyan, 350, 0);
    botStrategyNum = 2;
    EEPROM.write(0, botStrategyNum);
    EEPROM.commit();
    delay(1000);
    botMode = "MODO_STOP";

  }else if(botMode == "MODO_ESTRATEGIA_3"){
    fillLeds(CRGB::Cyan);
    flashLeds(CRGB::Cyan, 350, 0);
    flashLeds(CRGB::Cyan, 350, 0);
    flashLeds(CRGB::Cyan, 350, 0);
    botStrategyNum = 3;
    EEPROM.write(0, botStrategyNum);
    EEPROM.commit();
    delay(1000);
    botMode = "MODO_STOP";

  }else if(botMode == "MODO_DEBUG_SERIAL"){
    fillLeds(CRGB::Cyan);
    flashLeds(CRGB::White, 50, 0);
    lerSensoresDistancia();
    lerSensoresLinha();
    debugSerial();
  }
  //delay(33);
}

void lerSensoresDistancia(){
  int analogFactor = 16; // Para transformar a resolução 12 bits para 8 bits
  int minSafeVal = 93;

  //lDistanceVal = constrain((6762/(analogRead(lDistancePin)-9))-4, 0, maxDistanceVal);
  //cDistanceVal = constrain((6762/(analogRead(cDistancePin)-9))-4, 0, maxDistanceVal);
  //rDistanceVal = constrain((6762/(analogRead(rDistancePin)-9))-4, 0, maxDistanceVal);

  lDistanceRawVal = analogRead(lDistancePin) / analogFactor;
  cDistanceRawVal = analogRead(cDistancePin) / analogFactor;
  rDistanceRawVal = analogRead(rDistancePin) / analogFactor;


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
  int analogFactor = 16; // Para transformar a resolução 12 bits para 8 bits
  int analogOffset = 5; // Definir uma folga pros valores do sensor
  
  bool interiorInverted = false; // Caso o interior seja branco ao inves de preto (invertido)

  lLineRawVal = analogRead(lLinePin) / analogFactor;
  cLineRawVal = analogRead(cLinePin) / analogFactor;
  rLineRawVal = analogRead(rLinePin) / analogFactor;

  if (lLineInnerCalibrationMinVal < lLineOuterCalibrationMinVal){
    interiorInverted = true;
  }

  if(interiorInverted){
    if(lLineRawVal < (lLineOuterCalibrationMinVal - analogOffset)){
      lLineVal = false;
    }else{
      lLineVal = true;
    }
    if(cLineRawVal < (cLineOuterCalibrationMinVal - analogOffset)){
      cLineVal = false;
    }else{
      cLineVal = true;
    }
    if(rLineRawVal < (rLineOuterCalibrationMinVal - analogOffset)){
      rLineVal = false;
    }else{
      rLineVal = true;
    }
  }else{
    if(lLineRawVal > (lLineOuterCalibrationMaxVal + analogOffset)){
      lLineVal = false;
    }else{
      lLineVal = true;
    }
    if(cLineRawVal > (cLineOuterCalibrationMaxVal + analogOffset)){
      cLineVal = false;
    }else{
      cLineVal = true;
    }
    if(rLineRawVal > (rLineOuterCalibrationMaxVal + analogOffset)){
      rLineVal = false;
    }else{
      rLineVal = true;
    }
  }
}

void calibrarSensoresLinha(){
  // #### Calibrar parte interna da arena.
  // Variáveis para calibrar.

  int readingsCount = 20;
  
  int lLineValues[readingsCount];
  int cLineValues[readingsCount];
  int rLineValues[readingsCount];

  lLineInnerCalibrationMinVal = 255;
  lLineInnerCalibrationMaxVal = 0;
  cLineInnerCalibrationMinVal = 255;
  cLineInnerCalibrationMaxVal = 0;
  rLineInnerCalibrationMinVal = 255;
  rLineInnerCalibrationMaxVal = 0;

  flashLeds(CRGB::Purple, 100, 0);
  flashLeds(CRGB::Purple, 100, 0);
  flashLeds(CRGB::Purple, 100, 0);

  delay(5000);

  flashLeds(CRGB::Red, 100, 0);

  // Leitura dos sensores de linha.
  for(int i = 0; i < readingsCount; i++){
    lerSensoresLinha();

    lLineValues[i] = lLineRawVal;
    cLineValues[i] = cLineRawVal;
    rLineValues[i] = rLineRawVal;

    delay(33);
  }
  flashLeds(CRGB::Green, 1000, 0);

  // Definição de mínimo e máximo para calcular offset.
  for (int i = 0; i < readingsCount; i++) {
      if (lLineValues[i] >= lLineInnerCalibrationMaxVal) {
         lLineInnerCalibrationMaxVal = lLineValues[i];
      }
      if (lLineValues[i] <= lLineInnerCalibrationMinVal) {
         lLineInnerCalibrationMinVal = lLineValues[i];
      }
      if (cLineValues[i] >= cLineInnerCalibrationMaxVal) {
         cLineInnerCalibrationMaxVal = cLineValues[i];
      }
      if (cLineValues[i] <= cLineInnerCalibrationMinVal) {
         cLineInnerCalibrationMinVal = cLineValues[i];
      }
      if (rLineValues[i] >= rLineInnerCalibrationMaxVal) {
         rLineInnerCalibrationMaxVal = rLineValues[i];
      }
      if (rLineValues[i] <= rLineInnerCalibrationMinVal) {
         rLineInnerCalibrationMinVal = rLineValues[i];
      }
   }

  // Guardar em memória os valores.
  EEPROM.write(1, constrain(lLineInnerCalibrationMinVal, 0, 255));
  EEPROM.write(2, constrain(lLineInnerCalibrationMaxVal, 0, 255));
  EEPROM.write(3, constrain(cLineInnerCalibrationMinVal, 0, 255));
  EEPROM.write(4, constrain(cLineInnerCalibrationMaxVal, 0, 255));
  EEPROM.write(5, constrain(rLineInnerCalibrationMinVal, 0, 255));
  EEPROM.write(6, constrain(rLineInnerCalibrationMaxVal, 0, 255));

  EEPROM.commit();

  lLineOuterCalibrationMinVal = 255;
  lLineOuterCalibrationMaxVal = 0;
  cLineOuterCalibrationMinVal = 255;
  cLineOuterCalibrationMaxVal = 0;
  rLineOuterCalibrationMinVal = 255;
  rLineOuterCalibrationMaxVal = 0;

  // #### Calibrar faixa externa da arena!
  // Aguardar o reposicionamento do robô para os sensores frontais estarem sobre a faixa.
  flashLeds(CRGB::Purple, 100, 0);
  flashLeds(CRGB::Purple, 100, 0);
  flashLeds(CRGB::Purple, 100, 0);

  delay(5000);

  flashLeds(CRGB::Red, 100, 0);
  // Leitura dos sensores de linha.
  // Ler sensores da frente primeiro
  for(int i = 0; i < readingsCount; i++){
    lerSensoresLinha();

    lLineValues[i] = lLineRawVal;
    rLineValues[i] = rLineRawVal;

    delay(33);
  }
  flashLeds(CRGB::Green, 1000, 0);

  delay(1000);

  flashLeds(CRGB::Purple, 100, 0);
  flashLeds(CRGB::Purple, 100, 0);
  flashLeds(CRGB::Purple, 100, 0);

  delay(5000);

  flashLeds(CRGB::Red, 100, 0);
  // Aguardar para reposicionar o sensor de linha traseiro sobre a faixa.
  for(int i = 0; i < readingsCount; i++){
    lerSensoresLinha();
    cLineValues[i] = cLineRawVal;

    delay(33);
  }

  flashLeds(CRGB::Green, 1000, 0);

  // Definição de mínimo e máximo para calcular offset.
  for (int i = 0; i < readingsCount; i++) {
      if (lLineValues[i] >= lLineOuterCalibrationMaxVal) {
         lLineOuterCalibrationMaxVal = lLineValues[i];
      }
      if (lLineValues[i] <= lLineOuterCalibrationMinVal) {
         lLineOuterCalibrationMinVal = lLineValues[i];
      }
      if (cLineValues[i] >= cLineOuterCalibrationMaxVal) {
         cLineOuterCalibrationMaxVal = cLineValues[i];
      }
      if (cLineValues[i] <= cLineOuterCalibrationMinVal) {
         cLineOuterCalibrationMinVal = cLineValues[i];
      }
      if (rLineValues[i] >= rLineOuterCalibrationMaxVal) {
         rLineOuterCalibrationMaxVal = rLineValues[i];
      }
      if (rLineValues[i] <= rLineOuterCalibrationMinVal) {
         rLineOuterCalibrationMinVal = rLineValues[i];
      }
   }

  // Guardar em memória os valores.
  EEPROM.write(7, constrain(lLineOuterCalibrationMinVal, 0, 255));
  EEPROM.write(8, constrain(lLineOuterCalibrationMaxVal, 0, 255));
  EEPROM.write(9, constrain(cLineOuterCalibrationMinVal, 0, 255));
  EEPROM.write(10, constrain(cLineOuterCalibrationMaxVal, 0, 255));
  EEPROM.write(11, constrain(rLineOuterCalibrationMinVal, 0, 255));
  EEPROM.write(12, constrain(rLineOuterCalibrationMaxVal, 0, 255));

  EEPROM.commit();
  flashLeds(CRGB::Green, 500, 0);
  flashLeds(CRGB::Green, 500, 0);
  flashLeds(CRGB::Green, 500, 0);
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
      if (IrReceiver.decodedIRData.command == 0x45) {
        remoteButton = 1;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x47) {
        remoteButton = 2;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x40) {
        remoteButton = 3;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x7) {
        remoteButton = 4;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x15) {
        remoteButton = 5;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x9) {
        remoteButton = 6;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x19) {
        remoteButton = 7;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0xC) {
        remoteButton = 8;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x18) {
        remoteButton = 9;
      } else if (IrReceiver.decodedIRData.command == 0x5E) {
        remoteButton = 10;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x8) {
        remoteButton = 11;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x1C) {
        remoteButton = 12;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x5A) {
        remoteButton = 13;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x42) {
        remoteButton = 14;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x52) {
        remoteButton = 15;
        flashLeds(CRGB::White, 33, 1);
      } else if (IrReceiver.decodedIRData.command == 0x4A) {
        remoteButton = 16;
        flashLeds(CRGB::White, 33, 1);
      }
      IrReceiver.resume();
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
    botMode = "MODO_CALIBRAGEM_LINHA"; // MODO CALIBRAGEM SENSORES LINHA
  }else if(remoteButton == 11 && botMode == "MODO_STOP"){ //BOTAO D
    botMode = "MODO_ESTRATEGIA_1"; // ESTRATEGIA 1
  }else if(remoteButton == 12 && botMode == "MODO_STOP"){ //BOTAO E
    botMode = "MODO_ESTRATEGIA_2"; // ESTRATEGIA 2
  }else if(remoteButton == 13 && botMode == "MODO_STOP"){ //BOTAO F
    botMode = "MODO_ESTRATEGIA_3"; // ESTRATEGIA 3
  }else if(remoteButton == 2 && botMode == "MODO_STOP"){ //BOTAO RAIO
    botMode = "MODO_DEBUG_SERIAL"; // DEBUG MODE
  }else if(remoteButton == 15 && botMode == "MODO_STOP"){
    botMode = "MODO_DEBUG_SENSORES";
  }else if(remoteButton == 16 && botMode == "MODO_STOP"){
    botMode = "MODO_DEBUG_MOTORES";
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
  Serial.print("DL:");
  Serial.print(lDistanceRawVal);
  Serial.print(",");
  Serial.print("DC:");
  Serial.print(cDistanceRawVal);
  Serial.print(",");
  Serial.print("DD:");
  Serial.print(rDistanceRawVal);
  Serial.print(",");
  Serial.print("LL:");
  Serial.print(lLineRawVal);
  Serial.print(",");
  Serial.print("LC:");
  Serial.print(cLineRawVal);
  Serial.print(",");
  Serial.print("LD:");
  Serial.print(rLineRawVal);
  Serial.print(",");
  Serial.print("BControle:");
  Serial.print(remoteButton);
  Serial.print(",");
  Serial.print("Modo:");
  Serial.print(botMode);
  Serial.print(",");
  Serial.print("l_I_Min:");
  Serial.print(lLineInnerCalibrationMinVal);
  Serial.print(",");
  Serial.print("c_I_Min:");
  Serial.print(cLineInnerCalibrationMinVal);
  Serial.print(",");
  Serial.print("r_I_Min:");
  Serial.print(rLineInnerCalibrationMinVal);
  Serial.print(",");
  Serial.print("l_O_Min:");
  Serial.print(lLineOuterCalibrationMinVal);
  Serial.print(",");
  Serial.print("c_O_Min:");
  Serial.print(cLineOuterCalibrationMinVal);
  Serial.print(",");
  Serial.print("r_O:");
  Serial.print(rLineOuterCalibrationMinVal);
  Serial.print(",");
  Serial.print("LiOffset:");
  Serial.print(lLineInnerCalibrationMaxVal);
  Serial.print(",");
  Serial.print("CiOffset:");
  Serial.print(cLineInnerCalibrationMaxVal);
  Serial.print(",");
  Serial.print("RiOffset:");
  Serial.print(rLineInnerCalibrationMaxVal);
  Serial.print(",");
  Serial.print("LoOffset:");
  Serial.print(lLineOuterCalibrationMaxVal);
  Serial.print(",");
  Serial.print("CoOffset:");
  Serial.print(cLineOuterCalibrationMaxVal);
  Serial.print(",");
  Serial.print("RoOffset:");
  Serial.println(rLineOuterCalibrationMaxVal);
}
void fillLeds(CRGB color)
{
    for(int i = 0; i < NUM_LEDS; i++) {
        leds[i] = color;
    }
    FastLED.show();
    previousColor = color;
}
void flashLeds(CRGB color, int wait, bool usePreviousColor)
{
  CRGB oldColor = previousColor;
  fillLeds(color);
  FastLED.show();
  delay(wait);
  if(usePreviousColor){
    fillLeds(oldColor);
  }else{
    fillLeds(CRGB::Black);
  }
  FastLED.show();
  delay(wait);
}