// ##########################################
// #### Definição de Portas e Variaveis! ####
// ##########################################


// #### LEDs Endereçáveis
#define LED_PIN 19

// #### Motores
#define motStandbyPin 4 // Sinal de Standby dos motores. (desativar) | Na placa: Standby
#define lMotPinA 0 // Sinal de direção do motor esquerdo. | Na placa: AN1
#define lMotPinB 2 // Sinal de direção do motor esquerdo. | Na placa: AN2
#define lMotPinS 15 // Sinal de PWM do motor esquerdo. | Na placa: PWMA
#define rMotPinA 16 // Sinal de direção do motor direito. | Na placa: BN1
#define rMotPinB 17 // Sinal de direção do motor direito. | Na placa: BN2
#define rMotPinS 5 // Sinal de PWM do motor direito. | Na placa: PWMB

// #### Receptor de Controle Remoto
#define receiverPin 13 // Sinal do receptor de controle remoto. | Na placa: OUT

// #### Sensores de linha
#define lLinePin 25 // Sinal do sensor de linha esquerdo frontal. |
//#define lLineBackPin 14// Sinal do sensor de linha esquerdo traseiro. | NAO CONECTADO
//#define cLinePin 27 // Sinal do sensor de linha central traseiro. |
#define rLinePin 26// Sinal do sensor de linha direito frontal. |
//#define rLineBackPin 12// Sinal do sensor de linha esquerdo traseiro. | NAO CONECTADO

// #### Sensor de distancia
#define lDistancePin 33 // Sensor de distancia esquerdo. |
#define cDistancePin 32 // Sensor de distancia central. |
#define rDistancePin 34 // Sensor de distancia direito. |
