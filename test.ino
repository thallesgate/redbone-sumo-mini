#include <PS4Controller.h>
#define _DEBUG

#define SEND_MILLIS 100   // send commands to hoverboard every SEND_MILLIS millisesonds

#define REFRESH_RATE        30          // [Hz] Sending time interval
const long waitInterval = 1000 / REFRESH_RATE; // [ms] Wait time needed to achieve the desired refresh rate
unsigned long previousMillis = 0; // [ms] Used for the non-blocking delay

float linearCmd = 0;
float angularCmd = 0;
int stateCmd = 0;
int drivemodeCmd = 0;
int maxSpeed = 500;
bool enableSend = false;

#define LINEAR_LIMIT 4.0//0.170 //[M/s] -> 10 RPM +- 10m/minuto
#define ANGULAR_LIMIT 8.0 //1 //[RAD/s] -> 1 rotacao a cada 2 segundos
#define MOTOR_LIMIT 6.0 
float l_speed_cmd = 0.0;//Desired speed for left wheel in m/s
float r_speed_cmd = 0.0;//Desired speed for right wheel in m/s

int received_linear_cmd = 0;
int received_angular_cmd = 0;

#define WHEEL_RADIUS 0.065                 //DIAMETER Wheel radius, in M
#define WHEEL_BASE 0.130               //Wheelbase, in M
const double wheel_circumference = WHEEL_RADIUS * PI * 2; 
// ########################## DS4 ##########################
// PS4 analog range: -+128
#define CONTROLLER_DEADZONE 10.0

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  // Ensure input is within bounds
  if (x < in_min) {
    x = in_min;
  }
  if (x > in_max) {
    x = in_max;
  }
  
  // Calculate the mapped value
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void handleController()
{
  int lStickX = PS4.LStickY();
  int rStickY = PS4.RStickX();

  if(lStickX <= CONTROLLER_DEADZONE && lStickX >= (CONTROLLER_DEADZONE*-1.0)){
    linearCmd = 0.0f;
  }else{
    linearCmd = (float)lStickX;
  }
  if(rStickY <= CONTROLLER_DEADZONE && rStickY >= (CONTROLLER_DEADZONE*-1.0)){
    angularCmd = 0.0f;
  }else{
    angularCmd = (float)rStickY;
  }
  
  //PS4.LStickX(),
  //PS4.LStickY(),
  //PS4.RStickX(),
  //PS4.RStickY(),
  //PS4.Left(),
  //PS4.Down(),
  //PS4.Right(),
  //PS4.Up(),
  //PS4.Square(),
  //PS4.Cross(),
  //PS4.Circle(),
  //PS4.Triangle(),
  //PS4.L1(),
  //PS4.R1(),
  //PS4.L2(),
  //PS4.R2(),  
  //PS4.Share(),
  //PS4.Options(),
  //PS4.PSButton(),
  //PS4.Touchpad(),
  //PS4.Charging(),
  //PS4.Audio(),
  //PS4.Mic(),
  //PS4.Battery());

  if (millis() - lastTimeStamp > 50)
  {
    #ifdef _DEBUG
    //Serial.println("Enabled: " + String(enableSend) + " linearCmd: " + String(linearCmd) + " angularCmd: " + String(angularCmd));
    #endif
    lastTimeStamp = millis();
  }
}

void handleMotorCmd(float linearCmd, float angularCmd, float linearLimit, float angularLimit){
  //0-256

  double linear_velocity_x_req = 0.0;
  double angular_velocity_z_req = 0.0;
  
  linear_velocity_x_req = mapFloat(linearCmd, -128.0f, 128.0f, LINEAR_LIMIT*-1.0, LINEAR_LIMIT);
  angular_velocity_z_req = mapFloat(angularCmd, -128.0f, 128.0f, ANGULAR_LIMIT*-1.0, ANGULAR_LIMIT);

  double l_speed_req = linear_velocity_x_req - angular_velocity_z_req*(WHEEL_BASE/2.0);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  double r_speed_req = linear_velocity_x_req + angular_velocity_z_req*(WHEEL_BASE/2.0);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
  //Serial.println(" | linear_velocity: " + String(linear_velocity_x_req) + " angular_velocity: " + String(angular_velocity_z_req) + " l_speed_req: " + String(l_speed_req) + " r_speed_req: " + String(r_speed_req));
  //l_speed_cmd = (l_speed_req/WHEEL_RADIUS) / (2.0*PI); // Converting m/s to Rad/s to Rev/s * 60 to RPM
  //r_speed_cmd = (r_speed_req/WHEEL_RADIUS) / (2.0*PI); // Converting m/s to Rad/s to Rev/s * 60 to RPM

  l_speed_cmd = (l_speed_req / wheel_circumference) * 60.0;
  r_speed_cmd = (r_speed_req / wheel_circumference) * 60.0;

}

void onConnect()
{
  #ifdef _DEBUG
  Serial.println("Connected!. Enabling.");
  #endif
  enableSend = true;
}

void onDisConnect()
{
  #ifdef _DEBUG
  Serial.println("Disconnected!. Disabling.");
  #endif
  enableSend = false; 
}

void setup()
{
  #ifdef _DEBUG
    Serial.begin(115200);
  #endif
  PS4.attach(handleController);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin();
}
int reqSpeed = 0;

unsigned long iLast = 0;
unsigned long iNext = 0;
unsigned long iTimeNextState = 3000;

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= waitInterval) {
    previousMillis = currentMillis;
    //Non-blocking Task
      handleController();
    if(enableSend){
      handleMotorCmd(linearCmd, angularCmd, LINEAR_LIMIT, ANGULAR_LIMIT);
    }else{
      handleMotorCmd(0.0, 0.0, LINEAR_LIMIT, ANGULAR_LIMIT);
    }
  }
}