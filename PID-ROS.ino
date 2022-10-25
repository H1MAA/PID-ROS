#include <ros.h>
#include <std_msgs/Int16.h>
#include "Wire.h"
#include <CytronMotorDriver.h>


HardwareSerial Serial3(PB11, PB10);

ros::NodeHandle nh;

float setPoint;

void messageCb( const std_msgs::Int16& toggle_msg){
  setPoint = toggle_msg.data;
  
}

ros::Subscriber<std_msgs::Int16> get_setPoint("setPoint", &messageCb );

// Motor initialization
#define M1 PA8
#define M2 PA9
CytronMD motor(PWM_DIR, M1, M2);


#define ENCODERA PA0
#define ENCODERB PA1
#define res 540

//Encoder variables
long long count;
long long prv_count;
long long dcount;
int rpm;
int rps;


//Time variables
long long currentTime;
long long prv_time;
long long dt;


//PID CONSTANTS (Placeholder)
int kp = 3;
int ki = 3;
int kd = 0.05;


//PID Importatnt varialbles and parameters initialization
float output;                                                 //outut pwm
float error = 0, lastError = 0;
float errorRate, errorSum;



void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial3.begin(115200);
  pinMode(ENCODERA, INPUT_PULLUP);
  pinMode(ENCODERB, INPUT_PULLUP);
  attachInterrupt(ENCODERA, ISR_EncA, CHANGE);
  attachInterrupt(ENCODERB, ISR_EncB, CHANGE);
  
  (nh.getHardware())->setPort(&Serial3);
  (nh.getHardware())->setBaud(115200);
  nh.initNode();
  nh.subscribe(get_setPoint);
}

void loop() {
  //RPM Calculation
  currentTime = millis();
  dt = currentTime - prv_time;
  dcount = count - prv_count;
  rps = (dcount/(dt*res))*1000;
  rpm = rps * 60;

  output = calculatePID(rpm);
  motor.setSpeed(output);

  prv_count = count;
  prv_time = currentTime;
}

float calculatePID(float rpm){

  //set the dt
  currentTime = millis();
  dt = currentTime - prv_time;

  //calculate error, rate and sum
  error = setPoint - rpm;                 //for Kp element
  errorRate = (error - lastError)/dt;     //for Kd Element
  errorSum += error *dt;                  //for Ki Element

  //compute the PID output
  float out = (kp * error) + (kd * errorRate) + (ki * errorSum);

  //update factors
  lastError = error;
  prv_time = currentTime;
  return out;
}


void ISR_EncA(void){
  if (digitalRead(ENCODERA) != digitalRead(ENCODERB))
    count++;
  else
    count--; 
}

void ISR_EncB(void){
  if (digitalRead(ENCODERA) == digitalRead(ENCODERB))
    count++;
  else
    count--;  
}
