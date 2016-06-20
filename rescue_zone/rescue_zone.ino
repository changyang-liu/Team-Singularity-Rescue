#include <IR.h>
#include <Scaled.h>
#include <DualVNH5019MotorShield.h> // from https://github.com/pololu/dual-vnh5019-motor-shield
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <initLib.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
DualVNH5019MotorShield md;
Scaled light;
Initialization ini;
SharpIR irFront = SharpIR(A8);
SharpIR irRight = SharpIR(A9);
SharpIR irLeft = SharpIR(A10);

void doEncoderA1(){ini.PastB1?ini.encoder1Pos--:ini.encoder1Pos++;}
void doEncoderA2(){ini.PastB2?ini.encoder2Pos--:ini.encoder2Pos++;}
void doEncoderB1(){ini.PastB1=!ini.PastB1;}
void doEncoderB2(){ini.PastB2=!ini.PastB2;}

int rightDist, leftDist;
long distTrav;

long currentPos1 = 0;
long currentPos2 = 0;
long previousPos1 = 0;
long previousPos2 = 0; 

float dtheta1;
float dtheta2;

boolean done;
int prevState;

int samples;
float total;
float avg;

void setup(){ 
  ini.initialize();
  attachInterrupt(digitalPinToInterrupt(ini.encoder1PinA), doEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ini.encoder2PinA), doEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(ini.encoder1PinB), doEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ini.encoder2PinB), doEncoderB2, CHANGE); 
  //!accel.begin();          //Don't leave it in if accelerometer is not connected; the code will not proceed until it is connected
  md.init();
  done = false;
  if (ini.button() == 0) {prevState = 0;}
  else {prevState = 1;}
}

//23 counts is roughly 1 cm (adjusted for inertia) at speed 60

float currentDist;
float previousDist;
float maxDist;
float dx;
float sumdx;
float avedx;

void loop() 
{
  stopIfFault();
  if(ini.button() == 0 && done == false && prevState == 1){
    ini.encoder1Pos = 0;
    while(ini.button() == 0){
      md.setSpeeds(40, 40);
      Serial.println(ini.encoder1Pos);
    }
    currentDist = irLeft.distance();
    previousDist = currentDist; 
    while(ini.button() == 0){

    }
    while(ini.button() == 0){
      md.setBrakes(400, 400);
    }
    //entrance();
    //scan();
    //done = true;
  }
  else if (ini.button() == 0 && done == true) {md.setBrakes(400, 400);}
  else if(ini.button() == 1 && done == true) {done = false; prevState = 0;}
  else if(ini.button() == 1 && prevState == 0) {prevState = 1;}
  else {md.setBrakes(400, 400);}
}

void entrance(){
  ini.encoder1Pos=0;
  ini.encoder2Pos=0;
  while (ini.button() == 1) {md.setBrakes(400,400);}
  moveTime(60, 60, 300);
  while(abs(ini.encoder1Pos) < 1300 && ini.button() == 0){
    constSpeeds(60);
  }
  md.setBrakes(400, 400);
  delay(200);
  while (ini.button() == 1) {md.setBrakes(400,400);}
  moveCounts(50, -50, 430);
  while (ini.button() == 1) {md.setBrakes(400,400);}
  moveCounts(-50, -50, 200);
  while (ini.button() == 1) {md.setBrakes(400,400);}
  moveCounts(50, -50, 860);
  while (ini.button() == 1) {md.setBrakes(400,400);}
  moveTime(-80, -80, 2000);
  delay(300);
  }
  

void scanLeft(){
  float currentDist;
  float previousDist;
  float maxDist;
  float dx;
  float sumdx;
  float avedx;
  if(irLeft.distance() < 40){
    if(maxDist - irLeft.distance() > 10){
      break;
    }
  }else if(maxDist - irLeft.distance() > 15){
    break;
  }
  sumdx = 0;
  md.setSpeeds(40, 40);;
  for(int i = 0; i < 15; i++){
    currentDist = irLeft.distance();
    dx = currentDist - previousDist;
    sumdx += dx; 
    previousDist = currentDist;
  }
  avedx = sumdx/15;
  if(abs(avedx) < 0.1){
    maxDist = currentDist;
  }
}

void scanRight(){
  float currentDist;
  float previousDist;
  float maxDist;
  float dx;
  float sumdx;
  float avedx;
  if(irRight.distance() < 40){
    if(maxDist - irRight.distance() > 10){
      break;
    }
  }else if(maxDist - irRight.distance() > 15){
    break;
  }
  sumdx = 0;
  md.setSpeeds(40, 40);
  for(int i = 0; i < 15; i++){
    currentDist = irRight.distance();
    dx = currentDist - previousDist;
    sumdx += dx; 
    previousDist = currentDist;
  }
  avedx = sumdx/15;
  if(abs(avedx) < 0.1){
    maxDist = currentDist;
  }
}


void ballCollectLeft(){
   
}

void ballCollectRight(){
  moveCounts(-50, -50 , 200);
  moveCounts(40, -40, 100);
  
  long start_time = millis();
  long threshold = 100;
  boolean halt = false;
}

void printIR(){
  Serial.print("Front:");
  Serial.print(irFront.distance());
  Serial.print(" Left:");
  Serial.print(irLeft.distance());
  Serial.print(" Right:");
  Serial.println(irRight.distance());
}

void testEncoders(){
  Serial.print(ini.encoder1Pos); 
  Serial.print(" ");
  Serial.println(ini.encoder2Pos);
}

void moveCounts(int motor1Speed, int motor2Speed, long counts){
  ini.encoder1Pos=0;
  ini.encoder2Pos=0;
  if(motor1Speed==0){  //when only right motor is moving
    while(abs(ini.encoder2Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
    Serial.println(ini.encoder1Pos);
    }
   }
  else{
    while(abs(ini.encoder1Pos)<counts ){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
    Serial.println(ini.encoder1Pos);
    }
   } 
  md.setBrakes(400, 400);
  delay(200);
  ini.encoder1Pos=0;
  ini.encoder2Pos=0;
}

void moveDegs(int motor1Speed, int motor2Speed, int degs){
  ini.encoder1Pos=0;
  ini.encoder2Pos=0;
  unsigned long counts;
  if(motor1Speed==0){  //when only right motor is moving
    counts = (long)(degs*55/36+(100-motor2Speed)*0.3);
    while(abs(ini.encoder2Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
    }
   }
  else{
    counts = (long)(degs*55/36+(100-motor1Speed)*0.3);
    while(abs(ini.encoder1Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
    }
   }
  md.setBrakes(400, 400);
  delay(200);
  ini.encoder1Pos=0;
  ini.encoder2Pos=0;
}

void moveTime(int motor1Speed, int motor2Speed, long t){
  long elapsedtime=0;
  long starttime=millis();
  while(elapsedtime<t){ 
    md.setSpeeds(motor1Speed, motor2Speed);
    elapsedtime=millis()-starttime;
   } 
  md.setBrakes(400, 400);
 }

 void constSpeeds(int spd){ 
  float motor1Speed = spd;
  float motor2Speed = spd;
  currentPos1 = abs(ini.encoder1Pos);
  currentPos2 = abs(ini.encoder2Pos);
  dtheta1 = currentPos1 - previousPos1;
  dtheta2 = currentPos2 - previousPos2;
  previousPos1 = currentPos1;
  previousPos2 = currentPos2;
  if(dtheta1 == 0){dtheta1 = 1;}
  if(dtheta2 == 0){dtheta2 = 1;}
  motor2Speed = (dtheta1/dtheta2)*motor2Speed;
  md.setSpeeds(motor1Speed, motor2Speed);
}

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}
