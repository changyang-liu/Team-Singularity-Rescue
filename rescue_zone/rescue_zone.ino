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
long distTravelled;

long currentPos1 = 0;
long currentPos2 = 0;
long previousPos1 = 0;
long previousPos2 = 0; 

float dtheta1;
float dtheta2;

void setup(){ 
  ini.initialize();
  attachInterrupt(digitalPinToInterrupt(ini.encoder1PinA), doEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ini.encoder2PinA), doEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(ini.encoder1PinB), doEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ini.encoder2PinB), doEncoderB2, CHANGE); 
  !accel.begin();
  md.init();
}


void loop() 
{
  stopIfFault();
  //entrance();
  if(!ini.button()){
    scan();
  }else{
    md.setBrakes(400, 400);
  }
}

void entrance(){
  moveTime(60, 60, 300);
  while(abs(ini.encoder1Pos) < 1200){
    constSpeeds(60);
  }
  md.setBrakes(400, 400);
  delay(200);
  moveCounts(50, -50, 450);
  moveCounts(-50, -50, 200);
  moveCounts(50, -50, 900);
  moveTime(-80, -80, 2000);
  delay(300);
}

void scan() {
  rightDist = irRight.distance() - 5;
  leftDist = 80 - rightDist;
  distTravelled = 0;
  moveTime(40, 40, 200);
  while(irRight.distance()>rightDist){
      md.setSpeeds(40, 40);
      //distTravelled = distTravelled + dtheta1;
  }
  md.setBrakes(400, 400);
  ballCollectRight();
}


void ballCollectLeft(){
   
}

void ballCollectRight(){
  moveCounts(-50, -50 , 200);
  moveCounts(40, -40, 100);
  
  long start_time = millis();
  long threshold = 100;
  boolean halt = false;

  
while(!halt) {
    md.setSpeeds(40,-40);
//    Serial.print("Millis: ");
//    Serial.print(millis());
//    Serial.print(" Start: ");
//    Serial.println(start_time);
    Serial.println(irFront.distance());

//    if (irFront.distance() >= rightDist - 5) {
//      
//      if (millis() - start_time > threshold) {
////        md.setBrakes(400,400);
////        moveTime(-40,40,(millis()-start_time)/2);
////        md.setBrakes(400, 400);
////        moveTime(40, 40, 200);
////        while(irFront.distance()>10){
////          md.setSpeeds(40, 40);
////        }
////        md.setBrakes(400, 400); 
//        halt = true;
//      } else {
//        start_time = millis();
//      }
//    }
//
  }
//  while (1) {
//       md.setBrakes(400,400);
//  }
 



  
//  while(irFront.distance()>=rightDist){
//    md.setSpeeds(40, -40);
//  }
//  md.setBrakes(400, 400);
//  moveTime(40, 40, 200);
//  while(irFront.distance()>10){
//    md.setSpeeds(40, 40);
//  }
//  md.setBrakes(400, 400);  
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
  stopIfFault();
  if(motor1Speed==0){  //when only right motor is moving
    while(abs(ini.encoder2Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
      //Serial.println(abs(ini.encoder2Pos));
    }
   }
  else{
    while(abs(ini.encoder1Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
      //Serial.println(abs(ini.encoder1Pos));
    }
   } 
  //Serial.println(counts);
  md.setBrakes(400, 400);
  delay(200);
  ini.encoder1Pos=0;
  ini.encoder2Pos=0;
}

void moveDegs(int motor1Speed, int motor2Speed, int degs){
  ini.encoder1Pos=0;
  ini.encoder2Pos=0;
  stopIfFault();
  unsigned long counts;
  if(motor1Speed==0){  //when only right motor is moving
    counts = (long)(degs*55/36+(100-motor2Speed)*0.3);
    while(abs(ini.encoder2Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
      //Serial.println(abs(ini.encoder2Pos));
    }
   }
  else{
    counts = (long)(degs*55/36+(100-motor1Speed)*0.3);
    while(abs(ini.encoder1Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
      //Serial.println(abs(ini.encoder1Pos));
    }
   } 
  //Serial.println(counts);
  md.setBrakes(400, 400);
  delay(200);
  ini.encoder1Pos=0;
  ini.encoder2Pos=0;
}

void moveTime(int motor1Speed, int motor2Speed, long t){
  long elapsedtime=0;
  stopIfFault();
  long starttime=millis();
  while(elapsedtime<t){ 
    md.setSpeeds(motor1Speed, motor2Speed);
    elapsedtime=millis()-starttime;
    //Serial.println(elapsedtime);
   } 
  md.setBrakes(400, 400);
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

void constSpeeds(int spd){ // find rotational Speed
  float motor1Speed = spd;
  float motor2Speed = spd;
  stopIfFault();
  currentPos1 = abs(ini.encoder1Pos);
  currentPos2 = abs(ini.encoder2Pos);
  dtheta1 = currentPos1 - previousPos1;
//  Serial.print("dtheta1: ");
//  Serial.print(dtheta1);
  dtheta2 = currentPos2 - previousPos2;
//  Serial.print(" dtheta2: ");
//  Serial.print(dtheta2);
//  Serial.print(" 1Speed: ");
//  Serial.print(motor1Speed);
  previousPos1 = currentPos1;
  previousPos2 = currentPos2;
  if(dtheta1 == 0){dtheta1 = 1;}
  if(dtheta2 == 0){dtheta2 = 1;}
  motor2Speed = (dtheta1/dtheta2)*motor2Speed;
//  Serial.print(" 2Speed: ");
//  Serial.println(motor2Speed);
  md.setSpeeds(motor1Speed, motor2Speed);
  delay(50);
}


