#include <EnableInterrupt.h>
#include <Servo.h>
#include <IR.h>
#include <Scaled.h>
#include <DualVNH5019MotorShield.h> // from https://github.com/pololu/dual-vnh5019-motor-shield
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <initLib.h>
#include <MotorFunctions.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
DualVNH5019MotorShield md;
Scaled light;
Initialization ini;
SharpIR irFront = SharpIR(A8);
SharpIR irRight = SharpIR(A9);
SharpIR irLeft = SharpIR(A10);
Motors mtr = Motors(md);
Servo myservo;

int rightDist, leftDist;
volatile long distTrav;

void doEncoderA1() {mtr.getPastB1() ? mtr.subtrEncoder1() : mtr.addEncoder1();}
void doEncoderA2() {mtr.getPastB2() ? mtr.subtrEncoder2() : mtr.addEncoder2();}
void doEncoderB1() {mtr.setPastB1(!mtr.getPastB1());}
void doEncoderB2() {mtr.setPastB2(!mtr.getPastB2());}

boolean done;
int prevState;

int samples;
float total;
float avg;

long LLightTotal;
long RLightTotal;
long LLightAvg;
long RLightAvg;
int LDRsamples = 200;

bool ballAtLeft = 0;
bool ballAtRight = 0;
bool ballAtFront = 9;
int endCorner;

void setup() {
  ini.initialize();
  enableInterrupt(mtr.getEncoder1PinA(), doEncoderA1, RISING);
  enableInterrupt(mtr.getEncoder2PinA(), doEncoderA2, RISING);
  enableInterrupt(mtr.getEncoder1PinB(), doEncoderB1, CHANGE);
  enableInterrupt(mtr.getEncoder2PinB(), doEncoderB2, CHANGE);
  //!accel.begin();          //Don't leave it in if accelerometer is not connected; the code will not proceed until it is connected
  md.init();
  done = false;
  if (ini.button() == 0) {
    prevState = 0;
  }
  else {
    prevState = 1;
  }
  myservo.attach(9);
  pinMode(39, OUTPUT);
  pinMode(41, OUTPUT);
  digitalWrite(39, HIGH);
  digitalWrite(41, HIGH);
  Servo.write(180);
}

//23 counts is roughly 1 cm (adjusted for inertia) at speed 60

float currentDistLeft, previousDistLeft, maxDistLeft, dxLeft, sumdxLeft, avedxLeft;
float currentDistRight, previousDistRight, maxDistRight, dxRight, sumdxRight, avedxRight;

void loop()
{
//   LRavg();
//  Serial.print(LLightAvg);
//  Serial.print("   ");
//  Serial.println(RLightAvg);
//  printIR();
//  servoPos(140);
//  delay(500);
//  servoPos(0);
//  delay(500);
//  Serial.println(mtr.encoder1Pos);
//  LRavg();
//  Serial.print(LLightAvg);
//  Serial.print("   ");
//  Serial.println(RLightAvg);
  mtr.stopIfFault();
//  while(ini.button() == 0){
//    md.setSpeeds(40, 45);
//  }
  if (ini.button() == 0 && done == false && prevState == 1) {
    //entrance();
//    if(endCorner == 1){
//      mtr.moveCounts(40, 40, 1000);
//    }
    mtr.encoder1Pos = 0;
    while(ini.button() == 0){
      ballAtLeft = 0;
      ballAtRight = 0;
      ballAtFront = 0;
      md.setSpeeds(50, 50);
      scanLeft();
      scanRight();     
      if(ballAtLeft == 1 || ballAtRight == 1){
        break;
      }
      if(irFront.distance() < 15){
        ballAtFront = 1;
        break;
      }
    }
    distTrav+=mtr.encoder1Pos;
    //Serial.println(distTrav);
    if(ballAtLeft == 1 && distTrav > 700){
      md.setBrakes(400, 400);
      delay(2000);
      ballCollectLeft();
    }else if(ballAtRight == 1){
      md.setBrakes(400, 400);
      delay(2000);
      ballCollectRight();
    }else if(ballAtFront == 1 && distTrav < 2500){
      md.setBrakes(400, 400);
      delay(2000);
      ServoPos(0);
      ServoPos(140);
    }
    //done = true;
  }
  else if (ini.button() == 0 && done == true) {md.setBrakes(400, 400);}
  else if (ini.button() == 1 && done == true) {done = false; prevState = 1;}
  else if (ini.button() == 1 && prevState == 0) {prevState = 1;}
  else {md.setBrakes(400, 400);}
}

void entrance() {
  mtr.encoder1Pos = 0;
  mtr.encoder2Pos = 0;
  while (ini.button() == 1) {md.setBrakes(400, 400);}
  mtr.moveTime(60, 60, 300);
  while (abs(mtr.encoder1Pos) < 1650 && ini.button() == 0) {
    md.setSpeeds(60, 60);
  }
  md.setBrakes(400, 400);
  delay(100);
  LRavg();
  if (RLightAvg <=370) {
    endCorner = 1;
    mtr.moveCounts(-50, -50, 280);
    mtr.moveCounts(50, -50, 430);
    mtr.moveCounts(-50, -50, 200);
    mtr.moveCounts(-50, 50, 910);
    mtr.moveTime(-50, -50, 1000); 
  }
  else {
    mtr.moveCounts(50, 50, 200);
  }

  delay(3000);
  
//  while (ini.button() == 1) {md.setBrakes(400, 400);}
//  mtr.moveCounts(50, -50, 430);
//  while (ini.button() == 1) {md.setBrakes(400, 400);}
//  mtr.moveCounts(-50, -50, 200);
//  while (ini.button() == 1) {md.setBrakes(400, 400);}
//  mtr.moveCounts(50, -50, 860);
//  while (ini.button() == 1) {md.setBrakes(400, 400);}
//  mtr.moveTime(-80, -80, 2000);
//  delay(300);
}

void scanLeft() {
  ballAtLeft = 0;
  if (irLeft.distance() < 40) {
    if (maxDistLeft - irLeft.distance() > 10) {
      ballAtLeft = 1;
    }
  } else if (maxDistLeft - irLeft.distance() > 15) {
      ballAtLeft = 1;
  }
  sumdxLeft = 0;
  for (int i = 0; i < 10; i++) {
    currentDistLeft = irLeft.distance();
    dxLeft = currentDistLeft - previousDistLeft;
    sumdxLeft += dxLeft;
    previousDistLeft = currentDistLeft;
  }
  avedxLeft = sumdxLeft / 10;
  if (abs(avedxLeft) < 0.2) {
    maxDistLeft = currentDistLeft;
  }
}

void scanRight() {
  ballAtRight == 0;
  if (irRight.distance() < 40) {
    if (maxDistRight - irRight.distance() > 10) {
      ballAtRight = 1;
    }
  }else if (maxDistRight - irRight.distance() > 15) {
    ballAtRight = 1;
  }
  //Serial.println(avedx);
  sumdxRight = 0;
  for (int i = 0; i < 10; i++) {
    currentDistRight = irRight.distance();
    dxRight = currentDistRight - previousDistRight;
    sumdxRight += dxRight;
    previousDistRight = currentDistRight;
  }
  avedxRight = sumdxRight / 10;
  if (abs(avedxRight) < 0.2) {
    maxDistRight = currentDistRight;
  }
}


void ballCollectLeft() {
  

}

void ballCollectRight() {
  mtr.moveCounts(-50, -50 , 200);
  mtr.moveCounts(40, -40, 420);
  servoPos(0);
  mtr.moveTime(50, 50, 4000);
}

void printIR() {
  Serial.print("Front:");
  Serial.print(irFront.distance());
  Serial.print(" Left:");
  Serial.print(irLeft.distance());
  Serial.print(" Right:");
  Serial.println(irRight.distance());
}

void LRavg() {
LLightTotal = 0;
RLightTotal = 0;
for (int i=1; i<=LDRsamples;++i)
{LLightTotal+=analogRead(A6);
 RLightTotal+=analogRead(A15);}
LLightAvg = LLightTotal/LDRsamples;
RLightAvg = RLightTotal/LDRsamples;
}

void servoPos(int newPos){
  int currentPos = myservo.read();
  if(currentPos < newPos){
    for (int pos = currentPos; pos < newPos; pos++) { 
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(10);
    }   
  }else{
    for (int pos = currentPos; pos > newPos; pos--) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      if(pos > 60){
        delay(10);  
      }else{
        delay(5);
      }
    }
  }
}
