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

int rightDist, leftDist;
long distTrav;

void doEncoderA1() {mtr.getPastB1() ? mtr.subtrEncoder1() : mtr.addEncoder1();}
void doEncoderA2() {mtr.getPastB2() ? mtr.subtrEncoder2() : mtr.addEncoder2();}
void doEncoderB1() {mtr.setPastB1(!mtr.getPastB1());}
void doEncoderB2() {mtr.setPastB2(!mtr.getPastB2());}

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

long LLightTotal;
long RLightTotal;
long LLightAvg;
long RLightAvg;
int LDRsamples = 200;

void setup() {
  ini.initialize();
  attachInterrupt(digitalPinToInterrupt(mtr.getEncoder1PinA()), doEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(mtr.getEncoder2PinA()), doEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(mtr.getEncoder1PinB()), doEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(mtr.getEncoder2PinB()), doEncoderB2, CHANGE);
  //!accel.begin();          //Don't leave it in if accelerometer is not connected; the code will not proceed until it is connected
  md.init();
  done = false;
  if (ini.button() == 0) {
    prevState = 0;
  }
  else {
    prevState = 1;
  }
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
  mtr.stopIfFault();
  if (ini.button() == 0 && done == false && prevState == 1) {
    
    //currentDist = irLeft.distance();
    //previousDist = currentDist;
    entrance();
    //scan();
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
  delay(400);
  LRavg();
  if (RLightAvg <=720) {
    mtr.moveCounts(-50, -50, 100);
  }
  delay(50000);
  
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
  float currentDist;
  float previousDist;
  float maxDist;
  float dx;
  float sumdx;
  float avedx;
  if (irLeft.distance() < 40) {
    if (maxDist - irLeft.distance() > 10) {
      //break;
    }
  } else if (maxDist - irLeft.distance() > 15) {
    //break;
  }
  sumdx = 0;
  md.setSpeeds(40, 40);;
  for (int i = 0; i < 15; i++) {
    currentDist = irLeft.distance();
    dx = currentDist - previousDist;
    sumdx += dx;
    previousDist = currentDist;
  }
  avedx = sumdx / 15;
  if (abs(avedx) < 0.1) {
    maxDist = currentDist;
  }
}

void scanRight() {
  float currentDist;
  float previousDist;
  float maxDist;
  float dx;
  float sumdx;
  float avedx;
  if (irRight.distance() < 40) {
    if (maxDist - irRight.distance() > 10) {
      //break;
    }
  } else if (maxDist - irRight.distance() > 15) {
    //break;
  }
  sumdx = 0;
  md.setSpeeds(40, 40);
  for (int i = 0; i < 15; i++) {
    currentDist = irRight.distance();
    dx = currentDist - previousDist;
    sumdx += dx;
    previousDist = currentDist;
  }
  avedx = sumdx / 15;
  if (abs(avedx) < 0.1) {
    maxDist = currentDist;
  }
}


void ballCollectLeft() {

}

void ballCollectRight() {
  mtr.moveCounts(-50, -50 , 200);
  mtr.moveCounts(40, -40, 100);

  long start_time = millis();
  long threshold = 100;
  boolean halt = false;
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
{ LLightTotal = LLightTotal+=analogRead(A5);
  RLightTotal = RLightTotal+=analogRead(A15);}
LLightAvg = LLightTotal/LDRsamples;
RLightAvg = RLightTotal/LDRsamples;
}

