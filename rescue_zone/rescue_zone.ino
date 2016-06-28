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
Servo myservo;
Scaled light;
Initialization ini;
SharpIR irFront = SharpIR(A8);
SharpIR irRight = SharpIR(A9);
SharpIR irLeft = SharpIR(A10);
Motors mtr = Motors(md, myservo);

volatile long distTrav;

void doEncoderA1() {mtr.getPastB1() ? mtr.subtrEncoder1() : mtr.addEncoder1();}
void doEncoderA2() {mtr.getPastB2() ? mtr.subtrEncoder2() : mtr.addEncoder2();}
void doEncoderB1() {mtr.setPastB1(!mtr.getPastB1());}
void doEncoderB2() {mtr.setPastB2(!mtr.getPastB2());}

boolean done;
int prevState;

long LLightTotal;
long RLightTotal;
long LLightAvg;
long RLightAvg;
int LDRsamples = 20;

bool ballAtLeft = 0;
bool ballAtRight = 0;
bool ballAtFront = 0;
int endCorner;
bool entered = 0;

int leftBlack = 310;
int rightBlack= 200; 

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
  myservo.attach(5);
  pinMode(45, OUTPUT); // one side may be dim, change ports if so
  pinMode(41, OUTPUT);
  digitalWrite(45, HIGH);
  digitalWrite(41, HIGH);
  myservo.write(0);
  endCorner = 0;
}

//23 counts is roughly 1 cm (adjusted for inertia) at speed 60

float currentDistLeft, previousDistLeft, maxDistLeft, dxLeft, sumdxLeft, avedxLeft;
float currentDistRight, previousDistRight, maxDistRight, dxRight, sumdxRight, avedxRight;


void loop()
{
// Serial.print(mtr.encoder1Pos);
// Serial.print(" ");
// Serial.println(mtr.encoder2Pos);
 //myservo.write(179);
//  LRavg();
//  Serial.print(LLightAvg);
//  Serial.print("   ");
//  Serial.println(RLightAvg);
//  printIR();
//  servoPos(60);
//  delay(500);
//  servoPos(180);
//  delay(500);
  mtr.stopIfFault();
  if (ini.button() == 0 && done == false && prevState == 1) {
    if(entered == 0){
      Serial.println("entrance");
      entrance();
      entered = 1;
    } 
    mtr.encoder1Pos = 0;
    while(ini.button() == 0){
      ballAtLeft = 0;
      ballAtRight = 0;
      ballAtFront = 0;
      myservo.write(40);
      md.setSpeeds(40, 44);
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
    distTrav+=abs(mtr.encoder1Pos);
    Serial.println(distTrav);
    if(distTrav > 2500 && endCorner != 1){       //check end zones, then sweep(if end zone is not opposite entrance)
      Serial.println("end");
      checkEnd();                                      
      sweep();
    }else if(distTrav > 2800 && irFront.distance() < 10 && endCorner == 1){ //sweep both sides
      checkEnd();
      sweep();
    }else if(ballAtFront == 1 && distTrav < 2300){
      Serial.println("ball at front");
      md.setBrakes(400, 400);
      delay(500);
      servoPos(180);
      delay(500);
      servoPos(60);    
    }else if(ballAtLeft == 1 && distTrav > 800 && distTrav < 2500){
      Serial.println("ball at left");
      md.setBrakes(400, 400);
      delay(500);
      ballCollectLeft();
    }else if(ballAtRight == 1 && distTrav < 2500){
      Serial.println("ball at right");
      md.setBrakes(400, 400);
      delay(500);
      ballCollectRight();                                         //check sweep cases
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
  mtr.moveTime(60, 65, 300);
  LRavg();
  while (abs(mtr.encoder1Pos) < 1650 && RLightAvg > rightBlack  && ini.button() == 0) {
    LRavg();
    md.setSpeeds(60, 65);
  }
  md.setBrakes(400, 400);
  delay(100);
  LRavg();
  if (RLightAvg <=rightBlack) {
    endCorner = 1;
    mtr.moveCounts(-60, -50, 280);
    mtr.moveCounts(50, -50, 470);
    mtr.moveCounts(-50, -50, 300);
    mtr.moveCounts(-50, 50, 980);
    mtr.moveTime(-90, -90, 1500); 
  }
  else {
    servoPos(180);
    mtr.moveTime(60, 60, 2000);
    mtr.moveCounts(-50, -50, 100);
    servoPos(60);
    mtr.moveCounts(-60, -60, 470);
    mtr.moveCounts(50, -50, 450);
    mtr.moveCounts(-50, -50, 300);
    mtr.moveCounts(-50, 50, 980);
    mtr.moveTime(-90, -90, 1500); 
  }
}
  

void scanLeft() {
  ballAtLeft = 0;
  if (irLeft.distance() < 40) {
    if (maxDistLeft - irLeft.distance() > 10) {
      ballAtLeft = 1;
    }
  } else if (maxDistLeft - irLeft.distance() > 10) {
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
  }else if (maxDistRight - irRight.distance() > 10) {
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
  Serial.println("start ballCollectLeft");
  mtr.moveCounts(-50, -50 , 200);
  mtr.moveCounts(-50, 50, 510);
  servoPos(180);
  mtr.moveTime(100, 105, 2000);
  mtr.moveCounts(-50, -50, 100);
  delay(200);
  servoPos(60);
  delay(200);
  mtr.moveTime(60, 60, 2000);
  mtr.moveCounts(-50, -50, 1150);
  mtr.moveCounts(40, -40, 468);
  Serial.println("end ballCollectLeft");
}

void ballCollectRight() {
  Serial.println("start ballCollectRight");
  mtr.moveCounts(-50, -50 , 200);
  mtr.moveCounts(50, -50, 450);
  servoPos(180);
  mtr.moveTime(100, 105, 2000);
  mtr.moveCounts(-50, -50, 100);
  delay(200);
  servoPos(60);
  delay(200);
  mtr.moveTime(60, 60, 2000);
  mtr.moveCounts(-50, -55, 1150);
  mtr.moveCounts(-40, 40, 530);
  Serial.println("end ballCollectRight");
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

void checkEnd(){
  if(endCorner != 1){
    mtr.moveTime(80, 85, 2000);
    mtr.moveCounts(-50, -50 ,200);
    mtr.moveCounts(-50, 50, 500);
    mtr.encoder1Pos = 0;
    LRavg();
    while(mtr.encoder1Pos < 550 && RLightAvg > rightBlack){
      LRavg();
      Serial.println(RLightAvg);
      md.setSpeeds(40, 44);
    }
    md.setBrakes(400, 400);
    if(RLightAvg < rightBlack){
      endCorner = 3;
    }else{
      endCorner = 2;
    }
    Serial.print(endCorner);
    Serial.print(" ");
    Serial.println(RLightAvg);
  }
}

void sweep(){
  switch (endCorner){
    case 1:
      sweepLeft();
      sweepRight();
      mtr.moveCounts(50, -50, 910);
      mtr.moveTime(-80, -85, 2000);
      mtr.moveTime(50, 54, 8000);
      sweepEndLeft();
      break;
    case 2:
      sweepLeft();
      sweepEndRight(); 
      break;
    case 3:
      sweepEndLeft();
      sweepRight();
      endCorner = 1;
      sweepEndLeft();
      break;
  } 
}

void sweepEndLeft(){
  Serial.println("begin end left");
  if(endCorner == 1){
    mtr.moveTime(80, 85, 2000);
    mtr.moveCounts(-50, -50 ,1000);
    mtr.moveCounts(-50, 50, 250);
  }
  else if(endCorner == 3){
    mtr.moveCounts(-50, -50, 300);
    mtr.moveCounts(50, -50, 450);
    mtr.moveCounts(-50, -50, 700);
    mtr.moveCounts(-50, 50, 250);   
  }
  servoPos(180);
  mtr.moveTime(80, 85, 2000);
  mtr.moveCounts(-50, -50, 400);
  delay(100);
  servoPos(60);
  delay(100);
  // add more sweep if needed later
  mtr.moveCounts(50, -50, 910);
  mtr.moveTime(-50, -50, 3000);
  servoPos(0);
  mtr.moveCounts(50, 50, 700);
  mtr.moveCounts(-50, 50, 760);
}

void sweepEndRight(){
  mtr.moveCounts(-50, -50, 300);
  mtr.moveCounts(50, -50, 250);
  servoPos(180);
  mtr.moveTime(80, 85, 2000);
  mtr.moveCounts(-50, -50, 400);
  delay(100);
  servoPos(60);
  delay(100);
  // add more sweep if needed later
  mtr.moveCounts(50, -50, 910);
  mtr.moveTime(-50, -50, 3000);
  servoPos(0);
}

void sweepLeft(){
  if(endCorner == 1){
    mtr.moveTime(80, 85, 2000);
    mtr.moveCounts(-50, -50 ,200);
    mtr.moveCounts(-50, 50, 500);
  }else{
    mtr.moveCounts(-50, -50, 300);
  }
  servoPos(180);
  delay(100);
  mtr.moveTime(50, -50, 500);
  mtr.moveTime(60, 80, 2000);
  mtr.moveTime(80, 85, 1000);
  mtr.moveCounts(-50, -50, 400);
  delay(100);
  servoPos(60);
  delay(100);
  mtr.moveCounts(50, -50, 350);
  mtr.moveCounts(-60, -40, 500);
  mtr.moveCounts(-50, 50, 250);
  servoPos(180);
  delay(100);
  mtr.moveTime(80, 80, 2000);
  mtr.moveCounts(-50, -50, 100);
  delay(200);
  servoPos(60);
  delay(200);
  mtr.moveTime(60, 60, 2000);
  mtr.moveCounts(-50, -50, 1150);
  mtr.moveCounts(40, -40, 468);
}

void sweepRight(){
  mtr.moveTime(80, 85, 2000);
  mtr.moveCounts(-50, -50 ,200);
  mtr.moveCounts(50, -50, 460);
  servoPos(180);
  delay(100);
  mtr.moveTime(-50, 50, 500);
  mtr.moveTime(80, 70, 2000);
  mtr.moveTime(80, 85, 1000);
  mtr.moveCounts(-50, -50, 400);
  delay(100);
  servoPos(60);
  delay(100);
  mtr.moveCounts(-50, 50, 350);
  mtr.moveCounts(-40, -60, 500);
  mtr.moveCounts(50, -50, 220);
  servoPos(180);
  delay(100);
  mtr.moveTime(80, 80, 2000);
  mtr.moveCounts(-50, -50, 100);
  delay(200);
  servoPos(60);
  delay(200);
  mtr.moveTime(60, 60, 2000);
  mtr.moveCounts(-50, -55, 1150);
  mtr.moveCounts(-40, 40, 520); 
}

void servoPos(int newPos){
  int currentPos = myservo.read();
  if(currentPos > newPos){
    for (int pos = currentPos; pos > newPos; pos--) { 
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);
    }   
  }else{
    for (int pos = currentPos; pos < newPos; pos++) { // goes from 0 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      if(pos < 120){
        delay(10);  
      }else{
        delay(5);
      }
    }
  }
}
