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
int entranceCorner;
int endCorner;
bool entered = 0;

float currentDistLeft, previousDistLeft, maxDistLeft, dxLeft, sumdxLeft, avedxLeft;
float currentDistRight, previousDistRight, maxDistRight, dxRight, sumdxRight, avedxRight;

int leftBlack = 450;
int rightBlack= 300; 

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
  myservo.write(0);
  endCorner = 0;
  entranceCorner = 0;
}

//23 counts is roughly 1 cm (adjusted for inertia) at speed 60

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
//  servoPos(0);
//  delay(500);
  mtr.stopIfFault();
  if (ini.button() == 0 && done == false && prevState == 1) {
    Serial.println("entrance");
    if(entered == 0){
     scanEntrance();
     switch(entranceCorner){
      case 1:
        Serial.println(1);
        entrance1();
        break;
      case 2:
        Serial.println(2);
         entrance2();
        break;
      case 3:
        Serial.println(3);
        entrance3();
        break;
      case 4:
        Serial.println(4);
        entrance4();
        break; 
    }
      entered = 1;
    }
    mtr.encoder1Pos = 0;
    while(ini.button() == 0){
      ballAtLeft = 0;
      ballAtRight = 0;
      ballAtFront = 0;
      myservo.write(40);
      md.setSpeeds(35, 45);
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
    }else if(distTrav > 2800 && irFront.distance() < 10 && endCorner == 1){ 
      checkEnd();
      sweep();
    }else if(ballAtFront == 1 && distTrav < 2300){
      Serial.println("ball at front");
      md.setBrakes(400, 400);
      delay(500);
      servoPos(180);
      delay(500);
      servoPos(60);    
    }else if(ballAtLeft == 1){
      if(entranceCorner == 1 || entranceCorner == 2 && endCorner == 1){
        if(distTrav > 800 && distTrav < 2500){ 
          Serial.println("ball at left");
          md.setBrakes(400, 400);
          delay(500);
          ballCollectLeft();
        }
      }else if(distTrav < 2500){
          Serial.println("ball at left");
          md.setBrakes(400, 400);
          delay(500);
          ballCollectLeft();        
      }
    }else if(ballAtRight == 1){ 
      if(entranceCorner == 2 || entranceCorner == 1 && endCorner == 1){
        if(distTrav > 800 && distTrav < 2500){
          Serial.println("ball at right");
          md.setBrakes(400, 400);
          delay(500);
          ballCollectRight(); //check sweep cases
        }
      }else if(distTrav < 2500){
        Serial.println("ball at right");
        md.setBrakes(400, 400);
        delay(500);
        ballCollectRight(); 
      }
    }
  }
  else if (ini.button() == 0 && done == true) {md.setBrakes(400, 400);}
  else if (ini.button() == 1 && done == true) {done = false; prevState = 1;}
  else if (ini.button() == 1 && prevState == 0) {prevState = 1;}
  else {md.setBrakes(400, 400);}
}

void scanEntrance(){
  int maxIRLeft = 0;
  int maxIRRight = 0;
  int x;
  int y;
  mtr.encoder1Pos = 0;
  mtr.moveCounts(50, 60, 200); 
  LRavg();
  while (abs(mtr.encoder1Pos) < 1650 && LLightAvg > leftBlack && RLightAvg > rightBlack) {
    x = irLeft.distance();
    y = irRight.distance();
    LRavg();
    md.setSpeeds(40, 45);
    if(x > maxIRLeft){
      maxIRLeft = x;
    }
    if(y > maxIRRight){
      maxIRRight = y;
    }
    Serial.print(maxIRLeft);
    Serial.print(" ");
    Serial.println(maxIRRight);
  }
  md.setBrakes(400, 400);
  if(maxIRLeft >= 90 && maxIRRight < 20){
    entranceCorner = 1;
  }else if(maxIRLeft  < 20 && maxIRRight >= 90){
    entranceCorner = 2;
  }else if(maxIRLeft < 20 && maxIRRight < 90){
    entranceCorner = 3;
  }else if(maxIRLeft < 90 && maxIRRight < 20){
    entranceCorner = 4;
  }else{
    entranceCorner = 5;
  }
}

//void entrance() {
//  mtr.encoder1Pos = 0;
//  mtr.encoder2Pos = 0;
//  mtr.moveTime(50, 60, 300);
//  LRavg();
//  while (abs(mtr.encoder1Pos) < 1650 && RLightAvg > rightBlack  && ini.button() == 0) {
//    LRavg();
//    md.setSpeeds(50, 60);
//  }
//  md.setBrakes(400, 400);
//  delay(100);
//  if (RLightAvg <=rightBlack) {
//    endCorner = 1;
//    mtr.moveCounts(-60, -50, 280);
//    mtr.moveCounts(40, -40, 470);
//    mtr.moveCounts(-40, -40, 300);
//    mtr.moveCounts(-40, 40, 980);
//    mtr.moveTime(-90, -90, 1500); 
//  }
//  else {
//    mtr.moveCounts(-40, -40, 200);
//    servoPos(180);
//    delay(100);
//    mtr.moveTime(40, -40, 500);
//    mtr.moveTime(90, 80, 2000);
//    mtr.moveCounts(-40, -40, 100);
//    servoPos(60);
//    mtr.moveCounts(-60, -60, 470);
//    mtr.moveCounts(40, -40, 450);
//    mtr.moveCounts(-40, -40, 300);
//    mtr.moveCounts(-40, 40, 980);
//    mtr.moveTime(-90, -90, 1500); 
//  }
//}

void entrance1(){
  if (RLightAvg <=rightBlack) {
    endCorner = 1;
    mtr.moveCounts(-60, -50, 280);
    mtr.moveCounts(40, -40, 470);
    mtr.moveCounts(-40, -40, 300);
    mtr.moveCounts(-40, 40, 1100);
    mtr.moveTime(-90, -90, 1500); 
  }
  else {
    mtr.moveCounts(-40, -40, 200);
    servoPos(180);
    delay(100);
    mtr.moveTime(40, -40, 500);
    mtr.moveTime(90, 80, 2000);
    mtr.moveCounts(-40, -40, 100);
    servoPos(60);
    mtr.moveCounts(-60, -60, 470);
    mtr.moveCounts(40, -40, 450);
    mtr.moveCounts(-40, -40, 300);
    mtr.moveCounts(-40, 40, 1100);
    mtr.moveTime(-90, -90, 1500); 
  }
}

void entrance2(){
  //Serial.println(LLightAvg);
  if (LLightAvg <= leftBlack) {
    endCorner = 1;
    mtr.moveCounts(-60, -50, 280);
    mtr.moveCounts(-40, 40, 490);
    mtr.moveCounts(-40, -40, 300);
    mtr.moveCounts(40, -40, 980);
    mtr.moveTime(-90, -90, 1500); 
  }
  else {
    mtr.moveCounts(-40, -40, 200);
    servoPos(180);
    mtr.moveTime(-40, 40, 500);
    mtr.moveTime(80, 80, 2000);
    mtr.moveCounts(-40, -40, 100);
    servoPos(60);
    mtr.moveCounts(-60, -60, 470);
    mtr.moveCounts(-40, 40, 500);
    mtr.moveCounts(-40, -40, 300);
    mtr.moveCounts(40, -40, 1000);
    mtr.moveTime(-90, -90, 1500); 
  }
}

void entrance3(){
  mtr.moveCounts(-40, -40, 1350);
  mtr.moveCounts(-40, 40, 450);
  mtr.moveCounts(-40, -40, 300);
  mtr.moveCounts(-40, 40, 1200);
  mtr.moveTime(-80, -90, 1500);
    mtr.encoder1Pos = 0;
  mtr.encoder2Pos = 0;
  mtr.moveTime(50, 60, 300);
  LRavg();
  while (abs(mtr.encoder1Pos) < 1350 && RLightAvg > rightBlack  && ini.button() == 0) {
    LRavg();
    md.setSpeeds(50, 60);
  }
  md.setBrakes(400, 400);
  delay(100);
  if (RLightAvg <=rightBlack) {
    endCorner = 1;
    mtr.moveCounts(-60, -50, 280);
    mtr.moveCounts(40, -40, 470);
    mtr.moveCounts(-40, -40, 300);
    mtr.moveCounts(-40, 40, 980);
    mtr.moveTime(-90, -90, 1500); 
  }
  else {
    mtr.moveCounts(-40, -40, 200);
    servoPos(180);
    delay(100);
    mtr.moveTime(40, -40, 500);
    mtr.moveTime(90, 80, 2000);
    mtr.moveCounts(-40, -40, 100);
    servoPos(60);
    mtr.moveCounts(-60, -60, 470);
    mtr.moveCounts(40, -40, 470);
    mtr.moveCounts(-40, -40, 300);
    mtr.moveCounts(-40, 40, 980);
    mtr.moveTime(-90, -90, 1500); 
  }
}

void entrance4(){
  mtr.moveCounts(-40, -40, 1350);
  mtr.moveCounts(40, -40, 450);
  mtr.moveCounts(-40, -40, 300);
  mtr.moveCounts(-40, 40, 1000);
  mtr.moveTime(-80, -90, 1500);
  mtr.encoder1Pos = 0;
  mtr.encoder2Pos = 0;
  mtr.moveTime(50, 60, 300);
  LRavg();
  while (abs(mtr.encoder1Pos) < 1350 && RLightAvg > rightBlack  && ini.button() == 0) {
    LRavg();
    md.setSpeeds(50, 60);
  }
  md.setBrakes(400, 400);
  delay(100);
  if (LLightAvg <= leftBlack) {
    endCorner = 1;
    mtr.moveCounts(-60, -50, 280);
    mtr.moveCounts(-40, 40, 490);
    mtr.moveCounts(-40, -40, 300);
    mtr.moveCounts(40, -40, 980);
    mtr.moveTime(-90, -90, 1500); 
  }
  else {
    mtr.moveCounts(-40, -40, 200);
    servoPos(180);
    mtr.moveTime(-40, 40, 500);
    mtr.moveTime(80, 80, 2000);
    mtr.moveCounts(-40, -40, 100);
    servoPos(60);
    mtr.moveCounts(-60, -60, 470);
    mtr.moveCounts(-40, 40, 500);
    mtr.moveCounts(-40, -40, 300);
    mtr.moveCounts(40, -40, 1000);
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
  mtr.moveCounts(-40, -40 , 200);
  mtr.moveCounts(-40, 40, 510);
  servoPos(180);
  mtr.moveTime(100, 110, 2000);
  mtr.moveCounts(-50, -55, 100);
  delay(200);
  servoPos(60);
  delay(200);
  mtr.moveTime(60, 65, 2000);
  mtr.moveCounts(-50, -55, 1100);
  mtr.moveCounts(40, -40, 468);
  Serial.println("end ballCollectLeft");
}

void ballCollectRight() {
  Serial.println("start ballCollectRight");
  mtr.moveCounts(-40, -40 , 200);
  mtr.moveCounts(40, -40, 450);
  servoPos(180);
  mtr.moveTime(100, 110, 2000);
  mtr.moveCounts(-40, -40, 100);
  delay(200);
  servoPos(60);
  delay(200);
  mtr.moveTime(60, 65, 2000);
  mtr.moveCounts(-50, -55, 1100);
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
  digitalWrite(45, HIGH);
  digitalWrite(41, HIGH);
  delay(10);
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
    mtr.moveTime(80, 90, 2000);
    mtr.moveCounts(-40, -40 ,200);
    mtr.moveCounts(-40, 40, 500);
    mtr.encoder1Pos = 0;
    LRavg();
    while(mtr.encoder1Pos < 550 && RLightAvg > rightBlack){
      LRavg();
      Serial.println(RLightAvg);
      md.setSpeeds(40, 50);
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
      mtr.moveCounts(40, -40, 910);
      mtr.moveTime(-80, -90, 2000);
      mtr.moveTime(50, 60, 8000);
      if(entranceCorner == 1 || entranceCorner == 3){
        sweepEndLeft(); 
      }else{
        sweepEndRight();
      }
      done = true;
      break;
    case 2:
      sweepLeft();
      sweepEndRight(); 
      done = true;
      break;
    case 3:
      sweepEndLeft();
      sweepRight();
      endCorner = 1;
      sweepEndLeft();
      done = true;
      break;
  } 
}

void sweepEndLeft(){
  Serial.println("begin end left");
  if(endCorner == 1){
    mtr.moveTime(80, 90, 2000);
    mtr.moveCounts(-40, -40 ,1200);
    mtr.moveCounts(-40, 40, 250);
  }
  else if(endCorner == 3){
    mtr.moveCounts(-40, -40, 300);
    mtr.moveCounts(40, -40, 450);
    mtr.moveCounts(-40, -40, 700);
    mtr.moveCounts(-40, 40, 250);   
  }
  servoPos(180);
  mtr.moveTime(80, 90, 2000);
  mtr.moveCounts(-40, -40, 400);
  delay(100);
  servoPos(60);
  delay(100);
  // add more sweep if needed later
  mtr.moveCounts(40, -40, 950);
  mtr.moveTime(-60, -60, 3000);
  servoPos(0);
  mtr.moveCounts(50, 50, 700);
  mtr.moveCounts(-40, 40, 760);
}

void sweepEndRight(){
  if(endCorner == 1){
    mtr.moveTime(80, 90, 2000);
    mtr.moveCounts(-40, -40 ,1100);
  }
  mtr.moveCounts(-40, -40, 300);
  mtr.moveCounts(40, -40, 250);
  servoPos(180);
  mtr.moveTime(80, 90, 2000);
  mtr.moveCounts(-40, -40, 400);
  delay(100);
  servoPos(60);
  delay(100);
  // add more sweep if needed later
  mtr.moveCounts(40, -40, 950);
  mtr.moveTime(-40, -40, 3000);
  servoPos(0);
}

void sweepLeft(){
  if(endCorner == 1){
    mtr.moveTime(80, 90, 2000);
    mtr.moveCounts(-40, -40 ,200);
    mtr.moveCounts(-40, 40, 500);
  }else{
    mtr.moveCounts(-40, -40, 300);
  }
  servoPos(180);
  delay(100);
  mtr.moveTime(40, -40, 500);
  mtr.moveTime(80, 80, 2000);
  mtr.moveTime(80, 90, 1000);
  mtr.moveCounts(-40, -40, 400);
  delay(100);
  servoPos(60);
  delay(100);
  mtr.moveCounts(40, -40, 350);
  mtr.moveCounts(-60, -40, 500);
  mtr.moveCounts(-40, 40, 250);
  servoPos(180);
  delay(100);
  mtr.moveTime(80, 90, 2000);
  mtr.moveCounts(-40, -40, 100);
  delay(200);
  servoPos(60);
  delay(200);
  mtr.moveTime(60, 60, 2000);
  mtr.moveCounts(-40, -40, 1150);
  mtr.moveCounts(40, -40, 468);
}

void sweepRight(){
  mtr.moveTime(80, 90, 2000);
  mtr.moveCounts(-40, -40 ,200);
  mtr.moveCounts(40, -40, 460);
  servoPos(180);
  delay(100);
  mtr.moveTime(-40, 40, 500);
  mtr.moveTime(80, 80, 2000);
  mtr.moveTime(80, 90, 1000);
  mtr.moveCounts(-40, -40, 400);
  delay(100);
  servoPos(60);
  delay(100);
  mtr.moveCounts(-40, 40, 350);
  mtr.moveCounts(-40, -60, 500);
  mtr.moveCounts(40, -40, 220);
  servoPos(180);
  delay(100);
  mtr.moveTime(80, 80, 2000);
  mtr.moveCounts(-40, -40, 100);
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
