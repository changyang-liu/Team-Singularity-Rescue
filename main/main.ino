#include <Servo.h>
#include <EnableInterrupt.h>
#include <Scaled.h>
#include <DualVNH5019MotorShield.h> 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <initLib.h>
#include <PIDe.h>
#include <ColourSensor.h>
#include <MotorFunctions.h>
#include <IR.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
DualVNH5019MotorShield md;
Scaled light;
Initialization ini;
PIDe_Array pid = PIDe_Array(md,1.2,0.3,1.35,70,0.8,30);
PIDe_Single single = PIDe_Single(md,35, 3); //base spd, kp
ColourSensor2 colour2 = ColourSensor2();
ColourSensor3 colour3 = ColourSensor3();
Servo myservo;
Motors mtr = Motors(md, myservo);
SharpIR irFront = SharpIR(A8);
SharpIR irRight = SharpIR(A9);
SharpIR irLeft = SharpIR(A10);

float slopeAvg;
int loops = 0;
int maxloops = 200;
float gradient;
float gradSingle;

int slopeCount;
float prevFarL, prevCloseL, prevCloseR, prevFarR, downhillTime;

float far_left;
float close_left;
float close_right;
float far_right;

int counter = 0;

int counts;
int LGreen;
int RGreen;

bool rescue_zone;
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
volatile long distTrav;

float currentDistLeft, previousDistLeft, maxDistLeft, dxLeft, sumdxLeft, avedxLeft;
float currentDistRight, previousDistRight, maxDistRight, dxRight, sumdxRight, avedxRight;

void doEncoderA1(){mtr.getPastB1()?mtr.subtrEncoder1():mtr.addEncoder1();}
void doEncoderA2(){mtr.getPastB2()?mtr.subtrEncoder2():mtr.addEncoder2();}
void doEncoderB1(){mtr.setPastB1(!mtr.getPastB1());}
void doEncoderB2(){mtr.setPastB2(!mtr.getPastB2());}

void setup() {
  ini.initialize();
  enableInterrupt(mtr.getEncoder1PinA(), doEncoderA1, RISING);
  enableInterrupt(mtr.getEncoder2PinA(), doEncoderA2, RISING);
  enableInterrupt(mtr.getEncoder1PinB(), doEncoderB1, CHANGE);
  enableInterrupt(mtr.getEncoder2PinB(), doEncoderB2, CHANGE);
  md.init();
  rescue_zone = 0;
}

void loop(){  
//Serial.print(mtr.encoder1Pos);
//Serial.print("   ");
//Serial.println(mtr.encoder2Pos);
//Serial.print(colour2.green());
//Serial.print("  ");
//Serial.println(colour3.green());
// light.print();   
//light.printlog();
//Serial.print(LGreen);
//Serial.print("   ");
//Serial.println(RGreen);
    LGreen = 0;
    RGreen = 0;
//  Serial.println(slope());
//  if(slope() == 1) {md.setSpeeds(80,80);}
//  else if(slope() == -1) {md.setSpeeds(25,25);}
//  else{md.setSpeeds(50,50);}
//Serial.print(irFront.distance());
//Serial.print("   ");
//Serial.println(irRight.distance());

  if(!ini.button()){
    far_left = light.scale1();
    close_left = light.scale2();
    close_right = light.scale3();
    far_right = light.scale4();


  if(far_left > 70 && close_left >70 && close_right >70 && far_right > 70) {++counts;}
  else {counts = 0;}
  while(counts > 500) {
    md.setBrakes(400, 400);
    if(irFront.distance() >47 && irFront.distance() < 54 && irRight.distance() >9 && irRight.distance() < 14) {
      //rescue zone 
      rescue_zone = 1;
      mtr.moveCounts(-100,-100,300);
      rescueZone();
    }
    counts = 0;
  }
  
    
  if(!digitalRead(ini.touchSensorPin)) {      //obstacle code
    md.setBrakes(400, 400);
    delay(200);
    mtr.moveCounts(-60, -60, 240);
    mtr.moveCounts(-50, 50, 300);
    mtr.moveCounts(60, 60, 250);
    while(close_right > 50) {
      md.setSpeeds(70, 30);
      close_right = light.scale3();
    }
    mtr.moveTime(-50, 30, 450);
    mtr.moveCounts(40, 40, 150);
    mtr.moveTime(-50, 30, 1250);
    
  }
  
  if((far_left + close_left)/2 < 50 && (far_right + close_right)/2 < 50 && abs(far_left - far_right) < 30 && (far_left < 50 || far_right <50) ){
    md.setBrakes(400, 400);
  delay(200);
  for(int i = 0; i < 5; i++){
    md.setSpeeds(-30, -30);
    if(colour2.green()){
      LGreen = 1;
    }else if(colour3.green()){
      RGreen = 1;
    }
  }
  delay(200);
  for(int i = 0; i < 5; i++){
    md.setSpeeds(30, 30);
    if(colour2.green()){
      LGreen = 1;
    }else if(colour3.green()){
      RGreen = 1;
    }
  }
   
   if (LGreen == 1) {
      mtr.moveCounts(-50, -50, 50);
      delay(100);
      singleTrack(1, 700);
    }else if (RGreen == 1){
      mtr.moveCounts(-50, -50, 50);
      delay(100);
      singleTrack(2,700);
    }
       else {
      mtr.moveCounts(50, 50, 10);
       }
    }else{
      pid.track(far_left,close_left,close_right,far_right);
      if(slopeCount < 400) {++slopeCount;}
      if (slopeCount == 200) {prevFarL = far_left; prevCloseL = close_left; prevCloseR = close_right; prevFarR = far_right;}
      if (slopeCount == 400) {if(slope() == -1) {pid.setMaxSpeed(20);} else if (abs(far_left - prevFarL) < 3 && abs (close_left - prevCloseL) <3 && abs(close_right - prevCloseR) <3 && abs(far_right - prevFarR) < 3){mtr.moveCounts(110, 110, 200);} else {pid.setMaxSpeed(70);}slopeCount = 0;}
    }
  }else{
  md.setBrakes(400, 400);
  }
}

void singleTrack(int side, long t){
  if(side == 1){
    long start_time = millis();  
    while (millis() - start_time < t) {
      single.track(1, light.scale2(), light.scale3());
    }
  }else{
    long start_time = millis(); 
    while (millis() - start_time < t) {
      single.track(2, light.scale2(), light.scale3()); 
    }
  }
}

float slope() {
  for(int i=0;i < 50;++i){
  slopeAvg+=(atan2(analogRead(A2),analogRead(A0))*180/3.1415926)*100;
  }
  slopeAvg = slopeAvg/50;
  if(slopeAvg >=7050 ) {
    return -1;
  }
  else if(slopeAvg <=6900) {
    return 1;
  }
  else {return 0;}
  return slopeAvg;
  slopeAvg = 0;
}

void rescueZone(){
  myservo.attach(5);
  pinMode(43, OUTPUT);
  pinMode(41, OUTPUT);
  digitalWrite(43, HIGH);
  digitalWrite(41, HIGH);
  myservo.write(0);
  endCorner = 0;
  while(rescue_zone == 1){
    if(entered == 0){
      Serial.println("entrance");
      entrance();
      entered = 1;
    } 
    mtr.encoder1Pos = 0;
    while(rescue_zone == 1){
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
  }
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


