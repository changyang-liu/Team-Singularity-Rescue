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
#include <CS.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
DualVNH5019MotorShield md;
Scaled light;
Initialization ini;
PIDe_Array pid = PIDe_Array(md, 1.8,0,30,75,0.05,30);
//PIDe_Array pid = PIDe_Array(md,1.6.0,0.34,60,75,0.85,30);
PIDe_Single single = PIDe_Single(md,30, 3); //base spd, kp
ColourSensor2 colour2 = ColourSensor2();
ColourSensor3 colour3 = ColourSensor3();
Servo myservo;
Motors mtr = Motors(md, myservo);
SharpIR irFront = SharpIR(A8);
SharpIR irRight = SharpIR(A9);
SharpIR irLeft = SharpIR(A10);
Ultrasound usL = Ultrasound(37, 39);
Ultrasound usR = Ultrasound(43, 47);


int speedBump;
float prevFarL, prevCloseL, prevCloseR, prevFarR, downhillTime;

float far_left;
float close_left;
float close_right;
float far_right;

long LLightTotal, RLightTotal, LLightAvg, RLightAvg;
int LDRsamples = 20;

int counter = 0;
float accelReading, accelDiff, prevAccel, grad;

int gap_count = 0;
int threshold = 10;
long gap_end = 0;
long slow_time = 500;

int line;

int counts, gapCounts, rescue_counter = 0, obst_counter = 0, LGreen, RGreen;

void doEncoderA1(){mtr.getPastB1()?mtr.subtrEncoder1():mtr.addEncoder1();}
void doEncoderA2(){mtr.getPastB2()?mtr.subtrEncoder2():mtr.addEncoder2();}
void doEncoderB1(){mtr.setPastB1(!mtr.getPastB1());}
void doEncoderB2(){mtr.setPastB2(!mtr.getPastB2());}

volatile long distTrav;
boolean done;
int prevState;
float usRSum;

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
  md.init();
  pinMode(31,INPUT_PULLUP);
  pinMode(45, OUTPUT); // one side may be dim, change ports if so
  pinMode(41, OUTPUT);
  myservo.attach(5);
  myservo.write(0);
  myservo.detach();
}

void loop(){
 //Serial.print(mtr.encoder1Pos);
//Serial.print("   ");
//Serial.println(mtr.encoder2Pos);
// light.print();
//light.printlog();
    LGreen = 0;
    RGreen = 0;
    rescue_counter = 0;
//for(int w=0;w<5;w++){mtr.moveCounts(50,-50,10); if(close_left<70){line=1;} }
//for(int q=0;q<5;q++){mtr.moveCounts(-50,50,10); if(close_right<70){line=1;} }
//if(line=1){mtr.moveCounts(50, 50, 60);} //if there's a line in front of the junction
//      else{
//            //tJunction, check for left/right junction here
//          }
//      }
//Serial.println(analogRead(A3));
//colour3.green();
//Serial.println(digitalRead(31));
  if(!ini.button()){
    far_left = light.scale1();
    close_left = light.scale2();
    close_right = light.scale3();
    far_right = light.scale4();
//~~~~~~~~~~Rescue zone
//  if(digitalRead(31)) {              // rescue zone ** DON'T FORGET TO INCLUDE !!!!!!!
//    rescue_counter += 1;
//  }  
//  if(rescue_counter > 10){
//    md.setBrakes(400, 400);
//    mtr.moveCounts(50, 60, 30);
//    for(int v=0;v<20;v++) {usRSum+=usR.getDistance();}
//    usRSum=usRSum/20;
//    if(usRSum<15) {rescueZone();}
//    else{}
//    
//  }
  
//~~~~~~~~~~~Obstacle
  if(!digitalRead(ini.touchSensorPin)) {      //obstacle code
      obstacleLeft();
  }
//~~~~~~~~~~Double black
  if((far_left + close_left)/2 < 50 && (far_right + close_right)/2 < 50 && abs(far_left - far_right) <30 && (far_left < 50 || far_right <50) ){
    md.setBrakes(400, 400);
  delay(200);
  if (slope()==0){
  for(int i = 0; i < 9; i++){
    mtr.greenCounts(-40, -40, 6);
    delay(10);
    if(colour2.green()){
      LGreen = 1;
    }else if(colour3.green()){
      RGreen = 1;
    }
    }
  }
  else {
    for(int i = 0; i < 9; i++){
    mtr.greenCounts(-80, -80, 5);
    delay(10);
    if(colour2.green()){
      LGreen = 1;
    }else if(colour3.green()){
      RGreen = 1;
    }
  }
  }
     if (LGreen == 1) {
//         if(slope()==-1) {
//          mtr.moveCounts(50, 50, 320);
//          mtr.moveCounts(-70, 70, 440);
//          mtr.moveCounts(50, 50, 350);
//          mtr.moveCounts(50, 50, 60);
//          singleTrack(1, 800);
//        }
//        else if (slope()==1) {
//          mtr.moveCounts(90, 90, 250);
//          mtr.moveCounts(-70, 70, 440);
//          mtr.moveCounts(50, 50, 350);
//          mtr.moveCounts(50, 50, 60);
//          singleTrack(1, 800);
//        }

        delay(100);
        singleTrack(1, 950);
        
      }else if (RGreen == 1){
//        if(slope()==-1) {
//          mtr.moveCounts(50, 50, 320);
//          mtr.moveCounts(70, -70, 440);
//          mtr.moveCounts(50, 50, 350);
//          mtr.moveCounts(50, 50, 60);
//          singleTrack(2, 800);
//        }
//        else if (slope()==1) {
//          mtr.moveCounts(90, 90, 250);
//          mtr.moveCounts(-70, 70, 440);
//          mtr.moveCounts(50, 50, 350);
//          mtr.moveCounts(50, 50, 60);
//          singleTrack(2, 800);
//        }
//        else{
        delay(100);
        singleTrack(2, 950);
      }else {
        if(slope()==0){
          mtr.moveCounts(95, 95,300);
        }
        else if (slope()==1) {
          mtr.moveCounts(80, 80, 300);
        }
      }
//      for(int w=0;w<5;w++){mtr.moveCounts(50,-50,10); if(close_left<70){line=1;} }
//      for(int q=0;q<5;q++){mtr.moveCounts(-50,50,10); if(close_right<70){line=1;} }
//      if(line=1){mtr.moveCounts(50, 50, 60);} //if there's a line in front of the junction
//      else{
//            //tJunction, check for left/right junction here
//          }
    }else{
      pid.track(far_left,close_left,close_right,far_right);
      if(speedBump <1500) {speedBump++;}
      if(speedBump==750 ){prevFarL=far_left;prevCloseL=close_left;prevCloseR=close_right;prevFarR=far_right;} //Serial.println("Values")
      if (speedBump == 1500){if(abs(far_left-prevFarL)<4 && abs(close_left-prevCloseL)<4 && abs(close_right-prevCloseR)<4 && abs(far_right-prevFarR)<4) {    pid.setMaxSpeed(150);
      //Serial.println("Max");
    for(int r = 0;r<700;r++) {
    far_left = light.scale1();
    close_left = light.scale2();
    close_right = light.scale3();
    far_right = light.scale4();
    pid.track(far_left,close_left,close_right,far_right);
    }
    //Serial.println("Finish");
    pid.setMaxSpeed(70);}
        speedBump=0;}
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

void obstacleLeft(){
  md.setBrakes(400, 400);
  delay(200);
  mtr.greenCounts(-60, -60, 120);
  mtr.greenCounts(-50, 50, 300);
  mtr.greenCounts(60,60, 320);
  close_left = light.scale2();
  
  while(close_left > 60){
    md.setSpeeds(80, 35);
    close_left = light.scale2();
  }
  md.setBrakes(400,400);
  delay(100);
  close_left = light.scale2();
  while(close_left > 60) {
    md.setSpeeds(40,40);
    close_left = light.scale2();
  }
  md.setBrakes(400,400);
  delay(100);
  singleTrack(1, 900);
}

void obstacleRight(){
  md.setBrakes(400, 400);
  delay(200);
  mtr.greenCounts(-60, -60, 120);
  mtr.greenCounts(50, -50, 260);

  mtr.greenCounts(60,60, 200);
  close_right = light.scale3();
  while(close_right > 70){
    md.setSpeeds(25, 90);
    close_right = light.scale3();
  }
  md.setBrakes(400,400);
  delay(100);
  close_right = light.scale3();
  while(close_right > 70) {
    md.setSpeeds(40,40);
    close_right = light.scale3();
  }
  md.setBrakes(400,400);
  delay(100);
  singleTrack(2, 900);
}

void rescueZone(){
  myservo.attach(5);
  myservo.write(0);
  done = false;
  endCorner = 0;
  entranceCorner = 0;
  while(rescue_counter > 5){
    mtr.stopIfFault();
    if (ini.button() == 0 && done == false) {
      if(entered == 0){
       Serial.println("entrance");
       scanEntrance();
       switch(entranceCorner){
        case 0:
          Serial.println(0);
          entrance1();
          break;
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
        myservo.write(60);
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
          if(distTrav > 900 && distTrav < 2300){ 
            Serial.println("ball at left");
            md.setBrakes(400, 400);
            delay(500);
            ballCollectLeft();
          }
        }else if(distTrav < 2300){
            Serial.println("ball at left");
            md.setBrakes(400, 400);
            delay(500);
            ballCollectLeft();        
        }
      }else if(ballAtRight == 1){ 
        if(entranceCorner == 2 || entranceCorner == 1 && endCorner == 1){
          if(distTrav > 900 && distTrav < 2300){
            Serial.println("ball at right");
            md.setBrakes(400, 400);
            delay(500);
            ballCollectRight(); //check sweep cases
          }
        }else if(distTrav < 2300){
          Serial.println("ball at right");
          md.setBrakes(400, 400);
          delay(500);
          ballCollectRight(); 
        }
      }
    }
    else {md.setBrakes(400, 400);}
  }
}

void scanEntrance(){
  int maxIRLeft = 0;
  int maxIRRight = 0;
  int x;
  int y;
  mtr.encoder1Pos = 0;
  mtr.moveCounts(50, 60, 400); 
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
  if(maxIRLeft >= 100 && maxIRRight < 20){
    entranceCorner = 1;
  }else if(maxIRLeft  < 20 && maxIRRight >= 100){
    entranceCorner = 2;
  }else if(maxIRLeft < 20 && maxIRRight < 100){
    entranceCorner = 3;
  }else if(maxIRLeft < 100 && maxIRRight < 20){
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
    mtr.moveCounts(-60, -50, 310);
    mtr.moveCounts(40, -40, 470);
    mtr.moveCounts(-40, -40, 300);
    mtr.moveCounts(-40, 40, 990);
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
    mtr.moveCounts(-40, 40, 990);
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
  mtr.moveCounts(-40, -40, 1300);
  mtr.moveCounts(-40, 40, 450);
  mtr.moveCounts(-40, -40, 300);
  mtr.moveCounts(-40, 40, 1100);
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
  mtr.moveCounts(-40, -40, 1300);
  mtr.moveCounts(40, -40, 450);
  mtr.moveCounts(-40, -40, 300);
  mtr.moveCounts(-40, 40, 980);
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
    while(abs(mtr.encoder1Pos) < 550 && RLightAvg > rightBlack){
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
      }else if(entranceCorner == 2 || entranceCorner == 4){
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
  mtr.moveCounts(-40, 40, 850);
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
        delay(3);
      }
    }
  }
}

void accele() {
  for(int r=0;r<50;r++) {
    accelReading = analogRead(A3);
    accelDiff += abs(accelReading - prevAccel);
    prevAccel = accelReading;
  }
  accelDiff=accelDiff/20;
  if(accelDiff<150) {
    pid.setMaxSpeed(150);
    for(int r = 0;r<400;r++) {
    far_left = light.scale1();
    close_left = light.scale2();
    close_right = light.scale3();
    far_right = light.scale4();
    pid.track(far_left,close_left,close_right,far_right);
    }
    pid.setMaxSpeed(70);
  }
  else{}
}

float slope() {
  for(int y=0;y<200;y++) {
    grad+=atan2(analogRead(A5),analogRead(A3))*180/3.14159265358979323846; 
  }
  grad=grad/200;
  if(grad>27) {
    return -1;
  }
  else if(grad<21.2) {
    return 1;
  }
  else {
    return 0;
  }
}
