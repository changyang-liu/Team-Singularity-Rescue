#include <Servo.h>
#include <EnableInterrupt.h>
#include <IR.h>
#include <Scaled.h>
#include <DualVNH5019MotorShield.h> 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <initLib.h>
#include <PIDe.h>
#include <ColourSensor.h>
#include <MotorFunctions.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
DualVNH5019MotorShield md;
Scaled light;
Initialization ini;
PIDe_Array pid = PIDe_Array(md,1.5,0.6,1.8,70,0.08,30);
PIDe_Single single = PIDe_Single(md,35, 3); //base spd, kp
ColourSensor2 colour2 = ColourSensor2();
ColourSensor3 colour3 = ColourSensor3();
Motors mtr = Motors(md);
Servo servo;

float slopeAvg;
int loops = 0;
int maxloops = 200;
float gradient;
float gradSingle;

int slopeCount;
float prevFarL, prevCloseL, prevCloseR, prevFarR;

float far_left;
float close_left;
float close_right;
float far_right;

long LLightTotal;
long RLightTotal;
long LLightAvg;
long RLightAvg;
int LDRsamples = 500;

int counter = 0;


int LGreen;
int RGreen;

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

 


  if(!ini.button()){
    far_left = light.scale1();
    close_left = light.scale2();
    close_right = light.scale3();
    far_right = light.scale4();


//  if(far_left > 70 && close_left >70 && close_right >70 && far_right > 70) {++counts;}
//  else {counts = 0;}
//  if(counts > 500) {
//    md.setBrakes(400, 400);
//    delay(100);
////    if(RLightAvg > 400)
////    {
////      if(colour2._r <  && colour2._g <  && colour2._b <  && colour3._r <  && colour3._g <  && colour3._b <  ) {
////        mtr.moveCounts(-50,-50,200);
////        //rescue zone
////      }
////    }
//    counts = 0;
//  }
    
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
  
  if((far_left + close_left)/2 < 50 && (far_right + close_right)/2 < 50 && abs(far_left - far_right) < 40 && (far_left < 50 || far_right <50) ){
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
      if(slopeCount < 600) {++slopeCount;}
      if (slopeCount == 300) {prevFarL = far_left; prevCloseL = close_left; prevCloseR = close_right; prevFarR = far_right;}
      if (slopeCount == 600) {if (abs(far_left - prevFarL) < 3 && abs (close_left - prevCloseL) <3 && abs(close_right - prevCloseR) <3 && abs(far_right - prevFarR) < 3){mtr.moveCounts(110, 110, 200);}slopeCount = 0;}
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
  slopeAvg+=atan2(analogRead(A0),analogRead(A2))*180/3.1415926;
  }
  slopeAvg = slopeAvg/50;
  if(slopeAvg >= 18) {
    return -1;
  }
  else if(slopeAvg <=14) {
    return 1;
  }
  else {return 0;}
  return slopeAvg;
  slopeAvg = 0;
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

