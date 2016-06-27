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

long LLightTotal;
long RLightTotal;
long LLightAvg;
long RLightAvg;
int LDRsamples = 500;

int counter = 0;

int counts;

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
      mtr.moveCounts(-100,-100,300);
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

void LRavg() {
LLightTotal = 0;
RLightTotal = 0;
for (int i=1; i<=LDRsamples;++i)
{LLightTotal+=analogRead(A6);
 RLightTotal+=analogRead(A15);}
LLightAvg = LLightTotal/LDRsamples;
RLightAvg = RLightTotal/LDRsamples;
}

