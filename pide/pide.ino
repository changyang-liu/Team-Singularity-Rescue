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
PIDe_Single single = PIDe_Single(md,30, 3); //base spd, kp
ColourSensor2 colour2 = ColourSensor2();
ColourSensor3 colour3 = ColourSensor3();
Servo myservo;
Motors mtr = Motors(md, myservo);
SharpIR irFront = SharpIR(A8);
SharpIR irRight = SharpIR(A9);
SharpIR irLeft = SharpIR(A10);

int slopeCount;
float prevFarL, prevCloseL, prevCloseR, prevFarR, downhillTime;

float far_left;
float close_left;
float close_right;
float far_right;

long LLightTotal, RLightTotal, LLightAvg, RLightAvg;
int LDRsamples = 500;

int counter = 0;
int line;

int counts, gapCounts,rescue,LGreen,RGreen;

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
  pinMode(29,INPUT_PULLUP);
}

void loop(){  
//Serial.print(mtr.encoder1Pos);
//Serial.print("   ");
//Serial.println(mtr.encoder2Pos);
//Serial.print(colour2.green());
//Serial.print("  ");
//Serial.println(colour3.green());
// light.print();   
// light.printlog();
//Serial.println(colour2.green());
//colour3.green();
    LGreen = 0;
    RGreen = 0;
//    Serial.println(irLeft.distance());
//Serial.print(irFront.distance());
//Serial.print("   ");
//Serial.println(irRight.distance());
//colour3.green();

if(digitalRead(29)) {
  md.setBrakes(400, 400);
  delay(1000);
}


  if(!ini.button()){
    far_left = light.scale1();
    close_left = light.scale2();
    close_right = light.scale3();
    far_right = light.scale4();

//    if (far_left>110) {
//      while(int w=0;w<100;w++) {
//        md.setSpeeds(-35, -35);
//        if(irLeft.distance() <9) {
//          rescue =1;
//        }
//      }
//      else{
//      md.setBrakes(400,400);
//      delay(1000);
//    }
//    }

//  if(far_left > 98 && close_left > 98 && close_right > 98 && far_right > 98) {++gapCounts;}
//  if(gapCounts==3){ if(abs(pid._derivative) < 0.5){
//  mtr.moveCounts(-60, -60, 100);
//  float startGap = millis();
//  while (millis() - startGap < 500) {
//    far_left = light.scale1();
//    close_left = light.scale2();
//    close_right = light.scale3();
//    far_right = light.scale4();
//    pid.track(far_left,close_left,close_right,far_right);
//  }
//  mtr.moveCounts(60, 60, 70);
//  gapCounts = 0;
//  }
//}
  
    
//  if(!digitalRead(ini.touchSensorPin)) {      //obstacle code
//    md.setBrakes(400, 400);
//    delay(200);
//    mtr.moveCounts(-60, -60, 240);
//    mtr.moveCounts(-50, 50, 300);
//    mtr.moveCounts(60, 60, 750);
//    mtr.moveCounts(50, -50, 270);
//    mtr.moveCounts(50, 50, 600);
//    mtr.moveCounts(50, -50, 300);
//    while(close_right > 50) {
//      md.setSpeeds(50, 50);
//      close_right = light.scale3();
//    } 
//    singleTrack(1, 900);
////    mtr.moveTime(-50, 30, 450);
////    mtr.moveCounts(40, 40, 150);
////    mtr.moveTime(-50, 30, 1250);
//    
//  }

  
  
  if((far_left + close_left)/2 < 50 && (far_right + close_right)/2 < 50 && abs(far_left - far_right) <30 && (far_left < 50 || far_right <50) ){
		md.setBrakes(400, 400);
  delay(200);
  for(int i = 0; i < 9; i++){
    md.setSpeeds(-30, -30);
    if(colour2.green()){
      LGreen = 1;
    }else if(colour3.green()){
      RGreen = 1;
    }
  }
  delay(200);
  for(int i = 0; i < 8; i++){
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
        singleTrack(1, 800);
      }else if (RGreen == 1){
        mtr.moveCounts(-50, -50, 50);
        delay(100);
        singleTrack(2,800);
      }else {
        if (pid.greenIntegral > 35) {
          mtr.moveCounts(-50,50,300);
          mtr.moveCounts(50,50,100);
        } else if (pid.greenIntegral < -35) {
          mtr.moveCounts(50,-50,300);
          mtr.moveCounts(50,50,100);
        } else {
          mtr.moveCounts(50,50,150);
        }
      }
//      for(int w=0;w<5;w++){mtr.moveCounts(50,-50,10); if(close_left<70){line=1;} }
//      for(int q=0;q<5;q++){mtr.moveCounts(-50,50,10); if(close_right<70){line=1;} }
//      if(line=1){mtr.moveCounts(50, 50, 60);} //if there's a line in front of the junction
//      else{
//            //tJunction, check for left/right junction here
//          }
//		  }
    }else{
      pid.track(far_left,close_left,close_right,far_right);
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


