#include <IR.h>
#include <Scaled.h>
#include <DualVNH5019MotorShield.h> // from https://github.com/pololu/dual-vnh5019-motor-shield
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <initLib.h>
#include <PIDe.h>
#include <ColourSensor.h>
#include <MotorFunctions.h>
#include <EnableInterrupt.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
DualVNH5019MotorShield md;
Scaled light;
Initialization ini;
PIDe_Array pid = PIDe_Array(md,1.8,0.4,1.5,120,0.035,30);
PIDe_Single single = PIDe_Single(md,35, 3.5); //base spd, kp
ColourSensor2 colour2 = ColourSensor2();
ColourSensor3 colour3 = ColourSensor3();
Motors mtr = Motors(md);

int LGreen;
int RGreen;

int loops = 0;
int maxloops = 200;

int Lr1, Lg1, Lb1, Lr2, Lg2, Lb2, Rr1, Rg1, Rb1, Rr2, Rg2, Rb2, LrAvg, LgAvg, LbAvg, RrAvg, RgAvg, RbAvg;

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
  !accel.begin();
  md.init();

}

void loop(){
//  light.printlog();   //test values
// colour2.update();
//  colour3.update();
//  Serial.print(colour2.r());
//  Serial.print("  ");
//  Serial.print(colour2.g());
//  Serial.print("  ");
//  Serial.print(colour2.b());
//  Serial.print("  S2  ");
//  Serial.print(colour3.r());
//  Serial.print("  ");
//  Serial.print(colour3.g());
//  Serial.print("  ");
//  Serial.println(colour3.b());
//  if(70<colour2.r()<210 && 90<colour2.g()<235 && 75<colour2.b()<205 && abs(colour2.g()-colour2.r()) >20 && abs(colour2.g()-colour2.b()) >20) { 
//    Serial.println("LGreen");
//  }
//  if (70<colour3.r()<220 && 85<colour3.g()<230 && 75<colour3.b()<210 && abs(colour3.g()-colour3.r())>20 && abs(colour3.g()-colour3.b())>20) {
//  Serial.println("RGreen");
//  }

  
  if(!ini.button()){
    int far_left = light.scale1();
    int close_left = light.scale2();
    int close_right = light.scale3();
    int far_right = light.scale4();
    
//  if(digitalRead(ini.touchSensorPin) == 0) {      //obstacle code
//    md.setBrakes(400, 400);
//    delay(200);
//    mtr.moveCounts(-60, -60, 260);
//    mtr.moveCounts(-50, 50, 320);
//    mtr.moveCounts(60, 60, 250);
//    while((far_left+close_left)/2 > 50) {
//      md.setSpeeds(70, 40);
//    }
//  }

    if(loops == maxloops){
      if(slope() == 1){
        pid.setMaxSpeed(150);
      }else if (slope() == -1){
        pid.setMaxSpeed(10);
      }else{
        pid.setMaxSpeed(70);
      }
      //Serial.println(gradient);
      loops = 0;
    }
    ++loops;
  
    if((far_left + close_left)/2 < 40 && (far_right + close_right)/2 < 40){
      md.setBrakes(400, 400);
      Lcolour();
      Rcolour();

      if (LGreen == 0 && RGreen == 0) {
        mtr.moveCounts(60, 60, 23);
        //md.setBrakes(400,400);
      }
      LGreen = 0;
      RGreen = 0;
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


float accelX(){                                 //Accelerometer readings on X, Y and Z axis.
  sensors_event_t event; 
  accel.getEvent(&event);
  return event.acceleration.x;
}

float accelY(){
  sensors_event_t event; 
  accel.getEvent(&event);
  return event.acceleration.y;
}

float accelZ(){
  sensors_event_t event; 
  accel.getEvent(&event);
  return event.acceleration.z;
}

int slope(){                           //Function to detect uphill, downhill or flat
if (((atan2(accelZ(),accelY()) * 180) / 3.1415926)>-100&&((atan2(accelZ(),accelY()) * 180) / 3.1415926)<-75)
  {
  return 0;}
  else
  {
    if (((atan2(accelZ(),accelY()) * 180) / 3.1415926)>-90)
    {return 1;}
    else
    {return -1;}
  }
}

void Lcolour()    {colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
colour2.update();
      Lr1 = colour2.r();
      Lg1 = colour2.g();
      Lb1 = colour2.b();
      delay(100);
      colour2.update();
      colour2.update();
      colour2.update();
      colour2.update();
      colour2.update();
      colour2.update();
      colour2.update();
            colour2.update();
      colour2.update();
      colour2.update();
      colour2.update();
      colour2.update();
      colour2.update();
      colour2.update();
            colour2.update();
      colour2.update();
      colour2.update();
      colour2.update();
      colour2.update();
      colour2.update();
      colour2.update();
      Lr2 = colour2.r();
      Lg2 = colour2.g();
      Lb2 = colour2.b();
      LrAvg = (Lr1+Lr2)/2;
      LgAvg = (Lg1+Lg2)/2;
      LbAvg = (Lb1+Lb2)/2;
      if(70<LrAvg<210 && 90<LgAvg<235 && 75<LbAvg<205 && abs(LgAvg-LrAvg) >20 && abs(LgAvg-LbAvg) >20) {
        Serial.println("L");
        mtr.moveCounts(-50, -50, 50);
        delay(100);
        singleTrack(1, 1000);      //side, closeleft, closeright
        LGreen = 1;
      }
}


void Rcolour() { 
        colour3.update();
        colour3.update();
        colour3.update();
        colour3.update();
        colour3.update();
        colour3.update();
                colour3.update();
        colour3.update();
        colour3.update();
        colour3.update();
        colour3.update();
        colour3.update();
                colour3.update();
        colour3.update();
        colour3.update();
        colour3.update();
        colour3.update();
        colour3.update();
      Rr1 = colour3.r();
      Rg1 = colour3.g();
      Rb1 = colour3.b();
      delay(100);
      colour3.update();
      colour3.update();
      colour3.update();
      colour3.update();
      colour3.update();
      colour3.update();
            colour3.update();
      colour3.update();
      colour3.update();
      colour3.update();
      colour3.update();
      colour3.update();
            colour3.update();
      colour3.update();
      colour3.update();
      colour3.update();
      colour3.update();
      colour3.update();
      Rr2 = colour3.r();
      Rg2 = colour3.g();
      Rb2 = colour3.b();
      RrAvg = (Rr1+Rr2)/2;
      RgAvg = (Rg1+Rg2)/2;
      RbAvg = (Rb1+Rb2)/2;
      delay(100);
      if (70<RrAvg<220 && 85<RgAvg<230 && 75<RbAvg<210 && abs(RgAvg-RrAvg)>20 && abs(RgAvg-RrAvg)>20) {
        Serial.println("R");
        mtr.moveCounts(-50, -50, 50);
        delay(100);
        singleTrack(2, 1000);
        RGreen = 1;
      }

}

