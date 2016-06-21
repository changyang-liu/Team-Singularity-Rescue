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

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
DualVNH5019MotorShield md;
Scaled light;
Initialization ini;
PIDe_Array pid = PIDe_Array(md,1.8,0.4,1.5,120,0.04,30);
PIDe_Single single = PIDe_Single(md,50, 1); //base spd, kp
ColourSensor2 colour2 = ColourSensor2();
ColourSensor3 colour3 = ColourSensor3();
Motors mtr = Motors(md);

void doEncoderA1(){mtr.getPastB1()?mtr.subtrEncoder1():mtr.addEncoder1();}
void doEncoderA2(){mtr.getPastB2()?mtr.subtrEncoder2():mtr.addEncoder2();}
void doEncoderB1(){mtr.setPastB1(!mtr.getPastB1());}
void doEncoderB2(){mtr.setPastB2(!mtr.getPastB2());}

void setup() {
  ini.initialize();

  attachInterrupt(digitalPinToInterrupt(mtr.getEncoder1PinA()), doEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(mtr.getEncoder2PinA()), doEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(mtr.getEncoder1PinB()), doEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(mtr.getEncoder2PinB()), doEncoderB2, CHANGE);
  !accel.begin();
  md.init();

}

void loop(){
  //light.printlog();
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
    


    if((far_left + close_left)/2 < 50 && (far_right + close_right)/2 < 50){
      md.setBrakes(400, 400);
      delay(200);
      colour2.update();
      colour3.update();
      if (70<colour2.r()<210 && 90<colour2.g()<235 && 75<colour2.b()<205 && abs(colour2.g()-colour2.r())>20 && abs(colour2.g()-colour2.b())>10) {
        //mtr.moveCounts(-50, -50, 50);
        //single.track(1,0,0);      //side, closeleft, closeright
      } else if (70<colour3.r()<220 && 85<colour3.g()<240 && 75<colour3.b()<210 && abs(colour3.g()-colour3.r())>18 && abs(colour3.g()-colour3.b())>10) {
        //mtr.moveCounts(-50, -50, 50);
        //single.track(2,0,0);
      }
      else {
        //mtr.moveCounts(60, 60, 35);
        md.setBrakes(400,400);
      }
    }else {
      pid.track(far_left,close_left,close_right,far_right);
    }
  }else{
    md.setBrakes(400, 400);
  }
}


