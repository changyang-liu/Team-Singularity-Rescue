#include <IR.h>
#include <Scaled.h>
#include <DualVNH5019MotorShield.h> // from https://github.com/pololu/dual-vnh5019-motor-shield
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <initLib.h>
#include <PIDe.h>
#include <ColourSensor.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
DualVNH5019MotorShield md;
Scaled light;
Initialization ini;
PIDe_Array pid = PIDe_Array(1.5,0.01,95,0.1,15);
ColourSensor2 colour2 = ColourSensor2();
ColourSensor3 colour3 = ColourSensor3();

void doEncoderA1(){ini.PastB1?ini.encoder1Pos--:ini.encoder1Pos++;}
void doEncoderA2(){ini.PastB2?ini.encoder2Pos--:ini.encoder2Pos++;}
void doEncoderB1(){ini.PastB1=!ini.PastB1;}
void doEncoderB2(){ini.PastB2=!ini.PastB2;}


void setup() {
  ini.initialize();

  attachInterrupt(digitalPinToInterrupt(ini.encoder1PinA), doEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ini.encoder2PinA), doEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(ini.encoder1PinB), doEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ini.encoder2PinB), doEncoderB2, CHANGE); 
  !accel.begin();
  md.init();

}

void loop() {
  light.print();
  if(!ini.button()){
    int far_left = light.scale1();
    int close_left = light.scale2();
    int close_right = light.scale3();
    int far_right = light.scale4();

    if((far_left + close_left)/2 < 50 && (far_right + close_right)/2 < 50){
      md.setBrakes(400, 400);
    }else{
      pid.update(far_left,close_left,close_right,far_right);
      md.setSpeeds(pid.speed1(),pid.speed2());
    }
  }else{
    md.setBrakes(400, 400);
  }

}
