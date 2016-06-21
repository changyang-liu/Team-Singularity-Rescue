#include <DualVNH5019MotorShield.h>
#include <MotorFunctions.h>
#include <initLib.h>

DualVNH5019MotorShield md;
Motors motor = Motors(md);
Initialization ini;

void doEncoderA1(){motor.getPastB1()?motor.subtrEncoder1():motor.addEncoder1();}
void doEncoderA2(){motor.getPastB2()?motor.subtrEncoder2():motor.addEncoder2();}
void doEncoderB1(){motor.setPastB1(!motor.getPastB1());}
void doEncoderB2(){motor.setPastB2(!motor.getPastB2());}

void setup() {
  ini.initialize();
  attachInterrupt(digitalPinToInterrupt(motor.getEncoder1PinA()), doEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(motor.getEncoder2PinA()), doEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(motor.getEncoder1PinB()), doEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor.getEncoder2PinB()), doEncoderB2, CHANGE);
  md.init();

  motor.moveCounts(40,40,3000);
}

void loop() {
  

}


