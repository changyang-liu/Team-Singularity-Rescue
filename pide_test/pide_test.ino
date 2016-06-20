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
PIDe_Array pid = PIDe_Array(1.6,0.03,90,0.09,10);
PIDe_Single single = PIDe_Single(50, 1);
ColourSensor2 colour2 = ColourSensor2();
ColourSensor3 colour3 = ColourSensor3();

void doEncoderA1(){ini.PastB1?ini.encoder1Pos--:ini.encoder1Pos++;}
void doEncoderA2(){ini.PastB2?ini.encoder2Pos--:ini.encoder2Pos++;}
void doEncoderB1(){ini.PastB1=!ini.PastB1;}
void doEncoderB2(){ini.PastB2=!ini.PastB2;}

long currentPos1 = 0;
long currentPos2 = 0;
long previousPos1 = 0;
long previousPos2 = 0; 

float dtheta1;
float dtheta2;

void setup() {
  ini.initialize();

  attachInterrupt(digitalPinToInterrupt(ini.encoder1PinA), doEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ini.encoder2PinA), doEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(ini.encoder1PinB), doEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ini.encoder2PinB), doEncoderB2, CHANGE); 
  !accel.begin();
  md.init();

}

void loop(){

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

    
//  if(true)    obstacle code
//    //digitalRead(ini.touchSensorPin) == 1) {
//    md.setBrakes(400, 400);
//    delay(200);
//    moveCounts(-60, -60, 260);
//    moveCounts(-50, 50, 320);
//    moveCounts(60, 60, 250);
//    while((far_left+close_left)/2 > 50) {
//      md.setSpeeds(70, 40);
//    }
//    


    if((far_left + close_left)/2 < 50 && (far_right + close_right)/2 < 50){
      md.setBrakes(400, 400);
      delay(200);
      colour2.update();
      colour3.update();
      if (70<colour2.r()<210 && 90<colour2.g()<235 && 75<colour2.b()<205 && abs(colour2.g()-colour2.r())>20 && abs(colour2.g()-colour2.b())>10) {
        moveCounts(-50, -50, 50);
        //single.update(1,0,0);      //side, closeleft, closeright
      }
      if (70<colour3.r()<220 && 85<colour3.g()<240 && 75<colour3.b()<210 && abs(colour3.g()-colour3.r())>18 && abs(colour3.g()-colour3.b())>10) {
        moveCounts(-50, -50, 50);
        //single.update(2,0,0);
      }
    }else {
      pid.update(far_left,close_left,close_right,far_right);
      md.setSpeeds(pid.speed1(),pid.speed2());
    }
  }else{
    md.setBrakes(400, 400);
  }
}


void moveCounts(int motor1Speed, int motor2Speed, long counts){
  ini.encoder1Pos=0;
  ini.encoder2Pos=0;
  if(motor1Speed==0){  //when only right motor is moving
    while(abs(ini.encoder2Pos)<counts && ini.button() == 0){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
    }
   }
  else{
    while(abs(ini.encoder1Pos)<counts && ini.button() == 0){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
    }
   } 
  md.setBrakes(400, 400);
  delay(200);
  ini.encoder1Pos=0;
  ini.encoder2Pos=0;
}

void moveTime(int motor1Speed, int motor2Speed, long t){
  long elapsedtime=0;
  long starttime=millis();
  while(elapsedtime<t){ 
    md.setSpeeds(motor1Speed, motor2Speed);
    elapsedtime=millis()-starttime;
   } 
  md.setBrakes(400, 400);
 }

 void constSpeeds(int spd){ 
  float motor1Speed = spd;
  float motor2Speed = spd;
  currentPos1 = abs(ini.encoder1Pos);
  currentPos2 = abs(ini.encoder2Pos);
  dtheta1 = currentPos1 - previousPos1;
  dtheta2 = currentPos2 - previousPos2;
  previousPos1 = currentPos1;
  previousPos2 = currentPos2;
  if(dtheta1 == 0){dtheta1 = 1;}
  if(dtheta2 == 0){dtheta2 = 1;}
  motor2Speed = (dtheta1/dtheta2)*motor2Speed;
  md.setSpeeds(motor1Speed, motor2Speed);
  delay(50);
}

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

