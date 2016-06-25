#include <Servo.h>
#include <EnableInterrupt.h>
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
PIDe_Array pid = PIDe_Array(md,1.8,0.06,30,70,0.04,30);
PIDe_Single single = PIDe_Single(md,35, 3.5); //base spd, kp
ColourSensor2 colour2 = ColourSensor2();
ColourSensor3 colour3 = ColourSensor3();
Motors mtr = Motors(md);
Servo servo;

int loops = 0;
int maxloops = 200;
float gradient;
float gradSingle;

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
//  !accel.begin();
  md.init();
  //servo.attach(9);
}

void loop(){
  Serial.print(analogRead(A11));
  Serial.print(" ");
  Serial.print(analogRead(A12));
  Serial.print(" ");
  Serial.print(analogRead(A13));
  Serial.print(" ");
  Serial.println(analogRead(A14));
  
//Serial.print(mtr.encoder1Pos);
//Serial.print("   ");
//Serial.println(mtr.encoder2Pos);
//Serial.print(colour2.green());
//Serial.print("  ");
//Serial.println(colour3.green());
//  servo.write(90);
//  Serial.println(slope());
// light.print();   //test values
// light.printlog();
//Lcolour();
//Rcolour();
//Serial.print(LGreen);
//Serial.print("   ");
//Serial.println(RGreen);
    LGreen = 0;
    RGreen = 0;
  

  if(!ini.button()){
    int far_left = light.scale1();
    int close_left = light.scale2();
    int close_right = light.scale3();
    int far_right = light.scale4();
    if(counter==1){
      colour2.green();
      colour3.green();
      Serial.println(colour2.green());
      }else if(counter==100){
        counter=0;
      }
    counter++;
    
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

//    if(loops == maxloops){
//      //Serial.println(slope());
//      gradient = 0;
//      for(int i = 0; i < 10; i++){
//        gradient+=slope();
//      }
//      gradient = gradient/10;
//      if(gradient>=0.5){
//        pid.setMaxSpeed(150);
//      }else if (gradient <= -0.4){
//        pid.setMaxSpeed(10);
//      }else{
//        pid.setMaxSpeed(70);
//      }
//      //Serial.println(gradient);
//      loops = 0;
//    }
//    ++loops;
  
  if((far_left + close_left*1.2)/2.2 < 45 && (far_right + close_right*1.2)/2.2 < 45 && abs(far_left - far_right) < 30 ){
		md.setBrakes(400, 400);
//    gradSingle = 0;
//    for(int i = 0; i < 10; i++){
//    gradSingle+=slope();
//    }
//    gradSingle = gradSingle/10;
//
//    if(gradSingle<=-0.4) {
//     single.setMaxSpeed(10);
//    }else {
//      single.setMaxSpeed(35);
//    }
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
//      Serial.println("L");
      mtr.moveCounts(-50, -50, 50);
      delay(100);
      singleTrack(1, 1000);
    }else if (RGreen == 1){
//      Serial.println("R");
      mtr.moveCounts(-50, -50, 50);
      delay(100);
      singleTrack(2,1000);
    }
       else {
//      Serial.println("No");
      mtr.moveCounts(50, 50, 10);
       }
		
		
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
	if (((atan2(accelZ(),accelY()) * 180) / 3.1415926)>-100&&((atan2(accelZ(),accelY()) * 180) / 3.1415926)<-70){
		return 0;
	}else if(((atan2(accelZ(),accelY()) * 180) / 3.1415926)>-70){
		return 1;
	}else{
		return -1;
	}	
}
