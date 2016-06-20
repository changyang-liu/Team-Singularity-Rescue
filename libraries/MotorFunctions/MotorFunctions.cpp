#include <DualVNH5019MotorShield.h>
#include <initLib.h>
#include <MotorFunctions.h>

Initialization ini1;
DualVNH5019MotorShield md1;

void Motors::moveCounts(int motor1Speed, int motor2Speed, long counts){
  ini1.encoder1Pos=0;
  ini1.encoder2Pos=0;
  if(motor1Speed==0){  //when only right motor is moving
    while(abs(ini1.encoder2Pos)<counts && ini1.button() == 0){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md1.setSpeeds(motor1Speed, motor2Speed);
    }
   }
  else{
    while(abs(ini1.encoder1Pos)<counts && ini1.button() == 0){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md1.setSpeeds(motor1Speed, motor2Speed);
    }
   } 
  md1.setBrakes(400, 400);
  delay(200);
  ini1.encoder1Pos=0;
  ini1.encoder2Pos=0;
}

void Motors::moveDegs(int motor1Speed, int motor2Speed, int degs){
  ini1.encoder1Pos=0;
  ini1.encoder2Pos=0;
  unsigned long counts;
  if(motor1Speed==0){  //when only right motor is moving
    counts = (long)(degs*55/36+(100-motor2Speed)*0.3);
    while(abs(ini1.encoder2Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md1.setSpeeds(motor1Speed, motor2Speed);
    }
   }
  else{
    counts = (long)(degs*55/36+(100-motor1Speed)*0.3);
    while(abs(ini1.encoder1Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md1.setSpeeds(motor1Speed, motor2Speed);
    }
   }
  md1.setBrakes(400, 400);
  delay(200);
  ini1.encoder1Pos=0;
  ini1.encoder2Pos=0;
}

void Motors::moveTime(int motor1Speed, int motor2Speed, long t){
  long elapsedtime=0;
  long starttime=millis();
  while(elapsedtime<t){ 
    md1.setSpeeds(motor1Speed, motor2Speed);
    elapsedtime=millis()-starttime;
   } 
  md1.setBrakes(400, 400);
 }

 void Motors::constSpeeds(int spd){ 
  float motor1Speed = spd;
  float motor2Speed = spd;
  currentPos1 = abs(ini1.encoder1Pos);
  currentPos2 = abs(ini1.encoder2Pos);
  dtheta1 = currentPos1 - previousPos1;
  dtheta2 = currentPos2 - previousPos2;
  previousPos1 = currentPos1;
  previousPos2 = currentPos2;
  if(dtheta1 == 0){dtheta1 = 1;}
  if(dtheta2 == 0){dtheta2 = 1;}
  motor2Speed = (dtheta1/dtheta2)*motor2Speed;
  md1.setSpeeds(motor1Speed, motor2Speed);
  delay(50);
}

void Motors::stopIfFault()
{
  if (md1.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md1.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}
