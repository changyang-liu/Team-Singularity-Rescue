#include <Scaled.h>

#include "DualVNH5019MotorShield.h" // from https://github.com/pololu/dual-vnh5019-motor-shield

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>


Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
DualVNH5019MotorShield md;
Scaled light;


#define encoder1PinA  18
#define encoder1PinB  19
#define encoder2PinA  20
#define encoder2PinB  21   


volatile long encoder1Pos = 0;
volatile long encoder2Pos = 0;
volatile boolean PastA1 = 0;
volatile boolean PastA2 = 0;
volatile boolean PastB1 = 0;
volatile boolean PastB2 = 0;






void moveDegs(int motor1Speed, int motor2Speed, int degs){
  stopIfFault();
  unsigned long counts;
  if(motor1Speed==0){  //when only right motor is moving
    counts = (long)(degs*55/36+(100-motor2Speed)*0.3);
    while(abs(encoder2Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
      Serial.println(abs(encoder2Pos));
    }
   }
  else{
    counts = (long)(degs*55/36+(100-motor1Speed)*0.3);
    while(abs(encoder1Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
      Serial.println(abs(encoder1Pos));
    }
   } 
  //Serial.println(counts);
  md.setBrakes(400, 400);
  encoder1Pos=0;
  encoder2Pos=0;
}

void moveTime(int motor1Speed, int motor2Speed, long t){ //milliseconds
  long elapsedtime=0;
  stopIfFault();
  long starttime=millis();
  while(elapsedtime<t){ 
    md.setSpeeds(motor1Speed, motor2Speed);
    elapsedtime=millis()-starttime;
    //Serial.println(elapsedtime);
   } 
  md.setBrakes(400, 400);
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

void testSpeeds(){ // find rotational Speed
  long currentPos1;
  long currentPos2;
  long previousPos1 = 0;
  long previousPos2 = 0; 
  int dtheta1 = 0;
  int dtheta2 = 0;
  md.setSpeeds(100, 100);
  stopIfFault();
  currentPos1 = abs(encoder1Pos);
  currentPos2 = abs(encoder2Pos);
  dtheta1 = currentPos1 - previousPos1;
  Serial.print("dtheta1: ");
  Serial.println(dtheta1);
  dtheta2 = currentPos2 - previousPos2;
  Serial.print("dtheta2: ");
  Serial.println(dtheta2);
  previousPos1 = currentPos1;
  previousPos2 = currentPos2;
}


void lineTrack(){
  while(true){
    int far_left=light.scale1();
    int close_left=light.scale2();
    int close_right=light.scale3();
    int far_right=light.scale4();
    int left_average=(far_left+close_left)/2;
    int right_average=(far_right+close_right)/2;
    stopIfFault();
    int lightTarget=50;
    int motor1Speed, motor2Speed;
    float kp=2.5;
    int baseSpeed=0;
    int leftError, rightError;
    
    leftError=left_average-lightTarget;
    rightError=right_average-lightTarget;
    motor1Speed= kp*leftError+baseSpeed;
    motor2Speed= kp*rightError+baseSpeed;
    md.setSpeeds(motor1Speed, motor2Speed);
    light.print();
  }
}

const float e = 2.71828;
float integral = 0;
int maxIntegral = 10000;
int derivative = 0;
int lastError = 0;
float kp=1;
float ki = 1;
float integralFactor = 0.5;
float kd = 90;
int baseSpeed=50;
int motor1Speed, motor2Speed,error;
  
void lineTrack2(){
  stopIfFault();
  int far_left=light.scale1();
  int close_left=light.scale2();
  int close_right=light.scale3();
  int far_right=light.scale4();
  int left_average=(far_left+close_left)/2;
  int right_average=(far_right+close_right)/2;

  error=left_average-right_average;
  derivative = error-lastError;
  integral = integral*integralFactor + error;
  if (abs(integral) >= maxIntegral) {
    integral = maxIntegral;
  }
  float turn = kp*error+ki*integral+kd*derivative;
  float variableSpeed = 90/(1+pow(e,0.15*(abs(error)-15)));
     motor1Speed=variableSpeed + turn;
     motor2Speed=variableSpeed - turn  ;
     lastError = error;
     md.setSpeeds(motor1Speed, motor2Speed);
  /*if(slope() == 0)
    {if(left_average<45&&right_average<45)
    {
    md.setBrakes(400, 400);
    }
    else{
      motor1Speed=variableSpeed + turn;
      motor2Speed=variableSpeed - turn  ;
      lastError = error;
      md.setSpeeds(motor1Speed, motor2Speed);
      };
    }
   else
   {
    if(slope() == 1)
     {
      motor1Speed=variableSpeed*(80/90) + turn;
      motor2Speed=variableSpeed*(80/90) - turn;
      lastError = error;
      md.setSpeeds(motor1Speed, motor2Speed);
     }
     else
     {
      motor1Speed=variableSpeed*(100/90) + turn;
      motor2Speed=variableSpeed*(100/90) - turn  ;
      lastError = error;
      md.setSpeeds(motor1Speed, motor2Speed);
     };
   }*/
  /*Serial.print(left_average);
  Serial.print(" ");
  Serial.println(right_average);
  delay(100);*/
}

float accelX()
{
  sensors_event_t event; 
  accel.getEvent(&event);
  return event.acceleration.x;
}
float accelY()
{
  sensors_event_t event; 
  accel.getEvent(&event);
  return event.acceleration.y;
}
float accelZ()
{
  sensors_event_t event; 
  accel.getEvent(&event);
  return event.acceleration.z;
}



int slope()
{
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



void setup() 
{ //pinMode(22,INPUT);

 !accel.begin();

  pinMode(encoder1PinA, INPUT);
  //turn on pullup resistor
  digitalWrite(encoder1PinA, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  pinMode(encoder2PinA, INPUT);
  //turn on pullup resistor
  digitalWrite(encoder2PinA, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  pinMode(encoder1PinB, INPUT); 
  //turn on pullup resistor
  digitalWrite(encoder1PinB, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  pinMode(encoder2PinB, INPUT); 
  //turn on pullup resistor
  digitalWrite(encoder2PinB, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  PastA1 = (boolean)digitalRead(encoder1PinA); //initial value of channel A;
  PastA2 = (boolean)digitalRead(encoder2PinA); //initial value of channel A;
  PastB1 = (boolean)digitalRead(encoder1PinB); //and channel B
  PastB2 = (boolean)digitalRead(encoder2PinB); //and channel B

//To Speed up even more, you may define manually the ISRs
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), doEncoderB2, CHANGE);  
  Serial.begin(9600);
  md.init();
}


void loop() 
{


  


if (digitalRead(22) == LOW)  
  { lineTrack2();}
  else
  { md.setBrakes(400,400);}
  //Serial.println(slope());
  

 
 

  //Serial.println((atan2(accelZ(),accelY()) * 180) / 3.1415926);
  //Serial.print("  ");
  //Serial.print(accelY());
  //Serial.print("  ");
  //Serial.println(accelZ());
  
}

//you may easily modify the code  get quadrature..
//..but be sure this whouldn't let Arduino back!
 
void doEncoderA1()
{
     PastB1 ? encoder1Pos--:  encoder1Pos++;
}
void doEncoderA2()
{
     PastB2 ? encoder2Pos--:  encoder2Pos++;
}

void doEncoderB1()
{
     PastB1 = !PastB1; 
}

void doEncoderB2()
{
     PastB2 = !PastB2; 
}

