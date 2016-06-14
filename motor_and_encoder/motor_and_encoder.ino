#include <IR.h>
#include <Scaled.h>
#include <DualVNH5019MotorShield.h> // from https://github.com/pololu/dual-vnh5019-motor-shield
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <initLib.h>


int buttonPin = 53, greenPin = 51, redPin = 50, touchSensorPin = 52, foilPin = 27;

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
DualVNH5019MotorShield md;
Scaled light;
initialization ini;

const float e = 2.71828;


//Colour
unsigned char Re_buf[11], counter = 0, Re_buf1[11], counter1 = 0;
unsigned char sign = 0, sign1 = 0;
byte rgb[3] = {0}, rgb2[3] = {0};

//Button
int state = LOW, reading;
boolean once = false;
long time_passed = 0, debounce = 100;

//Line tracks
int far_left, close_left, close_right, far_right;
float left_average, right_average, error, turn;
float integral = 0, derivative = 0, lastError = 0;

float kp= 2.2;
float ki = 0.08;
float kd = 70;

int maxIntegral = 10000;
float integralFactor = 0.9;

int motor1Speed, motor2Speed;
int originalSpeed = 30, baseSpeed = originalSpeed;


void doEncoderA1(){ini.PastB1?ini.encoder1Pos--:ini.encoder1Pos++;}void doEncoderA2(){ini.PastB2?ini.encoder2Pos--:ini.encoder2Pos++;}void doEncoderB1(){ini.PastB1=!ini.PastB1;}void doEncoderB2(){ini.PastB2=!ini.PastB2;}

void setup(){ 
  ini.initialize();
  attachInterrupt(digitalPinToInterrupt(ini.encoder1PinA), doEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ini.encoder2PinA), doEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(ini.encoder1PinB), doEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ini.encoder2PinB), doEncoderB2, CHANGE); 
  !accel.begin();
  md.init();
}



void loop() 
{
  stopIfFault();
  reading = digitalRead(buttonPin);
  //Serial.println(state);
  //md.setM1Speed(50);
  //testSpeeds(40);
//  Serial.print(ini.encoder1Pos); 
//  Serial.print(" ");
//  Serial.println(ini.encoder2Pos);

  if (reading == LOW) {
    time_passed = millis();
    once = false;
  } else {
    if (millis() - time_passed > debounce && !once) {
      if (state == HIGH) {
        state = LOW;
      } else {
        state = HIGH;
      }
      once = true;
    }
  }
  
  
  far_left = light.scale1();
  close_left = light.scale2();
  close_right = light.scale3();
  far_right = light.scale4();
  
  left_average = (far_left + close_left)/2;
  right_average = (far_right + close_right)/2;

  if (state) {
    if (digitalRead(foilPin)==HIGH) {
      //rescue function
    } else if (digitalRead(touchSensorPin) == LOW) {
      //obstacle function
    } else if (left_average < 50 && right_average < 50) {  //double black
      md.setBrakes(400,400);
      colourSensor();
      colourSensor2();
      delay(200);
      
      if(rgb[0]>=80 && rgb[0]<=230  && rgb[1]>=100 && rgb[1]<=240 && rgb[2]>=40 && rgb[2]<=220 && abs(rgb[1] - rgb[0])>=10 && abs(rgb[1] - rgb[2])>=10){
        digitalWrite(greenPin, HIGH);
        if (slope() == -1) {  //downward slope
          moveTime(-150,-150, 200);
          singleTrack(1, 20, 4, 1200);
        } else {  //level ground
          moveTime(-100, -100, 200);
          singleTrack(1, 30, 4, 1200);
        }
        digitalWrite(greenPin,LOW);
      } else {
        digitalWrite(greenPin, LOW);
      }
    } else {
      lineTrack2();
    }
  }else{
    //md.setBrakes(400,400);
  }
}

void lineTrack2(){
  error = left_average - right_average;
  integral = integral*integralFactor + error;
  derivative = error - lastError;
  
  if (abs(integral) >= maxIntegral) {
    integral = maxIntegral;
  }
  
  float variableSpeed = 105/(1+pow(e,0.05*(abs(error)-15)));
  turn = kp*error+ki*integral+variableSpeed*1.4*derivative;

  motor1Speed = variableSpeed + turn;
  motor2Speed = variableSpeed - turn  ;
  
  lastError = error;
  
  md.setSpeeds(motor1Speed, motor2Speed);
}

void singleTrack(int side, int base, int p, int t){
  long startTime = millis();
  
  while (millis() - startTime < t) {
    int close_left=light.scale2();
    int close_right = light.scale3();
    
    if (side == 1) {
      error = close_left-50;
      turn = error*p;
      motor1Speed = base + turn;
      motor2Speed = base - turn;
    
    } else if (side == 2) {
      error = close_right-50;
      turn = error*p;
      motor1Speed = base - turn;
      motor2Speed = base + turn;
    }
    md.setSpeeds(motor1Speed,motor2Speed);
  } 
}

void color()
{
  Serial.print(rgb[0]);
  Serial.print(" ");
  Serial.print(rgb[1]);
  Serial.print(" ");
  Serial.print(rgb[2]);
  if(rgb[0]>=80 && rgb[0]<=230  && rgb[1]>=100 && rgb[1]<=240 && rgb[2]>=40 && rgb[2]<=220 && abs(rgb[1] - rgb[0])>=10 && abs(rgb[1] - rgb[2])>=10){
  //if(rgb[0]<200 && rgb[1]<200 && rgb[2]<200){
    Serial.print(" ");
    Serial.println("green");
  }
  else{
    Serial.println(" ");
  }
}


void color2()
{
  Serial.print(rgb2[0]);
  Serial.print(" ");
  Serial.print(rgb2[1]);
  Serial.print(" ");
  Serial.print(rgb2[2]);
  if(rgb2[0]>=80 && rgb2[0]<=230  && rgb2[1]>=100 && rgb2[1]<=240 && rgb2[2]>=40 && rgb2[2]<=220 && abs(rgb2[1] - rgb2[0])>=10 && abs(rgb2[1] - rgb2[2])>=10){
  //if(rgb[0]<200 && rgb[1]<200 && rgb[2]<200){
    Serial.print(" ");
    Serial.println("green");
  }
  else{
    Serial.println(" ");
  }
}


void SerialEvent() {
  while (Serial2.available()) {   
    Re_buf[counter] = (unsigned char)Serial2.read();
    if(counter==0&&Re_buf[0]!=0x5A) return;      // 检查帧头         
    counter++;       
    
    if(counter==8) {                //接收到数据
       counter=0;                 //重新赋值，准备下一帧数据的接收 
       sign=1;
    }      
  }
}

void SerialEvent2() {
  while (Serial3.available()) {   
    Re_buf[counter]=(unsigned char)Serial3.read();
    if(counter==0&&Re_buf[0]!=0x5A) return;      // 检查帧头         
    counter++;       
    if(counter==8) {    
       counter=0;                 //重新赋值，准备下一帧数据的接收 
       sign=1;
    }      
  }
}

void colourSensor () {
  unsigned char i=0,sum=0;
  SerialEvent();
  if(sign){   
     sign=0;
     
     for(i=0;i<7;i++){
      sum+=Re_buf[i]; 
      
      if(sum==Re_buf[i]){             
          rgb[0]=Re_buf[4];
          rgb[1]=Re_buf[5];
          rgb[2]=Re_buf[6];     
      }
    }
  }
}

void colourSensor2 (){
  unsigned char i=0,sum=0;
  SerialEvent2();
  if(sign){   
      
     sign=0;
     for(i=0;i<7;i++){
      sum+=Re_buf[i]; 
      if(sum==Re_buf[i]){             
          rgb2[0]=Re_buf[4];
          rgb2[1]=Re_buf[5];
          rgb2[2]=Re_buf[6];  
      }
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

void moveDegs(int motor1Speed, int motor2Speed, int degs){
  stopIfFault();
  unsigned long counts;
  if(motor1Speed==0){  //when only right motor is moving
    counts = (long)(degs*55/36+(100-motor2Speed)*0.3);
    while(abs(ini.encoder2Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
      Serial.println(abs(ini.encoder2Pos));
    }
   }
  else{
    counts = (long)(degs*55/36+(100-motor1Speed)*0.3);
    while(abs(ini.encoder1Pos)<counts){ //470 for 400, 500 for 200, 530 for 100 (about -30 counts per 100 Speed)  
      md.setSpeeds(motor1Speed, motor2Speed);
      Serial.println(abs(ini.encoder1Pos));
    }
   } 
  //Serial.println(counts);
  md.setBrakes(400, 400);
  ini.encoder1Pos=0;
  ini.encoder2Pos=0;
}

void moveTime(int motor1Speed, int motor2Speed, long t){
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
long currentPos1 = 0;
long currentPos2 = 0;
long previousPos1 = 0;
long previousPos2 = 0; 

void testSpeeds(int spd){ // find rotational Speed
  float dtheta1;
  float dtheta2;
  moveTime(spd, spd, 500);
  float motor1Speed = spd;
  float motor2Speed = spd;
  while(true){
    stopIfFault();
    currentPos1 = abs(ini.encoder1Pos);
    currentPos2 = abs(ini.encoder2Pos);
    dtheta1 = currentPos1 - previousPos1;
    Serial.print("dtheta1: ");
    Serial.print(dtheta1);
    dtheta2 = currentPos2 - previousPos2;
    Serial.print(" dtheta2: ");
    Serial.print(dtheta2);
    Serial.print(" 1Speed: ");
    Serial.print(motor1Speed);
    Serial.print(" 2Speed: ");
    Serial.println(motor2Speed);
    previousPos1 = currentPos1;
    previousPos2 = currentPos2;
    if(dtheta1 == 0){dtheta1 = 1;}
    if(dtheta2 == 0){dtheta2 = 1;}
    motor2Speed = (dtheta1/dtheta2)*motor2Speed;
    
    md.setSpeeds(motor1Speed, motor2Speed);
    delay(50);
  }
}


