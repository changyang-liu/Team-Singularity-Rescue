#include <Scaled.h>
#include "DualVNH5019MotorShield.h" // from https://github.com/pololu/dual-vnh5019-motor-shield
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#define encoder1PinA  18
#define encoder1PinB  19
#define encoder2PinA  20
#define encoder2PinB  21

int buttonPin = 24, ledPin = 25, touchSensorPin = 26, foilPin = 27;

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
DualVNH5019MotorShield md;
Scaled light;

const float e = 2.71828;

//Encoder
volatile long encoder1Pos = 0, encoder2Pos = 0;
volatile boolean PastA1 = 0, PastA2 = 0, PastB1 = 0, PastB2 = 0;

//Colour
unsigned char Re_buf[11], counter = 0;
unsigned char sign = 0;
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

void setup(){ 
  pinMode(buttonPin,INPUT);
  pinMode(touchSensorPin, INPUT_PULLUP);
  pinMode(foilPin, INPUT);

  pinMode(ledPin,OUTPUT);
  
  !accel.begin();
  
  pinMode(encoder1PinA, INPUT); //turn on pullup resistor
  digitalWrite(encoder1PinA, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  pinMode(encoder2PinA, INPUT);
  digitalWrite(encoder2PinA, HIGH); 
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB, HIGH);
  pinMode(encoder2PinB, INPUT);
  digitalWrite(encoder2PinB, HIGH);
  
  PastA1 = (boolean)digitalRead(encoder1PinA); //initial value of channel A;
  PastA2 = (boolean)digitalRead(encoder2PinA); //initial value of channel A;
  PastB1 = (boolean)digitalRead(encoder1PinB); //and channel B
  PastB2 = (boolean)digitalRead(encoder2PinB); //and channel B

//To Speed up even more, you may define manually the ISRs
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), doEncoderB2, CHANGE); 
   
  Serial.begin(19200);
  Serial2.begin(9600);
  Serial3.begin(9600);
  delay(1);
  Serial2.write(0XA5); 
  Serial2.write(0X81);    //8 - bit7 = 1; 1 - bit0 = 1
  Serial2.write(0X26);    //Sum of A5 and 81 (for verification)
  Serial3.write(0XA5); 
  Serial3.write(0X81); 
  Serial3.write(0X26);
  md.init();
}



void loop() 
{
  stopIfFault();

  reading = digitalRead(buttonPin);
  Serial.println(state);


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
    if (digitalRead(foilPin) == HIGH) {
      //do rescue zone
    } else if (digitalRead(touchSensorPin) == LOW) {
      //do obstacle
    } else if (left_average < 50 && right_average < 50) {  //double black
      md.setBrakes(400,400);
      colourSensor();
      colourSensor2();
      delay(200);
      
      if(rgb[0]>=80 && rgb[0]<=230  && rgb[1]>=100 && rgb[1]<=240 && rgb[2]>=40 && rgb[2]<=220 && abs(rgb[1] - rgb[0])>=10 && abs(rgb[1] - rgb[2])>=10){
        digitalWrite(ledPin, HIGH);
        if (slope() == -1) {  //downward slope
          moveTime(-150,-150, 200);
          singleTrack(1, 20, 4, 1200);
        } else {  //level ground
          moveTime(-100, -100, 200);
          singleTrack(1, 30, 4, 1200);
        }
        digitalWrite(ledPin,LOW);
      } else {
        digitalWrite(ledPin, LOW);
      }
    } else {
      lineTrack2();
    }
  }else{
    md.setBrakes(400,400);
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
    if(counter==8)                //接收到数据
    {    
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

void doEncoderA1(){
  PastB1 ? encoder1Pos--:  encoder1Pos++;
}
void doEncoderA2(){

  PastB2 ? encoder2Pos--:  encoder2Pos++;
}

void doEncoderB1(){
  PastB1 = !PastB1; 
}

void doEncoderB2(){
  PastB2 = !PastB2; 
}

