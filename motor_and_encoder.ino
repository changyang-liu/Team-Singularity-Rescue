#include "DualVNH5019MotorShield.h" // from https://github.com/pololu/dual-vnh5019-motor-shield

DualVNH5019MotorShield md;


//PIN's definition
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
  encoder1Pos=0;
  encoder2Pos=0;
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

void setup() 
{
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
  //moveDegs(100, 100, 500);
  //moveDegs(100, 0, 300);
  moveDegs(200, 200, 360);
  delay(1000);
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

