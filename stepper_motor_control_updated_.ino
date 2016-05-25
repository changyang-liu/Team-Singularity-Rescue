// NOTE 1/8 step for driver -- change mirostep to improve smoothness
// angularspeed varies with microstep

int M1dirpin = 4;
int M1steppin = 5;
int M2dirpin = 7;
int M2steppin = 6;
int steps;
float angularspeed; // 60-1150deg/s 

void motor1(float M1speed, float degs){ //M1speed and M2speed from -100 to 100
  steps=degs/0.225;
  if(M1speed>0){
    digitalWrite(M1dirpin,LOW);
  }else if (M1speed<0){
    digitalWrite(M1dirpin,HIGH);
  }else{
    steps=0;
  }
  angularspeed=0.5*(100+abs(M1speed*25));
  int t1=(int)(1000000/angularspeed)*0.225;
  delayMicroseconds(2);
  //int StartTime;
  //StartTime=millis();
  for(int j=0;j<steps;j++){
    digitalWrite(M1steppin,LOW);
    delayMicroseconds(2); //maxspeed=1 minspeed=900000
    digitalWrite(M1steppin,HIGH);
    delayMicroseconds(t1); //maxspeed=900 minspeed=90000
  }
 /*int CurrentTime = millis();
   int ElapsedTime = CurrentTime - StartTime;
   Serial.println(ElapsedTime); */
}

void motor2(float M2speed, float degs){
  steps=degs/0.225;
  if(M2speed>0){
    digitalWrite(M2dirpin,LOW);
  }else if (M2speed<0){
    digitalWrite(M2dirpin,HIGH);
  }else{
    steps=0;
  }
  angularspeed=0.5*(100+abs(M2speed*25));
  int t2=(int)(1000000/angularspeed)*0.225;
  delayMicroseconds(2);
  //int StartTime;
  //StartTime=millis();
  for(int j=0;j<steps;j++){
    digitalWrite(M2steppin,LOW);
    delayMicroseconds(2); //maxspeed=1 minspeed=900000
    digitalWrite(M2steppin,HIGH);
    delayMicroseconds(t2); //maxspeed=900 minspeed=90000
  }
/*int CurrentTime = millis();
 int ElapsedTime = CurrentTime - StartTime;
 Serial.println(ElapsedTime);*/ 
}


void setup()
{
  pinMode(M1dirpin,OUTPUT);
  pinMode(M1steppin,OUTPUT);
  pinMode(M2dirpin,OUTPUT);
  pinMode(M2steppin,OUTPUT);
  Serial.begin(9600);
}

void loop()
{
motor1(50, 360);
motor2(50, 360);
}
