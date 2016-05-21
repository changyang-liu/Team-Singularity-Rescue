// Separate M1 and M2 for use later on
// Maximum and minimum motorspeed may vary 

int M1dirpin = 4;
int M1steppin = 5;
int M2dirpin = 7;
int M2steppin = 6;
float degs=360; //degrees from 0 to 360
int steps;
float motorspeed; // 110-2600deg/s 
float relativespeed=100;//-99 to 100 
int t;

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
  steps=degs/1.8;
  if(relativespeed>0){
    digitalWrite(M1dirpin,LOW);
    digitalWrite(M2dirpin,LOW);
  }else if (relativespeed<0){
    digitalWrite(M1dirpin,HIGH);
    digitalWrite(M2dirpin,HIGH);
  }else{
    steps=0;
  }
  motorspeed=100+abs(relativespeed*25);
  t=(int)(1000000/motorspeed)*1.8;
  delayMicroseconds(2);
  //int StartTime;
  //for(int i=0;i<100;i++){ //testloop
    //motorspeed=motorspeed+50;
    //StartTime=millis();
    for(int j=0;j<steps;j++){
      digitalWrite(M1steppin,LOW);
      digitalWrite(M2steppin,LOW);
      delayMicroseconds(2); //maxspeed=1 minspeed=900000
      digitalWrite(M1steppin,HIGH);
      digitalWrite(M2steppin,HIGH);
      delayMicroseconds(t); //maxspeed=900 minspeed=90000
  }
 //}
 /*int CurrentTime = millis();
   int ElapsedTime = CurrentTime - StartTime;
   Serial.println(ElapsedTime); */
}
