#include "Arduino.h"
#include "Scaled.h"


Scaled::Scaled() 
{
  Serial.begin(9600);
  pinMode(22, OUTPUT);     
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  digitalWrite(22, HIGH);
  digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);
  digitalWrite(25, HIGH);
}


int Scaled::scale1()
{
  raw1 = analogRead(sensorPin1); 
  scaled1 = ((raw1-417)/range1) * 100;  
  if(scaled1<0) {scaled1=0;}


return scaled1;
}

int Scaled::scale2()
{
raw2 = analogRead(sensorPin2);
  scaled2 = ((raw2-440)/range2) * 100;
  if(scaled2<0) {scaled2=0;}
return scaled2;
}

int Scaled::scale3()
{
  raw3 = analogRead(sensorPin3);
  scaled3 = ((raw3-386)/range3) * 100;
    if(scaled3<0) {scaled3=0;}


return scaled3;
}

int Scaled::scale4()
{ 
 raw4 = analogRead(sensorPin4);
  scaled4 = ((raw4-415)/range4) * 100;
    if(scaled4<0) {scaled4=0;}

return scaled4;
}


