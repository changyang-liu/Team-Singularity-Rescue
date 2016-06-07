#include "Arduino.h"
#include "Scaled.h"


Scaled::Scaled() 
{}


int Scaled::scale1()
{
  raw1 = analogRead(sensorPin1); 
  scaled1 = ((raw1-50)/range1) * 100;  
	if (scaled1 <0) {scaled1=0;}
	if (scaled1 >100) {scaled1=100;}


return scaled1;
}

int Scaled::scale2()
{
raw2 = analogRead(sensorPin2);
  scaled2 = ((raw2-120)/range2) * 100;
if (scaled2 <0) {scaled2=0;}
	if (scaled2 >100) {scaled2=100;}

return scaled2;
}

int Scaled::scale3()
{
  raw3 = analogRead(sensorPin3);
  scaled3 = ((raw3-80)/range3) * 100;
if (scaled3 <0) {scaled3=0;}
	if (scaled3 >100) {scaled3=100;}

return scaled3;
}

int Scaled::scale4()
{ 
 raw4 = analogRead(sensorPin4);
  scaled4 = ((raw4-160)/range4) * 100;
if (scaled4 <0) {scaled4=0;}
	if (scaled4 >100) {scaled4=100;}

return scaled4;
}

void Scaled::print()
{ Serial.print("1: ");
Serial.print(scale1());
Serial.print(" 2: ");
Serial.print(scale2());
Serial.print(" 3: ");
Serial.print(scale3());
Serial.print(" 4: ");
Serial.println(scale4());
}


