#include "Arduino.h"
#include "Scaled.h"


Scaled::Scaled() 
{}

float Scaled::scale1()
{
  raw1 = log(analogRead(sensorPin1)); 
  scaled1 = ((raw1-min1)/range1) * 100;  
	if (scaled1 <0) {scaled1=0;}
	if (scaled1 >100) {scaled1=100;}


return scaled1;
}

float Scaled::scale2()
{
raw2 = log(analogRead(sensorPin2));
  scaled2 = ((raw2-min2)/range2) * 100;
if (scaled2 <0) {scaled2=0;}
	if (scaled2 >100) {scaled2=100;}

return scaled2;
}

float Scaled::scale3()
{
  raw3 = log(analogRead(sensorPin3));
  scaled3 = ((raw3-min3)/range3) * 100;
if (scaled3 <0) {scaled3=0;}
	if (scaled3 >100) {scaled3=100;}

return scaled3;
}

float Scaled::scale4()
{ 
 raw4 = log(analogRead(sensorPin4));
  scaled4 = ((raw4-min4)/range4) * 100;
if (scaled4 <0) {scaled4=0;}
	if (scaled4 >100) {scaled4=100;}

return scaled4;
}


void Scaled::printlog()
{ Serial.print("Far left: ");
Serial.print(log(analogRead(sensorPin1)));
Serial.print(" Close left: ");
Serial.print(log(analogRead(sensorPin2)));
Serial.print(" Close right: ");
Serial.print(log(analogRead(sensorPin3)));
Serial.print(" Far right: ");
Serial.println(log(analogRead(sensorPin4)));
}

void Scaled::print()
{ Serial.print("Far left: ");
Serial.print(scale1());
Serial.print(" Close left: ");
Serial.print(scale2());
Serial.print(" Close right: ");
Serial.print(scale3());
Serial.print(" Far right: ");
Serial.println(scale4());
}


