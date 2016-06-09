#include "Arduino.h"
#include "Scaled.h"


Scaled::Scaled() 
{}

/*
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
*/

int Scaled::scale1()
{
  raw1 = log(analogRead(sensorPin1)); 
  scaled1 = ((raw1-4.75)/range1) * 100;  
	if (scaled1 <0) {scaled1=0;}
	if (scaled1 >100) {scaled1=100;}


return scaled1;
}

int Scaled::scale2()
{
raw2 = log(analogRead(sensorPin2));
  scaled2 = ((raw2-5.23)/range2) * 100;
if (scaled2 <0) {scaled2=0;}
	if (scaled2 >100) {scaled2=100;}

return scaled2;
}

int Scaled::scale3()
{
  raw3 = log(analogRead(sensorPin3));
  scaled3 = ((raw3-4.44)/range3) * 100;
if (scaled3 <0) {scaled3=0;}
	if (scaled3 >100) {scaled3=100;}

return scaled3;
}

int Scaled::scale4()
{ 
 raw4 = log(analogRead(sensorPin4));
  scaled4 = ((raw4-4.72)/range4) * 100;
if (scaled4 <0) {scaled4=0;}
	if (scaled4 >100) {scaled4=100;}

return scaled4;
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


