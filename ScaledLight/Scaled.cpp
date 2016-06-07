#include "Arduino.h"
#include "Scaled.h"


Scaled::Scaled() 
{}


int Scaled::scale1()
{
  raw1 = analogRead(sensorPin1); 
  scaled1 = ((raw1-80)/range1) * 100;  
	if (scaled1 <0) {scaled1=0;}


return scaled1;
}

int Scaled::scale2()
{
raw2 = analogRead(sensorPin2);
  scaled2 = ((raw2-200)/range2) * 100;

return scaled2;
}

int Scaled::scale3()
{
  raw3 = analogRead(sensorPin3);
  scaled3 = ((raw3-170)/range3) * 100;

return scaled3;
}

int Scaled::scale4()
{ 
 raw4 = analogRead(sensorPin4);
  scaled4 = ((raw4-185)/range4) * 100;

return scaled4;
}



