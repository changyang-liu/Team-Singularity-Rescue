#include "Arduino.h"

#ifndef Scaled_h
#define Scaled_h

class Scaled 
{
  public:
        Scaled ();	  
	float scale1();
	float scale2();
	float scale3 ();
	float scale4 ();
	void print ();
	void printlog();
  
        int sensorPin1 = A11;
        int sensorPin2 = A12;
        int sensorPin3 = A13;
        int sensorPin4 = A14;

	float max1 = 6.45, min1 = 5.50;
	float max2 = 6.65, min2 = 5.80;
	float max3 = 6.30, min3 = 5.35;
	float max4 = 6.35, min4 = 5.55;

	
	float range1 = max1 - min1;
	float range2 = max2 - min2;
	float range3 = max3 - min3;
	float range4 = max4 - min4;


        float scaled1;
        float scaled2;
        float scaled3;
        float scaled4;

        float raw1;
        float raw2;
        float raw3;
        float raw4;
};




#endif
