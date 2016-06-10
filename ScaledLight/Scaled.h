#include "Arduino.h"

#ifndef Scaled_h
#define Scaled_h

class Scaled 
{
  public:
        Scaled ();	  
	int scale1();
	int scale2();
	int scale3 ();
	int scale4 ();
	void print ();
	void printlog();
  
        int sensorPin1 = A14;
        int sensorPin2 = A13;
        int sensorPin3 = A12;
        int sensorPin4 = A11;

	float max1 = 6.51, min1 = 4.32;
	float max2 = 6.83, min2 = 5.19;
	float max3 = 6.81, min3 = 5.65;
	float max4 = 6.62, min4 = 5.00;

        // float range1 = 330 - 50;
        // float range2 = 816 - 120;
        // float range3 = 513 - 80;
        // float range4 = 870 - 160;

	
	float range1 = max1 - min1;
	float range2 = max2 - min2;
	float range3 = max3 - min3;
	float range4 = max4 - min4;


        int scaled1;
        int scaled2;
        int scaled3;
        int scaled4;

        float raw1;
        float raw2;
        float raw3;
        float raw4;
};




#endif
