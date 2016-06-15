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
  
        int sensorPin1 = A11;
        int sensorPin2 = A12;
        int sensorPin3 = A13;
        int sensorPin4 = A14;

	float max1 = 6.31, min1 = 4.45;
	float max2 = 6.57, min2 = 4.70;
	float max3 = 6.80, min3 = 4.98;
	float max4 = 6.32, min4 = 4.15;

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
