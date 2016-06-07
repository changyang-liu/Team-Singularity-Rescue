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
  
        int sensorPin1 = A14;
        int sensorPin2 = A13;
        int sensorPin3 = A12;
        int sensorPin4 = A11;

        // float range1 = 330 - 50;
        // float range2 = 816 - 120;
        // float range3 = 513 - 80;
        // float range4 = 870 - 160;

	
	float range1 = 6.45 - 4.75;
	float range2 = 6.79 - 5.23;
	float range3 = 6.26 - 4.44;
	float range4 = 6.76 - 4.72;


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
