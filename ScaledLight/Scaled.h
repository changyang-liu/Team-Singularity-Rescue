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
  
        int sensorPin1 = A14;
        int sensorPin2 = A13;
        int sensorPin3 = A12;
        int sensorPin4 = A11;

        float range1 = 546 - 417;
        float range2 = 569 - 440;
        float range3 = 519 - 386;
        float range4 = 539 - 415;

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
