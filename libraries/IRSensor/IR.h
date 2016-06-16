#include <Arduino.h>

#ifndef IR_h
#define IR_h

class SharpIR
{
  public:
	SharpIR(int ir_pin);
	const double e =  2.71828;
double distance();

  private:
	int ir;
	int model = 1080;
	int delayTime = 0;
	int samples = 50;
	int maxDiff = 17;
	int i;

  	double total = 0;
 	double prev;
  	double ave;
   	double dist;
    	double raw;

};

#endif