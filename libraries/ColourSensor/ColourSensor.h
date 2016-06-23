#ifndef ColourSensor_h
#define ColourSensor_h

#include <Arduino.h>

class ColourSensor2 {
	public:
		ColourSensor2();
	
		int green();
		
	private:
		boolean _end = false;
		boolean _first = true;
		int _r,_g,_b;
		int _readings[7] = {0};
		int _counter = 0;
		long _persent;
		
		

};

class ColourSensor3 {
	public:
		ColourSensor3();
	
		int green();
		
	private:
		boolean _end = false;
		boolean _first = true;
		int _r,_g,_b;
		int _readings[7] = {0};
		int _counter = 0;
		long _persent;
		
		

};
#endif