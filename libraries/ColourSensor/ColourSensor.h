#ifndef ColourSensor_h
#define ColourSensor_h

#include <Arduino.h>

class ColourSensor2 {
	public:
		ColourSensor2();
	
		int green();
		int _r,_g,_b;
		int rgbV();
		int _rV,_gV,_bV;
		
	private:
		boolean _end = false;
		boolean _first = true;

		int _readings[8] = {0};
		int _counter = 0;
		long _persent;
		
		

};

class ColourSensor3 {
	public:
		ColourSensor3();
	
		int green();
		int _r,_g,_b;
		int rgbV();	
		int _rV,_gV,_bV;		
	private:
		boolean _end = false;
		boolean _first = true;
		int _readings[8] = {0};
		int _counter = 0;
		long _persent;
		
		

};
#endif