#ifndef ColourSensor_h
#define ColourSensor_h

#include <Arduino.h>

class ColourSensor2 {
	public:
		ColourSensor2();
	
		void update();
		
		byte r();
		byte g();
		byte b();
		
	private:
		unsigned char _i=0,_sum=0;
	
		unsigned char _Re_buf[11], _counter = 0;
		unsigned char _sign = 0;
		byte _rgb[3] = {0};
};

class ColourSensor3 {
	public:
		ColourSensor3();
		
		void update();
		
		byte r();
		byte g();
		byte b();
		
	private:
		unsigned char _i=0,_sum=0;
	
		unsigned char _Re_buf[11], _counter = 0;
		unsigned char _sign = 0;
		byte _rgb[3] = {0};
};

#endif