#include "Arduino.h"
#include "ColourSensor.h"

ColourSensor2::ColourSensor2() {};

void ColourSensor2::update() {
	_i = 0, _sum = 0;
	
	//SerialEvent
	while (Serial2.available()) {   
		_Re_buf[_counter] = (unsigned char)Serial2.read();
		
		if(_counter == 0 && _Re_buf[0] != 0x5A) return;        
		
		_counter++;       
		
		if(_counter == 8) {    
			_counter = 0;
			_sign = 1;
		}      
	}
	
	if(_sign) {   
		_sign = 0;
		for(_i = 0; _i<7; _i++)
			_sum += _Re_buf[_i]; 
		if(_sum == _Re_buf[_i] ) {  	           
			_rgb[0] = _Re_buf[4];
			_rgb[1] = _Re_buf[5];
			_rgb[2] = _Re_buf[6];  
		} 
	} 
}

byte ColourSensor2::r() {return _rgb[0];}

byte ColourSensor2::g() {return _rgb[1];}

byte ColourSensor2::b() {return _rgb[2];}



ColourSensor3::ColourSensor3() {};

void ColourSensor3::update() {
	_i = 0, _sum = 0;
	
	//SerialEvent
	while (Serial3.available()) {   
		_Re_buf[_counter] = (unsigned char)Serial3.read();
		
		if(_counter == 0 && _Re_buf[0] != 0x5A ) return;        
		{	
		_counter++;       
		}
		
	if(_Re_buf[0] != 0x5A && _Re_buf[1] != 0x5A && _Re_buf[2] != 0x45) {
		if(_counter == 8) {    
			_counter = 0;
			_sign = 1;
		}      
	}
	
	if(_sign) {   
		_sign = 0;
		for(_i = 0; _i<7; _i++)
			_sum += _Re_buf[_i]; 
		if(_sum == _Re_buf[_i] ) {  	           
			_rgb[0] = _Re_buf[4];
			_rgb[1] = _Re_buf[5];
			_rgb[2] = _Re_buf[6];  
		} 
	} 
}

byte ColourSensor3::r() {return _rgb[0];}

byte ColourSensor3::g() {return _rgb[1];}

byte ColourSensor3::b() {return _rgb[2];}