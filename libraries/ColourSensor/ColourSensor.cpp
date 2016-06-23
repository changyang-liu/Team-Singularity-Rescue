#include "Arduino.h"
#include "ColourSensor.h"

ColourSensor2::ColourSensor2() {};

int ColourSensor2::green() {
	int result;
	_end = false;
	_first = true;
	while (!_end) {
		while (!Serial2.available()) {int useless = 0;}
		int temp = Serial2.read();
		if (temp == 3) {
			if (_counter == 3 && !_first) {
				_r = _readings[4];
				_g = _readings[5];
				_b = _readings[6];
				
				_persent = (((long)_g*10000)/(_r+_g+_b));

				if (_persent > 3700) {
					result = 1;
				} else {
					result = 0;
				}
				
				_end = true;
			} else {
				_counter = 3;
			}
		}
		
		if (temp > 0 && temp < 256) {
			if (_counter == 8) {
				_counter = 0;
				_first = false;
			}
			_readings[_counter] = temp;
			++_counter;
		}
	}
	
	return result;
	
}

ColourSensor3::ColourSensor3() {};

<<<<<<< HEAD
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
=======
int ColourSensor3::green() {
	int result;
	_end = false;
	_first = true;
	while (!_end) {
		while (!Serial3.available()) {int useless = 0;}
		int temp = Serial3.read();
		if (temp == 3) {
			if (_counter == 3 && !_first) {
				_r = _readings[4];
				_g = _readings[5];
				_b = _readings[6];
				
				_persent = (((long)_g*10000)/(_r+_g+_b));

				if (_persent > 3700) {
					result = 1;
				} else {
					result = 0;
				}
				
				_end = true;
			} else {
				_counter = 3;
			}
		}
		
		if (temp > 0 && temp < 256) {
			if (_counter == 8) {
				_counter = 0;
				_first = false;
			}
			_readings[_counter] = temp;
			++_counter;
		}
>>>>>>> origin/master
	}
	
	return result;
	
}