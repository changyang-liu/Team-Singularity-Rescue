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

				if (_persent > 3800 && abs(_g - _r) >20 && abs(_g - _b) >20) {
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

				if (_persent > 3800 && abs(_g - _r) >20 && abs(_g - _b) >20) {
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