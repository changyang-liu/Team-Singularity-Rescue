#include "Arduino.h"
#include "PIDe.h"

PIDe_Array::PIDe_Array(float kp, float ki, float maximum, float gradient, float half_x) {
	_kp = kp;
	_ki = ki;
	
	_maximum = maximum;
	_gradient = gradient;
	_half_x = half_x;
}

void PIDe_Array::updateLight(int far_left, int close_left, int close_right, int far_right) {
	_far_left = far_left;
	_close_left = close_left;
	_close_right = close_right;
	_far_right = far_right
}

pair<int,int> PIDe_Array::speeds() {
	_left_ave = (_far_left + _close_left)/2;
	_right_ave = (_far_right + _close_right)/2;
	
	_error = _left_ave - _right_ave;
	_integral = _integral*_integral_factor + _error;
	_derivative = _error - _prev_error;
	
	if (abs(_integral) > _max_integral) {
		if (_integral >= 0) {
			_integral = _max_integral;
		} else {
			_integral = -_max_integral;
		}
	}
	
	_variable_speed = _maximum/(1+pow(e,_gradient*(abs(_error)-_half_x)));
	_turn = _kp*_error + _ki*_integral + _variable_speed*_kd_ratio*_derivative;
	
	_prev_error = _error;
	
	return make_pair(_variable_speed + _turn, _variable_speed - _turn);
}

PIDe_Single::PIDe_Single(int base, int p, int t) {
	_base = base;
	_p = p;
	_t = t;
}

void PIDe_Single::updateLight(int close_left, int close_right) {
	_close_left = close_left;
	_close_right = close_right;
}

pair<int, int> PIDe_Array::speeds(int side) {
	_start_time = millis();
	
	while(millis() - _start_time < _t) {
		if (side == 1) {
			_error = _close_left-50;
			_turn = _error*_p;
			
			return make_pair(_base + _turn, _base - _turn);
		} else if (side == 2) {
			_error = close_right-50;
			_turn = _error*_p;
			
			return make_pair(_base - _turn, _base + _turn);
		} else {
			return make_pair(0,0);
		}
	}

}
