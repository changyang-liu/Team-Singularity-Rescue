#include "Arduino.h"
#include <PIDe.h>
#include <DualVNH5019MotorShield.h>

PIDe_Array::PIDe_Array(DualVNH5019MotorShield md, float kp, float ki, float kd_ratio, float maximum, float gradient, float half_x) {
	_md = md;
	
	_kp = kp;
	_ki = ki;
	_kd_ratio = kd_ratio;
	
	_maximum = maximum;
	_gradient = gradient;
	_half_x = half_x;
}

void PIDe_Array::track(float far_left, float close_left, float close_right, float far_right) {
	_far_left = far_left;
	_close_left = close_left;
	_close_right = close_right;
	_far_right = far_right;

	_left_ave = (_far_left + _close_left*1.5)/2.5;
	_right_ave = (_far_right + _close_right*1.5)/2.5;
	
	_error = _left_ave - _right_ave;
	_integral = _integral*_integral_factor + _error;
	_derivative = _error - _prev_error;
	
	// if (abs(_integral) > _max_integral) {
		// if (_integral >= 0) {
			// _integral = _max_integral;
		// } else {
			// _integral = -_max_integral;
		// }
	// }
	
	
	_turn = _kp*_error + _ki*_integral + _variable_speed*_kd_ratio*_derivative;
	_variable_speed = _maximum/(1+pow(e,_gradient*(abs(_turn)-_half_x)));
	
	_prev_error = _error;
	
	_speed1 = _variable_speed + _turn;
	_speed2 = _variable_speed - _turn;
	
	_md.setSpeeds(_speed1,_speed2);
}

void PIDe_Array::setMaxSpeed(int max_speed) {
	_maximum = max_speed;
}

void PIDe_Array::debug() {
	Serial.print("Error: ");
	Serial.print(_error);
	Serial.print(" Turn: ");
	Serial.print(_turn);
	Serial.print(" Variable speed: ");
	Serial.println(_variable_speed);
}

PIDe_Single::PIDe_Single(DualVNH5019MotorShield md, int base, int p) {
	_md = md;
	_base = base;
	_p = p;
}

void PIDe_Single::track(int side, float close_left, float close_right) {
	_close_left = close_left;
	_close_right = close_right;
	
	if (side == 1) {
		_error = _close_left-50;
		_turn = _error*_p;
		
		_speed1 = _base + _turn;
		_speed2 = _base - _turn;
	} else if (side == 2) {
		_error = close_right-50;
		_turn = _error*_p;
		
		_speed1 = _base - _turn;
		_speed2 = _base + _turn;
	}
	
	_md.setSpeeds(_speed1,_speed2);
}