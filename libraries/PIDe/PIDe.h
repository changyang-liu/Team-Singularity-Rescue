#ifndef PIDe_h
#define PIDe_h

#include <Arduino.h>

class PIDe_Array {
	public:
		PIDe_Array(float kp, float ki, float maximum, float gradient, float half_x);
		void update(int far_left, int close_left, int close_right, int far_right);
		float speed1();
		float speed2();
		void debug();
		
		const double e = 2.71828;
		
	private:
		int _far_left, _close_left, _close_right, _far_right;
		float _left_ave, _right_ave;
		float _error, _turn;
		
		float _kp;
		float _ki;
		
		float _maximum;
		float _gradient;
		float _half_x;
		
		float _prev_error = 0;
		float _integral = 0;
		float _derivative = 0;
		
		int _max_integral = 10000;
		float _integral_factor = 0.8;
			
		float _variable_speed;
		float _kd_ratio = 1;
		
		float _speed1, _speed2;
};

class PIDe_Single{
	public:
		PIDe_Single(int base, int p);
		void update(int side, int close_left, int close_right);
		float speed1();
		float speed2();
		
	private:
		int _close_left, _close_right;
		int _base, _p;
		
		float _error;
		float _turn;
		
		float _speed1, _speed2;
		
};

#endif
