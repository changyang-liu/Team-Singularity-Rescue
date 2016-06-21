#ifndef PIDe_h
#define PIDe_h

#include <DualVNH5019MotorShield.h>
#include <Arduino.h>

class PIDe_Array {
	public:
		PIDe_Array(DualVNH5019MotorShield md, float kp, float ki, float kd_ratio, float maximum, float gradient, float half_x);
		void track(int far_left, int close_left, int close_right, int far_right);
		void debug();
		
		const double e = 2.71828;
		
	private:
		DualVNH5019MotorShield _md;
	
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
		float _kd_ratio;
		
		float _speed1, _speed2;
};

class PIDe_Single{
	public:
		PIDe_Single(DualVNH5019MotorShield md, int base, int p);
		void track(int side, int close_left, int close_right);
		
	private:
		DualVNH5019MotorShield _md;
		
		int _close_left, _close_right;
		int _base, _p, _side;
		
		long start_time;
		
		float _error;
		float _turn;
		
		float _speed1, _speed2;
		
};

#endif
