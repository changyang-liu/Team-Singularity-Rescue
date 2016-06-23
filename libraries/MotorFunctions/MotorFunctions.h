#ifndef MotorFunctions_h
#define MotorFunctions_h

#include <DualVNH5019MotorShield.h>
#include <Arduino.h>

class Motors{
	public:
		Motors(DualVNH5019MotorShield md); 
		
		void stopIfFault();
		
		void moveCounts(int motor1Speed, int motor2Speed, long counts);
		void moveDegs(int motor1Speed, int motor2Speed, int degs);
		void moveTime(int motor1Speed, int motor2Speed, long t);
		void constSpeeds(int spd);
		
		int getEncoder1PinA();
		int getEncoder1PinB();
		int getEncoder2PinA();
		int getEncoder2PinB();
		
		boolean getPastB1();
		boolean getPastB2();
		
		void setPastB1(boolean b1);
		void setPastB2(boolean b2);
		
		void addEncoder1();
		void addEncoder2();
		void subtrEncoder1();
		void subtrEncoder2();
		
		volatile long encoder1Pos = 0, encoder2Pos = 0;
		
		
	private:
		long _currentPos1 = 0;
		long _currentPos2 = 0;
		
		long _previousPos1 = 0;
		long _previousPos2 = 0;
		
		int _encoder1PinA = 18;
		int _encoder1PinB = 19;
		int _encoder2PinA = 51;
		int _encoder2PinB = 53;
		
		volatile boolean _PastA1 = 0, _PastA2 = 0, _PastB1 = 0, _PastB2 = 0;
		
		DualVNH5019MotorShield _md;
		int _motor1Speed, _motor2Speed;
		long _counts;
		
		int _degs;
		unsigned long _deg_counts;
		
		long _t;
		long _start_time;
		
		int _spd;
		float _motor1Float, _motor2Float;
		
		float _dtheta1;
		float _dtheta2;
		
};

#endif
