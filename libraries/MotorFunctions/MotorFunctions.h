#include <DualVNH5019MotorShield.h>
#include <initLib.h>

#ifndef MotorFunctions_h;
#define MotorFunctions_h;

class Motors{
	public:
		long currentPos1 = 0;
		long currentPos2 = 0;
		long previousPos1 = 0;
		long previousPos2 = 0; 
		float dtheta1;
		float dtheta2;
		
		void moveCounts(int motor1Speed, int motor2Speed, long counts);
		void moveDegs(int motor1Speed, int motor2Speed, int degs);
		void moveTime(int motor1Speed, int motor2Speed, long t);
		void stopIfFault();
		void constSpeeds(int spd);
};

#endif
