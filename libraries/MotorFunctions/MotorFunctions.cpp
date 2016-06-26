#include <DualVNH5019MotorShield.h>
#include <MotorFunctions.h>
#include <Servo.h>

Motors::Motors(DualVNH5019MotorShield md, Servo myservo) {
	_md = md;
  _myservo = myservo;
	
	pinMode(_encoder1PinA, INPUT);
	digitalWrite(_encoder1PinA, HIGH);
	pinMode(_encoder2PinA, INPUT);
	digitalWrite(_encoder2PinA, HIGH); 
	pinMode(_encoder1PinB, INPUT);
	digitalWrite(_encoder1PinB, HIGH);
	pinMode(_encoder2PinB, INPUT);
	digitalWrite(_encoder2PinB, HIGH);
	
	_PastA1 = (boolean)digitalRead(_encoder1PinA); //initial value of channel A;
	_PastA2 = (boolean)digitalRead(_encoder2PinA); //initial value of channel A;
	_PastB1 = (boolean)digitalRead(_encoder1PinB); //and channel B
	_PastB2 = (boolean)digitalRead(_encoder2PinB); //and channel B
}

void Motors::moveCounts(int motor1Speed, int motor2Speed, long counts){
	_motor1Speed = motor1Speed;
	_motor2Speed = motor2Speed;
	_counts = counts;
  _servoPos = _myservo.read();
	
	encoder1Pos = 0;
	encoder2Pos = 0;
	
	if (_motor1Speed == 0) {
		while(abs(encoder2Pos) < _counts) {
      _myservo.write(_servoPos);
			_md.setSpeeds(_motor1Speed, _motor2Speed);
		}
	} else {
		while(abs(encoder1Pos) < _counts) {
      _myservo.write(_servoPos);
			_md.setSpeeds(_motor1Speed, _motor2Speed);
		}
	}
	
	_md.setBrakes(400,400);
	delay(200);
	encoder1Pos = 0;
	encoder2Pos = 0;
}

void Motors::moveDegs(int motor1Speed, int motor2Speed, int degs){
	_motor1Speed = motor1Speed;
	_motor2Speed = motor2Speed;
	_degs = degs;
	
	encoder1Pos = 0;
	encoder2Pos = 0;
	
	if (_motor1Speed == 0) {
		_deg_counts = (long) (_degs*55/36+(100-_motor2Speed)*0.3);
		while(abs(encoder1Pos) < _deg_counts) {
			_md.setSpeeds(_motor1Speed,_motor2Speed);
		}
	} else {
		_deg_counts = (long) (_degs*55/36+(100-motor1Speed)*0.3);
		while(abs(encoder1Pos) < _deg_counts) {
			_md.setSpeeds(_motor1Speed,_motor2Speed);
		}
	}
	_md.setBrakes(400,400);
	delay(200);
	encoder1Pos = 0;
	encoder2Pos = 0;
}

void Motors::moveTime(int motor1Speed, int motor2Speed, long t){
	_motor1Speed = motor1Speed;
	_motor2Speed = motor2Speed;
	_t = t;
  _servoPos = _myservo.read();
	
	_start_time = millis();
	
	while (millis() - _start_time < _t) {
    _myservo.write(_servoPos);
		_md.setSpeeds(_motor1Speed,_motor2Speed);
	}
	
	_md.setBrakes(400,400);
}

 void Motors::constSpeeds(int spd){
	_spd = spd;
	
	_motor1Float = _spd;
	_motor2Float = _spd;
	
	_currentPos1 = abs(encoder1Pos);
	_currentPos2 = abs(encoder2Pos);
	
	_dtheta1 = _currentPos1 - _previousPos1;
	_dtheta2 = _currentPos2 - _previousPos2;
	
	_previousPos1 = _currentPos1;
	_previousPos2 - _currentPos2;
	
	if (_dtheta1 == 0) {_dtheta1 = 1;}
	if (_dtheta2 == 0) {_dtheta2 = 1;}
	
	_motor2Speed = (_dtheta1/_dtheta2) * _motor2Speed;
	_md.setSpeeds(_motor1Speed, _motor2Speed);
	
	delay(50);
}

void Motors::stopIfFault()
{
  if (_md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (_md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

int Motors::getEncoder1PinA() {return _encoder1PinA;}
int Motors::getEncoder1PinB() {return _encoder1PinB;}
int Motors::getEncoder2PinA() {return _encoder2PinA;}
int Motors::getEncoder2PinB() {return _encoder2PinB;}

boolean Motors::getPastB1() {return _PastB1;}
boolean Motors::getPastB2() {return _PastB2;}

void Motors::setPastB1(boolean b1) {_PastB1 = b1;}
void Motors::setPastB2(boolean b2) {_PastB2 = b2;}

void Motors::addEncoder1() {encoder1Pos++;}
void Motors::addEncoder2() {encoder2Pos++;}
void Motors::subtrEncoder1() {encoder1Pos--;}
void Motors::subtrEncoder2() {encoder2Pos--;}



