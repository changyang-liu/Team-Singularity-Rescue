#include <Arduino.h>
#include <initLib.h>
#include <DualVNH5019MotorShield.h>

initialization::initialization() {}


void initialization::initialize () {
   pinMode(buttonPin,INPUT);
  pinMode(touchSensorPin, INPUT_PULLUP);
  pinMode(foilPin, INPUT);

  pinMode(greenPin,OUTPUT);
  pinMode(redPin, OUTPUT);
  
  
  pinMode(encoder1PinA, INPUT); //turn on pullup resistor
  digitalWrite(encoder1PinA, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  pinMode(encoder2PinA, INPUT);
  digitalWrite(encoder2PinA, HIGH); 
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB, HIGH);
  pinMode(encoder2PinB, INPUT);
  digitalWrite(encoder2PinB, HIGH);
  
  PastA1 = (boolean)digitalRead(encoder1PinA); //initial value of channel A;
  PastA2 = (boolean)digitalRead(encoder2PinA); //initial value of channel A;
  PastB1 = (boolean)digitalRead(encoder1PinB); //and channel B
  PastB2 = (boolean)digitalRead(encoder2PinB); //and channel B

//To Speed up even more, you may define manually the ISRs
   
  Serial.begin(115200);
  Serial2.begin(9600);
  Serial3.begin(9600);
  delay(1);
  Serial2.write(0XA5); 
  Serial2.write(0X81);    //8 - bit7 = 1; 1 - bit0 = 1
  Serial2.write(0X26);    //Sum of A5 and 81 (for verification)
  Serial3.write(0XA5); 
  Serial3.write(0X81); 
  Serial3.write(0X26);
}

int initialization::state() {
	_reading = digitalRead(buttonPin);
	
	if (!_reading) {
		_time_start = millis();
		_once = false;
	} else {
		if (millis() - _time_start > _debounce && !_once) {
			(_state)? _state = LOW:_state = HIGH;
			_once = true;
		}
	}
	
	return _state;
}

