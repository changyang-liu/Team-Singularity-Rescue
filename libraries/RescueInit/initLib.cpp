#include <Arduino.h>
#include <initLib.h>
#include <DualVNH5019MotorShield.h>

Initialization::Initialization() {}


void Initialization::initialize () {
  pinMode(buttonPin,INPUT_PULLUP);
  pinMode(touchSensorPin, INPUT_PULLUP);
  pinMode(foilPin, INPUT);

  Serial.begin(9600);
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

int Initialization::button()
{
	return digitalRead(buttonPin);
}

