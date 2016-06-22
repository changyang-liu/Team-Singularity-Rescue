#include <Servo.h>

Servo myservo; 


void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  servoPos(140);
  delay(1000);
  servoPos(0);
}

void loop() {
  //a
}


void servoPos(int newPos){
  int currentPos = myservo.read();
  if(currentPos < newPos){
    for (int pos = currentPos; pos < newPos; pos++) { 
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(30);
    }   
  }else{
    for (int pos = currentPos; pos > newPos; pos--) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      if(pos > 60){
        delay(30);  
      }else{
        delay(10);
      }
    }
  }
} 

