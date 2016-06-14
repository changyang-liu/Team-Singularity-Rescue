#include <Arduino.h>

#ifndef initLib_h
#define initLib_h

class initialization {
  public:
  initialization();
  void doEncoderA1();
  void doEncoderA2();
  void doEncoderB1();
  void doEncoderB2();
  void initialize();

  int encoder1PinA = 18;
  int encoder1PinB = 19;
  int encoder2PinA = 20;
  int encoder2PinB = 21;
    



  volatile long encoder1Pos = 0, encoder2Pos = 0;
  volatile boolean PastA1 = 0, PastA2 = 0, PastB1 = 0, PastB2 = 0;
  int buttonPin = 53, greenPin = 51, redPin = 50, touchSensorPin = 52, foilPin = 27;
};

#endif

