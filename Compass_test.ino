#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
 
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);


 float startPosition;
 float zValue;

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Magnetometer Test"); Serial.println("");

  if(!mag.begin())
  {

    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  sensors_event_t event; 
  mag.getEvent(&event);
  float Pi = 3.14159;
  //startPosition = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  startPosition = 0;
  zValue = event.magnetic.z;
}

  
  
void loop(void) 
{
//magnetic reading
  sensors_event_t event;
  sensors_event_t event1;
  accel.getEvent(&event1); 
  mag.getEvent(&event);

//accelerometer reading 


 //z deviation from -9.81ms^2 -> lower means tilted, 0.00 when vertical
  float tilt = 9.81 + (event1.acceleration.z);
  
  float Pi = 3.14159;
  
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;

  float relativeHeading = startPosition - heading;
  
  if (relativeHeading < 0)
  {
    relativeHeading = 360 + startPosition - heading;
  }
  float zDiff = zValue - event.magnetic.z;
  Serial.print("Compass Heading: ");
  Serial.println(relativeHeading);
  delay(200);
}
