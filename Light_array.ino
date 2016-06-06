
int sensorPin1 = A14;
int sensorPin2 = A13;
int sensorPin3 = A12;
int sensorPin4 = A11;

int range1 = 545 - 417   //white value minus black value (to find the range of the sensor)
int range2 = 565 - 440
int range3 = 516 - 386
int range4 = 537 - 415

float rawRange = 1024; // 3.3v          //conversion to lux - not important as of now
float logRange = 5.0; // 3.3v = 10^5 lux   

void setup() 
{//
  Serial.begin(9600);
  pinMode(22, OUTPUT);      //power the sensors, which are connected to digital pins 22, 23, 24, 25
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  digitalWrite(22, HIGH);
  digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);
  digitalWrite(25, HIGH);
}
 
void loop() 
{
  int raw1 = analogRead(sensorPin1);   //read values from sensor
  int raw2 = analogRead(sensorPin2);
  int raw3 = analogRead(sensorPin3);
  int raw4 = analogRead(sensorPin4);

  int scaled1 = ((raw1-417)/range1) * 100    //scale it to 0-100 - get the disparity between reading and black
  int scaled2 = ((raw2-440)/range1) * 100
  int scaled3 = ((raw3-386)/range1) * 100
  int scaled4 = ((raw4-415)/range1) * 100
  
  Serial.print(" 1:");
  Serial.print(scaled1);
  Serial.print(" 2: ");
  Serial.print(scaled2);
  Serial.print(" 3: ");
  Serial.print(scaled3);
  Serial.print(" 4: ");
  Serial.println(scaled4);
  //Serial.print(" - Lux = ");
  //Serial.println(RawToLux(rawValue1)); 
  delay(500);
}
 
float RawToLux(int raw)                         //conversion to lux
{
  float logLux = raw * logRange / rawRange;
  return pow(10, logLux);
}
