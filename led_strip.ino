#define sensorPin A0

int samples = 30;
int led = 9;  
int brightness = 100;
int fadeAmount = 1;

float rawRange = 1024; // 3.3v
float logRange = 5.0; // 3.3v = 10^5 lux


void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL);
  pinMode(led, OUTPUT);
  pinMode(sensorPin, INPUT);
  
}
void loop() {
  float ave = 0;
  for (int i=1; i <= samples; i++) {
    int sensorRaw = analogRead(sensorPin);
    ave += sensorRaw;
  }

  analogWrite(led, brightness);
//  brightness += fadeAmount;
//  if (brightness > 154 || brightness < 1) {
//    fadeAmount = -fadeAmount;
//  }
  delay(10);
  Serial.println(RawToLux(ave/samples));
}

float RawToLux(float raw) {
  float logLux = raw * logRange / rawRange;
  return pow(10, logLux);
}

