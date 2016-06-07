int leftPin = A0; // select the input pin for the potentiometer
int rightPin = A1;
float rawRange = 1024; // 3.3v
float logRange = 5.0; // 3.3v = 10^5 lux

void setup() {
  analogReference(EXTERNAL); //
  Serial.begin(9600);
  Serial.println("Hello Serial");
}

void loop() {
  // read the raw value from the sensor:
  float rightRaw = analogRead(rightPin);
  float leftRaw = analogRead(leftPin);
  Serial.print("Left: ");
  Serial.print(RawToLux(leftRaw));
  Serial.print(" Right: " );
  Serial.println(RawToLux(rightRaw));
  delay(200);
}

float RawToLux(float raw) {
  float logLux = raw * logRange / rawRange;
  return pow(10, logLux);
}

