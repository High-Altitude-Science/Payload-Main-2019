int sensorPin = A18;   // select the analog input pin for the photoresistor

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(analogRead(sensorPin));
  delay(200);
}
