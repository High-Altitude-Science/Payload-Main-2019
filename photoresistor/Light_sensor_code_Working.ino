int sensorPin = A18;   // select the analog input pin for the photoresistor

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(analogRead(sensorPin));
  delay(10);
  
  unsigned long time;
  
  Serial.print("Time: ");
  time = millis();

  Serial.println(time); //prints time since program started
  delay(10);          // wait a second so as not to send massive amounts of data
}
