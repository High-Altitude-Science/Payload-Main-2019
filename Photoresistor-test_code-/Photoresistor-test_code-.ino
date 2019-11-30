int GPIO_ANALOG_A = 37;

void setup()   {
  Serial.begin(38400);
}

int photoVoltageRaw = 0;

void loop()                     
{
  photoVoltageRaw = analogRead(GPIO_ANALOG_A);
  Serial.println(photoVoltageRaw);
  delay(500);
  
}
