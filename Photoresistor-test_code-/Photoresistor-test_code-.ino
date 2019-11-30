int GPIO_PWM_A = 36;
int GPIO_ANALOG_A = 37;

void setup()   {                
  pinMode(GPIO_PWM_A, OUTPUT);
  Serial.begin(38400);
}

int i = 0;
int photoVoltageRaw = 0;

void loop()                     
{
  analogWrite(GPIO_PWM_A,i++);
  photoVoltageRaw = analogRead(GPIO_ANALOG_A);
  if (i == 255)
    i = 0;
  Serial.println(photoVoltageRaw);
  delay(500);
  
}
