int GPIO_PWM_A = 35;
int GPIO_ANALOG_A = 38;

void setup()   {                
  pinMode(GPIO_PWM_A, OUTPUT);
  Serial.begin(38400);
}

int i = 0;
int tempVoltageRaw = 0;

void loop()                     
{
  analogWrite(GPIO_PWM_A,i++);
  tempVoltageRaw = analogRead(GPIO_ANALOG_A);
  if (i == 255)
    i = 0;
  Serial.println(tempVoltageRaw);
  delay(500);
  
}
