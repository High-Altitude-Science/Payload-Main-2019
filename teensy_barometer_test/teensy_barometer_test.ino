
#include <Wire.h>

int address1 = 72;
int pin_SDA = 18; 
int pin_SCL = 19; 

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  Wire.setSDA(pin_SDA);
  Wire.setSCL(pin_SCL);
  delay(1000);
  Serial.println("Begin!");
}

void loop() 
{
  int c;

  Wire.beginTransmission(address1);  
  Wire.write(0);  
  Wire.endTransmission();  
  Wire.requestFrom(address1, 2);
  
  c = Wire.receive();
  
  Serial.print("Temperature: ");
  Serial.println(c);
  delay(1000);
}
