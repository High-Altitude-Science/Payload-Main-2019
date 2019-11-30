
#include <Wire.h>
//#include <i2c_t3.h>
#include <Adafruit_MPL3115A2.h>

//TODO This works, but we need to use the i2c_t3 library commented out above for multiple I2C connections.
//If we are using the i2c library, we will need to make our own version of the Adafruit_MPL3115A2 code so we can
//receive a different type of Wire object.

// Power by connecting Vin to 3-5V, GND to GND
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
// See the Wire tutorial for pinouts for each Arduino
// http://arduino.cc/en/reference/wire
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

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


void loop() {
  if (! baro.begin(&Wire)) {
    Serial.println("Couldnt find sensor");
    return;
  }
  
  float pascals = baro.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  Serial.print(pascals/3377); Serial.println(" Inches (Hg)");

  float altm = baro.getAltitude();
  Serial.print(altm); Serial.println(" meters");

  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println("*C");

  delay(1000);
}
