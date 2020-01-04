#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
// Power by connecting Vin to 3-5V, GND to GND
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
// See the Wire tutorial for pinouts for each Arduino
// http://arduino.cc/en/reference/wire
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();


//Thermometer with thermistor
/*thermistor parameters:
 * RT0: 10 000 Ω
 * B: 3977 K +- 0.75%
 * T0:  25 C
 * +- 5%
 */

//These values are in the datasheet
#define RT0 50000   // Ω
#define B 3952      // K
//--------------------------------------


#define VCC 3.3    //Supply voltage
#define R 10000  //R=50KΩ

#define THERMISTORSCOUNT 4
#define PHOTORESISTORCOUNT 4

//Variables
float RT, VR, ln, TX, T0, VRT;
float arrRT[5], arrVR[5], arrln[5], arrTX[5], arrVRT[5];
int thermistorPins[] = {A16, A17, A18, A19, A20};

int photoresistorPins[] = {A12, A13, A14, A15};

int pinBaroSDA = 4; 
int pinBaroSCL = 3;



void setup() {
  Serial.begin(9600);
  analogReadResolution(10);
  T0 = 25 + 273.15;                 //Temperature T0 from datasheet, conversion from Celsius to kelvin
  Wire2.setSDA(pinBaroSDA);
  Wire2.setSCL(pinBaroSCL);
  Wire2.begin();
  delay(1000);
  Serial.println("Begin!");
}

void loop() {
  readThermistors();
  readPhotoresistors();
  readBarometer();

  unsigned long time;
  Serial.print("Time: ");
  time = millis();
  Serial.println(time);

  delay(500);

}

void readThermistors(){
  for (int i = 0; i < THERMISTORSCOUNT; i++){
    arrVRT[i] = analogRead(thermistorPins[i]);
    arrVRT[i] = (3.30 / 1023.00) * arrVRT[i];
    arrVR[i] = VCC - arrVRT[i];
    arrRT[i] = arrVRT[i] / (arrVR[i] / R);               
  
    arrln[i] = log(arrRT[i] / RT0);
    arrTX[i] = (1 / ((arrln[i] / B) + (1 / T0)));
    arrTX[i] = arrTX[i] - 273.15;

    Serial.printf("Temperature %d: ", i);
    Serial.print(arrTX[i]);
    Serial.print("C\t");
  }
  Serial.println();
}

void readPhotoresistors(){
  for (int i = 0; i < PHOTORESISTORCOUNT; i++){
    Serial.printf("Photoresistor %d: %d\t", i, analogRead(photoresistorPins[i]));
  }
  Serial.println();
}

void readBarometer(){
  if (! baro.begin(&Wire2)) {
    Serial.println("Couldnt find sensor");
    delay(1000);
    return;
  }
  
  float pascals = baro.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  float altm = baro.getAltitude();
  float tempC = baro.getTemperature();
  
  Serial.printf("%.2f Inches (Hg)\t%.2f meters\t%.2f C\n", pascals/3377, altm, tempC);
}
