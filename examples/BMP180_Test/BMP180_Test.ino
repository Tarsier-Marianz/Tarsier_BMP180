#include "Wire.h"
#include "Tarsier_BMP180.h"



float _temperature;
float _pressure;
float _atm;
float _altitude;
float _sealevel;

Tarsier_BMP180 _bmp;

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Serial success...");
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  if(!_bmp.init()){
	  Serial.println("BMP180 connection failed...");
  }

}

void loop() {
  _temperature	= _bmp.getTemperature(_bmp.readUT());	//Get the temperature, ReadUT MUST be called first to start temperature measurement
  _pressure		= _bmp.getPressure(_bmp.readUP());			//Get the calculated pressure
  _altitude		= _bmp.getAltitude(_pressure);				//calculate absolute uncompensated altitude - in Meters
  _sealevel		= _bmp.calcPressureAtSeaLevel(_pressure, _altitude);
  _atm			= _pressure / 1013.25;

  Serial.print("Temperature: ");
  Serial.print(_temperature, 2);						//display 2 decimal places
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(_pressure, 0);							//whole number only.
  Serial.println(" Pa");

  Serial.print("Pressure At Sea level: ");
  Serial.print(_sealevel, 2);							//display 2 decimal places
  Serial.println(" Pa");

  Serial.print("Ralated Atmosphere: ");
  Serial.println(_atm, 4);								//display 4 decimal places

  Serial.print("Altitude: ");
  Serial.print(_altitude, 2);							//display 2 decimal places
  Serial.println(" m");

  Serial.println();
  delay(1000);

}