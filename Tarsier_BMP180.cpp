/* 
* Tarsier_BMP180.cpp
*
* Created: 12/8/2017 4:27:22 PM
* Author: TARSIER
*/


#include "Tarsier_BMP180.h"
#include <Wire.h>
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

// default constructor
Tarsier_BMP180::Tarsier_BMP180()
{
} //Tarsier_BMP180

/*
*	intialize BMP180 connections
*	@return 1 if success, 0 if failed
*/
char Tarsier_BMP180::init(void)
{
	Wire.begin();
	_AC1 = bmp180ReadInt(CAL_AC1);
	_AC2 = bmp180ReadInt(CAL_AC2);
	_AC3 = bmp180ReadInt(CAL_AC3);
	_AC4 = bmp180ReadInt(CAL_AC4);
	_AC5 = bmp180ReadInt(CAL_AC5);
	_AC6 = bmp180ReadInt(CAL_AC6);
	_B1 = bmp180ReadInt(CAL_B1);
	_B2 = bmp180ReadInt(CAL_B2);
	_MB = bmp180ReadInt(CAL_MB);
	_MC = bmp180ReadInt(CAL_MC);
	_MD = bmp180ReadInt(CAL_MD);
	if(_AC1 && _AC2 && _AC3 && _AC4 && _AC5 && _AC6 &&
		_B1 && _B2 && _MB && _MC && _MD){
		readUT();
		readUP();
		return (1);
	}
	return (0);
}
// Read 1 byte from the BMP085 at 'address'
// Return: the read byte;
char Tarsier_BMP180::bmp180Read(unsigned char address)
{
	//Wire.begin();
	unsigned char data;
	Wire.beginTransmission(BMP180_ADDRESS);
	Wire.write(address);
	Wire.endTransmission();

	Wire.requestFrom(BMP180_ADDRESS, 1);
	while(!Wire.available());
	return Wire.read();
}
// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int Tarsier_BMP180::bmp180ReadInt(unsigned char address)
{
	unsigned char msb, lsb;
	Wire.beginTransmission(BMP180_ADDRESS);
	Wire.write(address);
	Wire.endTransmission();
	Wire.requestFrom(BMP180_ADDRESS, 2);
	while(Wire.available()<2);
	msb = Wire.read();
	lsb = Wire.read();
	return (int) msb<<8 | lsb;
}
// Read the uncompensated temperature value
unsigned int Tarsier_BMP180::readUT()
{
	unsigned char msb, lsb;
	unsigned int ut;
	Wire.beginTransmission(BMP180_ADDRESS);
	Wire.write(CONTROL);
	Wire.write(READ_TEMPERATURE);
	Wire.endTransmission();
	delay(5);
	ut = bmp180ReadInt(CONTROL_OUTPUT_MSB);	
	return ut;
}
// Read the uncompensated pressure value
unsigned long Tarsier_BMP180::readUP()
{
	unsigned char msb, lsb, xlsb;
	unsigned long up = 0;
	Wire.beginTransmission(BMP180_ADDRESS);
	Wire.write(CONTROL);
	Wire.write(READ_PRESSURE + (_OSS<<6));
	Wire.endTransmission();
	delay(2 + (3<<_OSS));

	// Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
	msb = bmp180Read(CONTROL_OUTPUT_MSB);
	lsb = bmp180Read(CONTROL_OUTPUT_LSB);
	xlsb = bmp180Read(CONTROL_OUTPUT_XLSB);
	up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-_OSS);
	return up;
}
void Tarsier_BMP180::writeRegister(int deviceAddress, byte address, byte val)
{
	Wire.beginTransmission(deviceAddress);	// start transmission to device
	Wire.write(address);					// send register address
	Wire.write(val);         // send value to write
	Wire.endTransmission();     // end transmission
}
int Tarsier_BMP180::readRegister(int deviceAddress, byte address)
{
	int v;
	Wire.beginTransmission(deviceAddress);
	Wire.write(address); // register to read
	Wire.endTransmission();

	Wire.requestFrom(deviceAddress, 1); // read a byte

	while(!Wire.available()) {
		// waiting
	}

	v = Wire.read();
	return v;
}
float Tarsier_BMP180::getAltitude(float pressure)
{
	
	//float A = pressure/101325;	
	float A = pressure/MSLP;
	float B = 1/5.25588;
	float C = pow(A,B);
	C = 1 - C;
	C = C /0.0000225577;
	return C;
}

float Tarsier_BMP180::getTemperature()
{
	return getTemperature(this-readUT());
}
float Tarsier_BMP180::getTemperature(unsigned int ut)
{
	long x1, x2;

	x1 = (((long)ut - (long)_AC6)*(long)_AC5) >> 15;
	x2 = ((long)_MC << 11)/(x1 + _MD);
	_PressureCompensate = x1 + x2;

	float temp = ((_PressureCompensate + 8)>>4);
	temp = temp /10;

	return temp;
}

long Tarsier_BMP180::getPressure()
{
	return getPressure(this->readUP());
}
long Tarsier_BMP180::getPressure(unsigned long up)
{
	long x1, x2, x3, b3, b6, p;
	unsigned long b4, b7;
	b6 = _PressureCompensate - 4000;
	x1 = (_B2 * (b6 * b6)>>12)>>11;
	x2 = (_AC2 * b6)>>11;
	x3 = x1 + x2;
	b3 = (((((long)_AC1)*4 + x3)<<_OSS) + 2)>>2;

	// Calculate B4
	x1 = (_AC3 * b6)>>13;
	x2 = (_B1 * ((b6 * b6)>>12))>>16;
	x3 = ((x1 + x2) + 2)>>2;
	b4 = (_AC4 * (unsigned long)(x3 + 32768))>>15;

	b7 = ((unsigned long)(up - b3) * (50000>>_OSS));
	if (b7 < 0x80000000)
	p = (b7<<1)/b4;
	else
	p = (b7/b4)<<1;

	x1 = (p>>8) * (p>>8);
	x1 = (x1 * 3038)>>16;
	x2 = (-7357 * p)>>16;
	p += (x1 + x2 + 3791)>>4;

	long temp = p;
	return temp;
}


float Tarsier_BMP180::calcPressureAtSeaLevel(long pressure, float altitude){	
	return(pressure/pow(1-(altitude/44330.0),5.255));
}

// default destructor
Tarsier_BMP180::~Tarsier_BMP180()
{
} //~Tarsier_BMP180
