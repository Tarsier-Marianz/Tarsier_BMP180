/* 
* Tarsier_BMP180.h
*
* Created: 12/8/2017 4:27:22 PM
* Author: TARSIER
*/


#ifndef __TARSIER_BMP180_H__
#define __TARSIER_BMP180_H__

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

//#define BMP180_ADDRESS		  0xEE     // default I2C address
#define BMP180_ADDRESS 0x77

/* ---- Registers ---- */
#define CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define CAL_AC3           0xAE  // R   Calibration data (16 bits)
#define CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define CAL_B1            0xB6  // R   Calibration data (16 bits)
#define CAL_B2            0xB8  // R   Calibration data (16 bits)
#define CAL_MB            0xBA  // R   Calibration data (16 bits)
#define CAL_MC            0xBC  // R   Calibration data (16 bits)
#define CAL_MD            0xBE  // R   Calibration data (16 bits)
#define CONTROL           0xF4  // W   Control register
#define CONTROL_OUTPUT_MSB   0xF6  // R   Output registers 0xF6=MSB, 0xF7=LSB, 0xF8=XLSB
#define CONTROL_OUTPUT_LSB    0xF7  // R   Output registers 0xF6=MSB, 0xF7=LSB, 0xF8=XLSB
#define CONTROL_OUTPUT_XLSB    0xF8  // R   Output registers 0xF6=MSB, 0xF7=LSB, 0xF8=XLSB

// BMP180 Modes
#define MODE_ULTRA_LOW_POWER    0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
#define MODE_STANDARD           1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
#define MODE_HIGHRES            2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
#define MODE_ULTRA_HIGHRES      3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25

// Control register
#define READ_TEMPERATURE        0x2E
#define READ_PRESSURE           0x34
//Other
#define MSLP                    101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)
#define LOCAL_ADS_ALTITUDE      2500            //mm     altitude of your position now
#define PRESSURE_OFFSET         0               //Pa    Offset

class Tarsier_BMP180
{
//variables
public:	
	long _PressureCompensate;
protected:
private:
	int16_t _AC1, _AC2, _AC3, _B1, _B2, _MB, _MC, _MD, _OSS;
	uint16_t _AC4, _AC5, _AC6;	
	char _error;

//functions
public:
	Tarsier_BMP180();
	~Tarsier_BMP180();
	char init();
	long getPressure();
	long getPressure(unsigned long up);
	float getTemperature();
	float getTemperature(unsigned int ut);
	float getAltitude(float pressure);
	float calcPressureAtSeaLevel(long pressure, float altitude);
	unsigned int readUT(void);
	unsigned long readUP(void);
	char getError(void);

protected:
private:
	Tarsier_BMP180( const Tarsier_BMP180 &c );
	Tarsier_BMP180& operator=( const Tarsier_BMP180 &c );
	char bmp180Read(unsigned char address);
	int bmp180ReadInt(unsigned char address);
	void writeRegister(int deviceAddress, byte address, byte val);
    int readRegister(int deviceAddress, byte address);

}; //Tarsier_BMP180

#endif //__TARSIER_BMP180_H__
