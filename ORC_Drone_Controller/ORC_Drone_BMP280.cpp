/*
  ORC_Drone_BMP280.cpp - Library for setting up and read data from BMP280.
  Created by Dang Xuan Ba, October 18, 2024.
  Updated by Dang Xuan Ba, October 18, 2024.
  Released into the public domain.
*/

#include "Arduino.h"
#include "ORC_Drone_BMP280.h"
#include<Wire.h>
#include<ORC_Utilities.h>
#include <SimpleKalmanFilter.h>

/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_BMP280::ORC_Drone_BMP280(float devRate)
{
	//Khoi tao gia tri mac dinh cho cac bien
	_dig_T1 = 0;
	_dig_P1 = 0;
	_dig_T2 = 0;
	_dig_T3 = 0;
	_dig_P2 = 0;
	_dig_P3 = 0;
	_dig_P4 = 0;
	_dig_P5 = 0;    
	_dig_P6 = 0;
	_dig_P7 = 0;
	_dig_P8 = 0;
	_dig_P9 = 0; 
	_BMP280_ACCEPTDEV_RATE = devRate; //Nguong gioi han dao dong
}
//----------------------BEGIN OF BMP280 FUNCTIONS--------------------------------------------------------
/*-----------------------------------
 * SETUP BMP280
--------------------------------------*/
float ORC_Drone_BMP280::BMP280Setup()
{
	//Tao cac bien tam
	uint8_t data[24], i=0;
	float AltitudeBarometer = 0, n = 300; 
	int RateCalibrationNumber;	
	//Cai dat vung nho
	Wire.setClock(400000);
	Wire.begin();
	delay(250);

	Wire.beginTransmission(BMP280_I2C);
	Wire.write(0xF4);                          // Mode Setting on F4 register to be affordable with application 
	Wire.write(0x57);                          
	//       010 101 11
	Wire.endTransmission();   

	Wire.beginTransmission(BMP280_I2C);
	Wire.write(0xF5);                         
	Wire.write(0x34); //Tan so quet 62.5ms
	//       000 101 00
	Wire.endTransmission();   


	Wire.beginTransmission(BMP280_I2C);
	Wire.write(0x88);
	Wire.endTransmission();
	Wire.requestFrom(BMP280_I2C,24);   
	//Đọc cac du lieu nhiet do va ap suat
	while(Wire.available() && (i<24))   
	{
		data[i] = Wire.read();
		i++;
	}
	_dig_T1 = (data[1] << 8) | data[0]; 
	_dig_T2 = (data[3] << 8) | data[2];
	_dig_T3 = (data[5] << 8) | data[4];
	_dig_P1 = (data[7] << 8) | data[6]; 
	_dig_P2 = (data[9] << 8) | data[8];
	_dig_P3 = (data[11]<< 8) | data[10];
	_dig_P4 = (data[13]<< 8) | data[12];
	_dig_P5 = (data[15]<< 8) | data[14];
	_dig_P6 = (data[17]<< 8) | data[16];
	_dig_P7 = (data[19]<< 8) | data[18];
	_dig_P8 = (data[21]<< 8) | data[20];
	_dig_P9 = (data[23]<< 8) | data[22]; 
	delay(250);


	for (RateCalibrationNumber=0; RateCalibrationNumber<n; RateCalibrationNumber ++) {
		AltitudeBarometer = BMP280ReadData();
		_AltitudeBarometerStartUp+=AltitudeBarometer/n;
		delay(20);
	}
  _isSetupDone = true;
  return _AltitudeBarometerStartUp;
}

  
/*-----------------------------------
 * READ BMP280 BASIC SIGNAL
--------------------------------------*/
float ORC_Drone_BMP280::BMP280ReadBasicData() {
	if(_isSetupDone){
		Wire.beginTransmission(BMP280_I2C);                           //I2C slave interfaces (0x76 or 0x77)
		Wire.write(0xF7);                                        
		Wire.endTransmission();
		Wire.requestFrom(0x76,6);                               // Request 6-bit to read the register (0xF7-FC)
		uint32_t press_msb = Wire.read();                       // 0xF7
		uint32_t press_lsb = Wire.read();
		uint32_t press_xlsb = Wire.read();
		uint32_t temp_msb = Wire.read();
		uint32_t temp_lsb = Wire.read();
		uint32_t temp_xlsb = Wire.read();                       //0xFC

		// Given on Datasheet
		unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >>4);             //raw measurement 
		unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >>4);

		signed long int var1, var2;
		// Compensated and Calibrated temp
		// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. 
		// t_fine carries fine temperature as global value                                                                    
		var1 = ((((adc_T >> 3) - ((signed long int )_dig_T1 <<1)))* ((signed long int )_dig_T2)) >> 11;
		var2 = (((((adc_T >> 4) - ((signed long int )_dig_T1)) * ((adc_T>>4) - ((signed long int )_dig_T1)))>> 12) * ((signed long int )_dig_T3)) >> 14;
		signed long int t_fine = var1 + var2;  

		// Compensated and Calibrated pressure
		// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
		// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa                                                                           
		var1 = (((signed long int )t_fine)>>1) - (signed long int )64000;
		var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int )_dig_P6); 
		var2 = var2 + ((var1*((signed long int )_dig_P5)) <<1);
		var2 = (var2>>2)+(((signed long int )_dig_P4)<<16);
		var1 = (((_dig_P3 * (((var1>>2)*(var1>>2)) >> 13))>>3)+((((signed long int )_dig_P2) * var1)>>1))>>18;
		var1 = ((((32768+var1))*((signed long int )_dig_P1)) >>15);
		unsigned long int p = 0; 
		if (var1 == 0) { p=0;}    

		p = (((unsigned long int )(((signed long int ) 1048576)-adc_P)-(var2>>12)))*3125;

		if(p<0x80000000){ p = (p << 1) / ((unsigned long int ) var1);}
		else { p = (p / (unsigned long int )var1) * 2;  }

		var1 = (((signed long int )_dig_P9) * ((signed long int ) (((p>>3) * (p>>3))>>13)))>>12;
		var2 = (((signed long int )(p>>2)) * ((signed long int )_dig_P8))>>13;
		p = (unsigned long int)((signed long int )p + ((var1 + var2+ _dig_P7) >> 4));
		
		double pressure;
		pressure=(double)p/100 - 81;
		return ((float)(44330*(1-pow(pressure/1013.25, 1/5.255))*100));                             // Altitude measurement  
  }else 
  {
	  Serial.println("Chua setup BMP!");
	  return 0;
  }
}
/*-----------------------------------
 * READ BMP280 FINE SIGNAL
--------------------------------------*/
float ORC_Drone_BMP280::BMP280ReadData(){
	//Doc du lieu tho tu bara
	float abs_z_baro = BMP280ReadBasicData(); 
	//Xu ly mem
	if((abs_z_baro - AltitudeBarometerValue) > _BMP280_ACCEPTDEV_RATE)
		AltitudeBarometerValue = AltitudeBarometerValue + _BMP280_ACCEPTDEV_RATE;
	else if((abs_z_baro - AltitudeBarometerValue) < - _BMP280_ACCEPTDEV_RATE)
		AltitudeBarometerValue = AltitudeBarometerValue - _BMP280_ACCEPTDEV_RATE;
	else 
		AltitudeBarometerValue = abs_z_baro;
	return AltitudeBarometerValue;
}
//----------------------END OF BMP280 FUNCTIONS--------------------------------------------------------




