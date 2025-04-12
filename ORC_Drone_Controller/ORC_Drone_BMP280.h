/*
  ORC_Drone_BMP280.h - Library for setting up and read data from BMP280.
  Created by Dang Xuan Ba, October 18, 2024.
  Updated by Dang Xuan Ba, October 18, 2024.
  Released into the public domain.
*/

#ifndef ORC_Drone_BMP280_h
#define ORC_Drone_BMP280_h

#include "Arduino.h"
#include<Wire.h>
#include<ORC_Utilities.h>
#include <SimpleKalmanFilter.h>

#define BMP280_I2C 0x76

class ORC_Drone_BMP280
{
  public:
    ORC_Drone_BMP280(float devRate);//Ham khoi tao
	float BMP280Setup(); //Setup bmp	
	float BMP280ReadData(); //Doc du lieu tu bmp
	float AltitudeBarometerValue; //in [cm]
	
  private:
    uint16_t _dig_T1, _dig_P1;
	int16_t  _dig_T2, _dig_T3, _dig_P2, _dig_P3, _dig_P4, _dig_P5;  //dig_T# for temperature compensation   
	int16_t  _dig_P6, _dig_P7, _dig_P8, _dig_P9;                  //dig_P# for pressure compensation 
	float _AltitudeBarometerStartUp;
	float BMP280ReadBasicData(); //Doc du lieu basic tu bmp
	float _BMP280_ACCEPTDEV_RATE = 400;
	bool _isSetupDone = false;


};



#endif