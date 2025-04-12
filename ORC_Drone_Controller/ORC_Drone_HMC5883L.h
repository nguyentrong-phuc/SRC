/*
  ORC_Drone_HMC5883L.h - Library for basic functions of HMC5883L.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#ifndef ORC_Drone_HMC5883L_h
#define ORC_Drone_HMC5883L_h

#include "Arduino.h"
#include<Wire.h>
#include<ORC_Utilities.h>
#include <SimpleKalmanFilter.h>

#define HMC5883_I2C 0x1E
#define HMC5883_Gauss_LSB_XY 1090
#define SENSORS_GAUSS_TO_MICROTESLA   100

class ORC_Drone_HMC5883L
{
  public:
    ORC_Drone_HMC5883L(); //Ham khoi tao
	void HMCSetup(); //Ham setup
	float HMCReadData(); //Ham doc du lieu
	bool HMCCalibration(uint16_t calibTime); //Ham calib_dulieu
	void HMCCalibParaManualSet(float *offsetVal, float *disx, float *disy, float *disz); //Ham set gia tri calib manually
	float yawKalman;
	
  private:
    int _pin;
	//-------------HMC Parameter--------------
	//float _yaw_noKalman;
	ORC_Utilities ORCUti;
	SimpleKalmanFilter *hmcKFilter;
	//Calibration parameters
	float _offsetVal[3]={4.957024, -17.126465, 5.203447};
	float _disx[3] = {0.918457, 0.002606, 0.007833};
	float _disy[3] = {0.002606, 0.961122, 0.001134};
	float _disz[3] = {0.007833, 0.001134, 0.963156};
	
	// Read raw HMC value
	void HMCReadRawData(float *rawValue);

};



#endif