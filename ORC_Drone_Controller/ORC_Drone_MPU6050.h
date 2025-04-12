/*
  ORC_Drone_MPU6050.h - Library for basic functions manipulating devices and signal processing.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#ifndef ORC_Drone_MPU6050_h
#define ORC_Drone_MPU6050_h

#include "Arduino.h"
#include<ORCType.h>
#include<Wire.h>
#include<ORC_Utilities.h>
#include <SimpleKalmanFilter.h>

#define MPU6050_I2C 0x68
#define MPU6050_MODE_CONTROL 0
#define MPU6050_MODE_COMPUTATION 1
#define MPU6050_MAX_UNCHANGE	10
#define MPU6050_ACC_MAX_UNMOVE_VALUE 3 //3.2//2.5 //m/s2: da OK
#define MPU6050_GYRO_MAX_UNMOVE_VALUE 5 //deg/s

class ORC_Drone_MPU6050
{
  public:
    ORC_Drone_MPU6050();//Ham khoi tao
	void MPU6050Setup(byte mode); //Setup mpu
	bool MPU6050ReadData(float dt, orcmpu_data *mdataOut); //Doc du lieu tu mpu
	bool MPU6050SetOffsetValues(orcmpu_data *mdataOut, float aex, float aey, float aez, float gexi, float geyi, float gezi);
	//float accx, accy, accz, gx, gy, gz, cfRoll, cfPitch, rawRoll, rawPitch;
	bool MPU6050CheckData(orcmpu_data *pre_mdataOut, orcmpu_data *current_mdataOut,  int16_t *unchangeCnt);
  private:
  //  int _pin;
	//float _axe, _aye, _aze, _gxe, _gye, _gze;
	bool _isSetupDone = false;
};

#endif