/*
  ORC_Drone_Attitude.h - Library for reading attitude signal.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#ifndef ORC_Drone_Attitude_h
#define ORC_Drone_Attitude_h

#include "Arduino.h"
#include<ORC_Utilities.h>
#include <ORC_Drone_MPU6050.h>
#include <ORC_Drone_HMC5883L.h>
#include <ORC_Utilities_1rdKalmanFilter.h>

#define SBUF_SIZE 64
#define EBIPort Serial1

class ORC_Drone_Attitude
{
  public:
    ORC_Drone_Attitude(int maxReadErrCnt, float rollOffset, float pitchOffset, float yawOffset); //Ham khoi tao
	bool EBIReadData(); //Ham doc du lieu tu EBIMU
	bool AttitudeFromSensorSetup(); //Ham setup de tin toan goc roll pitch yaw data thu (su dung 9dof).
	bool AttitudeFromSensorGetData(float dt); //Ham tin toan goc roll pitch yaw data thu (su dung 9dof).
	float ebiroll, ebipitch, ebiyaw; //Goc roll pitch yaw nhan tu cam bien khac
	float croll, cpitch, cyaw; //Goc roll pitch yaw duoc tinh toan
	ORC_Drone_MPU6050 *_DroneMPU;  //Bien mpu6050
	
  private:
    int _maxReadErrCnt, _readErrCnt;
	bool EBIAsciiParser1(float *item, int number_of_item);
	bool EBIAsciiParser2(float *item, int number_of_item);
	int sbuf_cnt;
	char sbuf[SBUF_SIZE];
	float _rollOffset, _pitchOffset, _yawOffset;
	const int signEuler[3] = {-1, 1, -1};
	float tAngles[3] = {0, 0, 0};
	float euler_OUT[3] = {0, 0, 0};
	bool AttitudeComputationSetupDone = false;
	float _fgyx, _fgyy, _frroll, _frpit;
	bool isUsedIMUChar(char c);  //Ham kiem tra mot du lieu co nhan dung hay khong
	bool isInRangeIMUData(float in);  //Ham kiem tra mot du lieu co nam trong tam cho phep hay khong
	
	ORC_Drone_HMC5883L DroneHMC; //Bien hmc5883L
	ORC_Utilities ORCUti; //Bien chua cac ham co ban
	ORC_Utilities_1rdKalmanFilter *kfRoll, *kfPitch, *kfYaw;
	ORC_Utilities_1rdKalmanFilter *kfGyX, *kfGyY, *kfrroll, *kfrpitch;
	
};

//------------------------Varialble for EBIMU------------------------------
typedef struct{  
  float roll, pitch, yaw;                        // bien luu cac goc roll, pitch, yaw
  uint16_t error_cnt; //Bien dem loi
}EBIMU_data;


#endif