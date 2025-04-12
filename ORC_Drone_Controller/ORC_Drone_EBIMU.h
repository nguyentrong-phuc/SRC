/*
  ORC_Drone_EBIMU.h - Library for reading attitude signal.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#ifndef ORC_Drone_EBIMU_h
#define ORC_Drone_EBIMU_h

#include "Arduino.h"
#include<ORC_Utilities.h>
#include<ORCType.h>
#include <ORC_Drone_EBIMU.h>

#define SBUF_SIZE 64

#define GOOD_Attitude_Range 10
#define GOOD_Yaw_Range 10
#define EBI_MAX_UNCHANGE 30
#define EBIPort Serial1

class ORC_Drone_EBIMU
{
  public:
    ORC_Drone_EBIMU(int maxReadErrCnt); //Ham khoi tao
	bool EBIReadData(orcebi_data *ebiData); //Ham doc du lieu tu EBIMU
	bool EBISetOffsetValues(orcebi_data *ebiData, float re, float pe, float ye); //Ham doc du lieu tu EBIMU
	float ebiroll, ebipitch, ebiyaw; //Goc roll pitch yaw nhan tu cam bien khac
	int readErrCnt;
	bool CheckEBIData(orcebi_data *pre_ebiData, orcebi_data *current_ebiData, float headingAngle, int16_t *unchangeCnt);

	
  private:
    int _maxReadErrCnt;
	bool EBIAsciiParser2(float *item, int number_of_item);
	int sbuf_cnt;
	char sbuf[SBUF_SIZE];
	float _rollOffset, _pitchOffset, _yawOffset;
	const int signEuler[3] = {1, -1, -1};	
	bool isUsedIMUChar(char c);  //Ham kiem tra mot du lieu co nhan dung hay khong
	bool isInRangeIMUData(float in);  //Ham kiem tra mot du lieu co nam trong tam cho phep hay khong
	ORC_Utilities ORCUti;
	
	
};



#endif