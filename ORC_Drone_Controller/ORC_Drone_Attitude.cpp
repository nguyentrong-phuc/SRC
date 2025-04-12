/*
  ORC_Drone_Attitude.h - Library for reading attitude signal.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#include "Arduino.h"
#include "ORC_Drone_Attitude.h"
#include<ORC_Utilities.h>
#include <ORC_Utilities_1rdKalmanFilter.h>

/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_Attitude::ORC_Drone_Attitude(int maxReadErrCnt, float rollOffset, float pitchOffset, float yawOffset)
{
  _maxReadErrCnt = maxReadErrCnt; //Cap nhat bien error
  _readErrCnt = 0;
  sbuf_cnt = 0; //Reset bien dem
  _rollOffset = rollOffset; //Gia tri offset cua goc roll
  _pitchOffset = pitchOffset; //Gia tri offset cua goc pitch
  _yawOffset = yawOffset; //Gia tri offset cua goc yaw
  EBIPort.begin(115200);   
  EBIPort.setTimeout(2);
  _DroneMPU = new ORC_Drone_MPU6050();
  kfRoll = new ORC_Utilities_1rdKalmanFilter(10,0.2,5);
  kfPitch = new ORC_Utilities_1rdKalmanFilter(10,0.2,5);
  kfYaw = new ORC_Utilities_1rdKalmanFilter(10,0.2,5);
  kfGyX = new ORC_Utilities_1rdKalmanFilter(1,0.1,10);
  kfGyY = new ORC_Utilities_1rdKalmanFilter(1,0.1,10);
  kfrroll = new ORC_Utilities_1rdKalmanFilter(1,0.1,10);
  kfrpitch = new ORC_Utilities_1rdKalmanFilter(1,0.1,10);
  
}
//----------------------BEGIN OF ATTITUDE FUNCTIONS--------------------------------------------------------
/*-------------------------------------------------
* Kiem tra 1 goc co nam trong khoang cho phep hay khong: -180<=in <= 180
---------------------------------------------------*/
bool ORC_Drone_Attitude::isInRangeIMUData(float in)
{
	return ((in >= -180) && (in <= 180)); 	
}

/*-------------------------------------------------
* Kiem tra 1 ky tu co thoa man ascii number khong: 0,...,9,-,+,.,'
---------------------------------------------------*/
bool ORC_Drone_Attitude::isUsedIMUChar(char c)
{
	return ((c == '.')||(c=='-')||(c=='+')||(c==',')||(((c >= 48) && (c <= 57)))); //48 ~ '0'; 57 ~ '9'	
}
/*-----------------------------------------
* Doc du lieu tho tra ve tu cam bien V2
-------------------------------------------*/
bool ORC_Drone_Attitude::EBIAsciiParser2(float *item, int number_of_item)
{
  int n, i;
  int rbytes;
  char *addr;
  char readChar;
  float yaw;
  bool result = false;

  rbytes = EBIPort.available();
  for (n = 0; n < rbytes; n++)
  {
	  readChar = EBIPort.read();
	  if(readChar== '*')//Bat dau chuoi moi
	  {
		  if((sbuf_cnt >= 5 )&&(sbuf_cnt<SBUF_SIZE)){ //co the da nhan du du lieu --> xu ly du lieu
			addr = strtok(sbuf, ",");
			for (i = 0; i < number_of_item; i++)
			{
			  item[i] = atof(addr);
			  addr = strtok(NULL, ",");
			}
			item[0] = item[0] * signEuler[0];
			item[1] = item[1] * signEuler[1];
			item[2] = item[2] *signEuler[2];
			result = (isInRangeIMUData(item[0]) && isInRangeIMUData(item[1]) && isInRangeIMUData(item[2]));			
		  }		  
		  //Reset vi tri nhan de chuan bi doc vi tri moi
		  sbuf_cnt = 0;		  
	  }else
	  {
		//Neu du lieu dung kieu cho phep thi them vao bo nho
		if(isUsedIMUChar(readChar)){
			sbuf[sbuf_cnt] = readChar;
			//Doi toi diem thu 2
			sbuf_cnt++;
			if (sbuf_cnt >= SBUF_SIZE) sbuf_cnt = 0;
		}
	  }
  }
  return result;
}
/*-----------------------------------------
* Doc du lieu tho tra ve tu cam bien
-------------------------------------------*/
bool ORC_Drone_Attitude::EBIAsciiParser1(float *item, int number_of_item)
{
  int n, i;
  int rbytes;
  char *addr;
  float yaw;
  bool result = false;

  rbytes = EBIPort.available();
  for (n = 0; n < rbytes; n++)
  {
    sbuf[sbuf_cnt] = EBIPort.read();
    if (sbuf[sbuf_cnt] == 0x0a)
    {
      addr = strtok(sbuf, ",");
      for (i = 0; i < number_of_item; i++)
      {
        item[i] = atof(addr);
        addr = strtok(NULL, ",");
      }
      result = true;

    }
    else if (sbuf[sbuf_cnt] == '*')
    { sbuf_cnt = -1;
    }

    sbuf_cnt++;
    if (sbuf_cnt >= SBUF_SIZE) sbuf_cnt = 0;
  }
  //bu offset
  item[0] = item[0] * signEuler[0];
  item[1] = item[1] * signEuler[1];
  item[2] = item[2] *signEuler[2];
  //yaw = item[2];
  //bu goc yaw
  //item[2] = ORCUti.yawCompensation(yaw);
  return result;
}


/*-----------------------------------------
* Doc va xu ly du lieu tu EBIMU
-------------------------------------------*/
bool ORC_Drone_Attitude::EBIReadData(){
  float temp_euler[3] = {0, 0, 0}, euler[3] = {0, 0, 0};
  bool result = false;
  //IMU6050(acc_x, acc_y, acc_z, gy_x, gy_y, gy_z); // Gia tri cam bien MPU6050
  if (EBIAsciiParser2(temp_euler, 3)) { //Doc du lieu thanh cong
    if (!isnan(temp_euler[0]) && !isnan(temp_euler[1]) && !isnan(temp_euler[2])) {
		euler[2] = ORCUti.yawCompensation(temp_euler[2]);
		euler[0] = temp_euler[0] - _rollOffset; 
		euler[1] = temp_euler[1] - _pitchOffset; 
		_readErrCnt = 0;
		ebiroll = euler[0];
		ebipitch = euler[1];
		ebiyaw = euler[2];
		result = true;
    }else _readErrCnt++;	 
  }else _readErrCnt++;
  //Gioi han gia tri error
  if(_readErrCnt >= _maxReadErrCnt)
	_readErrCnt = _maxReadErrCnt;
  //Tra ve ket qua lam viec
  return result;
}
/*-----------------------------------------
* HAM SETUP CAC CAM BIEN CAN THIET DE UOC LUONG ATTITUDE
-------------------------------------------*/
bool ORC_Drone_Attitude::AttitudeFromSensorSetup(){  
  _DroneMPU->MPU6050Setup(MPU6050_MODE_COMPUTATION); //Set up MPU6050 o che do tinh toan
  //DroneHMC.HMCSetup(); //Khoi dong HMC
  AttitudeComputationSetupDone = true;
  return true;
}
/*-----------------------------------------
* HAM UOC LUONG ATTITUDE
* cac gia tri tra ve croll, cpitch, cyaw
-------------------------------------------*/
bool ORC_Drone_Attitude::AttitudeFromSensorGetData(float dt){  
	//B1: doc du lieu tu mpu
	float rawyaw = 0, fgyx, fgyy, frroll, frpit;
	orcmpu_data mdata;
	
	if(!_DroneMPU->MPU6050ReadData(dt, &mdata))
		return false;		
	//B2: doc du lieu hmc
	rawyaw = 0;//DroneHMC.HMCReadData();
	// B3: tinh toan goc roll, pitch, yaw
	//Loc nhieu
	fgyx = kfGyX->update1Value(mdata.gx);
	fgyy = kfGyY->update1Value(mdata.gy);
	frroll = kfrroll->update1Value(mdata.mroll);
	frpit = kfrpitch->update1Value(mdata.mpitch);
	
	
	//Uoc luong gia tri roll
	croll = kfRoll->update2Value(fgyx, frroll, dt);
	/*  if(fabs(fgyx) > 10){
		croll = kfRoll->update2Value(fgyx, frroll, dt);
	}else //Truong hop loi
	{
		kfRoll->_r = 0.9*(kfRoll->_r) + 1 + 10*fabs(frroll - _frroll);
		croll = kfRoll->update2Value(fgyx, frroll, dt);
	}  */
	_frroll = frroll;
	//croll = kfRoll->update2Value(_DroneMPU->gx, _DroneMPU->rawRoll, dt);
	if(fabs(fgyy) > 5){
		cpitch = kfRoll->update2Value(fgyy, frpit, dt);
	}else
		//cpitch = kfRoll->update2Value(fgyy, 0.9*cpitch, dt);
	//cpitch = kfPitch->update2Value(_DroneMPU->gy, _DroneMPU->rawPitch, dt);
	cyaw = kfYaw->update2Value(mdata.gz, rawyaw, dt);
    return true;
}
//----------------------END OF ATTITUDE FUNCTIONS--------------------------------------------------------




