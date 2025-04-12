/*
  ORC_Drone_EBIMU.h - Library for reading attitude signal.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#include "Arduino.h"
#include "ORC_Drone_EBIMU.h"
#include<ORC_Utilities.h>
#include<ORCType.h>
#include <ORC_Utilities_1rdKalmanFilter.h>

/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_EBIMU::ORC_Drone_EBIMU(int maxReadErrCnt)
{
  _maxReadErrCnt = maxReadErrCnt; //Cap nhat bien error
  readErrCnt = 0;
  sbuf_cnt = 0; //Reset bien dem
 // _rollOffset = rollOffset; //Gia tri offset cua goc roll
 // _pitchOffset = pitchOffset; //Gia tri offset cua goc pitch
 // _yawOffset = yawOffset; //Gia tri offset cua goc yaw
 #ifdef STM32F1
 #else 
    EBIPort.begin(115200);   
    EBIPort.setTimeout(2);  
  #endif
}
//----------------------BEGIN OF ATTITUDE FUNCTIONS--------------------------------------------------------
/*-------------------------------------------------
* Kiem tra 1 goc co nam trong khoang cho phep hay khong: -180<=in <= 180
---------------------------------------------------*/
bool ORC_Drone_EBIMU::isInRangeIMUData(float in)
{
	return ((in >= -180) && (in <= 180)); 	
}

/*-------------------------------------------------
* Kiem tra 1 ky tu co thoa man ascii number khong: 0,...,9,-,+,.,'
---------------------------------------------------*/
bool ORC_Drone_EBIMU::isUsedIMUChar(char c)
{
	return ((c == '.')||(c=='-')||(c=='+')||(c==',')||(((c >= 48) && (c <= 57)))); //48 ~ '0'; 57 ~ '9'	
}
/*-----------------------------------------
* Doc du lieu tho tra ve tu cam bien V2
-------------------------------------------*/
bool ORC_Drone_EBIMU::EBIAsciiParser2(float *item, int number_of_item)
{
  int n, i;
  int rbytes;
  char *addr;
  char readChar;
  float yaw;
  bool result = false;

  rbytes = EBIPort.available();
  if (rbytes > 0)
  {
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
  }  
  return result;
}

/*-----------------------------------------
* Doc va xu ly du lieu tu EBIMU
-------------------------------------------*/
bool ORC_Drone_EBIMU::EBIReadData(orcebi_data *ebiData){
  float temp_euler[3] = {0, 0, 0}, euler[3] = {0, 0, 0};
  bool result = false;
  //IMU6050(acc_x, acc_y, acc_z, gy_x, gy_y, gy_z); // Gia tri cam bien MPU6050
  if (EBIAsciiParser2(temp_euler, 3)) { //Doc du lieu thanh cong
    if (!isnan(temp_euler[0]) && !isnan(temp_euler[1]) && !isnan(temp_euler[2])) {
		euler[2] = ORCUti.yawCompensation(temp_euler[2] - ebiData->yawe);
		euler[0] = temp_euler[0] - ebiData->rolle; 
		euler[1] = temp_euler[1] - ebiData->pitche; 
		ebiData->readErrCnt = 0;
		ebiData->roll = euler[0];
		ebiData->pitch = euler[1];
		ebiData->yaw = euler[2];
		result = true;
    }else ebiData->readErrCnt++;	 
  }else ebiData->readErrCnt++;
  //Gioi han gia tri error
  if(ebiData->readErrCnt >= _maxReadErrCnt)
	ebiData->readErrCnt = _maxReadErrCnt;
  //Tra ve ket qua lam viec
  return result;
}

/*-----------------------------------------
* Doc va xu ly du lieu tu EBIMU
-------------------------------------------*/
bool ORC_Drone_EBIMU::EBISetOffsetValues(orcebi_data *ebiData, float re, float pe, float ye){
  #ifdef STM32F1
    EBIPort.begin(115200);   
    EBIPort.setTimeout(2);  
  #endif
  ebiData->rolle = re; ebiData->pitche = pe; ebiData->yawe = ye;
  return true;
}
/*-----------------------------------------
* Kiem tra du lieu EBIMU
* headingAngle: degree
-------------------------------------------*/
bool ORC_Drone_EBIMU::CheckEBIData(orcebi_data *pre_ebiData, orcebi_data *current_ebiData, float headingAngle, int16_t *unchangeCnt){
 
  bool result = true;
  int16_t ucCnt = 0;
  //==============Kiem tra su thay doi tin hieu==========================
  //Kiem tra goc roll
  if(current_ebiData->roll == pre_ebiData->roll)
  {
	ucCnt = unchangeCnt[0];
	ucCnt ++;
	if(ucCnt >= EBI_MAX_UNCHANGE){
		result = false;
		ucCnt = EBI_MAX_UNCHANGE;
		Serial.print("[LOI] EBI KHONG THAY DOI GIA TRI GOC ROLL: "); Serial.println(ucCnt);		
	}
	unchangeCnt[0] = ucCnt;
  }else unchangeCnt[0] = 0;

    //Kiem tra goc pitch
  if(current_ebiData->pitch == pre_ebiData->pitch)
  {
	ucCnt = unchangeCnt[1];
	ucCnt ++;
	if(ucCnt >= EBI_MAX_UNCHANGE){
		result = false;
		ucCnt = EBI_MAX_UNCHANGE;
		Serial.print("[LOI] EBI KHONG THAY DOI GIA TRI GOC PITCH: "); Serial.println(ucCnt);		
	}
	unchangeCnt[1] = ucCnt;
  }else unchangeCnt[1] = 0;
  
    //Kiem tra goc yaw
  if(current_ebiData->yaw == pre_ebiData->yaw)
  {
	ucCnt = unchangeCnt[2];
	ucCnt ++;
	if(ucCnt >= EBI_MAX_UNCHANGE){
		result = false;
		ucCnt = EBI_MAX_UNCHANGE;
		Serial.print("[LOI] EBI KHONG THAY DOI GIA TRI GOC YAW: "); Serial.println(ucCnt);		
	}
	unchangeCnt[2] = ucCnt;
  }else unchangeCnt[2] = 0;  
  // //Kiem tra goc roll
  // if((current_ebiData->roll == pre_ebiData->roll)||(current_ebiData->pitch == pre_ebiData->pitch)||(current_ebiData->yaw == pre_ebiData->yaw))
  // {
	// ucCnt ++;
	// if(ucCnt >= EBI_MAX_UNCHANGE){
		// result = false;
		// ucCnt = EBI_MAX_UNCHANGE;
		// Serial.print("[LOI] EBI KHONG THAY DOI GIA TRI: "); Serial.println(ucCnt);
	// }
  // }else ucCnt = 0;
  // *unchangeCnt = ucCnt;
  //==============Ket thuc kiem tra do thay doi cua goc====================
  //Kiem tra tam goc do duoc
  if((fabs(current_ebiData->roll)>GOOD_Attitude_Range)||(fabs(current_ebiData->pitch)>GOOD_Attitude_Range)||(fabs(current_ebiData->yaw - headingAngle) > GOOD_Yaw_Range))
  //if((fabs(current_ebiData->roll)>GOOD_Attitude_Range)||(fabs(current_ebiData->pitch)>GOOD_Attitude_Range)||(fabs(current_ebiData->yaw - pre_ebiData->yaw) > GOOD_Yaw_Range))
  {
	Serial.print("[LOI] EBI BI NGHIENG LOI: "); Serial.print(current_ebiData->roll);
	Serial.print(","); Serial.print(current_ebiData->pitch);Serial.print(",");
	Serial.print(current_ebiData->yaw);Serial.print(",");
	Serial.println(headingAngle);
	result = false;
  }
  //Cập nhật lại các dữ liệu
  pre_ebiData->roll = current_ebiData->roll;
  pre_ebiData->pitch = current_ebiData->pitch;
  pre_ebiData->yaw = current_ebiData->yaw;	
  
  
  return result;
}
//----------------------END OF ATTITUDE FUNCTIONS--------------------------------------------------------




