/*
  ORC_Drone_MPU6050.cpp - a Arduino library used to setup and control quadcopters.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#include "Arduino.h"
#include "ORC_Drone_MPU6050.h"
#include<Wire.h>
#include<ORC_Utilities.h>
#include <SimpleKalmanFilter.h>

/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_MPU6050::ORC_Drone_MPU6050()
{
  _isSetupDone = false;
}
//----------------------BEGIN OF PWM FUNCTIONS--------------------------------------------------------
void ORC_Drone_MPU6050::MPU6050Setup(byte mode)
{
   Wire.begin();                   // Initialize comunication
// #if ARDUINO >= 157
//   Wire.setClock(400000UL); // Set I2C frequency to 400kHz
// #else
//   TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
// #endif
Wire.setClock(400000UL); // Set I2C frequency to 400kHz

  #ifdef ESP32
    bool test = true;  //Chua xu ly
  #elif defined(ARDUINO_ARCH_SAM)  
    bool test = true; //chua xu ly
  #elif defined(ARDUINO_ARCH_STM32)
    bool test = true; // chua xu ly
  #else 
  	Wire.setWireTimeout(3000, true);
  #endif
  
  Wire.beginTransmission(0x68);   //This is the I2C address of the MPU 0x68
  Wire.write(0x6B);               //Accessing the register 6B 
  Wire.write(0x00);               //Setting SLEEP register to 0
  Wire.endTransmission(true); 
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x08);                  //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission(true);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                   // Set the register bits as 00001000 (500deg/s full scale) / 65.5
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);           // Tạo trạng thái Start I2C và gửi địa chỉ slave I2C 0x68 đến mpu 
  Wire.write(0x1A);                       // Gửi địa chỉ thanh ghi muốn ghi dữ liệu vào là 0x1A
  //if (mode == MPU6050_MODE_CONTROL)
	// Wire.write(0x03);                       // Ghi 0x04 vào thanh ghi 0x1A để chọn tần số bộ lọc thông thấp 1-184Hz 2-94Hz 3-44Hz 4-21Hz
	Wire.write(0x04);                       // Ghi 0x04 vào thanh ghi 0x1A để chọn tần số bộ lọc thông thấp 1-184Hz 2-94Hz 3-44Hz 4-21Hz
 // else
//	Wire.write(0x01);                       // Ghi 0x04 vào thanh ghi 0x1A để chọn tần số bộ lọc thông thấp 1-184Hz 2-94Hz 3-44Hz 4-21Hz
  Wire.endTransmission(true);  
  delay(20);
  _isSetupDone = true;
}
bool ORC_Drone_MPU6050::MPU6050ReadData(float dt, orcmpu_data *mdataOut) {


  if(_isSetupDone){
	  Wire.beginTransmission(0x68);
	  Wire.write(0x3B);
	  Wire.endTransmission(false);
	  byte len = Wire.requestFrom(0x68, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
	  //For a range of +-2g, we need to divide the raw values by 16384.0, according to the datasheet
	  float temp_acc_x = -(((int16_t)(Wire.read() << 8) | Wire.read()) / 8192.0 ); // X-axis value
	  float temp_acc_y = -(((int16_t)(Wire.read() << 8) | Wire.read()) / 8192.0 ); // Y-axis value
	  float temp_acc_z = ((int16_t)(Wire.read() << 8) | Wire.read()) / 8192.0 ; // Z-axis value

	 if((len == 0)||(temp_acc_x == 0 && temp_acc_y == 0 && (temp_acc_z) == 0)){
	//  if((len == 0)||((temp_acc_x == 0 && temp_acc_y == 0) || (fabs(temp_acc_z) < 0.15))){
		mdataOut->readErrCnt ++;
		if(mdataOut->readErrCnt >= 100) mdataOut->readErrCnt = 100;		  
		return false;
	  }else{	  
		  // === Read gyroscope data === //
		  Wire.beginTransmission(0x68);
		  Wire.write(0x43); // Gyro data first register address 0x43
		  Wire.endTransmission(false);
		  byte len2 = Wire.requestFrom(0x68, 6, true); // Read 4 registers total, each axis value is stored in 2 registers

			float temp_gy_x = -((int16_t)((Wire.read() << 8 | Wire.read())) / 65.5 - mdataOut->gex);
			float temp_gy_y = -((int16_t)((Wire.read() << 8 | Wire.read())) / 65.5 - mdataOut->gey);
			float temp_gy_z = (int16_t)((Wire.read() << 8 | Wire.read())) / 65.5 - mdataOut->gez;

		  
		  if(isnan(temp_acc_x) || isnan(temp_acc_y) || isnan(temp_acc_z)
			 || isnan(temp_gy_x) || isnan(temp_gy_y) || isnan(temp_gy_z)){
				mdataOut->readErrCnt++;
				if(mdataOut->readErrCnt >= 100) mdataOut->readErrCnt = 100;
			 return false; //Bi loi du lieu
			 }
		else{
		  //Cap nhat du lieu
		  mdataOut->accx = temp_acc_x; mdataOut->accy = temp_acc_y; mdataOut->accz = temp_acc_z;
		  mdataOut->gx = temp_gy_x; mdataOut->gy = temp_gy_y; mdataOut->gz = temp_gy_z; 
		  
		  float accAngleX = (atan(temp_acc_y/(sqrt(pow(temp_acc_x,2) + pow(temp_acc_z,2))))*180/PI);
		  float accAngleY = (atan(-(temp_acc_x)/(sqrt(pow(temp_acc_y,2) + pow(temp_acc_z,2))))*180/PI);
		  
		  if(!isnan(accAngleX) && !isnan(accAngleY)){
			  //rawRoll = accAngleX;
			  //rawPitch = accAngleY;
			//cfRoll  = 0.99 *(cfRoll + gx*dt)  + 0.01* accAngleX;
			//cfPitch = 0.99 *(cfPitch + gy*dt) + 0.01* accAngleY;
			mdataOut->mroll  = 0.99 *(mdataOut->mroll + temp_gy_x*dt)  + 0.01* accAngleX;
			mdataOut->mpitch = 0.99 *(mdataOut->mpitch + temp_gy_y*dt) + 0.01* accAngleY;
			mdataOut->readErrCnt = 0;
			return true;
		  } else  {
			mdataOut->readErrCnt++;
			if(mdataOut->readErrCnt > 100) mdataOut->readErrCnt = 100;
			return false;
		  }
		}
	  }
  }else return false;
}
/*---------------------------------------------
* SET OFFSET VALUES
---------------------------------------------*/
bool ORC_Drone_MPU6050::MPU6050SetOffsetValues(orcmpu_data *mdataOut, float aex, float aey, float aez, float gexi, float geyi, float gezi) {
  if(_isSetupDone){
	  mdataOut->accex = aex; mdataOut->accey = aey;mdataOut->accez = aez;
	  mdataOut->gex = gexi; mdataOut->gey = geyi;mdataOut->gez = gezi;
	  return true;
  }else return false;
}

/*---------------------------------------------
* CHECK AND UPDATE MPU DATA
---------------------------------------------*/
bool ORC_Drone_MPU6050::MPU6050CheckData(orcmpu_data *pre_mdataOut, orcmpu_data *current_mdataOut,  int16_t *unchangeCnt) {
  bool result = true;
  int16_t ucCnt = *unchangeCnt;
  //B1 kiem tra du lieu co thay doi khong: neu du lieu khong thay doi trong vai chu ky -> false
  if((pre_mdataOut->accx == current_mdataOut->accx)||(pre_mdataOut->accy == current_mdataOut->accy)
  ||(pre_mdataOut->accz == current_mdataOut->accz)||(pre_mdataOut->gx == current_mdataOut->gx)
  ||(pre_mdataOut->gy == current_mdataOut->gy)||(pre_mdataOut->gz == current_mdataOut->gz))
  {
	  ucCnt++;
	  
	  if(ucCnt >= MPU6050_MAX_UNCHANGE){
		result = false;
		ucCnt = MPU6050_MAX_UNCHANGE;
		Serial.println("Loi du lieu MPU tinh qua lau<.>");
	  }
  }else ucCnt = 0;
  *unchangeCnt = ucCnt;
  //Kiem tra du lieu co trong khoang cho phep khong
  if((fabs(current_mdataOut->accx*9.81) > MPU6050_ACC_MAX_UNMOVE_VALUE)||(fabs(current_mdataOut->accy*9.81) > MPU6050_ACC_MAX_UNMOVE_VALUE)
  ||(fabs((current_mdataOut->accz - 1)*9.81) > MPU6050_ACC_MAX_UNMOVE_VALUE)||(fabs(current_mdataOut->gx) > MPU6050_GYRO_MAX_UNMOVE_VALUE)
  ||(fabs(current_mdataOut->gy) > MPU6050_GYRO_MAX_UNMOVE_VALUE)||(fabs(current_mdataOut->gz) > MPU6050_GYRO_MAX_UNMOVE_VALUE)){
	result = false;
	Serial.print("ax:");Serial.println(current_mdataOut->accx*9.81);
	Serial.print("ay:");Serial.println(current_mdataOut->accy*9.81);
	Serial.print("az:");Serial.println(((current_mdataOut->accz - 1)*9.81));
	Serial.print("gx:");Serial.println(current_mdataOut->gx);
	Serial.print("gy:");Serial.println(current_mdataOut->gy);
	Serial.print("gz:");Serial.println(current_mdataOut->gz);
	Serial.println("Loi du lieu MPU tinh qua cao!");
  }
  //Cap nhat du lieu moi
  pre_mdataOut->accx = current_mdataOut->accx; pre_mdataOut->accy = current_mdataOut->accy;
  pre_mdataOut->accz = current_mdataOut->accz;pre_mdataOut->gx = current_mdataOut->gx;
  pre_mdataOut->gy = current_mdataOut->gy; pre_mdataOut->gz = current_mdataOut->gz;

  
  return result;  
}
//----------------------END OF PWM FUNCTIONS--------------------------------------------------------




