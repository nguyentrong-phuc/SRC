/*
  ORC_Drone_HMC5883L.cpp - Library for basic functions of HMC5883L.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#include "Arduino.h"
#include "ORC_Drone_HMC5883L.h"
#include<Wire.h>
#include<ORC_Utilities.h>
#include <SimpleKalmanFilter.h>

/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_HMC5883L::ORC_Drone_HMC5883L()
{
  //Khoi tao bo loc
  hmcKFilter = new SimpleKalmanFilter(0.05, 2, 0.1);
}
//----------------------BEGIN OF PWM FUNCTIONS--------------------------------------------------------
/*------------------------------------------
* Cai dat cac thong so ban dau cho cam bien
-------------------------------------------*/
void ORC_Drone_HMC5883L::HMCSetup(){
  Wire.begin();
  ORCUti.i2c_write8(HMC5883_I2C, 0x02, 0x00); //Setup HMC5883L, thanh ghi Mode Register, che do doc Continous
  ORCUti.i2c_write8(HMC5883_I2C, 0x01, 0x20); //Setup thanh ghi CRB voi gain la +-1.3
  ORCUti.i2c_write8(HMC5883_I2C, 0x00, 0x78); //Setup thanh ghi CRA voi F =75Hz
}
/*------------------------------------------
* Doc gia tri tho cua cam bien
-------------------------------------------*/
void ORC_Drone_HMC5883L::HMCReadRawData(float *rawValue) {
  uint8_t xhi,xlo, zhi, zlo,yhi, ylo;
  int16_t magX, magY, magZ;
  // Read the magnetometer
  Wire.beginTransmission(HMC5883_I2C);
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.requestFrom(HMC5883_I2C, (byte)6, true);
	//Doc du lieu
  xhi = Wire.read();
  xlo = Wire.read();
  zhi = Wire.read();
  zlo = Wire.read();
  yhi = Wire.read();
  ylo = Wire.read();
	//Tong hop du lieu
  magX = (int16_t)(xlo | ((int16_t)xhi << 8));
  magY = (int16_t)(ylo | ((int16_t)yhi << 8));
  magZ = (int16_t)(zlo | ((int16_t)zhi << 8));
  //Tinh toan du lieu ve gia tri thuc
  rawValue[0] = (float) magX / (float) HMC5883_Gauss_LSB_XY * (float) SENSORS_GAUSS_TO_MICROTESLA;
  rawValue[1] = (float) magY / (float) HMC5883_Gauss_LSB_XY * (float) SENSORS_GAUSS_TO_MICROTESLA;
  rawValue[2] = (float) magZ / 980 * (float) SENSORS_GAUSS_TO_MICROTESLA;  
}
/*------------------------------------------
* Doc gia tri cam bien
-------------------------------------------*/
float ORC_Drone_HMC5883L::HMCReadData() {
  float raw_HMC[3], calib_HMC[3], yaw_cal, yaw_HMC;
  HMCReadRawData(raw_HMC);
  //Calib offset va do bien dang du lieu
  //calib_HMC[0] = 0.918457 * (raw_HMC[0] - 4.957024) + 0.002606 * (raw_HMC[1] - (-17.126465)) + 0.007833 * (raw_HMC[2] - 5.203447);
  //calib_HMC[1] = 0.002606 * (raw_HMC[0] - 4.957024) + 0.961122 * (raw_HMC[1] - (-17.126465)) + 0.001134 * (raw_HMC[2] - 5.203447);
  //calib_HMC[2] = 0.007833 * (raw_HMC[0] - 4.957024) + 0.001134 * (raw_HMC[1] - (-17.126465)) + 0.963156 * (raw_HMC[2] - 5.203447);
  
  calib_HMC[0] = _disx[0] * (raw_HMC[0] - _offsetVal[0]) + _disx[1] * (raw_HMC[1] - _offsetVal[1]) + _disx[2] * (raw_HMC[2] - _offsetVal[2]);
  calib_HMC[1] = _disy[0] * (raw_HMC[0] - _offsetVal[0]) + _disy[1] * (raw_HMC[1] - _offsetVal[1]) + _disy[2] * (raw_HMC[2] - _offsetVal[2]);
  calib_HMC[2] = _disz[3] * (raw_HMC[0] - _offsetVal[0]) + _disz[1] * (raw_HMC[1] - _offsetVal[1]) + _disz[2] * (raw_HMC[2] - _offsetVal[2]);

  //Tinh ma tran da calib
  //  multiplyMatrixVector(A_HMC, b_HMC, calib_HMC);
  yaw_cal = -atan2(calib_HMC[1], calib_HMC[0]);
  //Chuyen sang do
  yaw_HMC = ORCUti.yawCompensation(yaw_cal * 180 / M_PI);
  
  yawKalman = hmcKFilter->updateEstimate(yaw_HMC);
  return yawKalman;
}
/*----------------------------------------------
* Dat cac gia tri offset cho cam bien thu cong
-----------------------------------------------*/
void ORC_Drone_HMC5883L::HMCCalibParaManualSet(float *offsetVal, float *disx, float *disy, float *disz){
	_offsetVal[0] = offsetVal[0]; _offsetVal[1] = offsetVal[1]; _offsetVal[2] = offsetVal[2];
	_disx[0] = disx[0]; _disx[1] = disx[1]; _disx[2] = disx[2];
	_disy[0] = disy[0]; _disy[1] = disy[1]; _disy[2] = disy[2];
	_disz[0] = disz[0]; _disz[1] = disz[1]; _disz[2] = disz[2];
}	

/*-------------------------------------------------------------
* Tu dong calib cac gia tri off va distorsion cua cam bien
--------------------------------------------------------------*/
bool ORC_Drone_HMC5883L::HMCCalibration(uint16_t calibTime){
	float raw_HMC[3], _minCalib[3] = {0.01,0.01,0.01}, _maxCalib[3] = {0.01,0.01,0.01};;
	if(calibTime > 0){
		Serial.println("Bat dau tien hanh calib HMC.....");
		for(uint16_t i = 0; i < calibTime; i++){
			delay(20); 
			HMCReadRawData(raw_HMC); //Doc gia tri tho
			//Tien hanh cap nhat cac nguong
			for (int j = 0;j < 3; j++){
				//Cap nhat min
				if(raw_HMC[j] < _minCalib[j]){
					_minCalib[j] = raw_HMC[j];
				}
				//Cap nhat max
				if(raw_HMC[j] > _maxCalib[j]){
					_maxCalib[j] = raw_HMC[j];
				}
			}
		}
		//chuyen doi sang cac gia tri thuc te
		for (int j = 0;j < 3; j++){
			//Offset
			_offsetVal[j] = 0.5*(_minCalib[j] + _maxCalib[j]);
			_disx[j] = (j==0)?(2/(_maxCalib[j] - _minCalib[j])):0;
			_disy[j] = (j==1)?(2/(_maxCalib[j] - _minCalib[j])):0;
			_disz[j] = (j==2)?(2/(_maxCalib[j] - _minCalib[j])):0;
		}
		
		Serial.println("Ket thuc calib HMC. Cac gia tri dat duoc:");
		Serial.print("Offset x: "); Serial.println(_offsetVal[0]);
		Serial.print("Offset y: "); Serial.println(_offsetVal[1]);
		Serial.print("Offset z: "); Serial.println(_offsetVal[2]);
		Serial.print("Distor x: "); Serial.println(_disx[0]);
		Serial.print("Distor y: "); Serial.println(_disy[1]);
		Serial.print("Distor z: "); Serial.println(_disz[2]);
		return true;		
	}else return false;
}
//----------------------END OF PWM FUNCTIONS--------------------------------------------------------




