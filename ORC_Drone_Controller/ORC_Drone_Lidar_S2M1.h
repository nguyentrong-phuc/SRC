
/*------------------------------------------------------------
  ORC_DRONE_LIDAR_S2M1 CLASS
* * Written by Le Duc Tai + Trieu Khanh Thi
* * Modified by Dang Xuan Ba. Email: 
* Date of version: 2025/01/10
* Hardware: Arduino Due
-------------------------------------------------------------*/

#ifndef ORC_Drone_Lidar_S2M1_h
#define ORC_Drone_Lidar_S2M1_h

#include "Arduino.h"
#include "RPLidar.h"
#include "ORC_Utilities.h"

#define CMD_STOP 0x25
#define CMD_RESET 0x40
#define CMD_GET_HEALTH 0x52
#define CMD_SCAN 0x20
#define CMD_GET_INFO 0x50
#define CMD_EXPRESS_SCAN 0x82

#define LIDARSerial Serial1
#define OUTSerial Serial2

#define defaultDistance 100000

#define defaultAngle 0

#define freeDistance 10000 //4000 //in [mm]
#define lockDistance 2000 //2000 //in [mm]
//#define maxAngle	   15//in [deg]

#define LidarTimeOut 2000 //in [ms]
#define CountCheckRecoverMode 3 // max check of recover mode

#define K_Angle_Free 0.95
#define K_Angle_Control 0.95
#define K_Angle_Timeout 0.9955
#define ControlTime 10// in [ms]
#define ObstacleAvoidanceControlAngle_Time 10// in [100 ms]
#define ObstacleAvoidanceRawData_Time 20// in [100 ms]


typedef struct{
      float lidarDistance; // in [mm]
	float lidarAngle; // in [deg]
	float controlRoll; // in [deg]
	float controlPitch;	//in [deg]
	unsigned long preTime; //in [ms]
	bool isTimeOut;
	bool isNew; 
}lidarDataType; 


class ORC_Drone_Lidar_S2M1
{
  public:
      ORC_Drone_Lidar_S2M1(); //Khao khoi tao
	bool LidarSetup(lidarDataType *lidarData); //Setup che do Read
	
	void runLoop(lidarDataType *lidarData);

		
	byte sendingData[35] = {0};
	long readingData[11] = {0};
	//long Rec[9];

	void _printData(float angle, float distance); //In du lieu ra UART0
  private:
  	//void _printData(float angle, float distance); //In du lieu ra UART0
	bool LidarInScanData(lidarDataType *lidarData); //Scan du lieu cho lidar
	bool LidarRecoverMode(uint8_t );
	byte LidarDataProcessing(lidarDataType *lidarData);
	void LidarSendOutData(lidarDataType *lidarData); //Ham send du lieu
    	lidarDataType _lidarData; 
      //============================================
	RPLidar _lidar; // các thao tác với Lidar
      //============================================
	
	bool _isSetup = false;
      
	//int _lidarMotorPin = 6;
	rplidar_response_device_info_t _info;
	float _minDistance = 100000;
	float _minAngle = 0;
	// void printMinPoint();
	ORC_Utilities _Uti;
	unsigned long _preProcessingTime; //in [ms]
	byte _loopCount = 0;
	bool isZeroSent = false;
	float _maxControlAngle = 0;

};



#endif