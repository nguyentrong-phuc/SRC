/*------------------------------------------------------------
  ORC_Drone_2DLidar.h - Library for manipulting with 2D Lidar.
  Created by Dang Xuan Ba, Dec 08, 2024.
  Updated by Dang Xuan Ba, Dec 08, 2024.
  Released into the public domain.
-------------------------------------------------------------*/

#ifndef ORC_Drone_2DLidar_h
#define ORC_Drone_2DLidar_h

#include "Arduino.h"
#include "ORCType.h"
#include "RPLidar.h"
#include "ORC_Utilities.h"

#define LIDARSerial Serial1
#define OUTSerial Serial2

#define defaultDistance 100000
#define defaultAngle 0

#define freeDistance 1000 //4000 //in [mm]
#define lockDistance 300 //2000 //in [mm]
#define maxAngle	   15//in [deg]

#define LidarTimeOut 2000 //in [ms]

#define K_Angle_Free 0.95
#define K_Angle_Control 0.95
#define K_Angle_Timeout 0.9955
#define ControlTime 10// in [ms]
#define ObstacleAvoidanceControlAngle_Time 5// in [100 ms]
#define ObstacleAvoidanceRawData_Time 10// in [100 ms]


typedef struct{
    float lidarDistance; // in [mm]
	float lidarAngle; // in [deg]
	float controlRoll; // in [deg]
	float controlPitch;	//in [deg]
	unsigned long preTime; //in [ms]
	bool isTimeOut;
	bool isNew; 
}lidarDataType; 


class ORC_Drone_2DLidar
{
  public:
    ORC_Drone_2DLidar(int lidarMotorPWMPin); //Khao khoi tao
	bool LidarSetup(lidarDataType *lidarData); //Setup che do Read
	
	void runLoop(lidarDataType *lidarData);
		
		
	byte sendingData[35] = {0};
	long readingData[11] = {0};
	//long Rec[9];

  private:
  	//void _printData(float angle, float distance); //In du lieu ra UART0
	bool LidarInScanData(lidarDataType *lidarData); //Scan du lieu cho lidar
	byte LidarDataProcessing(lidarDataType *lidarData);
	void LidarSendOutData(lidarDataType *lidarData); //Ham send du lieu
    lidarDataType _lidarData; 
	RPLidar _lidar;
	int _lidarMotorPin = 6;
	rplidar_response_device_info_t _info;
	float _minDistance = 100000;
	float _minAngle = 0;
	ORC_Utilities _Uti;
	unsigned long _preProcessingTime; //in [ms]
	byte _loopCount = 0;
	bool isZeroSent = false;

};



#endif