/*================================================================
  ORC_Drone_PumpHub.h - Library for navigating data and pump control.
  Created by Dang Xuan Ba, Dec 14, 2024.
  Updated by Dang Xuan Ba, Dec 14, 2024.
  Released into the public domain.
==================================================================*/

#ifndef ORC_Drone_PumpHub_h
#define ORC_Drone_PumpHub_h

#include "Arduino.h"
#include<ORC_Utilities.h>
#include<ORCType.h>
#include "ORC_Drone_RF.h"
#include "ORC_Drone_PWM.h"

#define HUBPort	Serial1  // HUB
#define RPLidarPort		Serial2  
#define ESP32Port	Serial3    // IOT
#define ET			0.01 //10ms

#define LED_RF_SEND	8
#define LED_HUB_DATA 29
#define LED_PUMP_DATA 30
#define LED_LIDAR3D_DATA 31 //NO DATA: 00; NO_SOLUTION: 01; FLOAT: 10; FIXED: 11

// #define SBUF_SIZE 64
// #define GOOD_Attitude_Range 10
// #define GOOD_Yaw_Range 10
// #define EBI_MAX_UNCHANGE 30

class ORC_Drone_PumpHub
{
  public:
    ORC_Drone_PumpHub(uint64_t rfReadId, uint64_t rfSendId); //Ham khoi tao
	// void LedGPSDisplay(long RTKStatus); //Hien thi GPS led
	bool PumpHubRun(); //Ham chay HUB Chinh
	bool PumpHubSetup();//Ham setup hub
	// bool SplitData2LongInStruct(String stringIn, orcstring_data *structIn);//Ham tach du lieu sang long
	bool PumpHubInSentOut();//Ham goi du lieu ra ngoai
	bool PumpHubInScan();//Ham scan tat ca du lieu
	void PumpHubDataProcessing(orcstring_data *pumpControlDataVar, orcstring_data *pumpFeedbackDataVar); //Ham xu ly du lieu
	ORC_Drone_PWM PumpPWM;
	void PumpControlOut(long flowIn); //Xuat tin hieu dieu khien pump
	void PumpControl(orcstring_data pumpDataIn);//Dieu khien pump
	// orcstring_data droneAngleData, dronePositionData, droneGPSData, dronePumpControlData, dronePumpFBData, droneMainWarningData;
	// orcgps_data gpsLongData;
  private:
	void RFLedDisplay(long *preMilliTime,bool stateIn);
    ORC_Drone_RF *DroneRFSend;
	ORC_Utilities ORCUti;
	orcstring_data droneAngleData, dronePositionData, droneGPSData, dronePumpControlData, dronePumpFBData, droneObstacleControlData,droneObstacleRawData,droneMainWarningData;
	// long currentTime, timeCnt;
	// String AttitudeDataPrefix = "*A";
	// String TSWarningDataPrefix = "*T";
	long _preMilliTime;
	
	
};



#endif