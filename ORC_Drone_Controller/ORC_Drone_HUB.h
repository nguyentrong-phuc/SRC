/*
  ORC_Drone_HUB.h - Library for navigating data.
  Created by Dang Xuan Ba, Nov 14, 2024.
  Updated by Dang Xuan Ba, Nov 14, 2024.
  Released into the public domain.
*/

#ifndef ORC_Drone_HUB_h
#define ORC_Drone_HUB_h

#include "Arduino.h"
#include<ORC_Utilities.h>
#include<ORCType.h>

#define MAINPort	Serial1  // Main
#define GPSPort		Serial2  // GPS
#define PUMPPort	Serial3  // Pump Hub
#define ET			0.01 //10ms

#define LED_RF_SEND	8
#define LED_PROGRAM_READY 30 // 29
// #define LED_GPS_DATA_00  31 // 30
// #define LED_GPS_DATA_01  29 // 31 NO DATA: 00; NO_SOLUTION: 01; FLOAT: 10; FIXED: 11
#define LED_GPS_DATA  31 // 30
#define LED_PUMP_DATA  29 // 31 NO DATA: 00; NO_SOLUTION: 01; FLOAT: 10; FIXED: 11
#define BASE_STABLE_MAX 10 //10 lan
#define BASE_STABLE_DEVIATION 4 //[cm]
// #define SBUF_SIZE 64
// #define GOOD_Attitude_Range 10
// #define GOOD_Yaw_Range 10
// #define EBI_MAX_UNCHANGE 30

class ORC_Drone_HUB
{
  public:
    ORC_Drone_HUB(); //Ham khoi tao
	void LedGPSDisplay(long RTKStatus); //Hien thi GPS led
	bool HUBRun(); //Ham chay HUB Chinh
	bool HUBSetup(long GPSRTK_Standard);//Ham setup hub
	bool SplitData2LongInStruct(String stringIn, orcstring_data *structIn);//Ham tach du lieu sang long
	bool inSentOut(long *timeGPSIOTCnt);//Ham goi du lieu ra ngoai
	bool inScan();//Ham scan tat ca du lieu
		
	orcstring_data droneAngleData, dronePositionData, droneGPSData, dronePumpControlData, dronePumpFBData, droneMainWarningData, droneObstacleControlData;
	orcgps_data gpsLongData;
  private:
    void AllLedDisplay();
	ORC_Utilities ORCUti;
	bool GPSBaseRequest = false;	
	long currentTime, timeCnt;
	long _pre_base_gps_long[4];
	long _pre_base_dev_cm[4];
	uint8_t _baseStableCnt;
	// String AttitudeDataPrefix = "*A";
	// String TSWarningDataPrefix = "*T";
	
};



#endif