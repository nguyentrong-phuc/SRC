/*------------------------------------------------------------
  ORC_Drone_RF.h - Library for manipulting with PWM data.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
  PWM_min = 0; PWM_max = 1800
-------------------------------------------------------------*/

#ifndef ORC_Drone_RF_h
#define ORC_Drone_RF_h

#include "Arduino.h"
#include "ORCType.h"
#include<SPI.h>
#include<RF24.h>
#define RF_Read_Cnt_Max	300

#if defined(STM32F1)
  #define CE_PIN PB12
  #define CSN_PIN PA15
  #define SCK_PIN PB13
  #define MISO_PIN PB14
  #define MOSI_PIN PB15
#endif

class ORC_Drone_RF
{
  public:
    ORC_Drone_RF(uint8_t cepin, uint8_t cspin, uint64_t readingAddress, uint64_t writingAddress); //Khao khoi tao
	bool RFReadSetup(); //Setup che do Read
	bool RFSendSetup(); //Setup che do Send
	bool RFSendData(); //Ham send du lieu
	int RFReadData(orcdrone_data *droneData); //Read du lieu RF
	
		
		
	//byte sendingData[35] = {0};
	esp32RFMessage sendingData;
	long readingData[11] = {0};
	//long Rec[9];

  private:
	bool flagHoverPoint;
    unsigned int _pin1;
	//RF24 radio(9, 53); //CE, CS
	RF24 *_radio; //CE, CS
	unsigned int _rfReadErr;
	uint64_t _readingAddress, _writingAddress; //Bien cuc bo dia chi ken RF
	//Các biến trạng thái chế độ
	bool _isSendMode = false, _isReadMode = false, _isSendModeSetup = false, _isReadModeSetup = false;
	
	
	//Cac ham con xu ly
	
	int DataControl(long *dataRecFromTX, long pointOut[][POINT_MEMORY_MAX_LEN], orcdrone_data *drone_in);
	void commandProcessing(long commandCode, orcdrone_data *drone_in);
	void flyModeProcessing(long flyMode_in, orcdrone_data *drone_in);
	void FlyingPointPlanning(long pointIn[][POINT_MEMORY_MAX_LEN], long pointOut[][POINT_MEMORY_MAX_LEN], orcdrone_data *drone_in);
	void FlyingPointPlanningAlgorithm(long pointIn[][POINT_MEMORY_MAX_LEN], orcdrone_data *drone_in);
	long MultipleLongFloat2Long(long longIn, float floatIn);
};



#endif