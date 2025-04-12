/*------------------------------------------------------------
  ORC_Drone_RF.h - Library for manipulting with PWM data.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
  PWM_min = 0; PWM_max = 1800
-------------------------------------------------------------*/

#include "Arduino.h"
#include "ORC_Drone_RF.h"
#include "ORCType.h"
#include<SPI.h>
#include<RF24.h>
/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_RF::ORC_Drone_RF(uint8_t cepin, uint8_t cspin, uint64_t readingAddress, uint64_t writingAddress)
{
  _readingAddress = readingAddress; //Set reading Address;
  _writingAddress = writingAddress; //Set writing Address;
  _rfReadErr = 0;//So lan nhan sai
  _radio = new RF24(cepin, cspin);
  flagHoverPoint = true;
}
//----------------------BEGIN OF RF FUNCTIONS--------------------------------------------------------
/*----------------------------------------------------------------------------------------------
 * CAI DAT CHE DO READING CHO RF
------------------------------------------------------------------------------------------------*/
bool ORC_Drone_RF::RFReadSetup()
{
  #if defined(STM32F1)
    SPI.setMOSI(MOSI_PIN);
    SPI.setMISO(MISO_PIN);
    SPI.setSCLK(SCK_PIN);
    SPI.begin();
  #endif
  //Starting the radio communication
  if(_radio->begin()){
	  _radio->openReadingPipe(1, _readingAddress); //Setup  Addresses send data        (new) set up thêm dòng này
	  _radio->openWritingPipe(_writingAddress); //Setup  Addresses receive data

	  _radio->setPALevel(RF24_PA_MIN);   //You can set it as minimum or maximum depending on the distance between the Sendmitter and receiver.        
	  _radio->setChannel(0x77);
	  _radio->enableDynamicPayloads();  
	  _radio->setDataRate(RF24_1MBPS); 
	  _radio->startListening();
	  //Bien bao che do
	  _isReadMode = true;
	  _isReadModeSetup = true;
	  _isSendMode = false;
	  _isSendModeSetup = false;
	  return true;
  }else return false;
}
/*----------------------------------------------------------------------------------------------
 * CAI DAT CHE DO SENDING CHO RF
------------------------------------------------------------------------------------------------*/
bool ORC_Drone_RF::RFSendSetup()
{
  #if defined(STM32F1)
    SPI.setMOSI(MOSI_PIN);
    SPI.setMISO(MISO_PIN);
    SPI.setSCLK(SCK_PIN);
    SPI.begin();
  #endif
  //Starting the radio communication
  if(_radio->begin()){
	  _radio->openReadingPipe(1, _readingAddress); //Setup  Addresses send data        (new) set up thêm dòng này
	  _radio->openWritingPipe(_writingAddress); //Setup  Addresses receive data

	  _radio->setPALevel(RF24_PA_MIN);   //You can set it as minimum or maximum depending on the distance between the Sendmitter and receiver.        
	  _radio->setChannel(0x77);
	  _radio->enableDynamicPayloads();  
	  _radio->setDataRate(RF24_1MBPS); 
	  //Bien bao che do
	  _isReadMode = false;
	  _isReadModeSetup = false;
	  _isSendMode = true;
	  _isSendModeSetup = true;
	  //radio.startListening();
	  return true;
  }else return false;
}

/*----------------------------------------------------------------------------------------------
 * HAM GOI DU LIEU RF: chi goi du lieu o che do send mode
------------------------------------------------------------------------------------------------*/
bool ORC_Drone_RF::RFSendData()
{
  // Serial.print("_isSendMode: ");  Serial.println(_isSendMode);
  // Serial.print("_isSendModeSetup: ");  Serial.println(_isSendModeSetup);
  if(_isSendMode && _isSendModeSetup){ //Chi thuc hien o che do send va da khoi dong dc he thong
    
    bool flag = _radio->write(&sendingData, sizeof(sendingData));
    // Serial.println(flag);
    return flag;
	//  return _radio->write(&sendingData, sizeof(sendingData));
  } else
	return	false;	
}


/*--------------------------------------
 * Ham xu ly nut nhat Start Stop Emer
---------------------------------------*/
void ORC_Drone_RF::commandProcessing(long commandCode, orcdrone_data *drone_in){
	////-----CAI DAT AN TOAN--------------
	//Chi cho phep bay khi nut start duoc nhan va nut Emer duoc nhan
	
  // -------------------- BUTTON PROCESSING  AND LED MODE ---------------------------------------------------------------------------------
  if (commandCode == RF_EMERPRESS) { // EMERGENCY MODE
    drone_in->desiredData.EmerCommand = true; 
	drone_in->desiredData.StartCommand = false;
	drone_in->desiredData.StopCommand = false;
	//Cho phep an toan
	if(drone_in->FlyingEnable < FlyingEnableThres) //Truong hop dang khoi dong
	{	
		if(drone_in->FlyingEnable == 1)
			drone_in->FlyingEnable = FlyingEnableThres;
	}
	
  } else{
    if (commandCode == RF_STOPPRESS) {   // STOP is Press   
      if(drone_in->desiredData.StartCommand){
        drone_in->desiredData.EmerCommand = false; 
		drone_in->desiredData.StartCommand = false;
		drone_in->desiredData.StopCommand = true;
      }
    }else {
      if (commandCode == RF_STARTPRESS) { // Start is Press
        drone_in->desiredData.EmerCommand = false; 
		drone_in->desiredData.StartCommand = true;
		drone_in->desiredData.StopCommand = false; 
		//Cho phep an toan
		if(drone_in->FlyingEnable < FlyingEnableThres) //Truong hop dang khoi dong
			drone_in->FlyingEnable = 1;
		
      }
    }
  }   
}
/*--------------------------------------
 * Ham xu ly switch  Fly mode
---------------------------------------*/
void ORC_Drone_RF::flyModeProcessing(long flyMode_in, orcdrone_data *drone_in){ 

   // -------------------- PROCESSING  FLYING MODE ---------------------------------------------------------------------------------
  if (flyMode_in == FLY_MODE_MANUAL_POSITION) { // FLYING MANUAL MODE FOR POSITION
    drone_in->desiredData.flyMode = FLY_MODE_MANUAL_POSITION;    
  } else
  {
    if (flyMode_in == FLY_MODE_MANUAL_ANGLE) { // FLYING MANUAL MODE FOR ANGLE
      drone_in->desiredData.flyMode = FLY_MODE_MANUAL_ANGLE;      
    }
    else {
      if (flyMode_in == FLY_MODE_ONE_POINT)
      {
        drone_in->desiredData.flyMode = FLY_MODE_ONE_POINT;        
      } else
      {
        if (flyMode_in == FLY_MODE_MULTI_POINT)
        {
          drone_in->desiredData.flyMode = FLY_MODE_MULTI_POINT;          
        }
      }
    }
  }
}
/*----------------------------------------------------------
 * POINT PLANNING FOR MULTIPLE POINT AUTOMATIC CONTROL:
 * CHI THUC HIEN KHI O CHE DO FLYING_MODE == MULTIPLE POINT, LEN_DATA > 1, COMMAND == EMER
 * 
-----------------------------------------------------------*/
void ORC_Drone_RF::FlyingPointPlanningAlgorithm(long pointIn[][POINT_MEMORY_MAX_LEN], orcdrone_data *drone_in){
  int cPointIn = 0, cPointOut = 0, lenPointOut = 0, iTemp = 0;  
  long pointOut[MaxPtoPnum][POINT_MEMORY_MAX_LEN];
  if((drone_in->desiredData.flyMode == FLY_MODE_MULTI_POINT)&&(drone_in->desiredData.EmerCommand==true)&&(pointIn[0][POINT_MEMORY_INDEX_LEN]>1)){
    cPointOut = 0;
    //----------------------------------Them Hover Point vao du lieu ra---------------------------------
    pointOut[cPointOut][POINT_MEMORY_INDEX_XD]= drone_in->gpsdata.pos_cm[0]; //Them x tai home in [cm]
    pointOut[cPointOut][POINT_MEMORY_INDEX_YD]= drone_in->gpsdata.pos_cm[1]; //them y tai hom in [cm]
    pointOut[cPointOut][POINT_MEMORY_INDEX_ZD]= Z_HOVER_MULTI_POINT; // them z tai home
    pointOut[cPointOut][POINT_MEMORY_INDEX_TIME]= TXY_AUTO_DEFAULT; ///default time
    pointOut[cPointOut][POINT_MEMORY_INDEX_LEN]= 0; //Tam thoi cho len = 0
    pointOut[cPointOut][POINT_MEMORY_INDEX_MISSION_STATUS]= -Fly_DONE; // Chua xong
    pointOut[cPointOut][POINT_MEMORY_INDEX_INTERNET_ID]= NO_INTERNET_MULTI_POINT; //Diem tam
    pointOut[cPointOut][POINT_MEMORY_INDEX_PUMPFLOW]= 0; // No pump
    lenPointOut = 1;

    //-------------------------Copy du lieu diem vao du lieu bay-----------------------------------------
    for(cPointIn = 0; cPointIn < pointIn[0][POINT_MEMORY_INDEX_LEN]; cPointIn++){
      //Them 1 diem trung gian neu |Z_HOVER_MULTI_POINT - Z_DIEM_MONG_MUON| > 50 (cm)
     // #ifdef flagHoverPoint
        if(abs(pointOut[cPointOut][POINT_MEMORY_INDEX_ZD] - pointIn[cPointIn][POINT_MEMORY_INDEX_ZD]) >= 50){
          cPointOut++; //Them 1 diem moi
          // Z_hover > Z_target1
          if(pointOut[cPointOut-1][POINT_MEMORY_INDEX_ZD] > pointIn[cPointIn][POINT_MEMORY_INDEX_ZD]){ //Quy hoach toi diem thap hon: giu nguyen z, lay diem x,y moi
            pointOut[cPointOut][POINT_MEMORY_INDEX_XD]= pointIn[cPointIn][POINT_MEMORY_INDEX_XD]; //Lay vi tri x moi
            pointOut[cPointOut][POINT_MEMORY_INDEX_YD]= pointIn[cPointIn][POINT_MEMORY_INDEX_YD]; //Lay vi tri y moi
            pointOut[cPointOut][POINT_MEMORY_INDEX_ZD]= pointOut[cPointOut - 1][POINT_MEMORY_INDEX_ZD]; // giu nguyen z          
          }else{ //Quy hoach toi diem cap hon: giu nguyen x, y, lay diem z
            pointOut[cPointOut][POINT_MEMORY_INDEX_XD]= pointOut[cPointOut - 1][POINT_MEMORY_INDEX_XD]; //Giu nguyen x
            pointOut[cPointOut][POINT_MEMORY_INDEX_YD]= pointOut[cPointOut - 1][POINT_MEMORY_INDEX_YD]; //Giu nguyen y
            pointOut[cPointOut][POINT_MEMORY_INDEX_ZD]= pointIn[cPointIn][POINT_MEMORY_INDEX_ZD]; //Lay vi tri z moi
          }
          //Copy du lieu cac diem con lai
          pointOut[cPointOut][POINT_MEMORY_INDEX_TIME]= TXY_AUTO_DEFAULT; //Default time
          pointOut[cPointOut][POINT_MEMORY_INDEX_LEN]= 0; //Tam thoi cho len = 0
          pointOut[cPointOut][POINT_MEMORY_INDEX_MISSION_STATUS]= -Fly_DONE; // Chua xong
          pointOut[cPointOut][POINT_MEMORY_INDEX_INTERNET_ID]= NO_INTERNET_MULTI_POINT; //Diem tam
          pointOut[cPointOut][POINT_MEMORY_INDEX_PUMPFLOW]= (cPointIn == 0)?0:pointIn[cPointIn][POINT_MEMORY_INDEX_PUMPFLOW]; // No pump  
          lenPointOut++; //Tang do dai len 1 don vi     
        }
    //  #endif
      //Copy du lieu vao diem moi
      cPointOut++; //Them 1 diem moi
      
      pointOut[cPointOut][POINT_MEMORY_INDEX_XD]= pointIn[cPointIn][POINT_MEMORY_INDEX_XD]; //Copy x
      pointOut[cPointOut][POINT_MEMORY_INDEX_YD]= pointIn[cPointIn][POINT_MEMORY_INDEX_YD]; //Copy y
      pointOut[cPointOut][POINT_MEMORY_INDEX_ZD]= pointIn[cPointIn][POINT_MEMORY_INDEX_ZD]; // Copy y
      pointOut[cPointOut][POINT_MEMORY_INDEX_TIME]= pointIn[cPointIn][POINT_MEMORY_INDEX_TIME]; //Copy time
      pointOut[cPointOut][POINT_MEMORY_INDEX_LEN]= pointIn[cPointIn][POINT_MEMORY_INDEX_LEN]; //Copy len
      pointOut[cPointOut][POINT_MEMORY_INDEX_MISSION_STATUS]= pointIn[cPointIn][POINT_MEMORY_INDEX_MISSION_STATUS]; // Copy status done
      pointOut[cPointOut][POINT_MEMORY_INDEX_INTERNET_ID]= pointIn[cPointIn][POINT_MEMORY_INDEX_INTERNET_ID]; //copy internet ID
      pointOut[cPointOut][POINT_MEMORY_INDEX_PUMPFLOW]= pointIn[cPointIn][POINT_MEMORY_INDEX_PUMPFLOW]; // Copy pump data 
		lenPointOut++; //Tang do dai len 1 don vi 
    }
    //-----------------THEM DU LIEU VE LAI DIEM HOVER-------------------------------------
    //Them 1 diem trung gian neu |Z_HOVER_MULTI_POINT - Z_DIEM_MONG_MUON| > 50 (cm)
    if(abs(pointOut[cPointOut][POINT_MEMORY_INDEX_ZD] - pointOut[0][POINT_MEMORY_INDEX_ZD]) >= 50){
      cPointOut++; //Them 1 diem moi
      
      if(pointOut[cPointOut-1][POINT_MEMORY_INDEX_ZD] > pointOut[0][POINT_MEMORY_INDEX_ZD]){ //Quy hoach toi diem thap hon: giu nguyen z, lay diem x,y moi
        pointOut[cPointOut][POINT_MEMORY_INDEX_XD]= pointOut[0][POINT_MEMORY_INDEX_XD]; //Lay vi tri x moi
        pointOut[cPointOut][POINT_MEMORY_INDEX_YD]= pointOut[0][POINT_MEMORY_INDEX_YD]; //Lay vi tri x moi
        pointOut[cPointOut][POINT_MEMORY_INDEX_ZD]= pointOut[cPointOut - 1][POINT_MEMORY_INDEX_ZD]; // giu nguyen z          
      }else{ //Quy hoach toi diem cap hon: giu nguyen x, y, lay diem z
        pointOut[cPointOut][POINT_MEMORY_INDEX_XD]= pointOut[cPointOut - 1][POINT_MEMORY_INDEX_XD]; //Giu nguyen x
        pointOut[cPointOut][POINT_MEMORY_INDEX_YD]= pointOut[cPointOut - 1][POINT_MEMORY_INDEX_YD]; //Giu nguyen y
        pointOut[cPointOut][POINT_MEMORY_INDEX_ZD]= pointOut[0][POINT_MEMORY_INDEX_ZD]; //Lay vi tri z moi
      }
      //Copy du lieu cac diem con lai
      pointOut[cPointOut][POINT_MEMORY_INDEX_TIME]= TXY_AUTO_DEFAULT; ///default time
      pointOut[cPointOut][POINT_MEMORY_INDEX_LEN]= 0; //Tam thoi cho len = 0
      pointOut[cPointOut][POINT_MEMORY_INDEX_MISSION_STATUS]= -Fly_DONE; // Chua xong
      pointOut[cPointOut][POINT_MEMORY_INDEX_INTERNET_ID]= NO_INTERNET_MULTI_POINT; //Diem tam
      pointOut[cPointOut][POINT_MEMORY_INDEX_PUMPFLOW]= 0; // No pump
       lenPointOut++; //Tang do dai len 1 don vi    
    }
    //Copy diem hover vao diem cuoi cung
    cPointOut++; //Them 1 diem moi
    lenPointOut++; //Tang do dai len 1 don vi
    for(iTemp = 0; iTemp < POINT_MEMORY_MAX_LEN; iTemp++){
      pointOut[cPointOut][iTemp] = pointOut[0][iTemp];
    }
    // Cap nhat len moi cho tat ca cac du lieu
    for(iTemp = 0; iTemp < lenPointOut; iTemp++){
      pointOut[iTemp][POINT_MEMORY_INDEX_LEN] = (long)lenPointOut;
    }
   // Serial.print("=============>Sau khi qui hoach:"); Serial.print(lenPointOut);
    // Copy lai du lieu cho bo nho goc
	iTemp = 0;
    for(int oTemp = 0; oTemp < MaxPtoPnum; oTemp++)
      for(iTemp = 0; iTemp < POINT_MEMORY_MAX_LEN; iTemp++)
      {
        pointIn[oTemp][iTemp]= pointOut[oTemp][iTemp];
      }
  }
}
/*----------------------------------------------------------
 * POINT PLANNING FOR MULTIPLE POINT AUTOMATIC CONTROL:
 * CHI THUC HIEN KHI O CHE DO FLYING_MODE == MULTIPLE POINT, LEN_DATA > 1, COMMAND == EMER
 * 
-----------------------------------------------------------*/
void ORC_Drone_RF::FlyingPointPlanning(long pointIn[][POINT_MEMORY_MAX_LEN], long pointOut[][POINT_MEMORY_MAX_LEN], orcdrone_data *drone_in){
  int cPointIn = 0, cPointOut = 0, lenPointOut = 0, iTemp = 0;  
  
  if((drone_in->desiredData.flyMode == FLY_MODE_MULTI_POINT)&&(drone_in->desiredData.EmerCommand==true)&&(pointIn[0][POINT_MEMORY_INDEX_LEN]>1)){
    cPointOut = 0;
    //----------------------------------Them Hover Point vao du lieu ra---------------------------------
    pointOut[cPointOut][POINT_MEMORY_INDEX_XD]= ((float)(drone_in->gpsdata.pos_cm[0]))/100; //Them x
    pointOut[cPointOut][POINT_MEMORY_INDEX_YD]= ((float)(drone_in->gpsdata.pos_cm[1]))/100; //them y
    pointOut[cPointOut][POINT_MEMORY_INDEX_ZD]= Z_HOVER_MULTI_POINT; // them z
    pointOut[cPointOut][POINT_MEMORY_INDEX_TIME]= 0; //No time
    pointOut[cPointOut][POINT_MEMORY_INDEX_LEN]= 0; //Tam thoi cho len = 0
    pointOut[cPointOut][POINT_MEMORY_INDEX_MISSION_STATUS]= -Fly_DONE; // Chua xong
    pointOut[cPointOut][POINT_MEMORY_INDEX_INTERNET_ID]= NO_INTERNET_MULTI_POINT; //Diem tam
    pointOut[cPointOut][POINT_MEMORY_INDEX_PUMPFLOW]= 0; // No pump
    lenPointOut = 1;

    //-------------------------Copy du lieu diem vao du lieu bay-----------------------------------------
    for(cPointIn = 0; cPointIn < pointIn[0][POINT_MEMORY_INDEX_LEN]; cPointIn++){
      //Them 1 diem trung gian neu |Z_HOVER_MULTI_POINT - Z_DIEM_MONG_MUON| > 50 (cm)
      if(abs(pointOut[cPointOut][POINT_MEMORY_INDEX_ZD] - pointIn[cPointIn][POINT_MEMORY_INDEX_ZD]) >= 50){
        cPointOut++; //Them 1 diem moi
        if(pointOut[cPointOut-1][POINT_MEMORY_INDEX_ZD] > pointIn[cPointIn][POINT_MEMORY_INDEX_ZD]){ //Quy hoach toi diem thap hon: giu nguyen z, lay diem x,y moi
          pointOut[cPointOut][POINT_MEMORY_INDEX_XD]= pointIn[cPointIn][POINT_MEMORY_INDEX_XD]; //Lay vi tri x moi
          pointOut[cPointOut][POINT_MEMORY_INDEX_YD]= pointIn[cPointIn][POINT_MEMORY_INDEX_YD]; //Lay vi tri x moi
          pointOut[cPointOut][POINT_MEMORY_INDEX_ZD]= pointOut[cPointOut - 1][POINT_MEMORY_INDEX_ZD]; // giu nguyen z          
        }else{ //Quy hoach toi diem cap hon: giu nguyen x, y, lay diem z
          pointOut[cPointOut][POINT_MEMORY_INDEX_XD]= pointOut[cPointOut - 1][POINT_MEMORY_INDEX_XD]; //Giu nguyen x
          pointOut[cPointOut][POINT_MEMORY_INDEX_YD]= pointOut[cPointOut - 1][POINT_MEMORY_INDEX_YD]; //Giu nguyen y
          pointOut[cPointOut][POINT_MEMORY_INDEX_ZD]= pointIn[cPointIn][POINT_MEMORY_INDEX_ZD]; //Lay vi tri z moi
        }
        //Copy du lieu cac diem con lai
        pointOut[cPointOut][POINT_MEMORY_INDEX_TIME]= 0; //No time
        pointOut[cPointOut][POINT_MEMORY_INDEX_LEN]= 0; //Tam thoi cho len = 0
        pointOut[cPointOut][POINT_MEMORY_INDEX_MISSION_STATUS]= -Fly_DONE; // Chua xong
        pointOut[cPointOut][POINT_MEMORY_INDEX_INTERNET_ID]= NO_INTERNET_MULTI_POINT; //Diem tam
        pointOut[cPointOut][POINT_MEMORY_INDEX_PUMPFLOW]= pointIn[cPointIn][POINT_MEMORY_INDEX_PUMPFLOW]; // No pump  
        lenPointOut++; //Tang do dai len 1 don vi     
      }
      //Copy du lieu vao diem moi
      cPointOut++; //Them 1 diem moi
      
      pointOut[cPointOut][POINT_MEMORY_INDEX_XD]= pointIn[cPointIn][POINT_MEMORY_INDEX_XD]; //Copy x
      pointOut[cPointOut][POINT_MEMORY_INDEX_YD]= pointIn[cPointIn][POINT_MEMORY_INDEX_YD]; //Copy y
      pointOut[cPointOut][POINT_MEMORY_INDEX_ZD]= pointIn[cPointIn][POINT_MEMORY_INDEX_ZD]; // Copy y
      pointOut[cPointOut][POINT_MEMORY_INDEX_TIME]= pointIn[cPointIn][POINT_MEMORY_INDEX_TIME]; //Copy time
      pointOut[cPointOut][POINT_MEMORY_INDEX_LEN]= pointIn[cPointIn][POINT_MEMORY_INDEX_LEN]; //Copy len
      pointOut[cPointOut][POINT_MEMORY_INDEX_MISSION_STATUS]= pointIn[cPointIn][POINT_MEMORY_INDEX_MISSION_STATUS]; // Copy status done
      pointOut[cPointOut][POINT_MEMORY_INDEX_INTERNET_ID]= pointIn[cPointIn][POINT_MEMORY_INDEX_INTERNET_ID]; //copy internet ID
      pointOut[cPointOut][POINT_MEMORY_INDEX_PUMPFLOW]= pointIn[cPointIn][POINT_MEMORY_INDEX_PUMPFLOW]; // Copy pump data 
		lenPointOut++; //Tang do dai len 1 don vi 
    }
    //-----------------THEM DU LIEU VE LAI DIEM HOVER-------------------------------------
    //Them 1 diem trung gian neu |Z_HOVER_MULTI_POINT - Z_DIEM_MONG_MUON| > 50 (cm)
    if(abs(pointOut[cPointOut][POINT_MEMORY_INDEX_ZD] - pointOut[0][POINT_MEMORY_INDEX_ZD]) >= 50){
      cPointOut++; //Them 1 diem moi
      
      if(pointOut[cPointOut-1][POINT_MEMORY_INDEX_ZD] > pointOut[0][POINT_MEMORY_INDEX_ZD]){ //Quy hoach toi diem thap hon: giu nguyen z, lay diem x,y moi
        pointOut[cPointOut][POINT_MEMORY_INDEX_XD]= pointOut[0][POINT_MEMORY_INDEX_XD]; //Lay vi tri x moi
        pointOut[cPointOut][POINT_MEMORY_INDEX_YD]= pointOut[0][POINT_MEMORY_INDEX_YD]; //Lay vi tri x moi
        pointOut[cPointOut][POINT_MEMORY_INDEX_ZD]= pointOut[cPointOut - 1][POINT_MEMORY_INDEX_ZD]; // giu nguyen z          
      }else{ //Quy hoach toi diem cap hon: giu nguyen x, y, lay diem z
        pointOut[cPointOut][POINT_MEMORY_INDEX_XD]= pointOut[cPointOut - 1][POINT_MEMORY_INDEX_XD]; //Giu nguyen x
        pointOut[cPointOut][POINT_MEMORY_INDEX_YD]= pointOut[cPointOut - 1][POINT_MEMORY_INDEX_YD]; //Giu nguyen y
        pointOut[cPointOut][POINT_MEMORY_INDEX_ZD]= pointOut[0][POINT_MEMORY_INDEX_ZD]; //Lay vi tri z moi
      }
      //Copy du lieu cac diem con lai
      pointOut[cPointOut][POINT_MEMORY_INDEX_TIME]= 0; //No time
      pointOut[cPointOut][POINT_MEMORY_INDEX_LEN]= 0; //Tam thoi cho len = 0
      pointOut[cPointOut][POINT_MEMORY_INDEX_MISSION_STATUS]= -Fly_DONE; // Chua xong
      pointOut[cPointOut][POINT_MEMORY_INDEX_INTERNET_ID]= NO_INTERNET_MULTI_POINT; //Diem tam
      pointOut[cPointOut][POINT_MEMORY_INDEX_PUMPFLOW]= 0; // No pump
       lenPointOut++; //Tang do dai len 1 don vi    
    }
    //Copy diem hover vao diem cuoi cung
    cPointOut++; //Them 1 diem moi
    lenPointOut++; //Tang do dai len 1 don vi
    for(iTemp = 0; iTemp < POINT_MEMORY_MAX_LEN; iTemp++){
      pointOut[cPointOut][iTemp] = pointOut[0][iTemp];
    }
    // Cap nhat len moi cho tat ca cac du lieu
    for(iTemp = 0; iTemp < lenPointOut; iTemp++){
      pointOut[iTemp][POINT_MEMORY_INDEX_LEN] = (long)lenPointOut;
    }
  }
  
}

/*----------------------------------------------------------------------------------------------
 * HAM DOC DU LIEU RF: chi doc du lieu o che do read mode
------------------------------------------------------------------------------------------------*/
int ORC_Drone_RF::RFReadData(orcdrone_data *droneData)
{
	int result = RF_NODATA_RECEIVED;
	if(_isReadMode && _isReadModeSetup){ //Chi thuc hien o che do read va da khoi dong dc he thong
		if (_radio->available()){
			//Serial.print("Nhan duoc RF:");
			byte receivedData[45]; // Mảng byte để lưu trữ dữ liệu nhận được
			int temp1 = 0, temp2 = 0;
			_radio->read(&receivedData, sizeof(receivedData)); //Dọc du lieu RF

      // Serial.print("reading:");
      // for(int i = 0; i < sizeof(receivedData); i++){
      //   if(i < (sizeof(receivedData) - 1)){
      //     Serial.print(receivedData[i]); Serial.print(",");
      //   }else Serial.println(receivedData[i]);

      // }

			//_rfReadErr = 0; //Reset lai bien theo doi RF
			droneData->desiredData.RFErrorCnt = 0;
			//Serial.println(receivedData[0]);
			if(receivedData[0] == '*'){//'*'

				// -----------  checksum ---------------
				// tổng giá trị của các byte data
				long checksum;
				memcpy(&checksum, &receivedData[24], sizeof(long));// tong gia tri cua du lieu		
				// tach du lieu thanh cac cum long
				long sumbyte =0, dataRec[6];
				for(int i = 0; i<6;i++){
					memcpy(&dataRec[i], &receivedData[i*4], sizeof(long));
				}
				//Tinh checksum tu du lieu
				for(int i = 0; i<24;i++){
					sumbyte += receivedData[i];
				}
				//Xu ly khi dung du lieu
				// Serial.print("Checksum:  "); Serial.print(checksum);
				if(sumbyte == checksum){
          
          
					temp1 = (int)receivedData[3]; //Bt state, BT mode
					temp2 = temp1%10;  //bt state
					temp1 = (temp1 - temp2)/10; // bt mode
					      // Serial.print("---------------Receive 3: "); Serial.print(receivedData[3]);
					// Serial.print("=> status: "); Serial.print(temp1);
					      // Serial.print("=> button: "); Serial.println(temp2);


					readingData[0] = (long)temp1;// status
					readingData[1] = (long)temp2;// button
					readingData[2] = ((long)receivedData[1])+ ((long)receivedData[2])*256;// 0  -> ID of diem

					readingData[3] = dataRec[1];                          // Dx, longtitude
					readingData[4] = dataRec[2];                          // Dy, lattitude
					readingData[5] = dataRec[3];                          // z
					readingData[6] = dataRec[4];                          // time, yaw
					readingData[7] = static_cast<long> (receivedData[20]);// n_lane
					readingData[8] = static_cast<long> (receivedData[21]);// st_count
					readingData[9] = static_cast<long> (receivedData[22]);// flow
					readingData[10] = static_cast<long> (receivedData[23]);// pump

          //Luu lai trang thai pump manual
          droneData->desiredData.pumpData.preManualPumpCommand = droneData->desiredData.pumpData.curManualPumpCommand;
          droneData->desiredData.pumpData.curManualPumpCommand = (readingData[10] > 0)?true:false;
          //droneData->pumpData.isPumpRequired = droneData->pumpData.curManualPumpCommand && (!droneData->pumpData.preManualPumpCommand);
					
          // for (int i = 0; i< 10; i++)
          // {
            // Serial.print(readingData[i]); Serial.print(", ");
          // }
		  // Serial.println();
          // Serial.println(readingData[3]);

					result = DataControl(readingData,droneData->desiredData.data_PtoP_Control,droneData);
					// result = true;
				}
			}
		}
		else 
		{
			droneData->desiredData.RFErrorCnt++;
			if((droneData->desiredData.RFErrorCnt) >= RF_Read_Cnt_Max)
				((droneData->desiredData.RFErrorCnt) = RF_Read_Cnt_Max);		
		}
	} 	
	//Tra ve ket qua	
	return result;
}
/*----------------------------------------------------------
 * COPY RF DATA TO DRONE CONTROL DATA
-----------------------------------------------------------*/
int ORC_Drone_RF::DataControl(long *dataRecFromTX,long pointOut[][POINT_MEMORY_MAX_LEN], orcdrone_data *drone_in)
{
  long temp1, n_len, longTemp1, st_count;
	int result = RF_NODATA_RECEIVED;
  //  Serial.print("DataControl: "); Serial.println(dataRecFromTX[0]);
	switch(dataRecFromTX[0]){
      case 1://Manual control in Position
           // Rec[0]= dataRecFromTX[3]; // Dx
           // Rec[1]= dataRecFromTX[4]; // Dy
           // Rec[2]= dataRecFromTX[5]; // Yaw
           // Rec[3]= dataRecFromTX[6]; // z_des
           // Rec[4]= dataRecFromTX[1]; // button state
           // Rec[5]= FLY_MODE_MANUAL_POSITION;  // auto switch
            //-----------Cap nhat du lieu vao bien Drone-------------        
            //Serial.print("Mode 1:"); Serial.println(dataRecFromTX[3]);
            //Xu ly thong tin vi tri
            drone_in->desiredData.manual_xd = ((float)dataRecFromTX[3])/1000; // Dx: cm -> m
            drone_in->desiredData.manual_yd =  ((float)dataRecFromTX[4])/1000; // Dy:  cm -> m
            drone_in->desiredData.manual_yawd= dataRecFromTX[5]; // Yaw
            drone_in->desiredData.manual_zd = dataRecFromTX[6]; // z_des
            //--Xy ly nut nhat start stop emer
            longTemp1 = dataRecFromTX[1];  // button state
            commandProcessing(longTemp1, drone_in);
            //--Xu ly che do bay
            longTemp1 = FLY_MODE_MANUAL_POSITION;  // Pos Manual Mode
            flyModeProcessing(longTemp1, drone_in);
            result = RF_DATA_RECEIVED_NO_ID;
        break;
      case 2://Auto control  Note: data_PtoP[MaxPtoPnum][6]={0};  //P = []{x, y, z, t, n_len,done, note}

//        data_PtoP[st_count][3] = dataRecFromTX[6]; //Get segmental flying time
//        data_PtoP[st_count][0] = dataRecFromTX[3]- gpsData.base_lat_long; //Get  x_des
//        data_PtoP[st_count][1] = dataRecFromTX[4]- gpsData.base_lon_long; //Get y_des
//        data_PtoP[st_count][2] = dataRecFromTX[5]; //Get segmental z_des
//        data_PtoP[st_count][4] = 1; //Insert numbers of points    
//        data_PtoP[st_count][5] = -Fly_DONE; //Insert numbers of points          
//         
//        Rec[3]= dataRecFromTX[5]; // z_des // cm
//        Rec[4]= dataRecFromTX[1]; // button state
//        Rec[5]= FLY_MODE_ONE_POINT;      // auto switch
//        //Goi lai thong tin phan hoi
//        RF_Response(st_count);Serial.println("SEnt response!!");
          st_count = 0;
          temp1 = dataRecFromTX[6];//Get segmental flying time
          if(!isnan(temp1))
          {
            pointOut[st_count][POINT_MEMORY_INDEX_TIME] = temp1; //Get segmental flying time
            temp1 = dataRecFromTX[3];
            
            if(!isnan(temp1)){
				//xd in [cm]
              pointOut[st_count][POINT_MEMORY_INDEX_XD] = MultipleLongFloat2Long((temp1 - drone_in->gpsdata.base_gps_long[0]),LAT_TO_CM); //Get segmental x_des in [m]
              temp1 = dataRecFromTX[4];
              
              if(!isnan(temp1)){
				 //yd in [cm]
                pointOut[st_count][POINT_MEMORY_INDEX_YD] = MultipleLongFloat2Long(-(temp1- drone_in->gpsdata.base_gps_long[1]), LON_TO_CM); //Get segmental y_des in [m]
                temp1 = dataRecFromTX[5];
                if(!isnan(temp1)){
                  pointOut[st_count][POINT_MEMORY_INDEX_ZD] = temp1; //Get segmental z_des in [cm]
                  pointOut[st_count][POINT_MEMORY_INDEX_LEN] = 1; //Insert numbers of points
                  pointOut[st_count][POINT_MEMORY_INDEX_MISSION_STATUS] = -Fly_DONE; //Insert numbers of points
                  
                  // Serial.print("z: "); Serial.println(pointOut[st_count][POINT_MEMORY_INDEX_ZD] );
                 // Rec[3]             = dataRecFromTX[5];  //z_des                       
                 // Rec[5]             = FLY_MODE_ONE_POINT;     // auto Point to Point switch
                  longTemp1 = FLY_MODE_ONE_POINT;  // Pos Manual Mode
                  flyModeProcessing(longTemp1, drone_in);
                  temp1 = dataRecFromTX[1];
                  if(!isnan(temp1)) {
					          //--Xy ly nut nhat start stop emer
					          commandProcessing(temp1, drone_in); // button state
					  
                     //Rec[4]             = temp1;  //button state
                     //state_auto_control = 1;static_pos_count = 0;
                     //Goi lai thong tin phan hoi
//                    RF_Response(st_count);Serial.println("SEnt response 2!!");                    

                  }
                }
              }
            }
          }
          result = RF_DATA_RECEIVED_NO_ID; // -1
        break;
      case 3://Auto control Point to Point   -> Note: data_PtoP[MaxPtoPnum][6]={0};  //P = []{x, y, z, t, n_len, note}
        result = RF_DATA_RECEIVED_NO_ID;
        temp1 = dataRecFromTX[1];
        // Serial.print(" button state mode 3: "); Serial.println(temp1);
        if(!isnan(temp1)) {
         //  Rec[4]   = temp1;  //button state
          commandProcessing(temp1, drone_in);
			    n_len = dataRecFromTX[7];  //n_len
			    if(!isnan(n_len))
              if((n_len <= MaxPtoPnum)&&(n_len > 1))  //ONLY working with limited points
              {
                // Serial.print("Auto_len: "); Serial.print(n_len);
                // Serial.print(" id: "); Serial.println(dataRecFromTX[8]);
                temp1 = dataRecFromTX[8]; //Get segmental frame
                if(!isnan(temp1))
                {
                  st_count = temp1; //Get segmental frame
                  if (st_count< n_len){
                    temp1 = dataRecFromTX[6];//Get segmental flying time
                  //Serial.print(" time: "); Serial.println(temp1);
                  if(!isnan(temp1))
                  {
                    pointOut[st_count][POINT_MEMORY_INDEX_TIME] = temp1; //Get segmental flying time
                    temp1 = dataRecFromTX[3];
                    //Serial.print("longtitude: "); Serial.println(temp1);
                    if(!isnan(temp1)){
                      //xd in [cm]
                      pointOut[st_count][POINT_MEMORY_INDEX_XD] = MultipleLongFloat2Long((temp1- drone_in->gpsdata.base_gps_long[0]),LAT_TO_CM); //Get segmental x_des
                      temp1 = dataRecFromTX[4];
                      // Serial.print("lattitude: "); Serial.println(temp1);
                      if(!isnan(temp1)){
                        //yd in [cm]
                        pointOut[st_count][POINT_MEMORY_INDEX_YD] = MultipleLongFloat2Long(-(temp1- drone_in->gpsdata.base_gps_long[1]),LON_TO_CM); //Get segmental y_des
                        temp1 = dataRecFromTX[5];
                        if(!isnan(temp1)){
                          pointOut[st_count][POINT_MEMORY_INDEX_ZD] = temp1; //Get segmental z_des
                          pointOut[st_count][POINT_MEMORY_INDEX_LEN] = n_len; //Insert numbers of points
                          pointOut[st_count][POINT_MEMORY_INDEX_MISSION_STATUS] = -Fly_DONE; //Insert numbers of points
                        // Rec[3]             = dataRecFromTX[5];  //z_des                       
                        // Rec[5]             = FLY_MODE_MULTI_POINT;     // auto Point to Point switch
                          longTemp1 = FLY_MODE_MULTI_POINT;  // Pos Manual Mode
                          flyModeProcessing(longTemp1, drone_in);
  //                        temp1 = dataRecFromTX[1];
  //                        //Serial.print(" button state: "); Serial.println(temp1);
  //                        if(!isnan(temp1)) {
  //                           Rec[4]   = temp1;  //button state
  //                           commandProcessing(temp1, drone_in);
                          //Xu ly gia tri pump flow
                          temp1 = dataRecFromTX[9];
                          if(!isnan(temp1)){
                            pointOut[st_count][POINT_MEMORY_INDEX_PUMPFLOW]=temp1; //Luu lai gia tri pump flow
							//Serial.print("Pump Auto: "); Serial.println(temp1);
                            //Goi lai thong tin phan hoi
                            if(drone_in->desiredData.EmerCommand) //Chi goi du lieu phan hoi khi o mat dat
                            {
                                result = st_count;
                  // Serial.println(st_count);
                            }
                            // Quy hoach lai cac diem bay khi nhan du du lieu
                            if(st_count == (n_len - 1))
                            {
								pointOut[st_count][POINT_MEMORY_INDEX_PUMPFLOW]=0; //diem cuoi cung se tat bom
                              // FlyingPointPlanning(pointTemp, pointOut, drone_in);
                              FlyingPointPlanningAlgorithm(pointOut, drone_in);
                            }
                            // break;
                          }                           
                        }
                      }
                    }
                  }
                }       

              }
            }
        }
        
        // Serial.print("Casse 3: ");
        // Serial.println(result);
        break;  
       case 4://Manual control for Angle
//      Serial.println("mode 111111");
          //  Rec[0]= dataRecFromTX[3]; // Dx
          //  Rec[1]= dataRecFromTX[4]; // Dy
          //  Rec[2]= dataRecFromTX[5]; // Yaw
          //  Rec[3]= dataRecFromTX[6]; // z_des
          //  Rec[4]= dataRecFromTX[1]; // button state
          //  Rec[5]= FLY_MODE_MANUAL_ANGLE;  // auto switch
            //-----------Cap nhat du lieu vao bien Drone-------------        
            
            //Xu ly thong tin vi tri
            drone_in->desiredData.manual_pitchd = ((float)dataRecFromTX[3])/10; // Pitch
            drone_in->desiredData.manual_rolld =  -((float)dataRecFromTX[4])/10; // Dy
            drone_in->desiredData.manual_yawd= dataRecFromTX[5]; // Yaw
            drone_in->desiredData.manual_zd = dataRecFromTX[6]; // z_des
            //--Xy ly nut nhat start stop emer
            longTemp1 = dataRecFromTX[1];  // button state
            commandProcessing(longTemp1, drone_in);
            //--Xu ly che do bay
            longTemp1 = FLY_MODE_MANUAL_ANGLE;  // Pos Manual Mode
            flyModeProcessing(longTemp1, drone_in);
            result = RF_DATA_RECEIVED_NO_ID;
        break;    
        default://Manual control for Angle
//      Serial.println("mode 111111");
//            Rec[0]= dataRecFromTX[3]; // Dx
//            Rec[1]= dataRecFromTX[4]; // Dy
//            Rec[2]= dataRecFromTX[5]; // Yaw
//            Rec[3]= dataRecFromTX[6]; // z_des
//            Rec[4]= dataRecFromTX[1]; // button state
//            Rec[5]= FLY_MODE_MANUAL_ANGLE;  // auto switch
//            //-----------Cap nhat du lieu vao bien Drone-------------        
//            
//            //Xu ly thong tin vi tri
//            drone_in->manual_pitch_d = dataRecFromTX[3]/10; // Pitch
//            drone_in->manual_roll_d =  -dataRecFromTX[4]/10; // Dy
//            drone_in->manual_yaw_d= dataRecFromTX[5]; // Yaw
//            drone_in->manual_z_d = dataRecFromTX[6]; // z_des
            //--Xy ly nut nhat start stop emer
            longTemp1 = dataRecFromTX[1];  // button state
            commandProcessing(longTemp1, drone_in);
            
            result = RF_DATA_RECEIVED_NO_ID;
            // Serial.print("Default: ");
            // Serial.println(result);
//            //--Xu ly che do bay
//            longTemp1 = FLY_MODE_MANUAL_ANGLE;  // Pos Manual Mode
//            flyModeProcessing(longTemp1, drone_in);
        break;  
    }
	return result;
}
/*-------------------------------------
* CHUYEN DU LIEU LONG * FLOAT SANG KIEU LONG
--------------------------------------*/
long ORC_Drone_RF::MultipleLongFloat2Long(long longIn, float floatIn){
	float temp = (float)longIn;
	return ((long)(temp*floatIn));	
}
//----------------------END OF RF FUNCTIONS--------------------------------------------------------




