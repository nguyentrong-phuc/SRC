/*================================================================
  ORC_Drone_PumpHub.h - Library for navigating data and pump control.
  Created by Dang Xuan Ba, Dec 14, 2024.
  Updated by Dang Xuan Ba, Dec 14, 2024.
  Released into the public domain.
==================================================================*/

#include "Arduino.h"
#include "ORC_Drone_PumpHub.h"
#include<ORC_Utilities.h>
#include<ORCType.h>
//#include <ORC_Utilities_1rdKalmanFilter.h>

/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_PumpHub::ORC_Drone_PumpHub(uint64_t rfReadId, uint64_t rfSendId)
{
   //Cau hinh cho du lieu angle: "*A 123, -2343, 1232, 34232#" ~ "*A r, p, y, CRC#" in (x10 deg)
  droneAngleData.isNew = false; 
  droneAngleData.preFixString = ORCUti.AttitudeDataPrefix;
  droneAngleData.len = 6;
  
  //Cau hinh cho du lieu vi tri: "*P 142, -455, 457, 0 34232#" ~ "*P x, y, z,status CRC#" in (cm)
  dronePositionData.isNew = false;
  dronePositionData.preFixString = ORCUti.PositionDataPrefix;
  dronePositionData.len = 5;
  
  //Cau hinh cho du lieu toa do: "*G 12458756, -154626, 165444, 1, 34232#" ~ "*G lat, lon, alt, status CRC#" in (x10^7 (lat, lon) cm)
  droneGPSData.isNew = false;
  droneGPSData.preFixString = ORCUti.GPSDataPrefix;
  droneGPSData.len = 5;
  
  //Cau hinh cho du lieu toa do: "*FC 12458756, 12458756#" ~ "*PC luuluong CRC#" in (x10 lpm)
  dronePumpControlData.isNew = false;
  dronePumpControlData.preFixString = ORCUti.FlowControlDataPrefix;
  dronePumpControlData.len = 2;
  
  //Cau hinh cho du lieu toa do: "*FF 12458756, 12458756#" ~ "*PC luuluong CRC#" in (x10 lpm)
  dronePumpFBData.isNew = false;
  dronePumpFBData.preFixString = ORCUti.FlowFeedbackDataPrefix;
  dronePumpFBData.len = 2;
  
  //Cau hinh cho du lieu canh bao: "*W 0, 10, 10#" ~ "*T ts CRC#" in (ms)
  droneMainWarningData.isNew = false;
  droneMainWarningData.preFixString = ORCUti.MainWarningDataPrefix;
  droneMainWarningData.len = 3;
  
	//Cau hinh cho du lieu goc dieu khien: "*LC 10, 10, 20#" in (10* DEG)
  droneObstacleControlData.isNew = false;
  droneObstacleControlData.preFixString = ORCUti.Lidar2DControlDataPrefix;
  droneObstacleControlData.len = 3;

  //Cau hinh cho du lieu toa do: "*LR 10, 10, 20#" in ([mm], [deg])
  droneObstacleRawData.isNew = false;
  droneObstacleRawData.preFixString = ORCUti.Lidar2DRawDataPrefix;
  droneObstacleRawData.len = 3;
	

//   currentTime = micros();
//   timeCnt = 0;
  

  
  DroneRFSend = new ORC_Drone_RF(9,53, rfReadId,rfSendId); //Khoi tao bien doc RF  
  
  pinMode(LED_RF_SEND, OUTPUT);
  pinMode(LED_HUB_DATA, OUTPUT);
  pinMode(LED_PUMP_DATA, OUTPUT);
  pinMode(LED_LIDAR3D_DATA, OUTPUT);
  
  digitalWrite(LED_RF_SEND, LOW);
  digitalWrite(LED_HUB_DATA, LOW);
  digitalWrite(LED_PUMP_DATA, LOW);
  digitalWrite(LED_LIDAR3D_DATA, LOW);

  
  _preMilliTime = millis();
}
//----------------------BEGIN OF HUB FUNCTIONS--------------------------------------------------------
/*-------------------------------------------------
* HIEN THI TAT CA CAC LED
---------------------------------------------------*/
void ORC_Drone_PumpHub::RFLedDisplay(long *preMilliTime,bool stateIn){
	long curMilliTime =millis();
	if(stateIn){ //Uu tien trang thai HIGH
		digitalWrite(LED_RF_SEND, stateIn);
	}else if(curMilliTime - (*preMilliTime) >100){
		digitalWrite(LED_RF_SEND, stateIn);
		*preMilliTime = curMilliTime;
	}
	
}

/*-------------------------------------------------
* HIEN THI TAT CA CAC LED
---------------------------------------------------*/
void ORC_Drone_PumpHub::PumpControl(orcstring_data pumpDataIn){	
	PumpControlOut(pumpDataIn.dataOut[0]);
}

/*-------------------------------------------------
* HIEN THI TAT CA CAC LED
---------------------------------------------------*/
void ORC_Drone_PumpHub::PumpControlOut(long flowIn){
	long pumpPWM = 0;
	unsigned int pumpPWMSendOut = 0;
	if(flowIn > 0)
	{
		pumpPWM = (flowIn > PUMP_FLOW_MAX)?PUMP_FLOW_MAX:flowIn;	
		pumpPWMSendOut = 	(unsigned int)((pumpPWM*1900)/PUMP_FLOW_MAX);
		digitalWrite(LED_PUMP_DATA, HIGH);
	}else digitalWrite(LED_PUMP_DATA, LOW);
	
	PumpPWM.setPWM(pumpPWMSendOut, pumpPWMSendOut, pumpPWMSendOut, pumpPWMSendOut);	
}

/*-------------------------------------------------
* SET UP HUB
---------------------------------------------------*/
bool ORC_Drone_PumpHub::PumpHubSetup()
{


	Serial.begin(115200); Serial.setTimeout(2); 
  	HUBPort.begin(115200);	HUBPort.setTimeout(2); 
  	RPLidarPort.begin(115200);RPLidarPort.setTimeout(2); 
  	ESP32Port.begin(115200);ESP32Port.setTimeout(2); 

	while(!DroneRFSend->RFSendSetup())
	{
		Serial.println("Dang doi setup RF");
	}
	// Serial.println("In setup");
	//Cai dat PWM cho pump
 	PumpPWM.PWMSetup();
	PumpPWM.setPWMMaxMin(0, 2000);  
	delay(10);
	Serial.println("Setup done!");
}

/*-------------------------------------------------
* QUET TAT CA CAC DU LIEU CONG VAO: CONG HUB, CONG LIDAR, CONG IOT
---------------------------------------------------*/
bool ORC_Drone_PumpHub::PumpHubInScan()
{
	String strTemp = "";
	bool result = false;
	// Serial.println("Hub1: ");
	//Kiem tra du lieu tu cong MAIN
	if(HUBPort.available()){ //Phat hien co du lieu: chi lam 3 dang: A, T, PC
		strTemp = HUBPort.readStringUntil('\n');
		// Serial.print("Hub: ");
		 // Serial.println(strTemp);
		//Kiem tra tung dang du lieu
		if(strTemp.startsWith(droneAngleData.preFixString)){ //Du lieu goc nghieng
			droneAngleData.stringIn = strTemp;
			droneAngleData.isNew = true;
			result = true;
			//Serial.print("A: ");
			//Serial.println(droneAngleData.stringIn);
			// if(SplitData2LongInStruct(strTemp, &droneAngleData)){
				// droneAngleData.isNew = true;
				// result = true;
			// }
		}else if(strTemp.startsWith(droneMainWarningData.preFixString)){ //Du lieu thoi gian lay mau
			droneMainWarningData.stringIn = strTemp;
			// droneMainWarningData.isNew = true;
			// result = true;

			if(ORCUti.SplitData2LongInStruct(strTemp, &droneMainWarningData)){
				droneMainWarningData.isNew = true;
				result = true;
			}
			// Serial.print("W: ");
			//Serial.println(droneMainWarningData.stringIn);
		}else if(strTemp.startsWith(dronePumpControlData.preFixString)){ //Du lieu pump control
			// dronePumpControlData.stringIn = strTemp;
			// dronePumpControlData.isNew = true;
			// result = true;
			if(ORCUti.SplitData2LongInStruct(strTemp, &dronePumpControlData)){
				dronePumpControlData.isNew = true;
				result = true;
				//Serial.print("Du lieu nhan tu HUB (ok): ");
				//Serial.println(strTemp);
			}
			//Serial.print("Du lieu nhan tu HUB: ");
			//Serial.println(strTemp);
			
		}else if (strTemp.startsWith(droneGPSData.preFixString)){ //Doc du lieu GPS
			droneGPSData.stringIn = strTemp;
			droneGPSData.isNew = true;
		}
			
		
		digitalWrite(LED_HUB_DATA, true); //Nhan duoc du lieu tu HUB
	}else
		digitalWrite(LED_HUB_DATA, false); //Khong Nhan duoc du lieu tu HUB

	//Kiem tra du lieu tu cong RPLidar
	if(RPLidarPort.available()){ //Phat hien co du lieu: chi lam 3 dang: A, T, PC
		strTemp = RPLidarPort.readStringUntil('\n');
		// Serial.println(strTemp);
		//Kiem tra tung dang du lieu
		if(strTemp.startsWith(droneObstacleControlData.preFixString)){ //Du lieu goc nghieng
			droneObstacleControlData.stringIn = strTemp;
			droneObstacleControlData.isNew = true;
			result = true;			
			// if(SplitData2LongInStruct(strTemp, &droneAngleData)){
				// droneAngleData.isNew = true;
				// result = true;
			// }
		}else if (strTemp.startsWith(droneObstacleRawData.preFixString)){ //Du lieu tho: khoang cach va goc
			droneObstacleRawData.stringIn = strTemp;
			droneObstacleRawData.isNew = true;
			result = true;			
			// if(SplitData2LongInStruct(strTemp, &droneAngleData)){
				// droneAngleData.isNew = true;
				// result = true;
			// }
		}
		//digitalWrite(LED_LIDAR3D_DATA, true); //Nhan duoc du lieu tu HUB
	}//else digitalWrite(LED_LIDAR3D_DATA, false); //Nhan duoc du lieu tu HUB

	return result;
}


/*-------------------------------------------------
* XU LY DU LIEU BOM NHAN DC
---------------------------------------------------*/
void ORC_Drone_PumpHub::PumpHubDataProcessing(orcstring_data *pumpControlDataVar, orcstring_data *pumpFeedbackDataVar)
{
	if(pumpControlDataVar->isNew){ //Xu ly khi co du lieu moi
		pumpFeedbackDataVar->dataOut[0] = pumpControlDataVar->dataOut[0]; //Tam thoi cho in bang out
		pumpFeedbackDataVar->isNew = true;
		pumpControlDataVar->isNew = false;
	}
}

/*-------------------------------------------------
* GOI DU LIEU RA CAC CONG: CONG MAIN, CONG GPS, CONG PUMP
---------------------------------------------------*/
bool ORC_Drone_PumpHub::PumpHubInSentOut()
{
	bool result = false;	
	bool rfSendResult = false;
	/*-------------------------------------------
	1) Goi du lieu ra MAIN: 1) goi du lieu tranh vat can
	
	2) Goi du lieu xuong pump: 1) Goi tin hieu Ts; 2) goi tin hieu x,y,z; 3) goi tin hieu lat long alt; 4) Tin hieu pump control
	---------------------------------------------*/
	// TRIEN KHAI
	//--GOI DU LIEU RA MAN HINH
	if(droneObstacleControlData.isNew && (Serial.availableForWrite()>= 63)){ //Goi GPS to UART0
		Serial.println(droneObstacleControlData.stringIn);
	}
	
				
	//1) Goi du lieu ra HUB: 1) Goi tranh vat can; 2) Tin hieu pump feedback
	if(HUBPort.availableForWrite()>= 63){
		//Goi du lieu GPS
		if(droneObstacleControlData.isNew){ //Khi co du lieu moi
			HUBPort.println(droneObstacleControlData.stringIn);
			droneObstacleControlData.isNew = false;	
			result = true;	
			digitalWrite(LED_LIDAR3D_DATA, true); //Nhan duoc du lieu tu HUB	
		}else{
			digitalWrite(LED_LIDAR3D_DATA, false);
		//Goi du lieu phan hoi tu pump
		if(dronePumpFBData.isNew){
			HUBPort.print(dronePumpFBData.preFixString);HUBPort.print(" ");
			HUBPort.print(dronePumpFBData.dataOut[0]);HUBPort.print(",");
			HUBPort.print(dronePumpFBData.dataOut[0]);
			HUBPort.println("#");
			dronePumpFBData.isNew = false;
			result = true;
			//Serial.print("Goi Du lieu qua HUB: ");
			//Serial.println(dronePumpFBData.dataOut[0]);
		}
		}
	}else digitalWrite(LED_LIDAR3D_DATA, false); //Nhan duoc du lieu tu HUB
	
	//2) Goi du lieu xuong pump: 1) Goi tin hieu Ts; 2) goi tin hieu x,y,z; 3) goi tin hieu lat long alt; 4) Tin hieu pump control
	if(ESP32Port.availableForWrite()>= 63){
		//Goi tin hieu goc
		if(droneAngleData.isNew){
			ESP32Port.println(droneAngleData.stringIn);
			droneAngleData.isNew = false;
			result = true;
		}else
		
		//Goi tin hieu Ts
		if(droneMainWarningData.isNew && (droneMainWarningData.dataOut[0] == MAIN_MSG_CODE_WARNINGFAULT)){
			ESP32Port.println(droneMainWarningData.stringIn);
			droneMainWarningData.isNew = false;
			result = true;
		}else
		//Goi tin hieu vi tri
		if(dronePositionData.isNew){
			ESP32Port.println(dronePositionData.stringIn);
			dronePositionData.isNew = false;
			result = true;
		}else
		//Goi tin hieu gps
		if(droneGPSData.isNew){
			ESP32Port.println(droneGPSData.stringIn);
			//Serial.println("11");
			//Serial.println(droneGPSData.stringIn);
			droneGPSData.isNew = false;
			result = true;
		}else
		//Goi tin hieu vat can
		if(droneObstacleRawData.isNew){
			ESP32Port.println(droneObstacleRawData.stringIn);
			droneObstacleRawData.isNew = false;
			result = true;
		}
		
	}
	// //Hien thi len UART0
	// if((Serial.availableForWrite()>= 63) && droneMainWarningData.isNew){
	// 	Serial.print("Du lieu goi qua RF:");
	// 	Serial.print(droneMainWarningData.dataOut[0]); //IN MESSAGE CODE
	// 	Serial.print(",");
	// 	Serial.print(droneMainWarningData.dataOut[1]); //IN DATA
	// 	Serial.print(",");
	// 	Serial.println(droneMainWarningData.dataOut[2]); //IN CRC
	// }
	//3) Goi du lieu qua song RF
	//Goi du lieu Fly Point Id
	// Serial.print("isNew: "); Serial.println(droneMainWarningData.isNew);
	// Serial.print("droneMainWarningData.dataOut[0] "); Serial.println(droneMainWarningData.dataOut[0]);
	if(droneMainWarningData.isNew && (droneMainWarningData.dataOut[0] == MAIN_MSG_CODE_FLY_ID)){
		//if(droneMainWarningData.dataOut[1]!=1){
		DroneRFSend->sendingData.rfheader = '*';
		DroneRFSend->sendingData.msgCode = droneMainWarningData.dataOut[0];
		DroneRFSend->sendingData.data = droneMainWarningData.dataOut[1];
		DroneRFSend->sendingData.checksum = droneMainWarningData.dataOut[2];
		// Serial.println("Feedback: ");
		// Serial.println(DroneRFSend->sendingData.rfheader);
		// Serial.println(DroneRFSend->sendingData.msgCode);
		// Serial.println(DroneRFSend->sendingData.data);
		// Serial.println(DroneRFSend->sendingData.checksum);
		// Serial.println("______________________________");
		rfSendResult = DroneRFSend->RFSendData();
		//}
		// Serial.print("rfSendResult: ");Serial.println(rfSendResult);	
		droneMainWarningData.isNew = false;
		result = true;
	}

	// if(droneMainWarningData.isNew && (droneMainWarningData.dataOut[0] == MAIN_MSG_CODE_FLY_ID)){
		// DroneRFSend->sendingData.rfheader = '*';
		// DroneRFSend->sendingData.msgCode = 1;
		// DroneRFSend->sendingData.data = 7; 
		// DroneRFSend->sendingData.checksum = 8;

		// // Serial.println("Feedback: ");
		// Serial.println(DroneRFSend->sendingData.rfheader);
		// Serial.println(DroneRFSend->sendingData.msgCode);
		// Serial.println(DroneRFSend->sendingData.data);
		// Serial.println(DroneRFSend->sendingData.checksum);
		// Serial.println("______________________________");
		// rfSendResult = DroneRFSend->RFSendData();

		// // Serial.print("rfSendResult: ");Serial.println(rfSendResult);	
		// droneMainWarningData.isNew = false;
		// result = true;
	// }

	// Serial.print("rfSendResult: ");Serial.println(rfSendResult);	
	//Hien thi du lieu len Led
	
	RFLedDisplay(&_preMilliTime, rfSendResult);
	
	// digitalWrite(LED_RF_SEND, HIGH);
	return result;
}



/*-------------------------------------------------
* RUN HUB
---------------------------------------------------*/
bool ORC_Drone_PumpHub::PumpHubRun()
{
	bool result = true;
	//Scan du lieu
	PumpHubInScan();

	//Ham xu ly du lieu
	PumpHubDataProcessing(&dronePumpControlData, &dronePumpFBData);

	//Dieu khien pump
	PumpControl(dronePumpControlData);
	//PumpPWM.setPWM(1000, 1000, 1000, 1000); 
	//Serial.println("Dang bom");

	//Sent out du lieu
	result = PumpHubInSentOut();
	return result;
}
//----------------------END OF ATTITUDE FUNCTIONS--------------------------------------------------------




