/*
  ORC_Drone_EBIMU.h - Library for reading attitude signal.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#include "Arduino.h"
#include "ORC_Drone_HUB.h"
#include<ORC_Utilities.h>
#include<ORCType.h>
//#include <ORC_Utilities_1rdKalmanFilter.h>

/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_HUB::ORC_Drone_HUB()
{
   //Cau hinh cho du lieu angle: "*A 123, -2343, 1232, 2,1,34232#" ~ "*A r, p, y, CRC#" in (x10 deg)
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
  
  //Cau hinh cho du lieu luu luong pump mong muon: "*FC 12458756, 12458756#" ~ "*PC luuluong CRC#" in (x10 lpm)
  dronePumpControlData.isNew = false;
  dronePumpControlData.preFixString = ORCUti.FlowControlDataPrefix;
  dronePumpControlData.len = 2;
  
  //Cau hinh cho du lieu pump phan hoi: "*FF 12458756, 12458756#" ~ "*PC luuluong CRC#" in (x10 lpm)
  dronePumpFBData.isNew = false;
  dronePumpFBData.preFixString = ORCUti.FlowFeedbackDataPrefix;
  dronePumpFBData.len = 2;
  
  //Cau hinh cho du lieu canh bao: "*W 1 10, 10#" ~ "*T ts CRC#" in (ms)
  droneMainWarningData.isNew = false;
  droneMainWarningData.preFixString = ORCUti.MainWarningDataPrefix;
  droneMainWarningData.len = 3;
 
  //Cau hinh cho du lieu goc dieu khien: "*LC 10, 10, 20#" in (10* DEG)
  droneObstacleControlData.isNew = false;
  droneObstacleControlData.preFixString = ORCUti.Lidar2DControlDataPrefix;
  droneObstacleControlData.len = 3;

  GPSBaseRequest = false;

  currentTime = micros();
  timeCnt = 0;
  _baseStableCnt = 0;
  
  MAINPort.begin(115200);	
  GPSPort.begin(115200);
  PUMPPort.begin(115200);
  
  
  pinMode(LED_RF_SEND, OUTPUT);
  pinMode(LED_PROGRAM_READY, OUTPUT);
  pinMode(LED_GPS_DATA, OUTPUT);
  pinMode(LED_PUMP_DATA, OUTPUT);
  
  digitalWrite(LED_RF_SEND, LOW);
  digitalWrite(LED_PROGRAM_READY, LOW);
  digitalWrite(LED_GPS_DATA, LOW);
  digitalWrite(LED_PUMP_DATA, LOW);
  
}
//----------------------BEGIN OF HUB FUNCTIONS--------------------------------------------------------
/*-------------------------------------------------
* QUET TAT CA CAC DU LIEU CONG VAO: CONG MAIN, CONG GPS, CONG PUMP
---------------------------------------------------*/
bool ORC_Drone_HUB::inScan()
{
	String strTemp = "";
	bool result = false;
	//Kiem tra du lieu tu cong MAIN
	if(MAINPort.available()){ //Phat hien co du lieu: chi lam 3 dang: A, T, PC
		strTemp = MAINPort.readStringUntil('\n');
		// Serial.println(strTemp);
		//Kiem tra tung dang du lieu
		/*
		if(strTemp.startsWith(droneAngleData.preFixString)){ //Du lieu goc nghieng
			droneAngleData.stringIn = strTemp;
			droneAngleData.isNew = true;
			result = true;
			
			// if(SplitData2LongInStruct(strTemp, &droneAngleData)){
				// droneAngleData.isNew = true;
				// result = true;
			// }
		}else if(strTemp.startsWith(droneMainWarningData.preFixString)){ //Du lieu thoi gian lay mau
			droneMainWarningData.stringIn = strTemp;
			droneMainWarningData.isNew = true;
			result = true;
			// if(SplitData2LongInStruct(strTemp, &droneTSData)){
				// droneTSData.isNew = true;
				// result = true;
			// }
		}else if(strTemp.startsWith(dronePumpControlData.preFixString)){ //Du lieu pump control
			dronePumpControlData.stringIn = strTemp;
			dronePumpControlData.isNew = true;
			result = true;
			// if(SplitData2LongInStruct(strTemp, &dronePumpControlData)){
				// dronePumpControlData.isNew = true;
				// result = true;
			// }
		}else */
			
		if(strTemp.startsWith(droneGPSData.preFixString)){ //Du lieu pump control
			GPSBaseRequest = true;
			result = true;
	
			// if(SplitData2LongInStruct(strTemp, &dronePumpControlData)){
				// dronePumpControlData.isNew = true;
				// result = true;
			// }
		}else if(strTemp.startsWith(droneMainWarningData.preFixString)){ //Du lieu thoi gian lay mau
			droneMainWarningData.stringIn = strTemp;
			droneMainWarningData.isNew = true;
			result = true;

			// if(SplitData2LongInStruct(strTemp, &droneTSData)){
				// droneTSData.isNew = true;
				// result = true;
			// }
			// Serial.print("Hub received from Main: ");
			// Serial.println(droneMainWarningData.stringIn);
		}else if(strTemp.startsWith(dronePumpControlData.preFixString)){ //Du lieu pump control
			dronePumpControlData.stringIn = strTemp;
			dronePumpControlData.isNew = true;
			result = true;
			
			// Serial.print("Nhan du lieu tu MAIN:");
			// Serial.println(strTemp);
			
			// if(SplitData2LongInStruct(strTemp, &dronePumpControlData)){
				// dronePumpControlData.isNew = true;
				// result = true;
			// }
		}
		
		
	}
	//Kiem tra du lieu tu cong GPS
	//digitalWrite(LED_GPS_DATA, LOW);
	// digitalWrite(LED_GPS_DATA, !(digitalRead(LED_GPS_DATA)));
	if(GPSPort.available()){//Phat hien co du lieu: chuyen du lieu string nhan dc vao du lieu GPS
		
		strTemp = GPSPort.readStringUntil('\n');
		
		
		droneGPSData.stringIn = strTemp;
		gpsLongData.readStrErrCnt = 0;
		if(ORCUti.SplitData2LongInStruct(strTemp, &droneGPSData)){
			//if(gpsLongData.pos_cm[3] == GPS_RTK_FIX)
			//digitalWrite(LED_GPS_DATA, !(digitalRead(LED_GPS_DATA)));
			
			//Serial.println(droneGPSData.dataOut[3]);
			// gpsLongData.current_lat_long = droneGPSData.dataOut[0];
			// gpsLongData.current_lon_long = droneGPSData.dataOut[1];
			// gpsLongData.current_alt = droneGPSData.dataOut[2];
			if(gpsLongData.isGotBase){//chi lay du lieu x,y,z
				gpsLongData.pos_cm[0] = (droneGPSData.dataOut[0] - gpsLongData.base_gps_long[0])*LAT_TO_CM;
				gpsLongData.pos_cm[1] = -(droneGPSData.dataOut[1] - gpsLongData.base_gps_long[1])*LON_TO_CM;
				gpsLongData.pos_cm[2] = (droneGPSData.dataOut[2] - gpsLongData.base_gps_long[2])/10;
				gpsLongData.pos_cm[3] = droneGPSData.dataOut[3];
				if(gpsLongData.pos_cm[3] == GPS_RTK_FIX)
					gpsLongData.isNew = true;
				
			}else{ //Chi lay du lieu base
				//Lay lai gia tri cua
				//Luu lai gia tri cua
				_pre_base_gps_long[0] = gpsLongData.base_gps_long[0];
				_pre_base_gps_long[1] = gpsLongData.base_gps_long[1];
				_pre_base_gps_long[2] = gpsLongData.base_gps_long[2];
				//Cap nhat base
				gpsLongData.base_gps_long[0] = droneGPSData.dataOut[0];
				gpsLongData.base_gps_long[1] = droneGPSData.dataOut[1];
				gpsLongData.base_gps_long[2] = droneGPSData.dataOut[2];
				gpsLongData.base_gps_long[3] = droneGPSData.dataOut[3];	
				if(droneGPSData.dataOut[3] == GPS_RTK_FIX){
					//Tinh do sai lech cua base
					_pre_base_dev_cm[0] = (droneGPSData.dataOut[0] - _pre_base_gps_long[0])*LAT_TO_CM;
				    _pre_base_dev_cm[1] = -(droneGPSData.dataOut[1] - _pre_base_gps_long[1])*LON_TO_CM;
				    _pre_base_dev_cm[2] = (droneGPSData.dataOut[2] - _pre_base_gps_long[2])/10;
					
					//du chat luong 
					if((abs(_pre_base_dev_cm[0])<= BASE_STABLE_DEVIATION)&&(abs(_pre_base_dev_cm[1])<= BASE_STABLE_DEVIATION)&&(abs(_pre_base_dev_cm[2])<= BASE_STABLE_DEVIATION))
					{
						_baseStableCnt++;
						if(_baseStableCnt >= BASE_STABLE_MAX){
							_baseStableCnt = BASE_STABLE_MAX;
							gpsLongData.isGotBase = true;
						}
					}else
						_baseStableCnt = 0; //Reset lai bien khao sat BASE
					
				}else{
					_baseStableCnt = 0; //Reset lai bien khao sat BASE
				}				
					
			}
			//chi lay du lieu tot nhat (FIXED)
			
			droneGPSData.isNew = true;
			gpsLongData.readErrCnt = 0;
			//LedGPSDisplay(droneGPSData.dataOut[3]); //Hien thi LED
			result = true;
		}else
			gpsLongData.readErrCnt = gpsLongData.readErrCnt + 1;
	}else {
		gpsLongData.readErrCnt = gpsLongData.readErrCnt + 1;
		gpsLongData.readStrErrCnt = gpsLongData.readStrErrCnt + 1;
	}
	
	//Kiem tra du lieu tu cong PUMP
	if(PUMPPort.available()){
		strTemp = PUMPPort.readStringUntil('\n');
		// dronePumpFBData.stringIn = strTemp;
		// Serial.print("Nhan du lieu tu Pump:");
		// Serial.println(strTemp);
		// dronePumpFBData.isNew = true;
		// result = true;
		
		if(strTemp.startsWith(droneObstacleControlData.preFixString)){ //Du lieu goc nghieng
			droneObstacleControlData.stringIn = strTemp;
			droneObstacleControlData.isNew = true;
			result = true;			
			// if(SplitData2LongInStruct(strTemp, &droneAngleData)){
				// droneAngleData.isNew = true;
				// result = true;
			// }
		}else if(strTemp.startsWith(dronePumpFBData.preFixString)){
			dronePumpFBData.stringIn = strTemp;
			dronePumpFBData.isNew = true;
			result = true;
		} //Kiem tra tung dang du lieu
	}
	return result;
}

/*-------------------------------------------------
* GOI DU LIEU RA CAC CONG: CONG MAIN, CONG PUMP
---------------------------------------------------*/
bool ORC_Drone_HUB::inSentOut(long *timeGPSIOTCnt)
{
	bool result = false;	
	long checksum = 0;
	/*-------------------------------------------
	1) Goi du lieu ra MAIN: 1) Goi GPS; 2) Tin hieu pump feedback
	
	2) Goi du lieu xuong pump: 1) Goi tin hieu Ts; 2) goi tin hieu x,y,z; 3) goi tin hieu lat long alt; 4) Tin hieu pump control
	---------------------------------------------*/
	// TRIEN KHAI
	//--GOI DU LIEU RA MAN HINH
	// if(gpsLongData.isNew && (Serial.availableForWrite()>= 63)){ //Goi GPS to UART0
		// Serial.print("*G ");
		// Serial.print(gpsLongData.base_gps_long[0]);Serial.print(",");
		// Serial.print(gpsLongData.base_gps_long[1]);Serial.print(",");
		// Serial.println(gpsLongData.base_gps_long[2]);		
	// //}
	// //if(gpsLongData.isNew && (Serial.availableForWrite()>= 63)){	 //Goi Pos to UART0	
		// Serial.print("*P ");
		// Serial.print(gpsLongData.pos_cm[0]);Serial.print(",");
		// Serial.print(gpsLongData.pos_cm[1]);Serial.print(",");
		// Serial.println(gpsLongData.pos_cm[2]);
	// }
	// TRIEN KHAI
	//--GOI DU LIEU RA MAN HINH
	//if(droneObstacleControlData.isNew && (Serial.availableForWrite()>= 63)){ //Goi GPS to UART0
	//	Serial.println(droneObstacleControlData.stringIn);
	//}
				
	//1) Goi du lieu ra MAIN: 1) Goi GPS; 2) Tin hieu pump feedback
	if(MAINPort.availableForWrite()>= 63){
		//Goi du lieu GPS
		if(gpsLongData.isNew){ //Khi co du lieu moi
			// if(timeGPSBaseCnt >= PGS_BASE_SEND_TIME){ //Goi du lieu base						
			// 	MAINPort.print(droneGPSData.preFixString);MAINPort.print(" ");
			// 	MAINPort.print(gpsLongData.base_gps_long[0]);MAINPort.print(",");
			// 	MAINPort.print(gpsLongData.base_gps_long[1]);MAINPort.print(",");
			// 	MAINPort.print(gpsLongData.base_gps_long[2]);MAINPort.print(",");
			// 	MAINPort.print(gpsLongData.base_gps_long[3]);MAINPort.print(",");
			// 	MAINPort.print(ORCUti.ChecksumOfLongArray(gpsLongData.base_gps_long, 4));
			// 	MAINPort.println("#");
			// 	result = true;
			// }
			//digitalWrite(LED_GPS_DATA, !(digitalRead(LED_GPS_DATA)));
			if(gpsLongData.isGotBase && GPSBaseRequest){ //Goi du lieu base						
				MAINPort.print(droneGPSData.preFixString);MAINPort.print(" ");
				MAINPort.print(gpsLongData.base_gps_long[0]);MAINPort.print(",");
				MAINPort.print(gpsLongData.base_gps_long[1]);MAINPort.print(",");
				MAINPort.print(gpsLongData.base_gps_long[2]);MAINPort.print(",");
				MAINPort.print(gpsLongData.base_gps_long[3]);MAINPort.print(",");
				MAINPort.print(ORCUti.ChecksumOfLongArray(gpsLongData.base_gps_long, 4));
				MAINPort.println("#");
				result = true;
				GPSBaseRequest = false;
			}
			else //goi x,y,z
			{
				checksum = ORCUti.ChecksumOfLongArray(gpsLongData.pos_cm, 4);
				MAINPort.print(dronePositionData.preFixString);MAINPort.print(" ");
				MAINPort.print(gpsLongData.pos_cm[0]);MAINPort.print(",");
				MAINPort.print(gpsLongData.pos_cm[1]);MAINPort.print(",");
				MAINPort.print(gpsLongData.pos_cm[2]);MAINPort.print(",");
				MAINPort.print(gpsLongData.pos_cm[3]);MAINPort.print(",");
				MAINPort.print(checksum);
				MAINPort.println("#");
				gpsLongData.isNew = false;
				result = true;
			}
		}else
		//Goi du lieu Obstacle Avoidance
		if(droneObstacleControlData.isNew){ //Khi co du lieu moi
			MAINPort.println(droneObstacleControlData.stringIn);
			droneObstacleControlData.isNew = false;	
			result = true;		
		}else
		//Goi du lieu phan hoi tu pump
		if(dronePumpFBData.isNew){
			MAINPort.println(dronePumpFBData.stringIn);
			dronePumpFBData.isNew = false;
			result = true;
		}
	}
	
	//2) Goi du lieu xuong pump: 1) Goi tin hieu Ts; 2) goi tin hieu x,y,z; 3) goi tin hieu lat long alt; 4) Tin hieu pump control
	
	if(PUMPPort.availableForWrite()>= 63){
		/*
		//Goi tin hieu goc
		if(droneAngleData.isNew){
			PUMPPort.println(droneAngleData.stringIn);
			droneAngleData.isNew = false;
			result = true;
		}*/
		//Goi tin hieu Ts
		if(droneMainWarningData.isNew){
			PUMPPort.println(droneMainWarningData.stringIn);
			droneMainWarningData.isNew = false;
			result = true;
			// Serial.print("Hub To Pump: ");
			// Serial.println(droneMainWarningData.stringIn);
			digitalWrite(LED_PUMP_DATA, HIGH);
		}else
		/*
		//Goi tin hieu x,y,x
		
		if(gpsLongData.isNew){ //Khi co du lieu moi
			if(timeGPSBaseCnt >= PGS_BASE_SEND_TIME){ //Goi du lieu base		
				PUMPPort.println(droneGPSData.stringIn);
				
				result = true;
			}else //goi x,y,z
			{
				PUMPPort.print(dronePositionData.preFixString);PUMPPort.print(" ");
				PUMPPort.print(gpsLongData.pos_cm[0]);PUMPPort.print(",");
				PUMPPort.print(gpsLongData.pos_cm[1]);PUMPPort.print(",");
				PUMPPort.print(gpsLongData.pos_cm[2]);PUMPPort.print(",");
				PUMPPort.print(gpsLongData.pos_cm[3]);PUMPPort.print(",");
				PUMPPort.print(ORCUti.ChecksumOfLongArray(gpsLongData.pos_cm, 4));
				PUMPPort.println("#");
				gpsLongData.isNew = false;
				result = true;
			}
		}*/
		//Goi tin hieu pump control
		if(dronePumpControlData.isNew){
			PUMPPort.println(dronePumpControlData.stringIn);
			dronePumpControlData.isNew = false;
			result = true;
			digitalWrite(LED_PUMP_DATA, LOW);
		}else if ((*timeGPSIOTCnt) >= GPS_IOT_SEND_TIME){
			PUMPPort.println(droneGPSData.stringIn);
			*timeGPSIOTCnt = 0;
		}
		
	}
	
	return result;
}

/*-------------------------------------------------
* SET UP HUB
---------------------------------------------------*/
bool ORC_Drone_HUB::HUBSetup(long GPSRTK_Standard)
{
	bool result = false;
	int readCnt = 0;
	String strTemp = "";
	// Cho doc duoc tin hieu GPS voi chat luong tot hon gia tri standard
	
	while(!result){
		//MAINPort.println("Wakeup!");
		/*
		if(GPSPort.available()){//Phat hien co du lieu: chuyen du lieu string nhan dc vao du lieu GPS
			strTemp = GPSPort.readStringUntil('\n');
			Serial.println(strTemp);
		}*/
		//KHONG CO GPS THI BO LENH NAY
		if(inScan()){ //Doc du lieu goi tu cac port
			result = gpsLongData.isNew && (gpsLongData.base_gps_long[3] >= GPSRTK_Standard);
		}
		readCnt++;
		if(readCnt >= 20){
			//Dao trang thai led Read
			digitalWrite(LED_PROGRAM_READY, !digitalRead(LED_PROGRAM_READY));
			readCnt = 0;
		}
		delay(10);
	}
	digitalWrite(LED_PROGRAM_READY, HIGH);	//He thong khoi dong xong
	Serial.println("HUB setup done!");
	return result;
}


/*-------------------------------------------------
* LED GPS 
---------------------------------------------------*/
void ORC_Drone_HUB::LedGPSDisplay(long RTKStatus)
{
	if(RTKStatus == GPS_RTK_FIX){
		digitalWrite(LED_GPS_DATA, HIGH);
		//digitalWrite(LED_GPS_DATA_01, HIGH);
	}else if(RTKStatus == GPS_RTK_FLOAT){
		digitalWrite(LED_GPS_DATA, !(digitalRead(LED_GPS_DATA)));
		//digitalWrite(LED_GPS_DATA_01, LOW);
	}else if(RTKStatus == GPS_RTK_NOSOLUTION){
		digitalWrite(LED_GPS_DATA, LOW);
		//digitalWrite(LED_GPS_DATA_01, LOW);
	}  
}

/*-------------------------------------------------
* LED GPS 
---------------------------------------------------*/
void ORC_Drone_HUB::AllLedDisplay()
{
	LedGPSDisplay(droneGPSData.dataOut[3]); //Hien thi LED GPS
	//Hien thi led Warning	
	digitalWrite(LED_RF_SEND, droneMainWarningData.isNew);
}
/*-------------------------------------------------
* RUN HUB
---------------------------------------------------*/
bool ORC_Drone_HUB::HUBRun()
{
	bool result = true;
	//Delay tinh 10ms
	//ORCUti.dynamicWaiting(ET*1000, &currentTime); //in msec
	timeCnt++; //Tang bien den de goi gps data ra main va esp32
	result &= inScan(); //Scan het du lieu
	
	if(timeCnt >= GPS_IOT_SEND_TIME) timeCnt = GPS_IOT_SEND_TIME;	
	//Hien thi led GPS
	if(gpsLongData.readErrCnt >= GPS_BASE_SEND_TIME){
		//digitalWrite(LED_GPS_DATA_00, LOW);
		//digitalWrite(LED_GPS_DATA_01, LOW);
		gpsLongData.readErrCnt = GPS_BASE_SEND_TIME;
	}
	//Hien thi led
	AllLedDisplay();


	result &= inSentOut(&timeCnt);//Goi het du lieu ra ngoai		
	return result;
}
//----------------------END OF ATTITUDE FUNCTIONS--------------------------------------------------------




