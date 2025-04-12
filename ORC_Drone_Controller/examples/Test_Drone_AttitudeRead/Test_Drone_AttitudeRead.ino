/*----------------------------------------------
  CHUONG TRINH TEST ATTITUDE READ
  DOC CAC GIA TRI Roll, pitch, yaw
  Written by Dang Xuan Ba <dang.xuanba@gmail.com>
  Date of version: 2024/10/07
  -------------------------------------------------*/
#include <ORC_Drone_EBIMU.h>
#include <ORC_Utilities.h>
#include "Arduino.h"

ORC_Drone_EBIMU DroneEBI_data(100); //Khoi tao bien voi cac gia tri offset la 0
ORC_Utilities ORCUti;
orcebi_data ebiData;

long currentTime = micros();

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup Done!");
	DroneEBI_data.EBISetOffsetValues(&ebiData, 0 , 0, 0);
}

void loop()
{
	//Delay
	ORCUti.dynamicWaiting(10, &currentTime); //in sec
	if(DroneEBI_data.EBIReadData(&ebiData)){    
		if(Serial.availableForWrite()>0) 
			Serial.print(ebiData.roll); Serial.print(",");
			Serial.print(ebiData.pitch); Serial.print(",");
			Serial.print(ebiData.yaw); Serial.print(",");
			Serial.println(ebiData.readErrCnt); 
	}
}
