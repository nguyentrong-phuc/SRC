/*----------------------------------------------
* CHUONG TRINH TEST HUB CONTROL
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/11/15
* Hardware: hub module
-------------------------------------------------*/
#include <ORC_Drone_HUB.h>
#include <ORCType.h>
#include <ORC_Utilities.h>

#define ET 0.01 //10ms

long currentTime = micros();
ORC_Drone_HUB hubVar;
ORC_Utilities ORCUti;
void setup()
{
	//Khoi tao Serial ket noi voi may tinh
	Serial.begin(115200);	
  Serial.println("BAt dau set up GPS");
	while (!hubVar.HUBSetup(GPS_RTK_FIX)){
		Serial.println("Dang cho du lieu gps");
	}
	Serial.println("Hoan thanh SETUP HUB");	
}

void loop()
{
	//Delay tinh 10ms
	ORCUti.dynamicWaiting(ET*1000, &currentTime); //in msec
	//------------RUN CONTROL HUB------------------------------------------
	hubVar.HUBRun();
//  Serial.print(hubVar.gpsLongData.readErrCnt); Serial.print(","); Serial.println(hubVar.gpsLongData.readStrErrCnt); 
}