/*----------------------------------------------
* CHUONG TRINH TEST RP LIDAR VS ARDUINO MEGA
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/12/08
* Hardware: pump control module
-------------------------------------------------*/
#include <ORC_Drone_PumpHub.h>

#define ET 0.01 //[s]
uint64_t rfReadId = 4, rfSendId = 3;
//Khai bao bien DroneController
ORC_Drone_PumpHub PumpHubVar(rfReadId, rfSendId);

ORC_Utilities ORCUti;
long currentTime = micros();

void setup()
{
  PumpHubVar.PumpHubSetup(); //Cai dat he thong doc lidar
}

void loop()
{
  ORCUti.dynamicWaiting(ET*1000, &currentTime); //in msec
  //delay(100);
  PumpHubVar.PumpHubRun();  
}