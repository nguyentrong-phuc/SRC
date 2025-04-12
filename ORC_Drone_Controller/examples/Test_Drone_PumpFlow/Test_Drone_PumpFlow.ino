/*----------------------------------------------
* CHUONG TRINH TEST RP LIDAR VS ARDUINO MEGA
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/12/08
* Hardware: pump control module
-------------------------------------------------*/
#include <ORC_Drone_PumpHub.h>

uint64_t rfReadId = 4, rfSendId = 3;
//Khai bao bien DroneController
ORC_Drone_PumpHub PumpHubVar(rfReadId, rfSendId);

void setup()
{
  PumpHubVar.PumpHubSetup(); //Cai dat he thong doc lidar
}

void loop()
{
  delay(10);
  PumpHubVar.PumpControlOut(2); //2 lit/phut
}