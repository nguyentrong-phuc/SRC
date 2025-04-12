/*----------------------------------------------
* CHUONG TRINH TEST RP LIDAR VS ARDUINO MEGA
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/12/08
-------------------------------------------------*/
#include <ORC_Drone_2DLidar.h>

#define RPLIDAR_MOTOR 6
//Khai bao bien DroneController
ORC_Drone_2DLidar RPLidarVar(RPLIDAR_MOTOR);
lidarDataType lidarData;

void setup()
{
  RPLidarVar.LidarSetup(&lidarData); //Cai dat he thong doc lidar
}

void loop()
{
  RPLidarVar.runLoop(&lidarData);
  
}