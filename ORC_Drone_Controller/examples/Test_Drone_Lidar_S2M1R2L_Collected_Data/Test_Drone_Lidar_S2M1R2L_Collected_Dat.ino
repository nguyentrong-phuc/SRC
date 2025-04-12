/*----------------------------------------------
* TEST RPLIDAR S2M1 IN ARDUINO DUE
* * Written by Le Duc Tai + Trieu Khanh Thi
* * Modified by Dang Xuan Ba
* Date of version: 2025/01/10
* Hardware: Arduino Due
-------------------------------------------------*/
#include <ORC_Drone_Lidar_S2M1.h>


ORC_Drone_Lidar_S2M1 RPLidarVar;
lidarDataType lidarData;

void setup()
{
  /**
   * @brief SET UP PORT SERIAL FOR LIDAR 
   * RUN RECOVER MODE AND BEGIN SCAN TO COLECCT DATA FROM LIDAR
   */
  RPLidarVar.LidarSetup(&lidarData); 
}

void loop()
{
  /**
   * @brief PROCESS DATA RECIVE BY LIDAR
   * UPDATING MORE FUNC TO PARSHING DATA
   */
  RPLidarVar.runLoop(&lidarData);
  
}