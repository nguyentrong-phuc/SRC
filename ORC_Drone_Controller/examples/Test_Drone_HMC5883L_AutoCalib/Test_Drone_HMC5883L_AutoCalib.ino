/*----------------------------------------------
* CHUONG TRINH TEST HMC5883L: AUTO CALIB
* DOC GOC YAW
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/10/07
-------------------------------------------------*/
#include <ORC_Drone_HMC5883L.h>
#include "Arduino.h"

//Khoi tao bien HMC
ORC_Drone_HMC5883L DroneHMC;


void setup()
{
  //Cai dat PWM
  Serial.begin(115200);
  DroneHMC.HMCSetup();
  //Calib cam bien
  if(DroneHMC.HMCCalibration(30000)){
	  Serial.println("Calib cam bien thanh cong.");
  }else Serial.println("Calib cam bien that bai.");
  
}

void loop()
{  
  delay(10);
  Serial.println(DroneHMC.HMCReadData());
}
