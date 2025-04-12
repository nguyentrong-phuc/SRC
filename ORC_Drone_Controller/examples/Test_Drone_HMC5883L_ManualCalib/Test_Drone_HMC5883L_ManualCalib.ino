/*----------------------------------------------
* CHUONG TRINH TEST HMC5883L: MANUAL CALIB
* DOC GOC YAW
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/10/07
-------------------------------------------------*/
#include <ORC_Drone_HMC5883L.h>
#include "Arduino.h"

//Khoi tao bien HMC
ORC_Drone_HMC5883L DroneHMC;
float offsetVal[3]={4.957024, -17.126465, 5.203447};
float disx[3] = {0.918457, 0.002606, 0.007833};
float disy[3] = {0.002606, 0.961122, 0.001134};
float disz[3] = {0.007833, 0.001134, 0.963156};

void setup()
{
  //Cai dat PWM
  Serial.begin(115200);
  DroneHMC.HMCSetup();
  //Manual Calib cam bien
  DroneHMC.HMCCalibParaManualSet(offsetVal, disx, disy, disz);
  Serial.println("Calib cam bien thu cong thanh cong.");
  
}

void loop()
{  
  delay(10);
  Serial.println(DroneHMC.HMCReadData());
}
