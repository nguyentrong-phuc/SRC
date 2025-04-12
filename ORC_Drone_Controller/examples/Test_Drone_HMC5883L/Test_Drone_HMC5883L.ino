/*----------------------------------------------
* CHUONG TRINH TEST HMC5883L
* DOC GOC YAW
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/10/07
-------------------------------------------------*/
#include <ORC_Drone_HMC5883L.h>
#include "Arduino.h"

ORC_Drone_HMC5883L DroneHMC;


void setup()
{
  //Cai dat PWM
  Serial.begin(115200);
  DroneHMC.HMC_Setup();
}

void loop()
{  
  delay(100);
  Serial.println(DroneHMC.HMC_read());
}
