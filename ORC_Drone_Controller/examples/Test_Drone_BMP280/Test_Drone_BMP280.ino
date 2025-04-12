/*----------------------------------------------
* CHUONG TRINH TEST BMP280
* DOC CAC GIA TRI ACC va GYRO
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/10/18
-------------------------------------------------*/
#include <ORC_Drone_BMP280.h>
#include "Arduino.h"

ORC_Drone_BMP280 DroneBMP(400); //Khoi tao bien voi cac gia tri offset la 0


void setup()
{
  //Cai dat PWM
  Serial.begin(115200);  
  DroneBMP.BMP280Setup(); //Setup BMP
  
}

void loop()
{  
  delay(20);
  Serial.println(DroneBMP.BMP280ReadData()); //In du lieu doc ra  
}
