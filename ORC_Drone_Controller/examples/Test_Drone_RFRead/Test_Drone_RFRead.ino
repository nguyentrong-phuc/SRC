/*----------------------------------------------
* CHUONG TRINH TEST NHAN RF
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/10/07
-------------------------------------------------*/
#include <ORC_Drone_RF.h>
#include <ORCType.h>
#include "Arduino.h"

uint64_t rfReadId = 2, reSendId = 1;
//Khai bao bien DroneRFRead
#if defined(STM32F1)
  ORC_Drone_RF DroneRFRead(PB12,PB5, rfReadId,reSendId);
#else 
  ORC_Drone_RF DroneRFRead(9,53, rfReadId,reSendId);
#endif

orcdrone_data droneData;
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  //Khoi tao Serial
  Serial.begin(9600);
  while (!DroneRFRead.RFReadSetup())
	Serial.println("Dang cho RF khoi dong...."); //Dang cho khoi dong
  Serial.println("RF da khoi dong xong.");     // Da khoi dong xong
}

void loop()
{
  delay(10);
  Serial.println(DroneRFRead.RFReadData(&droneData));
  if(DroneRFRead.RFReadData(&droneData)){
	  Serial.print("Doc duoc du lieu tu ID : "); //Dang cho khoi dong
	  Serial.println((long)rfReadId);
  }else	 Serial.println(".."); //Dang cho khoi dong  
}
