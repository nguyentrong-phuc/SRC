/*----------------------------------------------
* CHUONG TRINH TEST SEND RF
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/10/07
-------------------------------------------------*/
#include <ORC_Drone_RF.h>
#include "Arduino.h"

uint64_t rfReadId = 1, reSendId = 2;  
//Khai bao bien DroneController
#if defined(STM32F1)
  ORC_Drone_RF DroneRFSend(PB12,PB5, rfReadId,reSendId);
#else 
  ORC_Drone_RF DroneRFSend(9,53, rfReadId,reSendId);
#endif

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  //Khoi tao Serial
  Serial.begin(9600);
  while (!DroneRFSend.RFSendSetup())
	Serial.println("Dang cho RF khoi dong...."); //Dang cho khoi dong
  Serial.println("RF da khoi dong xong.");     // Da khoi dong xong
}

void loop()
{
  delay(10);
  //DroneRFSend.sendingData[0] = '*';
  DroneRFSend.sendingData.rfheader = '*';
  // Serial.println(DroneRFSend.RFSendData());
  if(DroneRFSend.RFSendData()){
	  Serial.print("Da goi dc du lieu toi dia chi ID : "); //Dang cho khoi dong
	  Serial.println((long)reSendId); //Dang cho khoi dong
  }else	 Serial.println(".."); //Dang cho khoi dong 
}
