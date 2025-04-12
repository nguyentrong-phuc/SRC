/*----------------------------------------------
* CHUONG TRINH TEST PWM CHO CAC DONG CO
* PWM_MIN = 100; PWM_MAX = 1800;
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/10/07
* Hardware: main module
-------------------------------------------------*/
#include <ORC_Drone_Controller.h>
HardwareSerial Serial2(PA3, PA2);
HardwareSerial Serial3(PB11, PB10);
uint64_t rfReadId = 2, rfSendId = 1;

//Khai bao bien DroneController
ORC_Drone_Controller DroneController(rfReadId,rfSendId);

int16_t lidar1DValue = 0;

void setup()
{
  //Cai dat PWM
  Serial.begin(9600);
  while(!DroneController.Lidar1D_read(&lidar1DValue))
  {
    Serial.println("Dang khoi dong lidar 1D.");
    delay(20);
   };
}

void loop()
{
  
  delay(100);
  if(DroneController.Lidar1D_read(&lidar1DValue))
  {
    Serial.print("Lidar 1D: ");
    Serial.println("lidar1DValue");
  }
  else
    Serial.println("Loi du lieu.");
  
}
