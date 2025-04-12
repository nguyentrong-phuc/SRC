/*----------------------------------------------
* CHUONG TRINH TEST MPU6050
* DOC CAC GIA TRI ACC va GYRO
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/10/07
* Hardware: main module
-------------------------------------------------*/
#include <ORC_Drone_MPU6050.h>
#include "Arduino.h"
#include "ORCType.h"

ORC_Drone_MPU6050 DroneMPU; //Khoi tao bien voi cac gia tri offset la 0
orcmpu_data mpuVar;

void setup()
{
  //Cai dat PWM
  Serial.begin(115200);
  DroneMPU.MPU6050Setup(MPU6050_MODE_CONTROL);
  //Set offset value
//  DroneMPU.MPU6050SetOffsetValues(&mpuVar, 0.03,-0.02,0.11,29,0,-2);
  
  while (!DroneMPU.MPU6050SetOffsetValues(&mpuVar, 0.05,0.01,0.1,-2.5,-5.8,4))
	Serial.println("Set offset value bi loi");
}

void loop()
{  
  delay(10);
  if(DroneMPU.MPU6050ReadData(0.001, &mpuVar)){
    //Serial.print("Accx: "); 
    Serial.print(mpuVar.accz);
    // Serial.print("\tAccy: "); Serial.print(mpuVar.accy);
    // Serial.print("\tAccz: "); Serial.print(mpuVar.accz);
    // Serial.print("\tgx: "); Serial.print(mpuVar.gx);
    // Serial.print("\tgy: "); Serial.print(mpuVar.gy);
    // Serial.print("\tgz: "); Serial.print(mpuVar.gz);
    // Serial.print("\troll: "); Serial.print(mpuVar.mroll);
    // Serial.print("\tpit: "); Serial.print(mpuVar.mpitch);
    Serial.println();  
    
    // Serial.print(mpuVar.mroll); Serial.print(",");  
    // Serial.print(mpuVar.mpitch); Serial.println(",");  
  }else
  {
    Serial.println("-------------------------------------------------LOI DOC DU LIEU--------------------------"); 
  }
  
}