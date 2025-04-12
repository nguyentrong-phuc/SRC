/*----------------------------------------------
* CHUONG TRINH TEST MPU6050
* DOC CAC GIA TRI ACC va GYRO
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/10/07
-------------------------------------------------*/
#include <ORC_Drone_MPU6050.h>
#include <ORC_Utilities.h>
#include "Arduino.h"
#include "ORCType.h"

#define ET 0.01
#define NOMOVE_ACC_VALUE 3 //[m/s]

ORC_Drone_MPU6050 DroneMPU; //Khoi tao bien voi cac gia tri offset la 0
orcmpu_data mpuVar;
ORC_Utilities ORCUti;
long currentTime = micros();
long wrongdataCnt = 0;
long totalTimePass = 100*60*10;

void setup()
{
  //Cai dat PWM
  Serial.begin(115200);
  DroneMPU.MPU6050Setup(MPU6050_MODE_CONTROL);
  //Set offset value
 // DroneMPU.MPU6050SetOffsetValues(&mpuVar, 0,0,0,0,0,0);
  
  //while (!DroneMPU.MPU6050SetOffsetValues(&mpuVar, 0,0,0,0,0,0))
	Serial.println("Set offset value bi loi");
}

void loop()
{  
  ORCUti.dynamicWaiting(ET*1000, &currentTime); //in msec
  totalTimePass--;
  if(totalTimePass >0){
	  Serial.print("Checking!");
	    if(DroneMPU.MPU6050ReadData(0.01, &mpuVar)){
		  if(fabs((mpuVar.accz -1)*9.81) > NOMOVE_ACC_VALUE)
		  {
			  wrongdataCnt++;
		  }  
		  Serial.print(" Accz Error: "); Serial.println(wrongdataCnt);
		  
		  
		// Serial.print("Accx: "); Serial.print(mpuVar.accx);
		// Serial.print(" Accy: "); Serial.print(mpuVar.accy);
		// Serial.print(" Accz: "); Serial.print(mpuVar.accz);
		// Serial.print(" gx: "); Serial.print(mpuVar.gx);
		// Serial.print(" gy: "); Serial.print(mpuVar.gy);
		// Serial.print(" gz: "); Serial.print(mpuVar.gz);
		// Serial.print(" roll: "); Serial.print(mpuVar.mroll);
		// Serial.print(" pit: "); Serial.print(mpuVar.mpitch);
		// Serial.println(".");  
	  }else
	  {
		  Serial.print("Reading error: "); Serial.println(mpuVar.readErrCnt);
		//Serial.println("-------------------------------------------------LOI DOC DU LIEU--------------------------"); 
	  }
  }else if(totalTimePass == 0){
	  if(wrongdataCnt > 0){
		  Serial.print("Checking done: FAIL!");
	  }else
		Serial.print("Checking done: OK!");
  }else
	  totalTimePass = -1;
	  

  
}