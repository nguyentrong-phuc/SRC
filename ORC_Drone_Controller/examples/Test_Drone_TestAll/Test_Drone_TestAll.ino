/*----------------------------------------------
* CHUONG TRINH TEST NHAN RF
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/10/07
-------------------------------------------------*/
#include <ORC_Drone_RF.h>
#include <ORC_Drone_EBIMU.h>
#include <ORCType.h>
#include <ORC_Drone_MPU6050.h>
#include <ORC_Drone_PWM.h>
#include "Arduino.h"
#include <ORC_Drone_HMC5883L.h>
#include <ORC_Drone_Controller.h>

uint64_t rfReadId = 2, reSendId = 1;
//Khai bao bien DroneRFRead
ORC_Drone_RF DroneRFRead(9,53, rfReadId,reSendId);
ORC_Drone_EBIMU DroneEBI_data(100); //Khoi tao bien voi cac gia tri offset la 0
ORC_Drone_MPU6050 DroneMPU; //Khoi tao bien voi cac gia tri offset la 0
ORC_Utilities ORCUti;
orcebi_data ebiData;
orcdrone_data droneData;
//Khai bao bien DroneController
ORC_Drone_Controller DroneController;
ORC_Drone_PWM DronePWM;
//Khoi tao bien HMC
ORC_Drone_HMC5883L DroneHMC;
float offsetVal[3]={4.957024, -17.126465, 5.203447};
float disx[3] = {0.918457, 0.002606, 0.007833};
float disy[3] = {0.002606, 0.961122, 0.001134};
float disz[3] = {0.007833, 0.001134, 0.963156};


long currentTime = micros();
float Ts = 0.01;

void setup()
{
	//Khoi tao Serial ket noi voi may tinh
	Serial.begin(115200);
	//Khoi dong RF
	while (!DroneRFRead.RFReadSetup())
		Serial.println("Dang cho RF khoi dong...."); //Dang cho khoi dong
	Serial.println("RF da khoi dong xong.");     // Da khoi dong xong
	//Khoi dong doc EBIMU
	DroneEBI_data.EBISetOffsetValues(&(droneData.ebimu_data), 0 , 0, 0);
	//Khoi dong bien mpu6050
	DroneMPU.MPU6050Setup(MPU6050_MODE_CONTROL);
	//Khoi dong doc Lidar1D
	while(!DroneController.Lidar1D_read(&(droneData.zLidar1D)))
	{
		Serial.println("Dang khoi dong lidar 1D.");
		delay(20);
	};
	//Khoi dong bien HMC
	DroneHMC.HMCSetup();
	//Manual Calib cam bien
	DroneHMC.HMCCalibParaManualSet(offsetVal, disx, disy, disz);
	Serial.println("Calib cam bien thu cong thanh cong.");
	//Cai dat PWM
	DronePWM.PWMSetup();
	DronePWM.setPWMMaxMin(0, 2000);
}

void loop()
{
  //Delay tinh 10ms
  ORCUti.dynamicWaiting(Ts*1000, &currentTime); //in msec
  //Doc tin hieu RF
  if(DroneRFRead.RFReadData(&droneData)){
	  Serial.print("Doc duoc du lieu, StartCommand: "); //Dang cho khoi dong
	  Serial.println(droneData.desiredData.StartCommand);
  }else	 Serial.println(".."); //Dang cho khoi dong  
  //Doc tin hieu tu EBIMU
  if(DroneEBI_data.EBIReadData(&(droneData.ebimu_data))){    
		if(Serial.availableForWrite()>0) 
			Serial.print(droneData.ebimu_data.roll); Serial.print(",");
			Serial.print(droneData.ebimu_data.pitch); Serial.print(",");
			Serial.print(droneData.ebimu_data.yaw); Serial.print(",");
			Serial.println(droneData.ebimu_data.readErrCnt); 
	}
	//Doc du lieu tu 6050
	if(DroneMPU.MPU6050ReadData(Ts, &(droneData.mpu_data))){
		Serial.print("Accx: "); Serial.print(droneData.mpu_data.accx);
		Serial.print(" Accy: "); Serial.print(droneData.mpu_data.accy);
		Serial.print(" Accz: "); Serial.print(droneData.mpu_data.accz);
		Serial.print(" gx: "); Serial.print(droneData.mpu_data.gx);
		Serial.print(" gy: "); Serial.print(droneData.mpu_data.gy);
		Serial.print(" gz: "); Serial.print(droneData.mpu_data.gz);
		Serial.print(" roll: "); Serial.print(droneData.mpu_data.mroll);
		Serial.print(" pit: "); Serial.print(droneData.mpu_data.mpitch);
		Serial.println(".");  
	  }else
	  {
		Serial.println("-------------------------------------------------LOI DOC DU LIEU--------------------------"); 
	  }
	//Doc du lieu lidar1D
	if(DroneController.Lidar1D_read(&(droneData.zLidar1D)))
	{
		Serial.print("Lidar 1D: ");
		Serial.println(droneData.zLidar1D);
	}
	else
		Serial.println("Loi du lieu.");
	//Doc du lieu HMC
	Serial.print("HMC: ");
	Serial.println(DroneHMC.HMCReadData());
	//Set PWM
	DronePWM.setPWM(droneData.Motor1_Speed, droneData.Motor2_Speed,droneData.Motor3_Speed,droneData.Motor4_Speed);
}