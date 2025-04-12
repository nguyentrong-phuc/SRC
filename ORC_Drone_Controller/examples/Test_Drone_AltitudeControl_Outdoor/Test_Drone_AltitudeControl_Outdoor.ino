/*----------------------------------------------
* CHUONG TRINH TEST NHAN RF
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/11/06
-------------------------------------------------*/
#include <ORC_Utilities.h>
#include <ORCType.h>
#include "Arduino.h"
#include <ORC_Drone_Controller.h>

#define ET 0.01 //s

uint64_t rfReadId = 2, rfSendId = 1;
//Khai bao bien DroneRFRead

ORC_Utilities ORCUti;
//orcebi_data ebiData;
orcdrone_data droneData;
//Khai bao bien DroneController
ORC_Drone_Controller DroneController(rfReadId, rfSendId);

long currentTime = micros();
//float Ts = 0.01;

void setup()
{
	//Khoi tao Serial ket noi voi may tinh
	Serial.begin(115200);
	//Setup drone system: Disable, control gains
	DroneController.setupDroneSystem(&droneData, true, GPS_RTK_FIX);	//Có su dụng GPS
	
	//droneData.gpsMode = 0;
	//Cai dat gia tri dieu khien	
  //Offset values cho gia toc
	droneData.awex = 0.035;
	droneData.awey = 0.025;
	droneData.awez = 1.016;
	droneData.mpu_data.gex = -4.2;
	droneData.mpu_data.gez= -0.7;
	setupAltitudeControl(&droneData);
	Serial.println("Bat dau kiem tra du lieu bay");	
	digitalWrite(LED_PROGRAM_READY, LOW);
	while (!DroneController.preCheckLoop(&droneData, 100))
		Serial.println("Dang kiem tra du lieu bay");
	Serial.println("Hoan thanh kiem tra du lieu bay");
	digitalWrite(LED_PROGRAM_READY, HIGH);	
}

void loop()
{
	//Delay tinh 10ms
	droneData.Ts = 0.001*ORCUti.dynamicWaiting(ET*1000, &currentTime); //in msec
	//------------RUN CONTROL PROGRAM------------------------------------------
	DroneController.runLoop(&droneData);
	//In du lieu he thong 
	printData2Serial0(&droneData);
}
/*--------------------------------------------
* Cai dat cac thong so dieu khien he thong
---------------------------------------------*/
void printData2Serial0(orcdrone_data *drone_in){
	//In thoi gian lay mau	
	if(Serial.availableForWrite()>= 63){
		Serial.print(drone_in->Ts*1000); Serial.print(", ");
		
		//Serial.print(droneData.zLidar1D - droneData.zLidar1DErr); Serial.print(", ");	//Serial.print(droneData.awz); Serial.print(", ");
		Serial.print(drone_in->z); Serial.print(", ");
		//	Serial.print(droneData.vz); Serial.print(", ");
		//Serial.print(droneData.mpu_data.accx); Serial.print(", ");
		// Serial.println(droneData.ebimu_data.roll); //Serial.print(", ");
		//Serial.println(droneData.ebimu_data.yaw); //Serial.print(", ");
		//Serial.println(droneData.mpu_data.gx); 
		//Serial.println(droneData.mpu_data.gz);
		//  Serial.println(droneData.ebimu_data.pitch); //Serial.print(", ");
		// Serial.println(droneData.mpu_data.gy); Serial.print(", ");
		//Serial.print(droneData.mpu_data.accy); Serial.print(", ");
		//Serial.print(droneData.mpu_data.accz); Serial.print(", ");
		//  Serial.println(droneData.mpu_data.gx);// Serial.print(", ");
		Serial.print(drone_in->zd); Serial.print(", ");
		Serial.println(drone_in->P_basic*0.01); 	
	}
}
/*--------------------------------------------
* Cai dat cac thong so dieu khien he thong
---------------------------------------------*/
void setupAltitudeControl(orcdrone_data *drone_in){
	//Set cac tham so dk khac bang 0
	drone_in->pitch_kp = 12; 	drone_in->pitch_ki = 0.5; 	drone_in->pitch_kd = 6; drone_in->pitch_ei = 0;
	drone_in->roll_kp = 12; 		drone_in->roll_ki = 0.5; 		drone_in->roll_kd = 6; 	drone_in->roll_ei = 0;
	drone_in->yaw_kp = 15; 		drone_in->yaw_ki = 0.5; 		drone_in->yaw_kd = 7; 	drone_in->yaw_ei = 0;
	//Cac he so dieu khien bien x
	drone_in->x_kp = 0; 	drone_in->x_ki = 0; 	drone_in->x_kd = 0; 	drone_in->x_ei1 = 0;drone_in->x_ei2 = 0;
	//Cac he so dieu khien bien y
	drone_in->y_kp = 0; 	drone_in->y_ki = 0; 	drone_in->y_kd = 0; 	drone_in->y_ei1 = 0; drone_in->y_ei2 = 0;
	//Cac he so dieu khien bien z
	drone_in->z_kp = 12; 	drone_in->z_ki = 1; 	drone_in->z_kd = 6; 	drone_in->z_ei1 = 0; drone_in->z_ei2 = 0;	
}