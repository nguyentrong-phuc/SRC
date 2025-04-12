/*----------------------------------------------
* CHUONG TRINH TEST NHAN RF
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/11/06
-------------------------------------------------*/
#include <ORC_Drone_RF.h>
#include <ORC_Drone_EBIMU.h>
#include <ORCType.h>
#include <ORC_Drone_MPU6050.h>
#include <ORC_Drone_PWM.h>
#include "Arduino.h"
#include <ORC_Drone_HMC5883L.h>
#include <ORC_Drone_Controller.h>

#define ET 0.01 //s

uint64_t rfReadId = 2, rfSendId = 1;
//Khai bao bien DroneRFRead
//ORC_Drone_RF DroneRFRead(9,53, rfReadId,reSendId);
ORC_Drone_EBIMU DroneEBI_data(100); //Khoi tao bien voi cac gia tri offset la 0
ORC_Drone_MPU6050 DroneMPU; //Khoi tao bien voi cac gia tri offset la 0
ORC_Utilities ORCUti;
//orcebi_data ebiData;
orcdrone_data droneData;
//Khai bao bien DroneController
ORC_Drone_Controller DroneController(rfReadId, rfSendId);
ORC_Drone_PWM DronePWM;
//Khoi tao bien HMC
ORC_Drone_HMC5883L DroneHMC;
float offsetVal[3]={4.957024, -17.126465, 5.203447};
float disx[3] = {0.918457, 0.002606, 0.007833};
float disy[3] = {0.002606, 0.961122, 0.001134};
float disz[3] = {0.007833, 0.001134, 0.963156};


long currentTime = micros();
//float Ts = 0.01;

void setup()
{
	//Khoi tao Serial ket noi voi may tinh
	Serial.begin(115200);
	//Setup drone system: Disable, control gains
	DroneController.setupDroneSystem(&droneData, false,GPS_RTK_NOSOLUTION);	
	
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
/*--------------------------------------------
* Cai dat cac thong so dieu khien he thong
---------------------------------------------*/
void setupAltitudeControl(orcdrone_data *drone_in){
	//Set cac tham so dk khac bang 0
	drone_in->pitch_kp = 10; 	drone_in->pitch_ki = 0.5; 	drone_in->pitch_kd = 4; drone_in->pitch_ei = 0;
	drone_in->roll_kp = 10; 		drone_in->roll_ki = 0.5; 		drone_in->roll_kd = 4; 	drone_in->roll_ei = 0;
	drone_in->yaw_kp = 0; 		drone_in->yaw_ki = 0; 		drone_in->yaw_kd = 0; 	drone_in->yaw_ei = 0;
	//Cac he so dieu khien bien x
	drone_in->x_kp = 0; 	drone_in->x_ki = 0; 	drone_in->x_kd = 0; 	drone_in->x_ei1 = 0;drone_in->x_ei2 = 0;
	//Cac he so dieu khien bien y
	drone_in->y_kp = 0; 	drone_in->y_ki = 0; 	drone_in->y_kd = 0; 	drone_in->y_ei1 = 0; drone_in->y_ei2 = 0;
	//Cac he so dieu khien bien z
	drone_in->z_kp = 12; 	drone_in->z_ki = 1; 	drone_in->z_kd = 4; 	drone_in->z_ei1 = 0; drone_in->z_ei2 = 0;	
}