/*----------------------------------------------
* CHUONG TRINH TEST GOC ROLL CHO DRONE 30KG
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2025/01/21
* Hardware: main module (Mega Pro 2560)
* Modified stably in 2025-01-04
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
	DroneController.setupDroneSystem(&droneData, false, GPS_RTK_FIX);	//Có su dụng GPS
	
	//droneData.gpsMode = 0;
	//Cai dat gia tri dieu khien	
  //Offset values cho gia toc cho cam bien thay vao ngay 13/12/2024
//	droneData.awex = -0.06;//-0.0357;//-0.35/9.81;
//	droneData.awey = -0.0275;
//	droneData.awez = 1;
	// droneData.mpu_data.gex = 0.45;//-1.45; -1.1
	// droneData.mpu_data.gey= 0.45;// 0.05; -3.65
 	// droneData.mpu_data.gez= -0.45;//-1.18; -0.65
	droneData.awex = -0.08;//-0.04;//-0.0357;//-0.35/9.81;
  droneData.awey = 0;//-0.0275;
  droneData.awez = 1.12;//1.03;
  droneData.mpu_data.gex = -2.3;//0.45;//-1.45; -1.1
  droneData.mpu_data.gey= 2.95;//0.35;// 0.05; -3.65
  droneData.mpu_data.gez= -0.8;//-0.15;//-1.18; -0.65
	setupAltitudeControl(&droneData);
	Serial.println("Bat dau kiem tra du lieu bay");	
	digitalWrite(LED_PROGRAM_READY, LOW);
//  	while (!DroneController.preCheckLoop(&droneData, 100))
//		Serial.println("Dang kiem tra du lieu bay");
	Serial.println("Hoan thanh kiem tra du lieu bay");
	digitalWrite(LED_PROGRAM_READY, HIGH);	
  droneData.P_basic = 700;
  droneData.autoTuneState = true;
}

void loop()
{
	//Delay tinh 10ms
	droneData.Ts = 0.001*ORCUti.dynamicWaiting(ET*1000, &currentTime); //in msec
	//------------RUN CONTROL PROGRAM------------------------------------------
	DroneController.runLoop(&droneData);
	//In du lieu he thong 
	printData2Serial0(&droneData);
//  digitalWrite(LED_RF_NHAN, HIGH);
}
/*--------------------------------------------
* Cai dat cac thong so dieu khien he thong
---------------------------------------------*/
void printData2Serial0(orcdrone_data *drone_in){
	//In thoi gian lay mau	
	if(Serial.availableForWrite()>= 63){
		Serial.print(drone_in->Ts*1000); Serial.print(", ");
//		
//		//Serial.print(droneData.zLidar1D - droneData.zLidar1DErr); Serial.print(", ");	//Serial.print(droneData.awz); Serial.print(", ");
		// Serial.print(drone_in->x); Serial.print(", ");
    // Serial.print(drone_in->xd); Serial.print(", ");

  // Serial.print(drone_in->y); Serial.print(", ");
    // Serial.print(drone_in->yd); Serial.print(", ");

//    Serial.print(drone_in->z); Serial.print(", ");
//    Serial.print(drone_in->zd); Serial.print(", ");
//			Serial.print(droneData.vz); Serial.print(", ");

//Serial.print(drone_in->mpu_data.accx); Serial.print(", ");
		 Serial.print(drone_in->ebimu_data.roll); Serial.print(", ");
    Serial.print(drone_in->roll_d);// Serial.print(", ");
//  Serial.print(drone_in->ebimu_data.pitch); //Serial.print(", ");
//	Serial.print(drone_in->ebimu_data.yaw); //Serial.print(", ");
//Serial.print(drone_in->mpu_data.gx); Serial.print(", ");
//Serial.print(drone_in->u_roll); //Serial.print(", ");

//Serial.print(drone_in->mpu_data.gy*10+0*100); //Serial.print(", ");
//Serial.print(drone_in->mpu_data.gz*10 - 0*100); //Serial.print(", ");
//		//  Serial.println(droneData.ebimu_data.pitch); //Serial.print(", ");
//		// Serial.println(droneData.mpu_data.gy); Serial.print(", ");
//Serial.print(drone_in->mpu_data.accy); Serial.print(", ");
//Serial.println(drone_in->mpu_data.accz); //Serial.print(", ");
//		//  Serial.println(droneData.mpu_data.gx);// Serial.print(", ");
//		Serial.print(drone_in->zd); Serial.print(", ");
////   Serial.print(drone_in->desiredData.flyMode); Serial.print(", ");
////   Serial.print(drone_in->xd); Serial.print(", ");
//   Serial.print(drone_in->gpsdata.pos_cm[2]); Serial.print(", ");
 //  Serial.print(drone_in->u_z); Serial.print(", ");
//   Serial.print((drone_in->gpsdata.isError?10:0)); Serial.print(", ");

//  Serial.print((drone_in->awx)*100/9.81); //Serial.print(", ");
//  Serial.print((drone_in->awy)*100/9.81); //Serial.print(", ");
//  Serial.print((drone_in->awz)/(10*9.81)); //Serial.print(", ");

   
//  Serial.print((drone_in->awx)/9.81); Serial.print(", ");
//  Serial.print((drone_in->awy)/9.81); Serial.print(", ");
//  Serial.print((drone_in->awz)/(981)); Serial.print(", ");
  
//  Serial.print(drone_in->gpsdata.readErrCnt); //Serial.print(", ");

   // Serial.print(drone_in->auto_current_point);Serial.print(", ");
   // Serial.print(drone_in->reachCount);
//   Serial.print(drone_in->obstacleAvoidanceVar.ov_roll_rec);Serial.print(", ");
//    Serial.print(drone_in->obstacleAvoidanceVar.ov_pitch_rec);Serial.print(", ");
//    Serial.print(drone_in->obstacleAvoidanceVar.ov_roll_d);Serial.print(", ");
//    Serial.print(drone_in->obstacleAvoidanceVar.ov_pitch_d);Serial.print(", ");
//      Serial.print(drone_in->obstacleAvoidanceVar.ov_xd);Serial.print(", ");
//   Serial.print(drone_in->obstacleAvoidanceVar.ov_yd);
////		Serial.println(drone_in->P_basic*0.01); 
  Serial.println()	;
	}
}
/* Cai dat cac thong so dieu khien he thong
---------------------------------------------*/
void setupAltitudeControl(orcdrone_data *drone_in){
	  //Set cac tham so dk khac bang 0
  // drone_in->pitch_kp = 12;  drone_in->pitch_ki = 0.5;   drone_in->pitch_kd = 6; drone_in->pitch_ei = 0;
  drone_in->pitch_kp = 0;  drone_in->pitch_ki = 0;   drone_in->pitch_kd = 0; drone_in->pitch_ei = 0;
//  drone_in->pitch_kp = 15;  drone_in->pitch_ki = 0.5;   drone_in->pitch_kd = 6; drone_in->pitch_ei = 0;
//  drone_in->pitch_kp = 4;  drone_in->pitch_ki = 0;   drone_in->pitch_kd = 2; drone_in->pitch_ei = 0;
  // drone_in->roll_kp = 12;     drone_in->roll_ki = 0.5;    drone_in->roll_kd = 6;  drone_in->roll_ei = 0;
  
    drone_in->roll_kp = 12;     drone_in->roll_ki = 0.5;    drone_in->roll_kd = 3;  drone_in->roll_ei = 0;

    
//  drone_in->roll_kp = 4;     drone_in->roll_ki = 0;    drone_in->roll_kd = 2;  drone_in->roll_ei = 0;
  drone_in->yaw_kp = 0;    drone_in->yaw_ki = 0;     drone_in->yaw_kd = 0;   drone_in->yaw_ei = 0;
//  drone_in->yaw_kp = 15;    drone_in->yaw_ki = 0.5;     drone_in->yaw_kd = 7;   drone_in->yaw_ei = 0;
  drone_in->yaw_kp = 0;    drone_in->yaw_ki = 0;     drone_in->yaw_kd = 0;   drone_in->yaw_ei = 0;
  //Cac he so dieu khien bien x
  drone_in->x_kp = 0;   drone_in->x_ki = 0;   drone_in->x_kd = 0;   drone_in->x_ei1 = 0;drone_in->x_ei2 = 0;
  //Cac he so dieu khien bien y
  drone_in->y_kp = 0;   drone_in->y_ki = 0;   drone_in->y_kd = 0;   drone_in->y_ei1 = 0; drone_in->y_ei2 = 0;
  //Cac he so dieu khien bien z
//  drone_in->z_kp = 20;  drone_in->z_ki = 1;   drone_in->z_kd = 7;   drone_in->z_ei1 = 0; drone_in->z_ei2 = 0; 
  drone_in->z_kp = 0;  drone_in->z_ki = 0;   drone_in->z_kd = 0;   drone_in->z_ei1 = 0; drone_in->z_ei2 = 0; 
//  drone_in->z_kp = 13;  drone_in->z_ki = 1;   drone_in->z_kd = 4.5;   drone_in->z_ei1 = 0; drone_in->z_ei2 = 0; 
//drone_in->z_kp = 15;  drone_in->z_ki = 1;   drone_in->z_kd = 7.8;   drone_in->z_ei1 = 0; drone_in->z_ei2 = 0; 
}