/*----------------------------------------------
* CHUONG TRINH TEST NHAN RF
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/11/06
* Hardware: main module
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

uint64_t rfReadId = 2, reSendId = 1;
//Khai bao bien DroneRFRead
ORC_Drone_RF DroneRFRead(9,53, rfReadId,reSendId);
ORC_Drone_EBIMU DroneEBI_data(100); //Khoi tao bien voi cac gia tri offset la 0
ORC_Drone_MPU6050 DroneMPU; //Khoi tao bien voi cac gia tri offset la 0
ORC_Utilities ORCUti;
//orcebi_data ebiData;
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
	droneData.desiredData.EmerCommand = true;
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
	
	
	droneData.P_basic = 700;
  DronePWM.PWMSetup();
  DronePWM.setPWMMaxMin(0, 2000);
	droneData.autoTuneState = true;
	//Cai dat gia tri dieu khien
	DroneController.setControlGains(&droneData);	
	setupPitchControl(&droneData);
	droneData.desiredData.EmerCommand = true;	
  droneData.mpu_data.gex = -4.2;
}

void loop()
{
	//Delay tinh 10ms
	droneData.Ts = 0.001*ORCUti.dynamicWaiting(ET*1000, &currentTime); //in msec
	//------------BƯỚC 1: DOC TIN HIEU DIEU KHIEN MONG MUON-------------------------
	//Doc tin hieu RF
	DroneRFRead.RFReadData(&droneData);

	//------------BUOC 2: XU LY CAC TRANG THAI NUT NHAT VA LOI-------------
	DroneController.Program_Processing(droneData.desiredData.data_PtoP_Control,&droneData);

	//------------BƯỚC 3: DOC CAC TRẠNG THAI HE THONG-------------------------
	//Doc tin hieu tu EBIMU
	DroneEBI_data.EBIReadData(&(droneData.ebimu_data));

	//Doc du lieu tu 6050
	DroneMPU.MPU6050ReadData(Ts, &(droneData.mpu_data));

	//Doc du lieu lidar1D
	DroneController.Lidar1D_read(&(droneData.zLidar1D));

	//Doc du lieu HMC
	DroneHMC.HMCReadData();


	//------------BUOC 4: TIEN HANH DIEU KHIEN----------------------------------
	//----BUOC 4-1: QUI HOACH DIEM DEN-------------------------------------------
	DroneController.move2NextPoint(droneData.desiredData.data_PtoP_Control, &droneData);
	//----BUOC 4-2: QUI HOACH QUY DAO-------------------------------------------
	DroneController.z_TrajectoryPlanning(droneData.desiredData.data_PtoP_Control, &droneData);
	DroneController.xy_TrajectoryPlanning(droneData.desiredData.data_PtoP_Control, &droneData);
	//----BUOC 4-3: TINH TOAN GIA TRI DIEU KHIEN---------------------------------
	//----BUOC 4-3-1: DIEU KHIEN CAP CAO X-Y---------------------------------
	DroneController.xy_Controller(&droneData);
	//----BUOC 4-3-2: DIEU KHIEN CAP CAO Z-----------------------------------
	DroneController.z_Controller(&droneData);
	//----BUOC 4-3-3: DIEU KHIEN CAP THAP-----------------------------------
	DroneController.rpy_Controller(&droneData);

	//----------- BUOC 5: XUAT TIN HIEU DIEU KHIEN RA HE THONG----------------------------------
	//----BUOC 5-1: TINH GIA TRI MOTOR SPEED
	DroneController.motorSpeed_Caculator(&droneData, 1);
	//----BUOC 5-2: XUAT PWN
	DronePWM.setPWMFromDrone(&droneData, 1);

  //In du lieu
  Serial.print(droneData.Ts*1000); Serial.print(", ");
  Serial.print(droneData.ebimu_data.pitch); Serial.print(", ");
//  Serial.println(droneData.mpu_data.gx);// Serial.print(", ");
  Serial.println(droneData.desiredData.manual_pitchd); 
}

void setupPitchControl(orcdrone_data *drone_in){
  drone_in->roll_kp = 0;   drone_in->roll_ki = 0;  drone_in->roll_kd = 0;  drone_in->roll_ei = 0;
	//Set cac tham so dk khac bang 0
	drone_in->pitch_kp = 12; 	drone_in->pitch_ki = 0.5; 	drone_in->pitch_kd = 4.5; drone_in->pitch_ei = 0;
	//Cac he so dieu khien goc yaw
	drone_in->yaw_kp = 0; 		drone_in->yaw_ki = 0; 		drone_in->yaw_kd = 0; 	drone_in->yaw_ei = 0;
	
	//Cac he so dieu khien bien x
	drone_in->x_kp = 0; 	drone_in->x_ki = 0; 	drone_in->x_kd = 0; 	drone_in->x_ei1 = 0;drone_in->x_ei2 = 0;
	//Cac he so dieu khien bien y
	drone_in->y_kp = 0; 	drone_in->y_ki = 0; 	drone_in->y_kd = 0; 	drone_in->y_ei1 = 0; drone_in->y_ei2 = 0;
	//Cac he so dieu khien bien z
	drone_in->z_kp = 0; 	drone_in->z_ki = 0; 	drone_in->z_kd = 0; 	drone_in->z_ei1 = 0; drone_in->z_ei2 = 0;	
}