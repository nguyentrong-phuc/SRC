/*
  ORC_Drone_Controller.h - Library for basic functions manipulating devices and signal processing.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#ifndef ORC_Drone_Controller_h
#define ORC_Drone_Controller_h

#include "Arduino.h"
#include "ORC_Drone_PWM.h"
#include "ORC_Drone_EBIMU.h"
#include "ORC_Drone_MPU6050.h"
#include "ORC_Drone_RF.h"
#include "ORCType.h"
#include "ORC_Drone_HMC5883L.h"
#include "ORC_Drone_WarningFault.h"

#define LIDAR1D_MAX_UNCHANGE 50  //500ms
#define LIDAR1D_GOOD_STANDRANGE 40  //20cm

#if defined(ARDUINO_ARCH_STM32)
	#define LED_RF_NHAN	PA15
	#define LED_PROGRAM_READY PB3
	#define LED_GPS_READY PB4
	#define LED_COMMAND_STATUS PA5
	#define LED_EMERCOMMAND_CNT_MAX PC13 //200ms
	#define LED_READYCOMMAND_CNT_MAX PC13 //200ms
#else
	#define LED_RF_NHAN	8
	#define LED_PROGRAM_READY 29
	#define LED_GPS_READY 30
	#define LED_COMMAND_STATUS 31
	#define LED_EMERCOMMAND_CNT_MAX 20 //200ms
	#define LED_READYCOMMAND_CNT_MAX 20 //200ms
#endif

#define LIDARPort Serial3
#define HUBPort Serial2

#define HUBSENDTIMEBASE	100
#define OBSTACLEAVOIDANCE_CONTROLDATA_TIMEOUT 150//150 ms
#define OBSTACLEAVOIDANCE_CONTROLDATA_ACCEPTANCE 5.8 //[deg]
#define OBSTACLEAVOIDANCE_ACTIVE_HEIGHT 550 //150[cm]
#define OBSTACLEAVOIDANCE_XY_POSITION_MAX 150 //150 //[m]

#define AUTOTUNE_POWER_STEP 1.5f //1.5 //0.7

class ORC_Drone_Controller
{
  public:
    ORC_Drone_Controller(uint64_t rfReadId, uint64_t rfSendId);	
	void setupDroneSystem(orcdrone_data *drone_in, bool gpsMode_in, long RTK_StandardIn);
	bool preCheckLoop(orcdrone_data *drone_in, int16_t maxCheckNumber);
	void runLoop(orcdrone_data *drone_in);
	ORC_Drone_PWM DronePWM;
	bool Lidar1D_read(int16_t *distance_out);
	//int16_t Lidar1_value;
  private:
	//ORC_Drone_PWM DronePWM;
    int _pin;
	int16_t Lidar1_strength;
	float exTemp, eyTemp, exi1Temp, exi2Temp, eyi1Temp, eyi2Temp,k_ccTemp, uzTemp,eATemp, eAiTemp;
	float uATemp,u1Temp, u2Temp, u3Temp, u4Temp, m1Temp, m2Temp, m3Temp, m4Temp;
	float ux, uy, yawTemp, cm1Temp, cm2Temp,ezTemp,ezi1Temp,ezi2Temp;
	float x_kalman_1[2], y_kalman_1[2],z_kalman_1[2];
	int16_t emerCommand_Cnt, readyCommand_Cnt,hubSendData_Cnt;
	bool commandStatusStore = true;
	ORC_Drone_RF *DroneRFRead;
	ORC_Drone_EBIMU *DroneEBI_data;
	ORC_Drone_MPU6050 DroneMPU;
	ORC_Drone_HMC5883L DroneHMC;
	ORC_Utilities ORCControllerUti;	
	orcstring_data dronePositionData, droneGPSData, droneObstacleControlData, dronePumpFBData;
	
	ORC_Drone_WarningFault Drone_WarningFault;
	// Function
	// bool Lidar1D_read(int16_t *distance_out);
	
	void Program_Processing(long dataPoint[][POINT_MEMORY_MAX_LEN],orcdrone_data *drone_in);
	void move2NextPoint(long dataPoint[][POINT_MEMORY_MAX_LEN], orcdrone_data *drone_in);
	void z_TrajectoryPlanning(long dataPoint[][POINT_MEMORY_MAX_LEN], orcdrone_data *drone_in);
	void xy_TrajectoryPlanning(long dataPoint[][POINT_MEMORY_MAX_LEN], orcdrone_data *drone_in);
	float Traj_Planning_const_Vel(float zd, float z, float step_size);
	void autoTunePminDrone(orcdrone_data *drone_in);
	void rp_TrajectoryPlanning(orcdrone_data *drone_in);
	void rpy_TrajectoryPlanning(orcdrone_data *drone_in);
	void xy_Controller(orcdrone_data *drone_in);
	void z_Controller(orcdrone_data *drone_in);
	void rpy_Controller(orcdrone_data *drone_in);
	void motorSpeed_Caculator(orcdrone_data *drone_in, float ScaleEng);
	float sgn(float x);
	void run(orcdrone_data *drone_in, uint16_t activeIn);
	void setControlGains(orcdrone_data *drone_in);
	
	float Kalman_1( float *xk, float ak, float zm, float dt1, int full_act, float k1, float k2);
	void MPUDataProcessing(orcdrone_data *drone_in);
	void AltitudeDataProcessing(orcdrone_data *drone_in);
	void PositionDataProcessing(orcdrone_data *drone_in);
	void Rotation(float ax, float ay, float az, float roll_i, float pitch_i, float yaw_i, float *Rx_i, float *Ry_i, float *Rz_i);
	
	bool Lidar1DCheckData(orcdrone_data *drone_in,int16_t *preLidarData, int16_t currentLidarData, int16_t *unchangeCnt, int16_t unchangeMax);
	
	void commandLedDisplay(orcdrone_data *drone_in);
	void readyLedDisplay(uint8_t in);
	void sendRPY2Hub(orcdrone_data *drone_in); //Goi goc rpy len internet
	void sendTSWarning(orcdrone_data *drone_in); //Goi canh bao toi tay cam khi TS khong on dinh
	void sendData2HUB(orcdrone_data *drone_in);//Goi du lieu canh bao
	void sendFlyPointID2Hub(orcdrone_data *drone_in, int id_in); //Goi Fly Point ID toi Hub
	bool GPSReadData(orcdrone_data *drone_in);
	bool HUBReadData(orcdrone_data *drone_in);
	void sendGPSBaseRequest2Hub(orcdrone_data *drone_in);
	void GPSLedDisplay(orcdrone_data *drone_in);//Hien thi LED GPS
	bool GPSCheckData(orcdrone_data *drone_in, int16_t unchangeMax); //Kiem tra chat luong tin hieu GPS
	void sendGPSWarning(orcdrone_data *drone_in);
	bool EBIMPUSelfCalibration(orcdrone_data *drone_in); //Tu calib EBIMU va MPU
	bool EBIReadandRecalib(orcdrone_data *drone_in);
	byte EBIAutoOffsetCompensation(orcdrone_data *drone_in);
	void sendWarningFaultMsg2Hub(orcdrone_data *drone_in);
	byte EBIAutoCorrection(orcdrone_data *drone_in);
	void ObstacleAvoidanceDataProcessing(orcdrone_data *drone_in); //Xu ly tranh vat can o main
	void PumpDataProcessing(orcdrone_data *drone_in); //Xu ly hoat dong pump o main
	void sendPumpControlData2Hub(orcdrone_data *drone_in); //Goi du lieu pump sang hub
	//ORC_Drone_PWM DronePWM;
	float _K_OV_Gain = 0.98;
	
};

//------------------------Structer for GPS------------------------------


#endif