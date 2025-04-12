/*
  ORC_Drone_Controller.cpp - a Arduino library used to setup and control quadcopters.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#include "Arduino.h"
#if defined(STM32F1)
	HardwareSerial Serial2(PA3, PA2);
	HardwareSerial Serial3(PB11, PB10);
#endif
#include "ORC_Drone_Controller.h"
#include "ORC_Drone_PWM.h"


/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_Controller::ORC_Drone_Controller(uint64_t rfReadId, uint64_t rfSendId)
{
  _pin = 13;	
  #if defined(STM32F1)
	// LIDARPort.begin(115200); LIDARPort.setTimeout(2); 
	// HUBPort.begin(115200); HUBPort.setTimeout(2); 
	DroneRFRead = new ORC_Drone_RF(PB12,PB5, rfReadId,rfSendId); 
  #else
	DroneRFRead = new ORC_Drone_RF(9,53, rfReadId,rfSendId); 
  #endif

  DroneEBI_data = new ORC_Drone_EBIMU(100);// Khoi tao bien doc EBIMU
  
  //Cau hinh cho du lieu vi tri: "*P 142, -455, 457, 0 34232#" ~ "*P x, y, z,status CRC#" in (cm)
  dronePositionData.isNew = false;
  dronePositionData.preFixString = ORCControllerUti.PositionDataPrefix;
  dronePositionData.len = 5;
  
  //Cau hinh cho du lieu toa do: "*G 12458756, -154626, 165444, 1, 34232#" ~ "*G lat, lon, alt, status CRC#" in (x10^7 (lat, lon) cm)
  droneGPSData.isNew = false;
  droneGPSData.preFixString = ORCControllerUti.GPSDataPrefix;
  droneGPSData.len = 5;
 
 //Cau hinh cho du lieu goc dieu khien: "*LC 10, 10, 20#" in (10* DEG)
  droneObstacleControlData.isNew = false;
  droneObstacleControlData.preFixString = ORCControllerUti.Lidar2DControlDataPrefix;
  droneObstacleControlData.len = 3;

  //Cau hinh cho du lieu flow feedback: "*FF 10, 10#" in (10* DEG)
  dronePumpFBData.isNew = false;
  dronePumpFBData.preFixString = ORCControllerUti.FlowFeedbackDataPrefix;
  dronePumpFBData.len = 2;

  }


/*----------------------------------------
* RUN FUNCTION
------------------------------------------*/
void ORC_Drone_Controller::run(orcdrone_data *drone_in, uint16_t activeIn)
{  
	delay(10);
}
/*----------------------------------------
Read lidar signal
------------------------------------------*/
bool ORC_Drone_Controller::Lidar1D_read(int16_t *distance_out){
  uint8_t       TFbuff[9] = {0};
  long          checksum  = 0 ;
  uint32_t serial1Timeout = micros() + 1000; // 1ms timeout
  int16_t distance_in = 0;
  int16_t strength_in = 0;

  if(LIDARPort.available()){  
	// Serial.println("...");
    TFbuff[0] = LIDARPort.read();
    TFbuff[1] = LIDARPort.read(); 
        
    while( (TFbuff[0] != 0x59) || (TFbuff[1] != 0x59)){
      TFbuff[0] = TFbuff[1];
      TFbuff[1] = LIDARPort.read();
	//   Serial.println(TFbuff[1]);
      if( micros() >  serial1Timeout) return false;
    }
    
    checksum += TFbuff[0];
    checksum += TFbuff[1];
  
    for(int i = 2;i < 8;i++){
      TFbuff[i] = LIDARPort.read();
      checksum += TFbuff[i];
    }
    
    TFbuff[8] = LIDARPort.read();
    checksum &= 0xff; 

    if(checksum == TFbuff[8]){
      distance_in = TFbuff[2]+TFbuff[3]*256;
      strength_in = TFbuff[4]+TFbuff[5]*256;
	  
      if(isnan(distance_in)) return false;
      if(distance_in<1200 && distance_in>0){
        *distance_out = distance_in - lidarDisOFFset;
        Lidar1_strength = strength_in;
        return true;    
      }
      else return false;
      
    }else{
        return false;
    }
  }
}
/*----------------------------------------
* Check lidar signal
------------------------------------------*/
bool ORC_Drone_Controller::Lidar1DCheckData(orcdrone_data *drone_in,int16_t *preLidarData, int16_t currentLidarData, int16_t *unchangeCnt, int16_t unchangeMax){
  int16_t   preLidar = *preLidarData, ucCnt = *unchangeCnt;
  bool result = true;
  /*
  //Kiem tra co su thay doi cua tin hieu khong
  if(preLidar == currentLidarData) {
	ucCnt++;	
	if(ucCnt >= unchangeMax) {
		ucCnt = unchangeMax;
		result = false;
		Serial.print("That bai vi lidar ucCnt = ");
		Serial.println(ucCnt);
	}
  }else
  {
	 ucCnt = 0; 
  }
  *unchangeCnt = ucCnt;
  */
  //Kiem tra tin hieu co nam trong mien cho phep hay khong
  if(((currentLidarData - drone_in->zLidar1DErr)< -LIDAR1D_GOOD_STANDRANGE) || ((currentLidarData - drone_in->zLidar1DErr) > LIDAR1D_GOOD_STANDRANGE))
	result = false;

	//Luu lai gia tri moi
	*preLidarData = currentLidarData;
	return result; 
  
}
/*---------------------------------------------------------------------
 * Xu ly chuong trinh he thong
 * Cap nhat bien: luu du lieu bay he thong
 ----------------------------------------------------------------------*/
void ORC_Drone_Controller::Program_Processing(long dataPoint[][POINT_MEMORY_MAX_LEN],orcdrone_data *drone_in){
	int temp = 0;
	if(drone_in->desiredData.EmerCommand == false) {
		//Che do an toan: khi drone dang sat mat dat ma co vat can den gian thi chuyen sang che do Emer
		if((drone_in->z < 20)&&((fabs(drone_in->obstacleAvoidanceVar.ov_roll_rec) > OBSTACLEAVOIDANCE_CONTROLDATA_ACCEPTANCE)||(fabs(drone_in->obstacleAvoidanceVar.ov_pitch_rec) > OBSTACLEAVOIDANCE_CONTROLDATA_ACCEPTANCE)))
		{
			drone_in->desiredData.EmerCommand = true; drone_in->desiredData.StopCommand = false; drone_in->desiredData.StartCommand = false;         
		}
		//Cac che do khac
		if (drone_in->autoTuneState){
			if(drone_in->desiredData.StopCommand){
			// STOP MOTOR KHI DRONE SAT MAT DAT
				if(drone_in->zd <=0)
				{
					if(drone_in->z < 5) //Chuyen qua mode Emer
					{
						drone_in->desiredData.EmerCommand = true; drone_in->desiredData.StopCommand = false; drone_in->desiredData.StartCommand = false;         
					}
				}
			}else{
				if(drone_in->desiredData.StartCommand){
					/*==================XU LY CAC TRUONG HOP LOI HE THONG========================*/
					if(drone_in->WarningFault != 0){
						/*--------Truong hop loi mat GPS----------------------------- 
						* Truong hop nay chi su dung duoc mode FLY_MODE_MANUAL_ANGLE, va dung he thong
						
						------------------------------------------------------------*/
						if(Drone_WarningFault.isWFRequiredError(drone_in->WarningFault, WF_Error_GPS_Timeout))
						{
							drone_in->desiredData.flyMode = FLY_MODE_MANUAL_ANGLE; //Chuyen he thong ve mode FLY_MODE_MANUAL_ANGLE
							//drone_in->desiredData.manual_rolld = 0;
							//drone_in->desiredData.manual_pitchd = 0;
							drone_in->desiredData.EmerCommand = false; drone_in->desiredData.StopCommand = true; drone_in->desiredData.StartCommand = false; 
						}
						
						/*--------Truong hop tin hieu GPS yeu (NO SOLUTION)-----------------------------
						* Dung lidar de do do cao (trong phan do luong)
						* Neu he thong o che do bay FLY_MODE_MULTI_POINT thi se chuyen ve che do FLY_MODE_ONE_POINT 
						va drone dung tai vi tri do
						--------------------------------------------------------------------------------*/
						if(Drone_WarningFault.isWFRequiredError(drone_in->WarningFault, WF_Error_GPS_NoSolution)){
							if(drone_in->desiredData.flyMode == FLY_MODE_MULTI_POINT){
								drone_in->desiredData.flyMode = FLY_MODE_ONE_POINT;
								//Lay vi tri du lieu hien thoi
								temp = drone_in->auto_current_point;
								if(temp < 0)	
									temp = 0;
								else if ((temp + 1) >=  dataPoint[0][POINT_MEMORY_INDEX_LEN])
									temp =  dataPoint[0][POINT_MEMORY_INDEX_LEN] - 1;
								//Cap nhat lai vi
								dataPoint[0][POINT_MEMORY_INDEX_XD] = dataPoint[temp][POINT_MEMORY_INDEX_XD];
								dataPoint[0][POINT_MEMORY_INDEX_YD] = dataPoint[temp][POINT_MEMORY_INDEX_YD];
								dataPoint[0][POINT_MEMORY_INDEX_LEN] = 1;
								drone_in->auto_current_point = 0;
							}
						}
						//----------XU LY TRUONG HOP MAT TIN HIEU RF RF-------------------------------
						if(Drone_WarningFault.isWFRequiredError(drone_in->WarningFault, WF_Error_RF_Timeout)){
							//Stop khi mat tin hieu o che do Manual Angle
							if(drone_in->desiredData.flyMode == FLY_MODE_MANUAL_ANGLE){ 
								drone_in->desiredData.StartCommand = false;
								drone_in->desiredData.StopCommand = true;
								drone_in->desiredData.manual_rolld = 0;
								drone_in->desiredData.manual_pitchd = 0;
								drone_in->desiredData.manual_yawd = 0;
							}else 
							//Stop khi mat tin hieu o che do Manual Pos
							if(drone_in->desiredData.flyMode == FLY_MODE_MANUAL_POSITION){ 
								drone_in->desiredData.StartCommand = false;
								drone_in->desiredData.StopCommand = true;
								//drone_in->desiredData.manual_xd = 0;
								//drone_in->desiredData.manual_yd = 0;
							}else 
							//Chi dung khi xong hanh trinh che do bay auto
							if (drone_in->desiredData.flyMode == FLY_MODE_MULTI_POINT){ 
								if((drone_in->auto_current_point > 0)&&(drone_in->auto_current_point < MaxPtoPnum)&&(dataPoint[0][POINT_MEMORY_INDEX_LEN] > 1)&&((drone_in->auto_current_point + 1)== dataPoint[0][POINT_MEMORY_INDEX_LEN])){
									if(dataPoint[drone_in->auto_current_point][POINT_MEMORY_INDEX_MISSION_STATUS] == Fly_DONE){
										drone_in->desiredData.StartCommand = false;
										drone_in->desiredData.StopCommand = true;
									}
								}            
							} else//Stop khi mat tin hieu o che do Manual Pos
							if(drone_in->desiredData.flyMode == FLY_MODE_ONE_POINT){ 
								drone_in->desiredData.StartCommand = false;
								drone_in->desiredData.StopCommand = true;
								//drone_in->desiredData.manual_xd = 0;
								//drone_in->desiredData.manual_yd = 0;
							}        
						//drone_in->RFErrorCnt = MAX_RF_ERROR;        
						}
						
						
					}				
					//----------XU LY TRUONG HOP CHUA GOI DU LIEU NHUNG NHAN NUT START O CHE DO BAY DA DIEM-------------------------------
					if((drone_in->desiredData.flyMode == FLY_MODE_MULTI_POINT)&&(dataPoint[0][POINT_MEMORY_INDEX_LEN] < 2)){
						drone_in->desiredData.StartCommand = false;
						drone_in->desiredData.StopCommand = false;
						drone_in->desiredData.EmerCommand = true;
					}
				}
			}
		}
	}
	//--------CHUYEN QUA CHE DO EMER KHI BO LOI------------
	//--------------STOP SYSTEM FOR FAIL CASES---------------------
	if (fabs(drone_in->ebimu_data.roll) >= FAIL_ANGLE|| fabs(drone_in->ebimu_data.pitch) >= FAIL_ANGLE){
		drone_in->desiredData.EmerCommand = true;
	}
	
}

/*---------------------------------------------------------------------
 * Check move to next point: chi ap dung cho truong hop bay da diem
 * Cap nhat bien: drone_in->auto_current_point
 ----------------------------------------------------------------------*/
void ORC_Drone_Controller::move2NextPoint(long dataPoint[][POINT_MEMORY_MAX_LEN], orcdrone_data *drone_in)
{
  int cPoint = 0;
  float zTemp = 0, xTemp = 0, yTemp = 0;
  float zGPSTemp = 0, xGPSTemp = 0, yGPSTemp = 0;
  long len_data = 0;
//  Serial.print("Auto tune: ");Serial.println(drone_in->autoTuneState);
  if((drone_in->desiredData.flyMode == FLY_MODE_MULTI_POINT)&&(drone_in->desiredData.StartCommand)&& (drone_in->autoTuneState)){ //Che do auto da diem
    //Chi ap dung cho truong hop bay da diem, chuong trinh dang chay va khoi dong xong
    cPoint = drone_in->auto_current_point;        
    if((cPoint >= 0) && (cPoint < MaxPtoPnum)){ //Dung dieu kien moi lam viec
      len_data = dataPoint[cPoint][POINT_MEMORY_INDEX_LEN];
	//  Serial.print("len: "); Serial.println(len_data); 
      if((cPoint < len_data) && (dataPoint[cPoint][POINT_MEMORY_INDEX_LEN] > 1)&&(dataPoint[cPoint][POINT_MEMORY_INDEX_MISSION_STATUS] != Fly_DONE)){
//        Serial.println("Check moving point");
		
        zTemp = (float) dataPoint[cPoint][POINT_MEMORY_INDEX_ZD];
        xTemp = (float) dataPoint[cPoint][POINT_MEMORY_INDEX_XD];
        xTemp = xTemp/100; //Chuyen sang [m]
        yTemp = (float) dataPoint[cPoint][POINT_MEMORY_INDEX_YD];
        yTemp = yTemp/100; //Chuyen sang [m]
		xGPSTemp = ((float)drone_in->gpsdata.pos_cm[0])/100;
		yGPSTemp = ((float)drone_in->gpsdata.pos_cm[1])/100;
		zGPSTemp = ((float)drone_in->gpsdata.pos_cm[2]);
		// Serial.print("cp:"); Serial.print(cPoint); Serial.print(" ,"); Serial.print(xGPSTemp-xTemp);
		// Serial.print(" , "); Serial.print(yGPSTemp-yTemp); Serial.print(" ,"); Serial.println(zGPSTemp-zTemp);
		
        if((fabs(zGPSTemp-zTemp)<= Z_REACH)&& (fabs(xGPSTemp-xTemp)<= X_REACH)&& (fabs(yGPSTemp-yTemp)<= Y_REACH)){ //Diem da thoa dieu kien
          drone_in->reachCount = drone_in->reachCount + 1;
          //Serial.print("REACH:");Serial.println(drone_in->reachCount);
          if(drone_in->reachCount >= TIME_REACH) //Cho diem den du tieu chuan
          {
            drone_in->reachCount = 0; //Reset lai bien dem
            dataPoint[cPoint][POINT_MEMORY_INDEX_MISSION_STATUS] = Fly_DONE; //Cap nhat hoan thanh nhiem vu
            drone_in->auto_current_point = (cPoint >= (len_data-1))?(len_data - 1):(cPoint+1); //Tang len 1 diem            
          }
        }else //Reset bien dem
          drone_in->reachCount = 0;     
        
      }          
    }
  }
}

/*--------------------------------------------
 * QUY HOACH QUY DAO CHO PHUONG Z
---------------------------------------------*/
void ORC_Drone_Controller::z_TrajectoryPlanning(long dataPoint[][POINT_MEMORY_MAX_LEN], orcdrone_data *drone_in)
{
  int cPoint = 0;
  float zTemp = 0;

  if(drone_in->desiredData.EmerCommand == false){
	if(drone_in->desiredData.StopCommand)//Qui hoach qui dao thong tin z khi nhan nut Stop
	{
		if(drone_in->StartedCommand == true){  //Truong hop he thong da hoat dong
		drone_in->zd = Traj_Planning_const_Vel(-5000, drone_in->zd, 0.2); //Gia do cao he thong
	//      Serial.println(drone_in->z_d);
		}   
	}
	else if(drone_in->desiredData.StartCommand)//Qui hoach qui dao thong tin z khi nhan nut Start
	{
		drone_in->StartedCommand = true; //Chuong trinh da khoi tao
		if(drone_in->autoTuneState == false){ //Quy hoach quy dao z khi khoi dong
			//  Serial.println("Auto tunennnnnnn!");
			autoTunePminDrone(drone_in); //Tang nang luong nang
			drone_in->zd = 30;//20;
			//drone_in->z_kd = z_kd_doneTune*2;

		}
		else{
			if((drone_in->desiredData.flyMode == FLY_MODE_MANUAL_ANGLE)||(drone_in->desiredData.flyMode == FLY_MODE_MANUAL_POSITION)){ //Che do manual
				drone_in->zd = Traj_Planning_const_Vel(drone_in->desiredData.manual_zd, drone_in->zd, 1);//0.5); //Gia do cao he thong
			}else if (drone_in->desiredData.flyMode == FLY_MODE_MULTI_POINT){ //Che do auto da diem
				// Serial.println("In multiple Point");
				cPoint = drone_in->auto_current_point;        
				if((cPoint >= 0) && (cPoint < MaxPtoPnum)){ //Dung dieu kien moi lam viec
					// Serial.println("In multiple Point1");
					if(dataPoint[cPoint][POINT_MEMORY_INDEX_LEN] > 1){
						// Serial.println("In Z: In multiple Point2");
						zTemp = (float) dataPoint[cPoint][POINT_MEMORY_INDEX_ZD];
						drone_in->zd = Traj_Planning_const_Vel(zTemp, drone_in->zd, 1);//0.5); //Gia do cao he thong
					}          
				}        
			}else if (drone_in->desiredData.flyMode == FLY_MODE_ONE_POINT){ //Che do auto don diem
				zTemp = dataPoint[0][POINT_MEMORY_INDEX_ZD];
				drone_in->zd = Traj_Planning_const_Vel(zTemp, drone_in->zd, 1);//0.2); //Gia do cao he thong      
				}    
		}
	}
  }
}

/*--------------------------------------------
 * QUY HOACH QUY DAO CHO PHUONG x-y
 * Chi qui hoach quy dao khi he thong khoi dong xong
---------------------------------------------*/
void ORC_Drone_Controller::xy_TrajectoryPlanning(long dataPoint[][POINT_MEMORY_MAX_LEN], orcdrone_data *drone_in)
{
  int cPoint = 0;
  long lTemp = 0;
  float zTemp = 0, fTemp = 0, ftTemp = 0, fTempPre, fTempCur, fX, fY;
  float betades = 0.8, sqrtTemp = 0;
//   drone_in->autoTuneState = true; // nho cmt no lai
  if((drone_in->autoTuneState)&&(drone_in->desiredData.EmerCommand == false)){      //chi ap dung khi he thong khoi dong xong
    if (drone_in->desiredData.flyMode == FLY_MODE_MANUAL_ANGLE) //Che do Manual Pos
    {
      rpy_TrajectoryPlanning(drone_in);
      drone_in->manualPosX_Locked = false;  //Mo khoa lock Manual Pos X
      drone_in->manualPosY_Locked = false;  //Mo khoa lock Manual Pos Y
    }
    else if(drone_in->desiredData.flyMode == FLY_MODE_MANUAL_POSITION) //Che do Manual Pos
    {
      //-------------------------------------Quy hoach cho vi tri X
      //Serial.print("Dx: "); Serial.println(drone_in->desiredData.manual_xd);
	  if(fabs(drone_in->desiredData.manual_xd) >= 0.02 ) 
      {
        drone_in->manualPosX_Locked = false; //Unlock Pos
        fTemp = drone_in->xd;
        drone_in->xd = fTemp + drone_in->desiredData.manual_xd;        
      }else{
        if(drone_in->manualPosX_Locked == false){
          drone_in->xd = drone_in->x;
          fTemp = drone_in->x_ei2;
          drone_in->x_ei2 = 0.5*fTemp;
          drone_in->manualPosX_Locked = true;
        }        
      }
      //-------------------------------------Quy hoach cho vi tri Y
      if(fabs(drone_in->desiredData.manual_yd) >= 0.02 ) //Quy hoach cho vi tri X
      {
        drone_in->manualPosY_Locked = false; //Unlock Pos
        fTemp = drone_in->yd;
        drone_in->yd = fTemp + drone_in->desiredData.manual_yd;        
      }else{
        if(drone_in->manualPosY_Locked == false){
          drone_in->yd = drone_in->y;
          fTemp = drone_in->y_ei2;
          drone_in->y_ei2 = 0.5*fTemp;
          drone_in->manualPosY_Locked = true;
        }        
      }
      
    }else 
    {
      drone_in->manualPosX_Locked = false;  //Mo khoa lock Manual Pos X
      drone_in->manualPosY_Locked = false;  //Mo khoa lock Manual Pos Y
    
      if(drone_in->desiredData.flyMode == FLY_MODE_ONE_POINT) //Che do Auto Single Point    
      {
        fTemp = (float)dataPoint[0][POINT_MEMORY_INDEX_XD];
        drone_in->xd = fTemp/100;
        fTemp = (float)dataPoint[0][POINT_MEMORY_INDEX_YD];
        drone_in->yd = fTemp/100;
        drone_in->auto_current_point = 0;
      } else if(drone_in->desiredData.flyMode == FLY_MODE_MULTI_POINT) //Che do Auto Multiple Point
      {

		// Serial.println("In x-Y: In multiple Point ");
        cPoint = (int)drone_in->auto_current_point;
        if(cPoint <= 0){//Diem dau tien
		// Serial.println("In x-Y: In multiple Point 1");
          fTemp = (float)dataPoint[0][POINT_MEMORY_INDEX_XD];
          drone_in->xd = fTemp/100;
		//   Serial.print("In x-Y: In multiple Point: ");
		//   Serial.println(drone_in->xd);
          fTemp = (float)dataPoint[0][POINT_MEMORY_INDEX_YD];
          drone_in->yd = fTemp/100;
        }else if(cPoint < dataPoint[cPoint][POINT_MEMORY_INDEX_LEN]){//Cac diem khac
			// Serial.println("In x-Y: In multiple Point 2");
          ftTemp = dataPoint[cPoint][POINT_MEMORY_INDEX_TIME];
		  
          if(ftTemp <= 0){ //No Planning: Planning theo van toc default
			//Kiem tra khoang cach toi diem tiep theo
		  	sqrtTemp = (float) (((dataPoint[cPoint][POINT_MEMORY_INDEX_XD] - dataPoint[cPoint-1][POINT_MEMORY_INDEX_XD])*(dataPoint[cPoint][POINT_MEMORY_INDEX_XD] - dataPoint[cPoint-1][POINT_MEMORY_INDEX_XD])) + ((dataPoint[cPoint][POINT_MEMORY_INDEX_YD] - dataPoint[cPoint-1][POINT_MEMORY_INDEX_YD])*(dataPoint[cPoint][POINT_MEMORY_INDEX_YD] - dataPoint[cPoint-1][POINT_MEMORY_INDEX_YD]))); 
			//Serial.print("D:"); Serial.println(sqrtTemp);
			//Neu co khoang cach thi qui hoach, khong co thi khong can qui hoach
			if(sqrtTemp > 0){
				// Serial.println("In x-Y: In multiple Point 3");
				//Qui hoach quy dao phuong x cho van toc default
				fTempCur = (float)dataPoint[cPoint][POINT_MEMORY_INDEX_XD];
				fTempCur = fTempCur/100; //chuyen sang [m]
				
				fTemp = (float)(VXY_AUTO_DEFAULT*(dataPoint[cPoint][POINT_MEMORY_INDEX_XD] - dataPoint[cPoint-1][POINT_MEMORY_INDEX_XD])); //Scale in 10ms 
				fTemp = fTemp/(sqrt(sqrtTemp)*100); //Scale in 10ms
				//Serial.print("XC:"); Serial.print(sqrtTemp); Serial.print(", ");
				drone_in->xd = Traj_Planning_const_Vel((fTempCur), drone_in->xd, fTemp);
				//Serial.print("XD:"); Serial.print(sqrt(sqrtTemp)); Serial.println(". ");
				//Serial.print("XD:"); Serial.print(drone_in->xd); Serial.println(". ");
				
				//drone_in->xd = fTemp/100;
				//Qui hoach quy dao phuong y cho van toc default			
				// Serial.print(" X_ d_3: "); Serial.println(drone_in->xd);
				fTempCur = (float)dataPoint[cPoint][POINT_MEMORY_INDEX_YD];
				fTempCur = fTempCur/100; //chuyen sang [m]
				//fTemp = VXY_AUTO_DEFAULT/100; //Scale in 10ms 
				fTemp = (float)(VXY_AUTO_DEFAULT*(dataPoint[cPoint][POINT_MEMORY_INDEX_YD] - dataPoint[cPoint-1][POINT_MEMORY_INDEX_YD])); //Scale in 10ms 
				fTemp = fTemp/(sqrt(sqrtTemp)*100); //Scale in 10ms
				//Serial.print("Fy Temp:"); Serial.println(fTempCur);  Serial.print(", ");
				drone_in->yd = Traj_Planning_const_Vel((fTempCur), drone_in->yd, fTemp);
				// Serial.print("yd_Fy TEmp:"); Serial.println(drone_in->yd); Serial.println(". ");
				//drone_in->yd = fTemp/100;
			}else{
				//Qui hoach quy dao phuong x cho van toc default
				fTempCur = (float)dataPoint[cPoint][POINT_MEMORY_INDEX_XD];
				drone_in->xd = fTempCur/100; //chuyen sang [m]
				
				//Qui hoach quy dao phuong y cho van toc default			
				// Serial.print(" X_ d_3: "); Serial.println(drone_in->xd);
				fTempCur = (float)dataPoint[cPoint][POINT_MEMORY_INDEX_YD];
				drone_in->yd = fTempCur/100; //chuyen sang [m]				
			}
          }else{
            //----------------XU LY X---------------------------
            //B1: Tnh step size
            fTempCur = (float)dataPoint[cPoint][POINT_MEMORY_INDEX_XD];
			fTempCur = fTempCur/100; //chuyen sang [m]
            fTempPre = (float)dataPoint[cPoint-1][POINT_MEMORY_INDEX_XD];
			fTempPre = fTempPre/100; //chuyen sang [m]
            fTemp = fabs((fTempCur - fTempPre)/(ftTemp*100)); //Scale in 10ms  
			// Serial.println("In x-Y: In multiple Point 4");
            //B2: Cap nhat vi tri
            drone_in->xd = Traj_Planning_const_Vel((fTempCur), drone_in->xd, fTemp);
			// Serial.print(" X_ d_4: "); Serial.println(drone_in->xd);
            //----------------XU LY Y---------------------------
            //B1: Tnh step size
            fTempCur = (float)dataPoint[cPoint][POINT_MEMORY_INDEX_YD];
			fTempCur = fTempCur/100; //chuyen sang [m]
            fTempPre = (float)dataPoint[cPoint-1][POINT_MEMORY_INDEX_YD];
			fTempPre = fTempPre/100; //chuyen sang [m]			
            fTemp = fabs((fTempCur - fTempPre)/(ftTemp*100)); //Scale in 10ms  

            //B2: Cap nhat vi tri
            drone_in->yd = Traj_Planning_const_Vel((fTempCur), drone_in->yd, fTemp);
          }
        }
      }
    }    
  }else{
    drone_in->manualPosX_Locked = false;  //Mo khoa lock Manual Pos X
    drone_in->manualPosY_Locked = false;  //Mo khoa lock Manual Pos Y
  }
}

/*---------------------------------------------
 * DIEU KHIEN VI TRI CAP CAO: Chi ap dung cho mode Position
 * x,xd --> pitch_d
 * y, yd -> roll_d
----------------------------------------------*/
void ORC_Drone_Controller::xy_Controller(orcdrone_data *drone_in){
 
  if((drone_in->desiredData.flyMode == FLY_MODE_MANUAL_POSITION)||(drone_in->desiredData.flyMode == FLY_MODE_ONE_POINT)||(drone_in->desiredData.flyMode == FLY_MODE_MULTI_POINT)){
	exTemp = (drone_in->xd + drone_in->obstacleAvoidanceVar.ov_xd) - drone_in->x; //Them tin hieu tranh vat can
    eyTemp = (drone_in->yd + drone_in->obstacleAvoidanceVar.ov_yd) - drone_in->y; //Them tin hieu tranh vat can
	
   // float exi = ex, eyi  = ey;
   // float ex_i_pre = *x_i, ey_i_pre = *y_i;
	exi1Temp = drone_in->x_ei1; eyi1Temp = drone_in->y_ei1;
	exi2Temp = drone_in->x_ei2; eyi2Temp = drone_in->y_ei2;
  //  Serial.print(ex); Serial.print(' ');
  //  Serial.print(ey); Serial.print(' ');
   // float k_cc;
    if(fabs(exTemp)>exy_k || fabs(eyTemp) > exy_k){
      if(fabs(exTemp)> fabs(eyTemp) ){
        k_ccTemp = exy_k/fabs(exTemp);
      }
      else{
        k_ccTemp = exy_k/fabs(eyTemp);
      }
      exTemp = exTemp*k_ccTemp;
      eyTemp = eyTemp*k_ccTemp;
    }
  
    exTemp = constrain(exTemp,-exy_max,exy_max);
    eyTemp = constrain(eyTemp,-exy_max,exy_max);
    
    
   // ====================== x PID =========================
    //*x_i = (*x_i + (T * ex)); 
    exi2Temp = 0.99*exi2Temp + 0.01*(drone_in->x_ki*(exTemp + 3*((exTemp>0)?1:((exTemp<0)?(-1):0)))); 
    exi2Temp = constrain(exi2Temp,-eixy_max,eixy_max);
    drone_in->x_ei2 = exi2Temp;
    
    ux = drone_in->x_kp*exTemp + exi2Temp - drone_in->x_kd*drone_in->vx;// - 0.001*CalAccX_filter;
    
	//Serial.print("ux: "); Serial.println(ux);
    // ====================== y PID =========================
  //  *y_i = (*y_i + (T * ey)); 
  //  *y_i = (*y_i + (T * eyi));
  //  *y_i = constrain(*y_i,-2000,2000);
    eyi2Temp = 0.99*eyi2Temp + 0.01*(drone_in->y_ki*(eyTemp + 3*((eyTemp>0)?1:((eyTemp<0)?(-1):0)))); 
    eyi2Temp = constrain(eyi2Temp,-eixy_max,eixy_max);
    drone_in->y_ei2 = eyi2Temp;
    
    uy = drone_in->y_kp*eyTemp + eyi2Temp - drone_in->y_kd*drone_in->vy;// - 0.001*CalAccX_filter;
  
    
    //Serial.print("uy: "); Serial.println(uy);
    // ====================== Convert =======================
    yawTemp = drone_in->ebimu_data.yaw*PI/180;
  //  float cm1 = (ux*sin(yaw_rad) - uy*cos(yaw_rad))/15;
  //  float cm2 = (ux*cos(yaw_rad) + uy*sin(yaw_rad))/15;
//    Serial.print(" ux: "); Serial.print( ux); Serial.print(" uy: "); Serial.println( uy);
    cm1Temp = (ux*sin(yawTemp) - uy*cos(yawTemp))/15;
    cm2Temp = (ux*cos(yawTemp) + uy*sin(yawTemp))/15;
//    Serial.print(" cm1Temp: "); Serial.print( cm1Temp); Serial.print(" cm2Temp: "); Serial.println( cm2Temp);
    cm1Temp = constrain(cm1Temp,-1,1);
    cm2Temp = constrain(cm2Temp,-1,1);
  //  roll_des = asin(cm1)*180/PI;
  //  pitch_des = atan(cm2)*180/PI;
    cm1Temp = (drone_in->roll_d)*0.95 + 0.05*asin(cm1Temp)*180/PI;
    cm2Temp = (drone_in->pitch_d)*0.95+ 0.05*atan(cm2Temp)*180/PI;
    
    drone_in->roll_d = constrain(cm1Temp,-DESIRED_ANGLE_AUTO_MAX,DESIRED_ANGLE_AUTO_MAX);
    drone_in->pitch_d = constrain(cm2Temp,-DESIRED_ANGLE_AUTO_MAX,DESIRED_ANGLE_AUTO_MAX); 
  }
}
/*---------------------------------------------
 * DIEU KHIEN VI TRI CAP CAO: chi dieu khien khi he thong khoi dong xong
 * z,zd --> u1
----------------------------------------------*/
void ORC_Drone_Controller::z_Controller(orcdrone_data *drone_in){
	float uz_temp = 0;
  if(drone_in->desiredData.EmerCommand){  //Khong dieu khien khi o che do Emer
    drone_in->u_z = 0;
  }else
    if((drone_in->autoTuneState)&& (drone_in->FlyingEnable >= FlyingEnableThres)){
      ezTemp = drone_in->zd - drone_in->z;
      ezTemp = constrain(ezTemp,-200,200);
      ezi1Temp = drone_in->z_ei1 + (drone_in->Ts)*ezTemp; //Cap nhat tich phan sai so 1
      //ezi1Temp  = constrain(ezi1Temp,-10000,10000);
	  
      drone_in->z_ei1 = ezi1Temp;
      ezi2Temp = 0.995*(drone_in->z_ei2) + drone_in->Ts*drone_in->z_ki*ezi1Temp; //Cap nhat tich phan sai so 2 
	  ezi2Temp  = constrain(ezi2Temp,-5000,5000);
      drone_in->z_ei2 = ezi2Temp;
	  
      uz_temp = drone_in->z_kp*ezTemp + ezi2Temp - drone_in->z_kd*drone_in->vz;
      if(!isnan(uz_temp))
        drone_in->u_z = uz_temp;    
    }else
		drone_in->u_z = 0;
}
/*---------------------------------------------
 * DIEU KHIEN VI TRI CAP thap: 
----------------------------------------------*/
void ORC_Drone_Controller::rpy_Controller(orcdrone_data *drone_in){
	float desired_angle = 0;

	if(drone_in->desiredData.EmerCommand){  //Khong dieu khien khi o che do Emer
		drone_in->u_roll = 0;drone_in->u_pitch = 0;drone_in->u_yaw = 0;
	}else{
		//--------------DIEU KHIEN GOC ROLL-----------------------------------
		//desired_angle = drone_in->roll_d + ((drone_in->zd > 150)?drone_in->obstacleAvoidanceVar.ov_roll_d:0);
		//desired_angle = constrain(desired_angle, -DESIRED_ANGLE_MANUAL_MAX, DESIRED_ANGLE_MANUAL_MAX);
		//eATemp = desired_angle - (drone_in->ebimu_data.roll); //Tinh sai so
		 eATemp = drone_in->roll_d - (drone_in->ebimu_data.roll); //Tinh sai so
		eAiTemp = 0.99*(drone_in->roll_ei) + drone_in->Ts*drone_in->roll_ki*(eATemp + 10*sgn(eATemp)); //Tinh tich phan
		drone_in->roll_ei = eAiTemp;   //Cap nhat tich phan
		uATemp = drone_in->roll_kp*eATemp + eAiTemp - drone_in->roll_kd*(drone_in->mpu_data.gx);
		if(!isnan(uATemp)){
			uATemp = constrain(uATemp, -300, 300);
			drone_in->u_roll = uATemp; //Cap nhat tin hieu dieu khien khi hop le
		}
		//--------------DIEU KHIEN GOC PITCH-----------------------------------
		//desired_angle = drone_in->pitch_d + ((drone_in->zd > 150)?drone_in->obstacleAvoidanceVar.ov_pitch_d:0);
		//desired_angle = constrain(desired_angle, -DESIRED_ANGLE_MANUAL_MAX, DESIRED_ANGLE_MANUAL_MAX);
		eATemp = drone_in->pitch_d - (drone_in->ebimu_data.pitch); //Tinh sai so
		//eATemp = desired_angle - (drone_in->ebimu_data.pitch); //Tinh sai so
		eAiTemp = 0.99*(drone_in->pitch_ei) + drone_in->Ts*drone_in->pitch_ki*(eATemp + 10*sgn(eATemp)); //Tinh tich phan
		drone_in->pitch_ei = eAiTemp;   //Cap nhat tich phan
		uATemp = drone_in->pitch_kp*eATemp + eAiTemp - drone_in->pitch_kd*(drone_in->mpu_data.gy);
		if(!isnan(uATemp)){
			uATemp = constrain(uATemp, -300, 300);
			drone_in->u_pitch = uATemp; //Cap nhat tin hieu dieu khien khi hop le
		}

		//--------------DIEU KHIEN GOC YAW-----------------------------------
		eATemp = drone_in->yaw_d - (drone_in->ebimu_data.yaw); //Tinh sai so
		//Calib goc ya
		if(eATemp > 180)
			eATemp = eATemp - 360;
		else if(eATemp < -180)
			eATemp = eATemp + 360;  
		eATemp = constrain(eATemp,-70,70); //Gioi han lai sai so goc yaw


		eAiTemp = 0.99*(drone_in->yaw_ei) + drone_in->Ts*drone_in->yaw_ki*(eATemp + 6*sgn(eATemp)); //Tinh tich phan
		drone_in->yaw_ei = eAiTemp;   //Cap nhat tich phan
		uATemp = drone_in->yaw_kp*eATemp + eAiTemp - drone_in->yaw_kd*(drone_in->mpu_data.gz);
	
		if(!isnan(uATemp)){
			uATemp = constrain(uATemp, -400, 400);
			drone_in->u_yaw = uATemp; //Cap nhat tin hieu dieu khien khi hop le
		}
	}  
}
/*----------------------------------------
* QUI HOACH QUY DAO VAN TOC HANG SO
-------------------------------------------*/
float ORC_Drone_Controller::Traj_Planning_const_Vel(float zd, float z, float step_size)
{
  float z_out = 0;  
  if((fabs(zd - z)) < fabs(step_size))
    return zd;
  else{
    z_out = z +  sgn(zd-z)*(fabs(step_size));
    return z_out;
  }
}

/*-----------------------------------
 * TU DONG TANG NANG LUONG CO BAN KHI DRONE O TREN MAT DAT
------------------------------------*/
void ORC_Drone_Controller::autoTunePminDrone(orcdrone_data *drone_in)
{
  if((drone_in->autoTuneState == false) && (drone_in->FlyingEnable >= FlyingEnableThres)){//Auto tuning Pm in process
    if(drone_in->z < 10){ // Chua cat canh
      if((drone_in->P_basic) >= Pmax){ //Gioi han max
        drone_in->autoTuneState = true;
        drone_in->P_basic = Pmax;
      }else{
        if(drone_in->z < 3) //Tang nhanh
          drone_in->P_basic = drone_in->P_basic + AUTOTUNE_POWER_STEP; //0.5:0k gps khung
        else        //Tang cham khi drone nhuc nhic
          drone_in->P_basic = drone_in->P_basic + 0.5*AUTOTUNE_POWER_STEP;
      }
    }else
    {
      drone_in->autoTuneState = true;
	  drone_in->P_basic = drone_in->P_basic - 40*AUTOTUNE_POWER_STEP;//D2: 10//20;50;
    }
  }  
}
//--------------- Flight Mode --------------------------------
// output: roll, pitch, yaw and z desired
//------------------------------------------------------------
void ORC_Drone_Controller::rp_TrajectoryPlanning(orcdrone_data *drone_in){
  if(drone_in->autoTuneState){
    if(drone_in->desiredData.flyMode == FLY_MODE_MANUAL_ANGLE){
      // Xu ly deadzone
      if(fabs(drone_in->desiredData.manual_rolld) < 0.1) drone_in->desiredData.manual_rolld = 0;
      if(fabs(drone_in->desiredData.manual_pitchd) < 0.1) drone_in->desiredData.manual_pitchd = 0;
      //Loc thong thap
      drone_in->roll_d = 0.9*drone_in->roll_d + 0.1*drone_in->desiredData.manual_rolld;
      drone_in->pitch_d = 0.9*drone_in->pitch_d + 0.1*drone_in->desiredData.manual_pitchd;
    }    
  }
}
//--------------- Flight Mode --------------------------------
// output: roll, pitch, yaw and z desired
//------------------------------------------------------------
void ORC_Drone_Controller::rpy_TrajectoryPlanning(orcdrone_data *drone_in){
  if(drone_in->autoTuneState){
    if(drone_in->desiredData.flyMode == FLY_MODE_MANUAL_ANGLE){
      // Xu ly deadzone
      if(fabs(drone_in->desiredData.manual_rolld) < 0.1) drone_in->desiredData.manual_rolld = 0;
      if(fabs(drone_in->desiredData.manual_pitchd) < 0.1) drone_in->desiredData.manual_pitchd = 0;
	  if(fabs(drone_in->desiredData.manual_yawd) < 3) drone_in->desiredData.manual_yawd = 0;
      //Loc thong thap
      drone_in->roll_d = 0.8*drone_in->roll_d + 0.2*drone_in->desiredData.manual_rolld;
      drone_in->pitch_d = 0.8*drone_in->pitch_d + 0.2*drone_in->desiredData.manual_pitchd;
	  drone_in->yaw_d = 0.95*drone_in->yaw_d + 0.05*drone_in->desiredData.manual_yawd;
    }    
  }
}
/*--------------------------------------------------
 * MOTOR SPEED COMPUTATION
---------------------------------------------------*/
void ORC_Drone_Controller::motorSpeed_Caculator(orcdrone_data *drone_in, float ScaleEng){

//(float uz_in, float rol_pid, float pit_pid, float ya_pid, float ro, float pit, unsigned int *O1_u, unsigned int *O2_u, unsigned int *O3_u, unsigned int *O4_u, float ScaleEng){
  float ms = cos((drone_in->ebimu_data.roll)*PI/180)*cos((drone_in->ebimu_data.pitch)*PI/180);
  if(drone_in->desiredData.EmerCommand){ //Truong hop nguy hiem ->tat he thong cho an toan
    drone_in->Motor1_Speed = 0;
    drone_in->Motor3_Speed = 0;
    
    drone_in->Motor2_Speed = 0;
    drone_in->Motor4_Speed = 0;  
    
  }else if(ms == 0)
  {
    //Serial.println("set 0");
    drone_in->Motor1_Speed = 0; drone_in->Motor2_Speed = 0; drone_in->Motor3_Speed = 0; drone_in->Motor4_Speed = 0;
  }
  else if((fabs(drone_in->ebimu_data.roll) > FAIL_ANGLE)||(fabs(drone_in->ebimu_data.pitch) > FAIL_ANGLE)){ //Truong hop nguy hiem ->tat he thong cho an toan
    drone_in->Motor1_Speed = 0;
    drone_in->Motor3_Speed = 0;
    
    drone_in->Motor2_Speed = 0;
    drone_in->Motor4_Speed = 0;  
    
  }else
  {
//  float u1, u2, u3, u4;
    u1Temp = (drone_in->P_basic + (drone_in->u_z/10))/ms;
    u2Temp = drone_in->u_roll;
    u3Temp = drone_in->u_pitch;
    u4Temp = drone_in->u_yaw;
//    Serial.print(" u2:");Serial.print(u2);
//    Serial.print(" u3:");Serial.print(u3);
//    Serial.print(" u4:");Serial.print(u4);
    m1Temp = u1Temp - u3Temp - u4Temp;
    m2Temp = u1Temp - u2Temp + u4Temp;
    m3Temp = u1Temp + u3Temp - u4Temp;
    m4Temp = u1Temp + u2Temp + u4Temp;

    if (m1Temp<100) m1Temp =100;
    if (m2Temp<100) m2Temp =100;
    if (m3Temp<100) m3Temp =100;
    if (m4Temp<100) m4Temp =100;

//    unsigned int O1_i = m1* ScaleEng;
//    unsigned int O3_i = m3* ScaleEng;
//    unsigned int O2_i = m2* ScaleEng;
//    unsigned int O4_i = m4* ScaleEng;

    // Serial.println(String(O1_i) + " " + String(O2_i) + " " + String(O3_i) + " " + String(O4_i));

    
    drone_in->Motor1_Speed = constrain((unsigned int)(m1Temp* ScaleEng), 100, 1800);
    drone_in->Motor3_Speed = constrain((unsigned int)(m3Temp* ScaleEng), 100, 1800);
    
    drone_in->Motor2_Speed = constrain((unsigned int)(m2Temp* ScaleEng), 100, 1800);
    drone_in->Motor4_Speed = constrain((unsigned int)(m4Temp* ScaleEng), 100, 1800);   
  }
}

/*------------------------------------------
 * SIGN function
--------------------------------------------*/
float ORC_Drone_Controller::sgn(float x){ 
  return ((x < 0)? (-1):((x > 0)?1:x));
}
/*--------------------------------------------
 * Cai dat cac thong so dieu khien cho he thong
---------------------------------------------*/
void ORC_Drone_Controller::setControlGains(orcdrone_data *drone_in)
{
	//Cac he so dieu khien goc roll
	// drone_in->roll_kp = 12; 	drone_in->roll_ki = 0.5; 	drone_in->roll_kd = 6; 	drone_in->roll_ei = 0;
	drone_in->roll_kp = 15; 	drone_in->roll_ki = 0.5; 	drone_in->roll_kd = 6; 	drone_in->roll_ei = 0;
	
	//Cac he so dieu khien goc pitch
	// drone_in->pitch_kp = 12; 	drone_in->pitch_ki = 0.5; 	drone_in->pitch_kd = 6; drone_in->pitch_ei = 0;
	drone_in->pitch_kp = 15; 	drone_in->pitch_ki = 0.5; 	drone_in->pitch_kd = 6; drone_in->pitch_ei = 0;
	//Cac he so dieu khien goc yaw
	drone_in->yaw_kp = 15; 		drone_in->yaw_ki = 0.5; 		drone_in->yaw_kd = 7; 	drone_in->yaw_ei = 0;
	
	//Cac he so dieu khien bien x
	drone_in->x_kp = 0.45; 	drone_in->x_ki = 0.05; 	drone_in->x_kd = 0.84; 	drone_in->x_ei1 = 0;drone_in->x_ei2 = 0;
	//Cac he so dieu khien bien y
	drone_in->y_kp = 0.45; 	drone_in->y_ki = 0.05; 	drone_in->y_kd = 0.84; 	drone_in->y_ei1 = 0; drone_in->y_ei2 = 0;
	//Cac he so dieu khien bien z
	drone_in->z_kp = 18; 	drone_in->z_ki = 1; 	drone_in->z_kd = 8; 	drone_in->z_ei1 = 0; drone_in->z_ei2 = 0;  
}

// ===================== Estimate ==============================================================================
/*----------------------------------------------------------------------------------------
* Ham loc vi tri tu gia toc va vi tri dua vao
-----------------------------------------------------------------------------------------*/
float ORC_Drone_Controller::Kalman_1( float *xk, float ak, float zm, float dt1, int full_act, float k1, float k2)
{
  // F = [1 0; dt 1] G = [dt 0]; H = [0 1];
  float xk_1 = 0, xk_2 = 0, temp1  = 0;

  //Read xk
  xk_1 = *xk; temp1 = xk_1;
  xk++; xk_2 = *xk;

  //Time update
  xk_1 = xk_1 + dt1 * ak - full_act * dt1 * k1 * k2 * (xk_2 - zm); //cap nhat van toc: 0.4=0.01*2*20; Kv = Ts*K1*k2 
  xk_2 = xk_2 + temp1 * dt1 - full_act * dt1 * (k1 + k2) * (xk_2 - zm); //cap nhat vi tri: 0.22=0.01*(2+20); KP = Ts*(k1+k2)
  /*
    if(full_act){
      xk_1 = xk_1 + 0.5*(zm - xk_2);
      xk_2 = xk_2 + 0.5*(zm - xk_2);
    }*/
  // Reupdate xk
  *xk = xk_2; xk--; *xk = xk_1;
  return xk_2;
}
/*--------------------------------------------
 * Xử lý du lieu phuong z
---------------------------------------------*/
void ORC_Drone_Controller::MPUDataProcessing(orcdrone_data *drone_in)
{
	//Setup safety	
	float Rx = 0, Ry = 0, Rz = 0;
	Rotation((drone_in->mpu_data.accx), (drone_in->mpu_data.accy), (drone_in->mpu_data.accz), (drone_in->ebimu_data.roll), (drone_in->ebimu_data.pitch), (drone_in->ebimu_data.yaw), &Rx, &Ry, &Rz);	  
	drone_in->awx = (Rx - (drone_in->awex))*9.81; //m/s2
	drone_in->awy = (Ry - (drone_in->awey))*9.81; //m/s2
	drone_in->awz = (Rz - (drone_in->awez))*100*9.81; //cm/s2
	
	if (fabs(drone_in->awx) <= 0.20)  drone_in->awx = 0;
	if (fabs(drone_in->awy) <= 0.20)  drone_in->awy = 0;
	if (fabs(drone_in->awz) <= 20)  	drone_in->awz = 0;
}
/*--------------------------------------------
 * Xử lý du lieu phuong z
---------------------------------------------*/
void ORC_Drone_Controller::AltitudeDataProcessing(orcdrone_data *drone_in)
{
	//Setup safety
	if ((drone_in->gpsMode == GPS_ENABLE)&& (drone_in->gpsdata.isError == false)) //(drone_in->gpsdata.pos_cm[3] > GPS_RTK_NOSOLUTION)&& (!(Drone_WarningFault.isWFRequiredError(drone_in->WarningFault, WF_Error_GPS_Timeout))))
		//drone_in->z = Kalman_1(z_kalman_1,  drone_in->awz, (float) (drone_in->gpsdata.pos_cm[2]), drone_in->Ts, 1, 2, 20);
		drone_in->z = Kalman_1(z_kalman_1,  drone_in->awz, (float) (drone_in->gpsdata.pos_cm[2]), drone_in->Ts, 1, 1, 20);
	else
		drone_in->z = Kalman_1( z_kalman_1,  drone_in->awz, (float)((drone_in->zLidar1D) - (drone_in->zLidar1DErr)), drone_in->Ts, 1, 2, 20);	  
	//Cap nhat van toc
	drone_in->vz = z_kalman_1[0];
}
/*--------------------------------------------
 * Xử lý du lieu phuong x,y
---------------------------------------------*/
void ORC_Drone_Controller::PositionDataProcessing(orcdrone_data *drone_in)
{
	//Xu ly phuong x in [m]
	drone_in->x = Kalman_1(x_kalman_1,  drone_in->awx, ((float) (drone_in->gpsdata.pos_cm[0]))/100, drone_in->Ts, 1, 2, 20);
	drone_in->vx = x_kalman_1[0];
	//Xu ly phuong y in [m]
	drone_in->y = Kalman_1(y_kalman_1,  drone_in->awy, ((float) (drone_in->gpsdata.pos_cm[1]))/100, drone_in->Ts, 1, 2, 20);
	drone_in->vy = y_kalman_1[0];
}
/*--------------------------------------------
 * Calib cam bien EBI và MPU:
 * B1: doc du lieu tu EBI
 * B2: Doc du lieu tu MPU
 * B3: Tinh goc lech giua roll, pitch giua 2 cam bien => mpu_roll_off, mpu_pitch_off
---------------------------------------------*/
bool ORC_Drone_Controller::EBIMPUSelfCalibration(orcdrone_data *drone_in)
{
	float mpuRollTemp, mpuPitchTemp;
	bool result = false;
	int Cnt_max = 10, cnt = Cnt_max;
	//while((!result )&& (cnt <= 0)){
	drone_in->mpu_data.mroll_offset = 0;
	drone_in->mpu_data.mpitch_offset = 0;
	while((!result )&& (cnt <= 0)){
		//B1: doc du lieu tu EBI
		DroneEBI_data->EBIReadData(&(drone_in->ebimu_data));
		//B2: Doc du lieu tu MPU
		DroneMPU.MPU6050ReadData(drone_in->Ts, &(drone_in->mpu_data));
		//B3: Tinh goc lech giua roll, pitch giua 2 cam bien => mpu_roll_off, mpu_pitch_off
		mpuRollTemp = (atan(drone_in->mpu_data.accy/(sqrt(pow(drone_in->mpu_data.accx,2) + pow(drone_in->mpu_data.accz,2))))*180/PI);
		mpuPitchTemp = (atan(-(drone_in->mpu_data.accx)/(sqrt(pow(drone_in->mpu_data.accy,2) + pow(drone_in->mpu_data.accz,2))))*180/PI);
		if(!isnan(mpuRollTemp) && !isnan(mpuPitchTemp)){
			drone_in->mpu_data.mroll_offset = drone_in->mpu_data.mroll_offset + (drone_in->ebimu_data.roll - mpuRollTemp)/Cnt_max;
			drone_in->mpu_data.mpitch_offset = drone_in->mpu_data.mpitch_offset + (drone_in->ebimu_data.pitch - mpuPitchTemp)/Cnt_max;
			drone_in->ebimu_data.pre_roll = drone_in->ebimu_data.roll;
			drone_in->ebimu_data.pre_pitch = drone_in->ebimu_data.pitch;
			cnt--;
			result = true;
		}else
			delay(30);
	}
	return result;
}
/*--------------------------------------------
 * Tu dong correct du lieu nhan duoc tu EBIMU
---------------------------------------------*/
byte ORC_Drone_Controller::EBIAutoCorrection(orcdrone_data *drone_in){
	float dr   = (drone_in->ebimu_data.roll - drone_in->ebimu_data.pre_roll)/(drone_in->Ts+drone_in->ebimu_data.roll_e_time);
  float dp   = (drone_in->ebimu_data.pitch - drone_in->ebimu_data.pre_pitch)/(drone_in->Ts+drone_in->ebimu_data.pitch_e_time);
  float dyaw =  (drone_in->ebimu_data.yaw - drone_in->ebimu_data.pre_yaw);
  byte result = 0;

//  Serial.print(" ip:"); Serial.print(eul[2]);

  if(fabs(dr-drone_in->mpu_data.gx)>100){               // neu co loi doc du lieu cam bien ebimu
    drone_in->ebimu_data.roll_out = drone_in->ebimu_data.roll_out + drone_in->mpu_data.gx*drone_in->Ts;  // lay gia tri tu van toc goc bu vao
    drone_in->ebimu_data.roll_e_time += drone_in->Ts;               // cong don thoi gian
	result = 1;
  } else{ // du lieu binh thuong
    drone_in->ebimu_data.roll_e_time = 0;
    drone_in->ebimu_data.roll_out = drone_in->ebimu_data.roll;
    drone_in->ebimu_data.pre_roll = drone_in->ebimu_data.roll;
  }
  
  if(fabs(dp-drone_in->mpu_data.gy)>100){               // neu co loi doc du lieu cam bien ebimu
    drone_in->ebimu_data.pitch_out = drone_in->ebimu_data.pitch_out + drone_in->mpu_data.gy*drone_in->Ts;  // lay gia tri tu van toc goc bu vao
    drone_in->ebimu_data.pitch_e_time += drone_in->Ts;               // cong don thoi gian
	result = 2;
  } else{ // du lieu binh thuong
    drone_in->ebimu_data.pitch_e_time = 0;
    drone_in->ebimu_data.pitch_out = drone_in->ebimu_data.pitch;
    drone_in->ebimu_data.pre_pitch = drone_in->ebimu_data.pitch;
  }

  if(fabs(dyaw)>5){               // neu co loi doc du lieu cam bien ebimu
    drone_in->ebimu_data.yaw_out  = drone_in->ebimu_data.pre_yaw + ((dyaw > 5)?5:-5);
    drone_in->ebimu_data.pre_yaw = drone_in->ebimu_data.yaw_out ;
	result = 3;
  } else{ // du lieu binh thuon
    drone_in->ebimu_data.yaw_out  = drone_in->ebimu_data.yaw;
    drone_in->ebimu_data.pre_yaw = drone_in->ebimu_data.yaw;
  }

//  Serial.print(" et:"); Serial.print(et*1000);

//  Serial.print(" er:"); Serial.print(gx*et);
//  Serial.print(" ep:"); Serial.print(gy*et);
//  Serial.print(" ey:"); Serial.print(gz*et);
  
//  Serial.print(" er:"); Serial.print((dr-gx)/10);
//  Serial.print(" ep:"); Serial.print((dp-gy)/10);
//  Serial.print(" ey:"); Serial.print((dyaw-gz)/10);

//  Serial.print(" r:"); Serial.print(eulOUT[0]-4);
//  Serial.print(" p:"); Serial.print(eulOUT[1]-4);
//  Serial.print(" y:"); Serial.print(eulOUT[2]-4);

//  Serial.print(" lr:"); Serial.print(laeul[0]-2);
//  Serial.print(" lp:"); Serial.print(laeul[1]-2);
//  Serial.print(" ly:"); Serial.print(laeul[2]-2);
  
  return result;
}
/*--------------------------------------------
 * Ham tu dong bu offset cho EBIMU tu du lieu MPU
---------------------------------------------*/
byte ORC_Drone_Controller::EBIAutoOffsetCompensation(orcdrone_data *drone_in)
{
	float keu = 0.995;
  // BIEN TAM CHO GOC ROLL
  //float r_6050_filter_temp = *r_6050_filter;
  //float r_eb_filter_temp = *r_eb_filter;  
  float eroll = 0; 
  //float r_off_temp = *r_off2;
  //float r_off1_temp = *r_off1;
  
  // BIEN TAM CHO GOC PTICH
  //float p_6050_filter_temp = *p_6050_filter;
  //float p_eb_filter_temp = *p_eb_filter; 
  float epitch = 0; 
  //float p_off_temp = *p_off2;
  //float p_off1_temp = *p_off1;
  byte result = 0;
  // AP DUNG CHINH OFFSET CHO GOC ROLL
//   r_6050_filter_temp = 0.995*r_6050_filter_temp + 0.005*r_6050;
  drone_in->mpu_data.mroll_filter = 0.995*drone_in->mpu_data.mroll_filter + 0.005*drone_in->mpu_data.mroll;
  //r_eb_filter_temp = 0.995*r_eb_filter_temp + 0.005*r_eb;
  drone_in->ebimu_data.roll_filter = 0.995*drone_in->ebimu_data.roll_filter + 0.005*drone_in->ebimu_data.roll;
  
  eroll = drone_in->ebimu_data.roll_filter - drone_in->mpu_data.mroll_filter; 
  
//  *r_c = r_eb + r_off_temp;
  if(fabs(eroll) > 5){
	result = 1;
      if(fabs(drone_in->mpu_data.mroll_filter) < 30){ 
		drone_in->ebimu_data.roll_offset2 = 0.995*drone_in->ebimu_data.roll_offset2 - 0.005*eroll;
	  }else if(fabs(eroll) <3)  drone_in->ebimu_data.roll_offset2 = keu*drone_in->ebimu_data.roll_offset2;
  }
  else if(fabs(eroll) <3) drone_in->ebimu_data.roll_offset2 = keu*drone_in->ebimu_data.roll_offset2;
  drone_in->ebimu_data.roll_offset1 = 0.99*drone_in->ebimu_data.roll_offset1 + 0.01*drone_in->ebimu_data.roll_offset2;
  
  //Return values
  //*r_6050_filter = r_6050_filter_temp;
  //*r_eb_filter = r_eb_filter_temp;
  //*r_c = r_eb + r_off1_temp;
  drone_in->ebimu_data.roll = drone_in->ebimu_data.roll_out + drone_in->ebimu_data.roll_offset1;
  //*r_off2 = r_off_temp;
  //*r_off1 = r_off1_temp;

//-------AP DUNG CHO GOC PITCH--------------------
  //p_6050_filter_temp = 0.995*p_6050_filter_temp + 0.005*p_6050;
  drone_in->mpu_data.mpitch_filter = 0.995*drone_in->mpu_data.mpitch_filter + 0.005*drone_in->mpu_data.mpitch;
  //p_eb_filter_temp = 0.995*p_eb_filter_temp + 0.005*p_eb;
  drone_in->ebimu_data.pitch_filter = 0.995*drone_in->ebimu_data.pitch_filter + 0.005*drone_in->ebimu_data.pitch;

  //epitch = p_eb_filter_temp - p_6050_filter_temp; 
  epitch = drone_in->ebimu_data.pitch_filter - drone_in->mpu_data.mpitch_filter; 

  if(fabs(epitch) > 5){
	result = 2;
    if(fabs(drone_in->mpu_data.mpitch_filter) < 30)
	// p_off_temp = 0.995*p_off_temp - 0.005*(p_eb_filter_temp - p_6050_filter_temp);
		drone_in->ebimu_data.pitch_offset2 = 0.995*drone_in->ebimu_data.pitch_offset2 - 0.005*epitch;
    else if(fabs(epitch) <3)  drone_in->ebimu_data.pitch_offset2 = keu*drone_in->ebimu_data.pitch_offset2;
  }
  else if(fabs(epitch) <3) drone_in->ebimu_data.pitch_offset2 = keu*drone_in->ebimu_data.pitch_offset2;
  //p_off1_temp = 0.99*p_off1_temp + 0.01*p_off_temp;
  drone_in->ebimu_data.pitch_offset1 = 0.99*drone_in->ebimu_data.pitch_offset1 + 0.01*drone_in->ebimu_data.pitch_offset2;


  //Return values
  //*p_6050_filter = p_6050_filter_temp;
  //*p_eb_filter = p_eb_filter_temp;
  //*p_c = p_eb + p_off1_temp;
  drone_in->ebimu_data.pitch = drone_in->ebimu_data.pitch_out + drone_in->ebimu_data.pitch_offset1;
  //*p_off2 = p_off_temp;
  //*p_off1 = p_off1_temp;
//
//  float epitch = p_eb - p_6050; 
//  float p_off_temp = *p_off;
//  *p_c = p_eb + p_off_temp;
//  if(fabs(epitch) > 8){
//      if(fabs(p_6050) < 30) *p_off = p_off_temp - 0.01*(*p_c - p_6050);
//      else if(fabs(epitch) <3) *p_off = keu*p_off_temp;
//  }
//  else if(fabs(epitch) <3) *p_off = keu*p_off_temp;
//  *r_c = r_eb;
  /* *p_c = p_eb; */
  return true; 
}

/*--------------------------------------------
 * Doc va tu dong recal cam bien EBIMU
 * B1: doc du lieu tu EBI
 * B2: Kiem tra du lieu nhan ve co bi loi hay khong
 * B3: Tu dong bu offset cho roll pitch
---------------------------------------------*/
bool ORC_Drone_Controller::EBIReadandRecalib(orcdrone_data *drone_in)
{
	bool result = false;
	//byte result_byte = 0;
	
	//B1: doc du lieu tu EBI
	result = DroneEBI_data->EBIReadData(&(drone_in->ebimu_data));
	//* B2: Kiem tra du lieu nhan ve co bi loi hay khong
	byte result_byte = EBIAutoCorrection(drone_in);

	//B3: Tu dong bu offset cho roll pitch
	EBIAutoOffsetCompensation(drone_in);
	return result;
}
// ============================= Rotation ======================================================================
void ORC_Drone_Controller::Rotation(float ax, float ay, float az, float roll_i, float pitch_i, float yaw_i, float *Rx_i, float *Ry_i, float *Rz_i) {
  float r_i = roll_i * PI / 180;
  float p_i = pitch_i * PI / 180;
  float y_i = yaw_i * PI / 180;


  *Rx_i = (cos(p_i) * cos(y_i)) * ax + (sin(r_i) * sin(p_i) * cos(y_i) - cos(r_i) * sin(y_i)) * ay + (cos(r_i) * sin(p_i) * cos(y_i) + sin(r_i) * sin(y_i)) * az;
  *Ry_i = (cos(p_i) * sin(y_i)) * ax + (sin(r_i) * sin(p_i) * sin(y_i) + cos(r_i) * cos(y_i)) * ay + (cos(r_i) * sin(p_i) * sin(y_i) - sin(r_i) * cos(y_i)) * az;
  *Rz_i = -sin(p_i) * ax + sin(r_i) * cos(p_i) * ay + cos(p_i) * cos(r_i) * az;
}

/*---------------------------------------------------------------------------
 * Chuong trinh hien thi led Command:
 * StartCommand: Led on
 * StopCommand: Led off
 * EmerCommand: Led blink
--------------------------------------------------------------------------------*/
void ORC_Drone_Controller::commandLedDisplay(orcdrone_data *drone_in){
	if(drone_in->desiredData.EmerCommand){
		emerCommand_Cnt++;
		if(emerCommand_Cnt >= LED_EMERCOMMAND_CNT_MAX){
			//digitalWrite(LED_COMMAND_STATUS, ~digitalRead(LED_COMMAND_STATUS)); //Dao trang thai led
			commandStatusStore = !commandStatusStore;
			emerCommand_Cnt = 0;
		}
	}else if(drone_in->desiredData.StartCommand){
		//digitalWrite(LED_COMMAND_STATUS, HIGH);
		commandStatusStore = true;		
	}else commandStatusStore = false; 
	digitalWrite(LED_COMMAND_STATUS, commandStatusStore);
	
}

/*---------------------------------------------------------------------------
 * Chuong trinh hien thi led READY:
 * in = 0: Led off
 * in = 1: Led blink
 * in = 2: Led on
--------------------------------------------------------------------------------*/
void ORC_Drone_Controller::readyLedDisplay(uint8_t in){
	if(in == 1){
		readyCommand_Cnt++;
		if(readyCommand_Cnt >= LED_READYCOMMAND_CNT_MAX){
			digitalWrite(LED_PROGRAM_READY, !digitalRead(LED_PROGRAM_READY)); //Dao trang thai led
			//commandStatusStore = !commandStatusStore;
			readyCommand_Cnt = 0;
		}
	}else if(in == 0){
		digitalWrite(LED_PROGRAM_READY, HIGH);
				
	}else digitalWrite(LED_PROGRAM_READY, LOW);
	
}
/*---------------------------------------------------------------------------
 * Chuong trinh hien thi led GPS:
 * readErrCnt < GPS_TIME_OUT ->LED GPS ON
 * else LED GPS OFF
--------------------------------------------------------------------------------*/
void ORC_Drone_Controller::GPSLedDisplay(orcdrone_data *drone_in){
	if((drone_in->gpsMode)&&(drone_in->gpsdata.readErrCnt == 0)){
		digitalWrite(LED_GPS_READY, HIGH);		
	}else digitalWrite(LED_GPS_READY, LOW);
	
}
//===========================================BAT DAU HE THONG CAC HAM XU LY UART VOI HUB====================================================
/*----------------------------------
* GOI DU LIEU Roll pitch yaw to hub
------------------------------------*/
void ORC_Drone_Controller::sendRPY2Hub(orcdrone_data *drone_in)
{
	long rpy[5];	
	if(HUBPort.availableForWrite()>= 63){
		rpy[0] = (long) (drone_in->ebimu_data.roll*10);
		rpy[1] = (long) (drone_in->ebimu_data.pitch*10);
		rpy[2] = (long) (drone_in->ebimu_data.yaw*10);
		rpy[3] = (long) (drone_in->desiredData.flyMode);
		rpy[4] = (drone_in->WarningFault);
		//rpy[3] = (long) (drone_in->gpsdata.readErrCnt);
		
		HUBPort.print(ORCControllerUti.AttitudeDataPrefix); HUBPort.print(" ");
		HUBPort.print(rpy[0]); HUBPort.print(",");
		HUBPort.print(rpy[1]); HUBPort.print(",");
		HUBPort.print(rpy[2]); HUBPort.print(",");
		HUBPort.print(rpy[3]); HUBPort.print(",");
		HUBPort.print(rpy[4]); HUBPort.print(",");
		HUBPort.print(ORCControllerUti.ChecksumOfLongArray(rpy,5)); HUBPort.println("#");
	}
}
/*----------------------------------
* GOI DU LIEU Warning khi Ts tang cao toi Hub
------------------------------------*/
void ORC_Drone_Controller::sendTSWarning(orcdrone_data *drone_in)
{
	long TSArr[2];	
	if((drone_in->Ts > 12)&&(HUBPort.availableForWrite()>= 63)){		
		TSArr[0] = ORCControllerUti.ORCWarningErrorTable[TSWarningIndex][0];
		TSArr[1] = (long)drone_in->Ts;
		HUBPort.print(ORCControllerUti.MainWarningDataPrefix); HUBPort.print(" ");
		HUBPort.print(TSArr[0]); HUBPort.print(",");
		HUBPort.print(TSArr[1]); HUBPort.print(",");
		HUBPort.print(ORCControllerUti.ChecksumOfLongArray(TSArr,2)); HUBPort.println("#");	
	}
}
/*----------------------------------
* GOI DU LIEU Warning khi chat luong GPS xuong thap hon muc qui dinh
------------------------------------*/
void ORC_Drone_Controller::sendGPSWarning(orcdrone_data *drone_in)
{
	long GPSArr[2];	
	if((!(drone_in->gpsMode))&&((drone_in->gpsdata.pos_cm[3])<(drone_in->gpsdata.RTK_Standard))&&(HUBPort.availableForWrite()>= 63)){	
		GPSArr[0] = ORCControllerUti.ORCWarningErrorTable[GPSLOWSTANDARDWarningIndex][0];
		GPSArr[1] = drone_in->gpsdata.pos_cm[3];
		HUBPort.print(ORCControllerUti.MainWarningDataPrefix); HUBPort.print(" ");
		HUBPort.print(GPSArr[0]); HUBPort.print(",");
		HUBPort.print(GPSArr[1]); HUBPort.print(",");
		HUBPort.print(ORCControllerUti.ChecksumOfLongArray(GPSArr,2)); HUBPort.println("#");		
	}
}
/*----------------------------------
* Goi yeu cau goi du lieu base
------------------------------------*/
void ORC_Drone_Controller::sendGPSBaseRequest2Hub(orcdrone_data *drone_in)
{
	//int temp = 0;
	if(drone_in->gpsdata.isGotBase == false)
	{
		//temp = HUBPort.availableForWrite();
		//Serial.println(temp);
		if(HUBPort.availableForWrite()>=63){
		//if(temp >= 63){
			//Serial.println(droneGPSData.preFixString);
			HUBPort.println(droneGPSData.preFixString);
		}
	}
}
/*----------------------------------
* GOI DU LIEU Warning khi chat luong GPS xuong thap hon muc qui dinh
------------------------------------*/
void ORC_Drone_Controller::sendWarningFaultMsg2Hub(orcdrone_data *drone_in)
{
	long GPSArr[2];	
	if(drone_in->WarningFault != 0){ //Neu co loi hoac warning xay ra thi goi sang tay cam
		if(HUBPort.availableForWrite()>= 63){ //Chi goi khi buffer trong
			GPSArr[0] = MAIN_MSG_CODE_WARNINGFAULT;
			GPSArr[1] = drone_in->WarningFault;
			HUBPort.print(ORCControllerUti.MainWarningDataPrefix); HUBPort.print(" ");
			HUBPort.print(GPSArr[0]); HUBPort.print(",");
			HUBPort.print(GPSArr[1]); HUBPort.print(",");
			HUBPort.print(ORCControllerUti.ChecksumOfLongArray(GPSArr,2)); HUBPort.println("#");
			// Serial.println("Send Waring");
		}
	}
}

/*----------------------------------
* GOI DU LIEU Warning khi chat luong GPS xuong thap hon muc qui dinh
------------------------------------*/
void ORC_Drone_Controller::sendFlyPointID2Hub(orcdrone_data *drone_in, int id_in)
{
	long GPSArr[2];	
	if((drone_in->desiredData.EmerCommand)&&(id_in >= 0)){ //Chi goi du lieu khi o che do Emer
		if(HUBPort.availableForWrite()>= 63){ //Chi goi khi buffer trong
			GPSArr[0] = MAIN_MSG_CODE_FLY_ID;
			GPSArr[1] = id_in;
			HUBPort.print(ORCControllerUti.MainWarningDataPrefix); HUBPort.print(" ");
			HUBPort.print(GPSArr[0]); HUBPort.print(",");
			HUBPort.print(GPSArr[1]); HUBPort.print(",");
			HUBPort.print(ORCControllerUti.ChecksumOfLongArray(GPSArr,2)); HUBPort.println("#");
			//Serial.println("Send sendFlyPointID2Hub");
		}
	}
}

/*----------------------------------
* GOI DU LIEU PUMP SANG HUB
------------------------------------*/
void ORC_Drone_Controller::sendPumpControlData2Hub(orcdrone_data *drone_in)
{	
	if(drone_in->desiredData.pumpData.isPumpOnRequired){ //Chi goi du lieu khi o che do Emer
		if(drone_in->desiredData.pumpData.feedbackFlow != drone_in->desiredData.pumpData.desiredFlow){
			//Serial.println("Vao ham goi du lieu pump ON sang Hub");
			drone_in->desiredData.pumpData.pumpOnSendCnt++;
			if(drone_in->desiredData.pumpData.pumpOnSendCnt >= PUMP_FLOW_SEND_COUNT_MAX){
				drone_in->desiredData.pumpData.pumpOnSendCnt = 0;
				if(HUBPort.availableForWrite()>= 63){ //Chi goi khi buffer trong			
					HUBPort.print(ORCControllerUti.FlowControlDataPrefix); HUBPort.print(" ");
					HUBPort.print(drone_in->desiredData.pumpData.desiredFlow); HUBPort.print(",");			
					HUBPort.print(drone_in->desiredData.pumpData.desiredFlow); HUBPort.println("#");	
					//Serial.println("Da goi du lieu pump ON sang Hub");
				}
			}
		}
		else drone_in->desiredData.pumpData.isPumpOnRequired = false;

	}else if(drone_in->desiredData.pumpData.isPumpOffRequired){ //Chi goi du lieu khi o che do Emer
		if(drone_in->desiredData.pumpData.feedbackFlow != drone_in->desiredData.pumpData.desiredFlow){
			//Serial.println("Vao ham goi du lieu pump OFF sang Hub");
			drone_in->desiredData.pumpData.pumpOffSendCnt++;
			if(drone_in->desiredData.pumpData.pumpOffSendCnt >= PUMP_FLOW_SEND_COUNT_MAX){
				drone_in->desiredData.pumpData.pumpOffSendCnt = 0;
				if(HUBPort.availableForWrite()>= 63){ //Chi goi khi buffer trong			
					HUBPort.print(ORCControllerUti.FlowControlDataPrefix); HUBPort.println(" 0,0#");
					//Serial.println("Da goi du lieu pump OFF sang Hub");					
				}
			}
		} else drone_in->desiredData.pumpData.isPumpOffRequired = false;
	}
}
/*----------------------------------
* GOI DU LIEU QUA HUB
------------------------------------*/
void ORC_Drone_Controller::sendData2HUB(orcdrone_data *drone_in){
	hubSendData_Cnt++; //tang thoi gian dem
	if(hubSendData_Cnt == HUBSENDTIMEBASE){ //Goi du lieu TS loi
		sendWarningFaultMsg2Hub(drone_in);
	}else if(hubSendData_Cnt >= (2*HUBSENDTIMEBASE)){ //Goi du lieu rpy
		sendRPY2Hub(drone_in);
		hubSendData_Cnt = 0;
	}	
}
/*-------------------------------------------------
* QUET DU LIEU GPS TU HUB
---------------------------------------------------*/
bool ORC_Drone_Controller::GPSReadData(orcdrone_data *drone_in)
{
	String strTemp = "";
	bool result = false;
	if(!(drone_in->gpsMode)){
		result = true;
		drone_in->gpsdata.isGotBase = true;
	}else{
		//Kiem tra du lieu tu cong GPS
		if(HUBPort.available()){//Phat hien co du lieu: chuyen du lieu string nhan dc vao du lieu GPS
			strTemp = HUBPort.readStringUntil('\n');	
			if(strTemp.startsWith(dronePositionData.preFixString)){ //Du lieu vi tri
				if(ORCControllerUti.SplitData2LongInStruct(strTemp, &dronePositionData)){
					drone_in->gpsdata.pos_cm[0] = dronePositionData.dataOut[0];
					drone_in->gpsdata.pos_cm[1] = dronePositionData.dataOut[1];
					drone_in->gpsdata.pos_cm[2] = dronePositionData.dataOut[2];
					drone_in->gpsdata.pos_cm[3] = dronePositionData.dataOut[3];
					drone_in->gpsdata.readErrCnt = 0;
					result = true;
				}
			}else if ((!(drone_in->gpsdata.isGotBase))&&(strTemp.startsWith(droneGPSData.preFixString))){ //Du lieu toa do --> base
				if(ORCControllerUti.SplitData2LongInStruct(strTemp, &droneGPSData)){
					drone_in->gpsdata.base_gps_long[0] = droneGPSData.dataOut[0];
					drone_in->gpsdata.base_gps_long[1] = droneGPSData.dataOut[1];
					drone_in->gpsdata.base_gps_long[2] = droneGPSData.dataOut[2];
					drone_in->gpsdata.base_gps_long[3] = droneGPSData.dataOut[3];				
					drone_in->gpsdata.isGotBase = true;
					result = true;
				}		
			}		
		}else{
			drone_in->gpsdata.readErrCnt = drone_in->gpsdata.readErrCnt + 1; //Tang bien dem loi
			if(drone_in->gpsdata.readErrCnt >= GPS_TIME_OUT) drone_in->gpsdata.readErrCnt = GPS_TIME_OUT; //Gioi han bien dem loi
		}		
	}
	return result;
}
/*-------------------------------------------------
* QUET DU LIEU GPS TU HUB
---------------------------------------------------*/
bool ORC_Drone_Controller::HUBReadData(orcdrone_data *drone_in)
{
	String strTemp = "";
	bool result = false;	
	//Kiem tra du lieu tu cong GPS
	if(HUBPort.available()){//Phat hien co du lieu: chuyen du lieu string nhan dc vao du lieu GPS
		strTemp = HUBPort.readStringUntil('\n');
		//Serial.println(strTemp);
		if(strTemp.startsWith(droneObstacleControlData.preFixString)){ //Doc du lieu tranh vat can
			if(ORCControllerUti.SplitData2LongInStruct(strTemp, &droneObstacleControlData)){
				drone_in->obstacleAvoidanceVar.ov_roll_rec = ((float)(droneObstacleControlData.dataOut[0]))/10;
				drone_in->obstacleAvoidanceVar.ov_pitch_rec = ((float)(droneObstacleControlData.dataOut[1]))/10;				
				drone_in->obstacleAvoidanceVar.readErrCnt = 0;
				result = true;
			}else
				drone_in->obstacleAvoidanceVar.readErrCnt = drone_in->obstacleAvoidanceVar.readErrCnt + 1;
			drone_in->gpsdata.readErrCnt = drone_in->gpsdata.readErrCnt + 1; //Tang bien dem loi cho GPS
		}else{
			drone_in->obstacleAvoidanceVar.readErrCnt = drone_in->obstacleAvoidanceVar.readErrCnt + 1;
			//Doc tin hieu feedback tu pump
			if(strTemp.startsWith(dronePumpFBData.preFixString)){
				//Serial.print("Nhan Du lieu pump tu HUB: ");
				//Serial.println(strTemp);
				if(ORCControllerUti.SplitData2LongInStruct(strTemp, &dronePumpFBData)){
					drone_in->desiredData.pumpData.feedbackFlow = dronePumpFBData.dataOut[0];					
				}
				drone_in->gpsdata.readErrCnt = drone_in->gpsdata.readErrCnt + 1; //Tang bien dem loi cho GPS
			}else 
				if(!(drone_in->gpsMode)){
					result = true;
					drone_in->gpsdata.isGotBase = true;
					drone_in->gpsdata.readErrCnt = 0;
				}else{
					if(strTemp.startsWith(dronePositionData.preFixString)){ //Du lieu vi tri
						if(ORCControllerUti.SplitData2LongInStruct(strTemp, &dronePositionData)){
							drone_in->gpsdata.pos_cm[0] = dronePositionData.dataOut[0];
							drone_in->gpsdata.pos_cm[1] = dronePositionData.dataOut[1];
							drone_in->gpsdata.pos_cm[2] = dronePositionData.dataOut[2];
							drone_in->gpsdata.pos_cm[3] = dronePositionData.dataOut[3];
							drone_in->gpsdata.readErrCnt = 0;
							//Danh gia chat luong Du lieu
							if(drone_in->gpsdata.pos_cm[3] == GPS_RTK_NOSOLUTION){
								if(drone_in->gpsdata.nosolutionCnt < GPS_RTK_NOSOLUTION_CNT_MAX)
									drone_in->gpsdata.nosolutionCnt = drone_in->gpsdata.nosolutionCnt + 1;
							} else drone_in->gpsdata.nosolutionCnt = 0;
							
							result = true;
						}else
							drone_in->gpsdata.readErrCnt = drone_in->gpsdata.readErrCnt + 1; //Tang bien dem loi
					}else if ((!(drone_in->gpsdata.isGotBase))&&(strTemp.startsWith(droneGPSData.preFixString))){ //Du lieu toa do --> base
						if(ORCControllerUti.SplitData2LongInStruct(strTemp, &droneGPSData)){
							drone_in->gpsdata.base_gps_long[0] = droneGPSData.dataOut[0];
							drone_in->gpsdata.base_gps_long[1] = droneGPSData.dataOut[1];
							drone_in->gpsdata.base_gps_long[2] = droneGPSData.dataOut[2];
							drone_in->gpsdata.base_gps_long[3] = droneGPSData.dataOut[3];				
							drone_in->gpsdata.isGotBase = true;
							result = true;
						}else
							drone_in->gpsdata.readErrCnt = drone_in->gpsdata.readErrCnt + 1; //Tang bien dem loi	
					}else
						drone_in->gpsdata.readErrCnt = drone_in->gpsdata.readErrCnt + 1; //Tang bien dem loi
				}

		}

				
	}else{
		drone_in->gpsdata.readErrCnt = drone_in->gpsdata.readErrCnt + 1; //Tang bien dem loi
		drone_in->obstacleAvoidanceVar.readErrCnt = drone_in->obstacleAvoidanceVar.readErrCnt + 1;
		
	}		
	if(drone_in->gpsdata.readErrCnt > GPS_TIME_OUT){
		drone_in->gpsdata.readErrCnt = GPS_TIME_OUT; //Gioi han bien dem loi
		drone_in->gpsdata.isError = true;
	}
	if(drone_in->obstacleAvoidanceVar.readErrCnt > OBSTACLEAVOIDANCE_CONTROLDATA_TIMEOUT){
		drone_in->obstacleAvoidanceVar.readErrCnt = OBSTACLEAVOIDANCE_CONTROLDATA_TIMEOUT;
		drone_in->obstacleAvoidanceVar.ov_roll_rec = 0;
		drone_in->obstacleAvoidanceVar.ov_pitch_rec = 0;	

	} 
	return result;
}
//==========================================------KET THUC HE THONG CAC HAM XU LY UART VOI HUB------====================================================
/*----------------------------------------
* xu ly du lieu tranh vat can
------------------------------------------*/
void ORC_Drone_Controller::ObstacleAvoidanceDataProcessing(orcdrone_data *drone_in){
	if(drone_in->zd > OBSTACLEAVOIDANCE_ACTIVE_HEIGHT){
		//xu ly goc
		drone_in->obstacleAvoidanceVar.ov_roll_d = _K_OV_Gain*drone_in->obstacleAvoidanceVar.ov_roll_d + (1 - _K_OV_Gain)*drone_in->obstacleAvoidanceVar.ov_roll_rec;
		drone_in->obstacleAvoidanceVar.ov_pitch_d = _K_OV_Gain*drone_in->obstacleAvoidanceVar.ov_pitch_d + (1 - _K_OV_Gain)*drone_in->obstacleAvoidanceVar.ov_pitch_rec; 
	}else
	{
		drone_in->obstacleAvoidanceVar.ov_roll_d = _K_OV_Gain*drone_in->obstacleAvoidanceVar.ov_roll_d;
		drone_in->obstacleAvoidanceVar.ov_pitch_d = _K_OV_Gain*drone_in->obstacleAvoidanceVar.ov_pitch_d;
	}
	//Xu ly vi tri y
	if(fabs(drone_in->obstacleAvoidanceVar.ov_pitch_d) > 0.5) //Xu ly binh thuong
	{
		drone_in->obstacleAvoidanceVar.ov_xd = drone_in->obstacleAvoidanceVar.ov_xd + 0.5*drone_in->obstacleAvoidanceVar.ov_pitch_d*drone_in->Ts;
		if(drone_in->obstacleAvoidanceVar.ov_xd > OBSTACLEAVOIDANCE_XY_POSITION_MAX){
			drone_in->obstacleAvoidanceVar.ov_xd = OBSTACLEAVOIDANCE_XY_POSITION_MAX;
		}else if(drone_in->obstacleAvoidanceVar.ov_xd < -OBSTACLEAVOIDANCE_XY_POSITION_MAX){
			drone_in->obstacleAvoidanceVar.ov_xd = -OBSTACLEAVOIDANCE_XY_POSITION_MAX;
		}
	}else //deadzone
	{
		drone_in->obstacleAvoidanceVar.ov_xd = 0.999*drone_in->obstacleAvoidanceVar.ov_xd;	//Van toc lui ve		
	}
	//Xu ly vi tri x
	if(fabs(drone_in->obstacleAvoidanceVar.ov_roll_d) > 0.5) //Xu ly binh thuong
	{
		drone_in->obstacleAvoidanceVar.ov_yd = drone_in->obstacleAvoidanceVar.ov_yd - 0.5*drone_in->obstacleAvoidanceVar.ov_roll_d*drone_in->Ts;
		if(drone_in->obstacleAvoidanceVar.ov_yd > OBSTACLEAVOIDANCE_XY_POSITION_MAX){
			drone_in->obstacleAvoidanceVar.ov_yd = OBSTACLEAVOIDANCE_XY_POSITION_MAX;
		}else if(drone_in->obstacleAvoidanceVar.ov_yd < -OBSTACLEAVOIDANCE_XY_POSITION_MAX){
			drone_in->obstacleAvoidanceVar.ov_yd = -OBSTACLEAVOIDANCE_XY_POSITION_MAX;
		}
	}else //deadzone
	{
		drone_in->obstacleAvoidanceVar.ov_yd = 0.999*drone_in->obstacleAvoidanceVar.ov_yd;			
	}
	
}

/*----------------------------------------
* Ham xu ly he thong bom:
* Qui trinh:
  + cho he thong pump khi nhan tin hieu pump enable tu tay cam: co canh len duong --> goi tin hieu mo pump, co canh xuong am goi tin hieu tat pump
  + Cho bom khi o che do bay tu dong va diem dang xet co gia tri pump flow lon hon khong
------------------------------------------*/
void ORC_Drone_Controller::PumpDataProcessing(orcdrone_data *drone_in){
	//Doc gia tri pump o che do auto
	if(drone_in->desiredData.flyMode == FLY_MODE_MULTI_POINT){ //Chi xet o che do auto da diem
		drone_in->desiredData.pumpData.preAutoPumpCommand = drone_in->desiredData.pumpData.curAutoPumpCommand; //Luu lai gia tri auto pump flow
		if((drone_in->auto_current_point>1)&&(drone_in->auto_current_point < MaxPtoPnum)){
			//Luu lai gia tri auto pump flow
			drone_in->desiredData.pumpData.curAutoPumpCommand = (drone_in->desiredData.data_PtoP_Control[drone_in->auto_current_point -1][POINT_MEMORY_INDEX_PUMPFLOW]>0)?true:false;
			//Ra quyet dinh dieu khien pump
			// MO PUMP: CHI MO PUMP --> FEEDBACK VE SE TẮT PUMP
			if(drone_in->desiredData.pumpData.curAutoPumpCommand && (!drone_in->desiredData.pumpData.preAutoPumpCommand)){
				drone_in->desiredData.pumpData.isPumpOnRequired = true;		
				drone_in->desiredData.pumpData.desiredFlow = drone_in->desiredData.data_PtoP_Control[drone_in->auto_current_point - 1][POINT_MEMORY_INDEX_PUMPFLOW];
			}
		}
		else{
			drone_in->desiredData.pumpData.curAutoPumpCommand = false;
			//drone_in->desiredData.pumpData.desiredFlow = 0;
		}
			
		// TAT PUMP: CHI TAT PUMP
		if((!drone_in->desiredData.pumpData.curAutoPumpCommand) && drone_in->desiredData.pumpData.preAutoPumpCommand){
			drone_in->desiredData.pumpData.isPumpOffRequired = true;
			drone_in->desiredData.pumpData.desiredFlow = 0;
		}
	}
	
	if(drone_in->desiredData.pumpData.curManualPumpCommand && (!drone_in->desiredData.pumpData.preManualPumpCommand)){
		drone_in->desiredData.pumpData.isPumpOnRequired = true;	
		drone_in->desiredData.pumpData.desiredFlow = MANUAL_PUMP_FLOW_DEFAULT;
	} 
	// else{
	// 	drone_in->desiredData.pumpData.isPumpOnRequired = false;	
	// 	drone_in->desiredData.pumpData.desiredFlow = 0;
	// }
	if((!drone_in->desiredData.pumpData.curManualPumpCommand) && drone_in->desiredData.pumpData.preManualPumpCommand){
		drone_in->desiredData.pumpData.isPumpOffRequired = true;	
		drone_in->desiredData.pumpData.desiredFlow = 0;
	} 
	// else{
	// 	drone_in->desiredData.pumpData.isPumpOffRequired = false;	
	// }	
}
/*----------------------------------------
* Check GPS signal
------------------------------------------*/
bool ORC_Drone_Controller::GPSCheckData(orcdrone_data *drone_in, int16_t unchangeMax){
  
	bool result = true;
	if(drone_in->gpsMode){
		//Kiem tra co su thay doi cua tin hieu khong
		if(drone_in->gpsdata.readErrCnt >= unchangeMax){
			Serial.print("gpsdata.readErrCnt: "); Serial.println(drone_in->gpsdata.readErrCnt);
			result = false;
		}

		//Kiem tra tin hieu co nam trong mien cho phep hay khong
		if((abs(drone_in->gpsdata.pos_cm[0])>GPS_DEVIATION_MAX_XY)|| (abs(drone_in->gpsdata.pos_cm[1])>GPS_DEVIATION_MAX_XY)|| (abs(drone_in->gpsdata.pos_cm[2])>GPS_DEVIATION_MAX_Z)){
			result = false;
		}
	}
	return result; 
  
}

/*--------------------------------------------
 * Cai dat cac thong so dieu khien cho he thong
---------------------------------------------*/
void ORC_Drone_Controller::setupDroneSystem(orcdrone_data *drone_in, bool gpsMode_in, long RTK_StandardIn)
{
	LIDARPort.begin(115200); LIDARPort.setTimeout(2); 
	HUBPort.begin(115200); HUBPort.setTimeout(2); 
	float offsetVal[3]={4.957024, -17.126465, 5.203447};
	float disx[3] = {0.918457, 0.002606, 0.007833};
	float disy[3] = {0.002606, 0.961122, 0.001134};
	float disz[3] = {0.007833, 0.001134, 0.963156};
	int16_t lidarTemp = 0, n_invest = 10;
	int pointTemp = 0, pointColumTemp = 0;
	//Setup safety
	drone_in->desiredData.EmerCommand = true;
	drone_in->desiredData.StartCommand = false;
	drone_in->desiredData.StopCommand = false;
	drone_in->FlyingEnable = 0;
	drone_in->FlyingEnable = WF_Error_RF_GOOD;
	drone_in->gpsdata.isError = false;
	drone_in->gpsMode = gpsMode_in;
	
	drone_in->P_basic = Pmin;
	drone_in->autoTuneState = false;
	drone_in->gpsdata.RTK_Standard = RTK_StandardIn;
	drone_in->Ts = 0.01;

	//Reset cac he so pump
	drone_in->desiredData.pumpData.preManualPumpCommand = false;
	drone_in->desiredData.pumpData.curManualPumpCommand = false;
	drone_in->desiredData.pumpData.preAutoPumpCommand = false;
	drone_in->desiredData.pumpData.curAutoPumpCommand = false;
	drone_in->desiredData.pumpData.isPumpOnRequired = false;
	drone_in->desiredData.pumpData.isPumpOffRequired = false;
	drone_in->desiredData.pumpData.feedbackFlow = 0;
	drone_in->desiredData.pumpData.desiredFlow = 0;
	drone_in->desiredData.pumpData.pumpOnSendCnt = PUMP_FLOW_SEND_COUNT_MAX;
	drone_in->desiredData.pumpData.pumpOffSendCnt = PUMP_FLOW_SEND_COUNT_MAX;


	//Set status led
	pinMode(LED_RF_NHAN, OUTPUT);
	pinMode(LED_PROGRAM_READY, OUTPUT);
	pinMode(LED_GPS_READY, OUTPUT);
	pinMode(LED_COMMAND_STATUS, OUTPUT);
	
	digitalWrite(LED_RF_NHAN, LOW);
	digitalWrite(LED_PROGRAM_READY, LOW);
	digitalWrite(LED_GPS_READY, LOW);
	digitalWrite(LED_COMMAND_STATUS, LOW);
	
	
	//Setup control gains
	setControlGains(drone_in);
	
	//Goi tin hieu tat bom sang pumphub
	drone_in->desiredData.pumpData.feedbackFlow = 1;
	drone_in->desiredData.pumpData.isPumpOffRequired = true;
	drone_in->desiredData.pumpData.desiredFlow = 0;
	sendPumpControlData2Hub(drone_in);//Goi du lieu pump sang hub
	
	//Reset bien OV
	drone_in->obstacleAvoidanceVar.ov_xd = 0;
	drone_in->obstacleAvoidanceVar.ov_yd = 0;
	
	// Reset lai bo nho cho qua trinh bay da diem
	pointTemp = 0;
	for(pointTemp = 0; pointTemp < MaxPtoPnum; pointTemp++)
	{
		pointColumTemp = 0;
		for(pointColumTemp = 0; pointColumTemp < POINT_MEMORY_MAX_LEN; pointColumTemp++)
		{
			drone_in->desiredData.data_PtoP_Control[pointTemp][pointColumTemp] = 0;			
		}
	}

	//Khoi dong RF
	while (!(DroneRFRead->RFReadSetup()))
		Serial.println("Dang cho RF khoi dong...."); //Dang cho khoi dong
	Serial.println("RF da khoi dong xong.");     // Da khoi dong xong
	
	
	
	//Doc GSPBase
	if(drone_in->gpsMode){ //Khi su dung mode GPS
		while(!(drone_in->gpsdata.isGotBase)){
			//Goi yeu cau tin hieu base
			sendGPSBaseRequest2Hub(drone_in);
			delay(50);
			//Doc du lieu tu GPS data
			HUBReadData(drone_in);
			//GPSReadData(drone_in);
			
			Serial.println("Dang doi lay tin hieu GPS base"); //Dang cho khoi dong
			GPSLedDisplay(drone_in); //thay doi trang thai led GPS			
		}
		digitalWrite(LED_GPS_READY, HIGH);
		Serial.println("Da lay GPS base xong.");     // Da khoi dong xong
	}
	//Khoi dong doc EBIMU
	// DroneEBI_data->EBISetOffsetValues(&(drone_in->ebimu_data), -1 ,0.72, -176);
	// DroneEBI_data->EBISetOffsetValues(&(drone_in->ebimu_data), 0.4 ,0.59, -166);//-176);
	// DroneEBI_data->EBISetOffsetValues(&(drone_in->ebimu_data), 3 ,-0.5, -166);//-176); D2
	DroneEBI_data->EBISetOffsetValues(&(drone_in->ebimu_data), 0, 7.6, -166);//-176); D2
	//Khoi dong bien mpu6050
	DroneMPU.MPU6050Setup(MPU6050_MODE_CONTROL);
	//Khoi dong doc Lidar1D
	while(!Lidar1D_read(&(drone_in->zLidar1D)))
	{
		Serial.println("Dang khoi dong lidar 1D.");
		delay(20);
	};
	for (int i = 0; i < n_invest; i++){
		lidarTemp += drone_in->zLidar1D;
		delay(20);
	}
	drone_in->zLidar1DErr = lidarTemp/n_invest; //khoang cach ban dau cua lidar
	//Khoi dong bien HMC
	DroneHMC.HMCSetup();
	//Manual Calib cam bien
	DroneHMC.HMCCalibParaManualSet(offsetVal, disx, disy, disz);

	// Tu dong calib EBI va MPU
	EBIMPUSelfCalibration(drone_in);

	Serial.println("Calib cam bien thu cong thanh cong.");
	//Cai dat PWM
	DronePWM.PWMSetup();
	DronePWM.setPWMMaxMin(0, 2000);
}

/*---------------------------------------------------------------------------
 * Chuong trinh kiem tra tat ca cac cam bien cua he thong truoc khi cho drone cat canh
 * cac dong co drone se qua nhe
 * he thong se check cac cam bien EBIMU, MPU6050, LIDAR, RF de dam bao co du lieu, du lieu trong vung cho phep, du lieu co su bien thien
--------------------------------------------------------------------------------*/
bool ORC_Drone_Controller::preCheckLoop(orcdrone_data *drone_in, int16_t maxCheckNumber)
{
	int16_t checkCnt = 0, maxCheckCnt = 0, ucEBICnt[3] = {0,0,0}, ucMPUCnt = 0, ucLIDARCnt = 0, wrongdataCnt=0;
	bool result = true, temp1 = true;
	float headingAngle = 0;
	int16_t preLidar1D = 0;
	int rfResult;
	orcebi_data preEBIData;
	orcmpu_data preMPUData;
	orcgps_data preGPSData;
	//Tinh so thoi gian can check
	maxCheckCnt = maxCheckNumber;//(maxCheckNumber > MAX_RF_ERROR)?maxCheckNumber:(MAX_RF_ERROR + 10);
	//Doc gia tri lan dau tien
	
	while(!EBIReadandRecalib(drone_in))
		Serial.println("Cho cho den khi doc duoc gia tri EBIMU!");
	while (!(DroneEBI_data->CheckEBIData(&preEBIData, &(drone_in->ebimu_data), preEBIData.yaw, ucEBICnt))){
		EBIReadandRecalib(drone_in);
	}
	
	//Khoi dong xu ly Lidar
	while(!Lidar1D_read(&(drone_in->zLidar1D)))
		Serial.println("Cho cho den khi doc duoc gia tri Lidar!");
	preLidar1D = drone_in->zLidar1D;
	
	
	while(!(drone_in->desiredData.StartCommand)){
		
		Serial.println("Cho cho den khi nhan nut Start!");
		//Doc tin hieu RF va kiem tra loi tin hieu RF
		rfResult = DroneRFRead->RFReadData(drone_in);
		temp1 = (rfResult >= RF_DATA_RECEIVED_NO_ID)?true:false;
		sendFlyPointID2Hub(drone_in, rfResult);
		//Led RF and Command on/off
		digitalWrite(LED_RF_NHAN, temp1);
		commandLedDisplay(drone_in);
		delay(10);
	}
	
	
	
	
	//Cho dong co quay
	DronePWM.setPWMFromDrone(drone_in, 1);
	delay(1000);
	//Bat dau check tin hieu
	preLidar1D = -100;
	while (!((drone_in->FlyingEnable >= FlyingEnableThres)&& result && (checkCnt >= maxCheckCnt))){
		delay(20);
		
		
		//------------BƯỚC 1: DOC TIN HIEU DIEU KHIEN MONG MUON-------------------------
		//Doc tin hieu RF va kiem tra loi tin hieu RF
		temp1 = DroneRFRead->RFReadData(drone_in);
		//Led RF and Command on/off
		digitalWrite(LED_RF_NHAN, temp1);
		commandLedDisplay(drone_in);		
		result &= (drone_in->desiredData.RFErrorCnt < MAX_RF_ERROR);
		if(!result){
			Serial.print("That bai do doc RF:");Serial.println(drone_in->desiredData.RFErrorCnt);
		}
		
		result &= ((fabs(drone_in->mpu_data.mroll_offset) < 5)&&(fabs(drone_in->mpu_data.mpitch_offset) < 5));
		if(!result){
			Serial.print("That bai do doc mroll_offset:"); Serial.print((drone_in->mpu_data.mroll_offset));
			Serial.print(","); Serial.print((drone_in->mpu_data.mpitch_offset));
			Serial.print(","); Serial.print((drone_in->mpu_data.mroll));
			Serial.print(","); Serial.println((drone_in->mpu_data.mpitch));
		}
		
		if(checkCnt <= maxCheckCnt){
			
			//Led ready on/off		
			readyLedDisplay(1);
			//------------BƯỚC 3: DOC CAC TRẠNG THAI HE THONG-------------------------
			//Doc tin hieu tu EBIMU
			EBIReadandRecalib(drone_in);
			
			result &= (drone_in->ebimu_data.readErrCnt < 4);
			Serial.print("ebimu_data.readErrCnt: "); Serial.println(drone_in->ebimu_data.readErrCnt);
			if(!result){
				Serial.println("That bai do doc EBI!");
			}
			
			Serial.print(drone_in->ebimu_data.roll); Serial.print(", "); 
			Serial.print(drone_in->ebimu_data.pitch); Serial.print(", ");
			Serial.println(drone_in->ebimu_data.yaw); 
			
			//Read HMC data
			headingAngle = DroneHMC.HMCReadData();
			//Kiem tra du lieu EBIMU
			//result &= DroneEBI_data->CheckEBIData(&preEBIData, &(drone_in->ebimu_data), headingAngle, &ucEBICnt);
			result &= DroneEBI_data->CheckEBIData(&preEBIData, &(drone_in->ebimu_data), preEBIData.yaw, ucEBICnt);
			if(!result){
				// Serial.print(drone_in->ebimu_data.roll); Serial.print(", "); 
				// Serial.print(drone_in->ebimu_data.pitch); Serial.print(", ");
				// Serial.print(drone_in->ebimu_data.yaw); Serial.print(", ");
				// Serial.print(ucEBICnt); Serial.print(", ");
				Serial.println("That bai do check EBI!");
			}
			// Serial.print(drone_in->ebimu_data.roll); Serial.print(", "); 
			// Serial.print(drone_in->ebimu_data.pitch); Serial.print(", ");
			// Serial.println(drone_in->ebimu_data.yaw); //Serial.print(", ");
			//Serial.println(drone_in->zLidar1DErr);
			//result &= (ucEBICnt > ((int16_t)0.3*maxCheckCnt)); //Nêu so lan du lieu khong thay doi nhieu qua
			
			//Doc du lieu tu 6050
			DroneMPU.MPU6050ReadData(drone_in->Ts, &(drone_in->mpu_data));
			result &= (drone_in->mpu_data.readErrCnt <= 2); //Cho phep doc loi 2 lan lien tiep (20ms)
			if(!result){
				Serial.print("mpu_data.readErrCnt: "); Serial.print(drone_in->mpu_data.readErrCnt);
				Serial.println(". That bai do doc MPU!");
			}
			//MPUDataProcessing(drone_in);
			//Kiem tra du lieu co phu hop hay khong
			result &= DroneMPU.MPU6050CheckData(&preMPUData, &(drone_in->mpu_data),  &ucMPUCnt);
			if(!result)
				Serial.println("That bai do check MPU!");
			
			//=========CHECK RIENG ACCZ
			if(fabs((drone_in->mpu_data.accz -1)*9.81) > MPU6050_ACC_MAX_UNMOVE_VALUE)
			{
				wrongdataCnt++;
				Serial.print(" --------------->Accz Error: "); Serial.println((drone_in->mpu_data.accz -1)*9.81);
				result = false;
			}  
			Serial.print(" Accz Error Number: "); Serial.println(wrongdataCnt);
			
			//===========END CHECK ACCZ
			
			
			// Serial.print(drone_in->mpu_data.accx); Serial.print(", "); 
			// Serial.print(drone_in->mpu_data.accy); Serial.print(", ");
			// Serial.print(drone_in->mpu_data.accz); Serial.print(", "); 
			// Serial.print(drone_in->mpu_data.gx); Serial.print(", ");
			// Serial.print(drone_in->mpu_data.gy); Serial.print(", "); 
			// Serial.println(drone_in->mpu_data.gz); 
			
			//Doc du lieu lidar1D
			Lidar1D_read(&(drone_in->zLidar1D));
			//Serial.print(drone_in->zLidar1D); Serial.print("....."); Serial.println(drone_in->zLidar1DErr);
			result &= Lidar1DCheckData(drone_in,&preLidar1D, drone_in->zLidar1D, &ucLIDARCnt, maxCheckCnt);
			if(!result)
				Serial.println("That bai do check lidar!");

			//Doc du lieu tu GPS va obstacle avoidance data
			HUBReadData(drone_in);

			//Doc tin hieu GPS			
			//GPSReadData(drone_in);

			//Kiem tra chat luong tin hieu GPS
			result &= GPSCheckData(drone_in, ((maxCheckCnt < GPS_TIME_OUT)?maxCheckCnt:GPS_TIME_OUT));
			if(!result)
				Serial.println("That bai do check GPS!");

			// // Kiem tra co nguoi hoac vat can trong tam cua drone hay khong: he thong on khi co du lieu vat can va du lieu do bang khong
			// result &= ((drone_in->obstacleAvoidanceVar.readErrCnt < OBSTACLEAVOIDANCE_CONTROLDATA_TIMEOUT)&&(fabs(drone_in->obstacleAvoidanceVar.ov_roll_rec)<OBSTACLEAVOIDANCE_CONTROLDATA_ACCEPTANCE)&&(fabs(drone_in->obstacleAvoidanceVar.ov_pitch_rec)<OBSTACLEAVOIDANCE_CONTROLDATA_ACCEPTANCE));
			// if(!result)
				// Serial.println("That bai do co vat can gan Drone!");
		
		}else
		{
			readyLedDisplay(result?0:2);//Hien thi ket qua kiem tra loi
		}
		checkCnt++;
		if(checkCnt > maxCheckCnt) checkCnt = maxCheckCnt+1;
		if((drone_in->desiredData.EmerCommand)||(drone_in->desiredData.StopCommand)) //Nhan nut Stop hoac Emer
		{
			
			if(drone_in->desiredData.StopCommand){//Cho phep tien hanh kiem tra lai
				Serial.println("Stop command!");
				result = true;
				checkCnt = 0;	
				drone_in->desiredData.StopCommand = false;
			}
			if (drone_in->desiredData.EmerCommand) //Tat dong co
			{
				DronePWM.setPWM(0, 0, 0, 0);
			}
		}
		else if(drone_in->desiredData.StartCommand) //Nhan nut Start
		{
			DronePWM.setPWM(PWM_TEST_LEVEL, PWM_TEST_LEVEL, PWM_TEST_LEVEL, PWM_TEST_LEVEL);
		}
		if((checkCnt >= maxCheckCnt)&& (!result)){
			Serial.print("Ket qua kiem tra that bai:");
			Serial.println(result);
			
		}
	}
	
	return true;
}
/*---------------------------------------------------------------------------
 * Chuong trinh chinh hoat dong cua he thong
--------------------------------------------------------------------------------*/
void ORC_Drone_Controller::runLoop(orcdrone_data *drone_in)
{
	int rfResult = RF_NODATA_RECEIVED;
	//------------BƯỚC 1: DOC TIN HIEU DIEU KHIEN MONG MUON-------------------------
	//Doc tin hieu RF
	rfResult = DroneRFRead->RFReadData(drone_in);

	//------------BƯỚC 2: DOC CAC TRẠNG THAI HE THONG-------------------------
	

	//Doc du lieu HMC
	DroneHMC.HMCReadData();
	//Doc du lieu tu 6050
	DroneMPU.MPU6050ReadData(drone_in->Ts, &(drone_in->mpu_data));
	//Doc tin hieu tu EBIMU
	//DroneEBI_data->EBIReadData(&(drone_in->ebimu_data));
	EBIReadandRecalib(drone_in);
	//Xu ly cac tin hieu gia toc
	MPUDataProcessing(drone_in);
	
	//Doc cac tin hieu tu Hub
	HUBReadData(drone_in);
	//Doc tin hieu tu GPS
	//GPSReadData(drone_in);
	PositionDataProcessing(drone_in);

	//Xu ly thuat toan tranh vat can
	ObstacleAvoidanceDataProcessing(drone_in);
	
	//Doc du lieu lidar1D
	Lidar1D_read(&(drone_in->zLidar1D));
	AltitudeDataProcessing(drone_in);

	//Kiem tra loi he thong
	Drone_WarningFault.WFCheckandSetError(drone_in);
	
	
	//------------BUOC 3: XU LY CAC TRANG THAI NUT NHAT VA LOI-------------
	Program_Processing(drone_in->desiredData.data_PtoP_Control,drone_in);
	
	
	
	//------------BUOC 4: TIEN HANH DIEU KHIEN----------------------------------	
	//----BUOC 4-1: QUI HOACH DIEM DEN-------------------------------------------
	move2NextPoint(drone_in->desiredData.data_PtoP_Control, drone_in);

	//----BUOC 4-2: QUI HOACH QUY DAO-------------------------------------------
	z_TrajectoryPlanning(drone_in->desiredData.data_PtoP_Control, drone_in);
	xy_TrajectoryPlanning(drone_in->desiredData.data_PtoP_Control, drone_in);
	//----BUOC 4-3: TINH TOAN GIA TRI DIEU KHIEN---------------------------------
	//----BUOC 4-3-1: DIEU KHIEN CAP CAO X-Y---------------------------------
	xy_Controller(drone_in);
	//----BUOC 4-3-2: DIEU KHIEN CAP CAO Z-----------------------------------
	z_Controller(drone_in);
	//----BUOC 4-3-3: DIEU KHIEN CAP THAP-----------------------------------
	rpy_Controller(drone_in);

	//----------- BUOC 5: XUAT TIN HIEU DIEU KHIEN RA HE THONG----------------------------------
	//----BUOC 5-1: TINH GIA TRI MOTOR SPEED
	motorSpeed_Caculator(drone_in, 1);
	//----BUOC 5-2: XUAT PWN
	DronePWM.setPWMFromDrone(drone_in, 1);
	

	//----------- BUOC 6: XU LY HE THONG BOM--------------------------------
	PumpDataProcessing(drone_in);


	//----------- BUOC 7: CANH BAO VA QUAN SAT DU LIEU-----------------------------
	digitalWrite(LED_RF_NHAN, (rfResult > RF_NODATA_RECEIVED));
	commandLedDisplay(drone_in);
	GPSLedDisplay(drone_in);//Hien thi LED GPS
	//sendData2HUB(drone_in); //Goi du lieu sang Hub
	sendFlyPointID2Hub(drone_in, rfResult); //Goi du lieu diem bay qua Hub
	// sendFlyPointID2Hub(drone_in, 1);
	sendPumpControlData2Hub(drone_in);//Goi du lieu pump sang hub
}