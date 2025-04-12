/*------------------------------------------------------------
  ORC_DRONE_LIDAR_S2M1 CLASS
* * Written by Le Duc Tai + Trieu Khanh Thi
* * Modified by Dang Xuan Ba
* Date of version: 2025/01/10
* Hardware: Arduino Due
-------------------------------------------------------------*/

#include "Arduino.h"
#include "ORC_Drone_Lidar_S2M1.h"


/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_Lidar_S2M1::ORC_Drone_Lidar_S2M1()
{
//   _lidarMotorPin = lidarMotorPWMPin;;
	_isSetup = false;
  _minDistance = defaultDistance;  //Cai dat khoang cach defau cho he thong
  _minAngle = defaultAngle;
  _maxControlAngle = OBSTACLE_ANGLE_LIDAR_MAX;

}
//----------------------BEGIN OF Lidar FUNCTIONS--------------------------------------------------------
/*----------------------------------------------------------------------------------------------
 * CAI DAT CHE DO READING CHO LIdar
------------------------------------------------------------------------------------------------*/
bool ORC_Drone_Lidar_S2M1::LidarSetup(lidarDataType *lidarData)
{
      _isSetup = false;
      //Cai dat cac cong com Xuat ra man hinh va goi sang cac thiet bi khac
      Serial.begin(115200); 
      OUTSerial.begin(115200); 
	    delay(10);
      Serial.println("succes setup monitor baudrate!!!");

      // setup Hardware Serial for Lidar
      _lidar.begin(LIDARSerial);

      // Run Recover Mode to check lidar status
      if(!LidarRecoverMode(CountCheckRecoverMode)) 
      {
        return false;
      }

      // begin scaning 
      _lidar.startScan();
      //In du lieu thong bao cai dat thanh cong     
      Serial.println("Init Successfull!!!!!");
      _isSetup = true;

      lidarData->preTime = millis(); //Luu lai thoi gian ghi nhan du lieu
     _preProcessingTime = millis();
      return true;
}


/*----------------------------------------------------------------------------------------------
 * PRINT MINDISTANCE OF POINT IN ONE LOOP SCAN
------------------------------------------------------------------------------------------------*/
void ORC_Drone_Lidar_S2M1::_printData(float distance, float ang){
    Serial.print("Min Distance: ");
    Serial.print(distance);
    Serial.print(" -- Min Angel: ");
    Serial.println(ang);
}
/*----------------------------------------------------------------------------------------------
 * END PRINT
------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------
 * DOC DU LIEU LIDAR
------------------------------------------------------------------------------------------------*/
bool ORC_Drone_Lidar_S2M1::LidarInScanData(lidarDataType *lidarData)
{
  float dist, ang;
  bool result = false;   
  unsigned long currentTime = millis(); //Lay thoi gian hien tai
  if (IS_OK(_lidar.waitPoint())) {
    
    //perform data processing here... 
    dist = _lidar.getCurrentPoint().distance;
    ang = _lidar.getCurrentPoint().angle;  // 0-360 deg


    if (_lidar.getCurrentPoint().startBit) {
      //Print min data of Previos Scanning
      // printMinPoint();
       // a new scan, display the previous data...

      lidarData->lidarDistance = _minDistance;
      lidarData->lidarAngle = _minAngle;

      lidarData->preTime = millis(); //Luu lai thoi gian ghi nhan du lieu
      _minDistance = defaultDistance;
      _minAngle = defaultAngle;
      lidarData->isNew = true;
	  
	  //Serial.println(lidarData->lidarDistance);
      result = true;
    } else {
       if ( dist > 0 &&  dist < _minDistance) { //Lay khoang cach ngan nhat: trong 1 khoang lam viec (>lockDistance)
	   // if ( (dist > lockDistance) &&  (dist < _minDistance)){
          _minDistance = dist;
          _minAngle = ang;
       }
      
    }
  }
  // else {
  //     Serial.println("No Data in waitPoint Queue Begin Scan!!!!!!!!");

  // }
  //Kiem tra timeout
  lidarData->isTimeOut = ((currentTime - lidarData->preTime) > LidarTimeOut);
  return result;
}

/*----------------------------------------------------------------------------------------------
 * BEGIN RECOVER MODE FOR LIDAR S2M1
------------------------------------------------------------------------------------------------*/

bool ORC_Drone_Lidar_S2M1::LidarRecoverMode(uint8_t MaxCheck){

  rplidar_response_device_health_t checkHealthStatus;
  uint8_t CountCheck = MaxCheck;
  while(IS_FAIL(_lidar.getHealth(checkHealthStatus)) && CountCheck-- > 0){
      _lidar.resetLidar();
      delay(1000);
  }
  if(CountCheck <= 0){
    return false;
  }
  return true;
}

/*----------------------------------------------------------------------------------------------
 * END RECOVER MODE 
------------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------
 * HAM XU LY DU LIEU LIDAR DE TAO RA GOC TRANH:
 * Neu vi tri vat can lon hon vi tri cho phep: giam cac goc ve 0
 * Neu vi tri can can nho ho vi tri cho phep thi tang goc tranh len ti le voi khoang cach tranh
 * Qua thoi gian nhan du lieu (timeout) ma khong nhan duoc du lieu: giam cac goc tranh ve khong
------------------------------------------------------------------------------------------------*/
byte ORC_Drone_Lidar_S2M1::LidarDataProcessing(lidarDataType *lidarData)
{
  float ctl_x, ctl_y, ov_roll, ov_pitch, distControl, angControl;
  unsigned long cTime = millis();
  byte result = 0;
  float dt = 0;
  dt = (float)(cTime - _preProcessingTime);
  if(fabs(dt) >= ControlTime){
    _preProcessingTime = cTime; //Luu lai thoi gian moi
    _loopCount++; //Dem vong thoi gian
    if(_loopCount >= ObstacleAvoidanceRawData_Time)
      _loopCount = 0;
    //Xu ly phuong x
    if(lidarData->isTimeOut){ //Truong hop timeout
      lidarData->controlRoll = K_Angle_Timeout*lidarData->controlRoll;
      lidarData->controlPitch = K_Angle_Timeout*lidarData->controlPitch;
      result = 1;
    }else{
      if((lidarData->lidarDistance > freeDistance)||(lidarData->lidarDistance < lockDistance)){ //Gia su quet hinh tron
         lidarData->controlPitch = K_Angle_Free*lidarData->controlPitch; 
         lidarData->controlRoll = K_Angle_Free*lidarData->controlRoll;
         result += 100;
      }else{
          distControl = (freeDistance - lidarData->lidarDistance);
          //angControl = 180 - fmod(lidarData->lidarAngle -45 + 360.0, 360.0); //Chuyen ve he truc toa do Drone
		  angControl = _Uti.yawCompensation(- fmod(lidarData->lidarAngle -45 + 360.0, 360.0)); //Chuyen ve he truc toa do Drone
          //Serial.println(angControl);
          
          ctl_x = distControl*cos(angControl*PI/180);
          ctl_y = distControl*sin(angControl*PI/180);

          ov_pitch = ctl_x*_maxControlAngle/(freeDistance - lockDistance);
          
        //   ov_pitch = (freeDistance - fabs(obstacle_x))*maxAngle/(freeDistance - lockDistance);//*(maxAngle/(freeDistance - lockDistance)); //Tinh goc can tranh (tuyen tinh)
        //   //ov_pitch = ov_pitch*maxAngle/(freeDistance - lockDistance);
        //  Serial.print("obstacle_x: "); Serial.print(obstacle_x);
        //  Serial.print("; PitCon: "); Serial.println(ov_pitch);
          if(ov_pitch > _maxControlAngle) //Gioi han goc can tranh
            ov_pitch = _maxControlAngle;
        //   if(obstacle_x < 0){ //Chon chieu tranh
        //     ov_pitch = -ov_pitch;
        //   }
          //Tinh toan gia tri dieu khien
          lidarData->controlPitch = K_Angle_Control*lidarData->controlPitch + (1-K_Angle_Control)*ov_pitch;
          result += 3;



          ov_roll = -ctl_y*_maxControlAngle/(freeDistance - lockDistance); //Tinh goc can tranh (tuyen tinh)
          if(ov_roll > _maxControlAngle) //Gioi han goc can tranh
            ov_roll = _maxControlAngle;
          lidarData->controlRoll = K_Angle_Control*lidarData->controlRoll + (1-K_Angle_Control)*ov_roll;
          result += 30; 
   
      }
    }
  }
  return result;
}

/*----------------------------------------------------------------------------------------------
 * GOI DU LIEU LIDAR RA CAC HE THONG KHAC
------------------------------------------------------------------------------------------------*/
void ORC_Drone_Lidar_S2M1::LidarSendOutData(lidarDataType *lidarData)
{
  long ang[2] = {0,0};
  //====== Goi du lieu ra UART===============
  if((Serial.availableForWrite()>=63) && lidarData->isNew){
    //In du lieu goc va khoang cac ngan nhat
    Serial.print("dist: ");  Serial.print(lidarData->lidarDistance);
    Serial.print("; angle: ");  Serial.print(lidarData->lidarAngle);
    Serial.print("; OA_roll: ");  Serial.print(lidarData->controlRoll);
    Serial.print("; OA_pitch: ");  Serial.println(lidarData->controlPitch);
    //lidarData->isNew = false;
  }
  //====== Goi du lieu ra OUT port===============
  if(_loopCount == 0){
    // if((OUTSerial.availableForWrite()>=63)&& lidarData->isNew){
	if((OUTSerial.availableForWrite()>=63)){
      ang[0] = (long) (lidarData->lidarDistance);
      ang[1] = (long) (lidarData->lidarAngle);
      //In du lieu goc va khoang cac ngan nhat
      OUTSerial.print(_Uti.Lidar2DRawDataPrefix); OUTSerial.print(" ");
      OUTSerial.print(ang[0]); OUTSerial.print(",");
      OUTSerial.print(ang[1]);OUTSerial.print(",");
      OUTSerial.print(_Uti.ChecksumOfLongArray(ang, 2));
      OUTSerial.println("#");
      lidarData->isNew = false;
    }
  }else if(_loopCount == ObstacleAvoidanceControlAngle_Time){ //Goi du lieu goc dieu khien qua he thong chinh
    // if((OUTSerial.availableForWrite()>=63)&& lidarData->isNew){ 
	if((OUTSerial.availableForWrite()>=63)){ 
      ang[0] = (long) (lidarData->controlRoll*10);
      ang[1] = (long) (lidarData->controlPitch*10);

      if((ang[0] == 0)&&(ang[1] == 0)){  //Truong hop khong co vat can, chi goi 1 lan duy nhat
        if(!isZeroSent){
          isZeroSent = true;
          OUTSerial.print(_Uti.Lidar2DControlDataPrefix); OUTSerial.println(" 0,0,0#");
          // OUTSerial.print(ang[0]); OUTSerial.print(",");
          // OUTSerial.print(ang[1]);OUTSerial.print(",");
          // OUTSerial.print(_Uti.ChecksumOfLongArray(ang, 2));
          // OUTSerial.println("#");
          lidarData->isNew = false;
        }

      }else{
        isZeroSent = false;
        OUTSerial.print(_Uti.Lidar2DControlDataPrefix); OUTSerial.print(" ");
        OUTSerial.print(ang[0]); OUTSerial.print(",");
        OUTSerial.print(ang[1]);OUTSerial.print(",");
        OUTSerial.print(_Uti.ChecksumOfLongArray(ang, 2));
        OUTSerial.println("#");
        lidarData->isNew = false;
      }
      
      
    }
  }
  
}
/*---------------------------------------------------------------------------
 * Chuong trinh chinh hoat dong cua he thong
--------------------------------------------------------------------------------*/

void ORC_Drone_Lidar_S2M1::runLoop(lidarDataType *lidarData){
//  //Doc du lieu tu lidar
//   if(LidarInScanData(lidarData)){
//     _printData(lidarData->lidarDistance,lidarData->lidarAngle);
//   } else {
//     // Serial.println("Fail Collect Data!!!");
//     ;
//   }
    byte processingStatue = 0;
    //Doc du lieu tu lidar
    LidarInScanData(lidarData);

    //Xu ly du lieu
    processingStatue = LidarDataProcessing(lidarData);
    // if(processingStatue > 0)
    //   Serial.println(processingStatue);

    //Goi cac du lieu can thiet ra ngoai
    LidarSendOutData(lidarData);
  
}
//----------------------END OF RF FUNCTIONS--------------------------------------------------------




