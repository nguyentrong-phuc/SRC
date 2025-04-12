/*------------------------------------------------------------
  ORC_Drone_2DLidar.h - Library for manipulting with 2D Lidar.
  Created by Dang Xuan Ba, Dec 08, 2024.
  Updated by Dang Xuan Ba, Dec 08, 2024.
  Released into the public domain.
-------------------------------------------------------------*/

#include "Arduino.h"
#include "ORC_Drone_2DLidar.h"
#include "ORCType.h"

/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_2DLidar::ORC_Drone_2DLidar(int lidarMotorPWMPin)
{
  _lidarMotorPin = lidarMotorPWMPin;
}
//----------------------BEGIN OF Lidar FUNCTIONS--------------------------------------------------------
/*----------------------------------------------------------------------------------------------
 * CAI DAT CHE DO READING CHO LIdar
------------------------------------------------------------------------------------------------*/
bool ORC_Drone_2DLidar::LidarSetup(lidarDataType *lidarData)
{
  Serial.begin(115200); Serial.setTimeout(2); 
  LIDARSerial.begin(115200);  LIDARSerial.setTimeout(2); // For RPLidar
  OUTSerial.begin(115200);  OUTSerial.setTimeout(2); // For RPLidar
  _lidar.begin(LIDARSerial);
  pinMode(_lidarMotorPin, OUTPUT);  // set pin modes
  lidarData->preTime = millis(); //Luu lai thoi gian ghi nhan du lieu
  _preProcessingTime = millis();
}
/*----------------------------------------------------------------------------------------------
 * DOC DU LIEU LIDAR
------------------------------------------------------------------------------------------------*/
bool ORC_Drone_2DLidar::LidarInScanData(lidarDataType *lidarData)
{
  float dist, ang;
  bool result = false; 
  unsigned long currentTime = millis();
  if (IS_OK(_lidar.waitPoint())) {
    
    //perform data processing here... 
    dist = _lidar.getCurrentPoint().distance;
    ang = _lidar.getCurrentPoint().angle;  // 0-360 deg
    
    if (_lidar.getCurrentPoint().startBit) {
       // a new scan, display the previous data...
       lidarData->lidarDistance = _minDistance;
        lidarData->lidarAngle = _minAngle;

       //printData(angleAtMinDist, minDistance);
       lidarData->preTime = millis(); //Luu lai thoi gian ghi nhan du lieu
       _minDistance = defaultDistance;
       _minAngle = defaultAngle;
       lidarData->isNew = true;
       result = true;
    } else {
       if ( dist > 0 &&  dist < _minDistance) { //Lay khoang cach ngan nhat
          _minDistance = dist;
          _minAngle = ang;
       }
    }
  }
  else {
    analogWrite(_lidarMotorPin, 0); //stop the rplidar motor
    // Try to detect RPLIDAR
    //rplidar_response_device_info_t info;
    if (IS_OK(_lidar.getDeviceInfo(_info, 100))) {
      // Serial.print("OK");
       // Detected
       _lidar.startScan();
       analogWrite(_lidarMotorPin, 255);
       delay(1000);
    }
  }
  //Kiem tra timeout
  lidarData->isTimeOut = ((currentTime - lidarData->preTime) > LidarTimeOut);
  return result;
}

/*----------------------------------------------------------------------------------------------
 * HAM XU LY DU LIEU LIDAR DE TAO RA GOC TRANH:
 * Neu vi tri vat can lon hon vi tri cho phep: giam cac goc ve 0
 * Neu vi tri can can nho ho vi tri cho phep thi tang goc tranh len ti le voi khoang cach tranh
 * Qua thoi gian nhan du lieu (timeout) ma khong nhan duoc du lieu: giam cac goc tranh ve khong
------------------------------------------------------------------------------------------------*/
byte ORC_Drone_2DLidar::LidarDataProcessing(lidarDataType *lidarData)
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
    //obstacle_x = lidarData->lidarDistance*cos(lidarData->lidarAngle*PI/180);
    //obstacle_y = lidarData->lidarDistance*sin(-lidarData->lidarAngle*PI/180);
    //Xu ly phuong x
    if(lidarData->isTimeOut){ //Truong hop timeout
      lidarData->controlRoll = K_Angle_Timeout*lidarData->controlRoll;
      lidarData->controlPitch = K_Angle_Timeout*lidarData->controlPitch;
      result = 1;
    }else{
      if(lidarData->lidarDistance > freeDistance){ //Gia su quet hinh tron
         lidarData->controlPitch = K_Angle_Free*lidarData->controlPitch; 
         lidarData->controlRoll = K_Angle_Free*lidarData->controlRoll;
         result += 100;
      }else{
          distControl = (freeDistance - lidarData->lidarDistance);
          angControl = 180-lidarData->lidarAngle;
          ctl_x = distControl*cos(angControl*PI/180);
          ctl_y = distControl*sin(angControl*PI/180);

          ov_pitch = ctl_x*maxAngle/(freeDistance - lockDistance);
          
        //   ov_pitch = (freeDistance - fabs(obstacle_x))*maxAngle/(freeDistance - lockDistance);//*(maxAngle/(freeDistance - lockDistance)); //Tinh goc can tranh (tuyen tinh)
        //   //ov_pitch = ov_pitch*maxAngle/(freeDistance - lockDistance);
        //  Serial.print("obstacle_x: "); Serial.print(obstacle_x);
        //  Serial.print("; PitCon: "); Serial.println(ov_pitch);
          if(ov_pitch > maxAngle) //Gioi han goc can tranh
            ov_pitch = maxAngle;
        //   if(obstacle_x < 0){ //Chon chieu tranh
        //     ov_pitch = -ov_pitch;
        //   }
          //Tinh toan gia tri dieu khien
          lidarData->controlPitch = K_Angle_Control*lidarData->controlPitch + (1-K_Angle_Control)*ov_pitch;
          result += 3;



          ov_roll = -ctl_y*maxAngle/(freeDistance - lockDistance); //Tinh goc can tranh (tuyen tinh)
          if(ov_roll > maxAngle) //Gioi han goc can tranh
            ov_roll = maxAngle;
          lidarData->controlRoll = K_Angle_Control*lidarData->controlRoll + (1-K_Angle_Control)*ov_roll;
          result += 30; 

        // //Xu ly phuong y
        // if(fabs(obstacle_y) > freeDistance){ //Vat can nam ngoai khoang cach cho phep --> giam goc ve khong
        //   lidarData->controlRoll = K_Angle_Free*lidarData->controlRoll;
        //   result += 20;
        // }else // Tang tu tu goc tranh
        // {
        //   ov_roll = (freeDistance - fabs(obstacle_y))*maxAngle/(freeDistance - lockDistance); //Tinh goc can tranh (tuyen tinh)
        //   if(ov_roll > maxAngle) //Gioi han goc can tranh
        //     ov_roll = maxAngle;
        //   if(obstacle_y > 0){ //Chon chieu tranh
        //     ov_roll = -ov_roll;
        //   }
        //   //Tinh toan gia tri dieu khien
        //   lidarData->controlRoll = K_Angle_Control*lidarData->controlRoll + (1-K_Angle_Control)*ov_roll;
        //   result += 30;
        // }
      }
    }
  }
  return result;
}
/*----------------------------------------------------------------------------------------------
 * GOI DU LIEU LIDAR RA CAC HE THONG KHAC
------------------------------------------------------------------------------------------------*/
void ORC_Drone_2DLidar::LidarSendOutData(lidarDataType *lidarData)
{
  long ang[2] = {0,0};
  //====== Goi du lieu ra UART===============
  if((Serial.availableForWrite()>=63) && lidarData->isNew){
    //In du lieu goc va khoang cac ngan nhat
    Serial.print("dist: ");  Serial.print(lidarData->lidarDistance);
    Serial.print("; angle: ");  Serial.print(lidarData->lidarAngle);
    Serial.print("; OA_roll: ");  Serial.print(lidarData->controlRoll);
    Serial.print("; OA_pitch: ");  Serial.println(lidarData->controlPitch);
    lidarData->isNew = false;
  }
  //====== Goi du lieu ra OUT port===============
  if(_loopCount == 0){
    if((OUTSerial.availableForWrite()>=63)&& lidarData->isNew){
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
    if((OUTSerial.availableForWrite()>=63)&& lidarData->isNew){ 
      ang[0] = (long) (lidarData->controlRoll*10);
      ang[1] = (long) (lidarData->controlPitch*10);

      if((ang[0] == 0)&&(ang[1] == 0)){  //Truong hop khong co vat can, chi goi 1 lan duy nhat
        if(!isZeroSent){
          isZeroSent = true;
          OUTSerial.print(_Uti.Lidar2DControlDataPrefix); OUTSerial.print(" ");
          OUTSerial.print(ang[0]); OUTSerial.print(",");
          OUTSerial.print(ang[1]);OUTSerial.print(",");
          OUTSerial.print(_Uti.ChecksumOfLongArray(ang, 2));
          OUTSerial.println("#");
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
void ORC_Drone_2DLidar::runLoop(lidarDataType *lidarData){
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




