/*----------------------------------------------
  CHUONG TRINH TEST ATTITUDE COMPUTATION
  DOC CAC GIA TRI ACC va GYRO
  Written by Dang Xuan Ba <dang.xuanba@gmail.com>
  Date of version: 2024/10/07
  -------------------------------------------------*/
#include <ORC_Drone_Attitude.h>
#include <ORC_Utilities.h>
#include "Arduino.h"

ORC_Drone_Attitude DroneAtti(100, 0, 0, 0); //Khoi tao bien voi cac gia tri offset la 0
ORC_Utilities ORCUti;

long currentTime = micros();
int cnt = 0;
float Ts = 2; //2 ms
void setup()
{
  //Cai dat PWM
  Serial.begin(115200);
  //Serial.println("Bat dau setup cac sensor can thiet.");
  DroneAtti.AttitudeFromSensorSetup();
  //Serial.println("Da setup xong setup cac sensor can thiet.");
}

void loop()
{
  float et = ORCUti.dynamicWaiting(Ts, &currentTime); //in sec
  cnt++;
  if (cnt > 5)
    cnt = 0;
  if (DroneAtti.AttitudeFromSensorGetData(et*0.001)) {
    
    if (cnt == 0) {
      if (Serial.availableForWrite() > 0)  {
        Serial.print("et: "); Serial.print(et);      
        Serial.print("; roll: "); Serial.println(DroneAtti.croll);
        Serial.print("; pit: "); Serial.print(DroneAtti.cpitch);
        Serial.print("; yaw: "); Serial.println(DroneAtti.cyaw);
      }
    }
  }
  else
  {
    Serial.println("-------------------------------------------------LOI DOC DU LIEU--------------------------");
  }
}
