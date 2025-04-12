/*----------------------------------------------
* CHUONG TRINH TEST PWM CHO CAC DONG CO
* PWM_MIN = 100; PWM_MAX = 1800;
* Written by Dang Xuan Ba <dang.xuanba@gmail.com>
* Date of version: 2024/10/07
* Hardware: main module
-------------------------------------------------*/
#include <ORC_Drone_Controller.h>

//Khai bao bien DroneController
ORC_Drone_Controller DroneController(1,2);
unsigned int pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0;

void setup()
{
  //Cai dat PWM
  DroneController.DronePWM.PWMSetup();
}

void loop()
{
  
  delay(100);
  // D11 -PWM1; D12-PWM2; D6-PWM3; D7-PWM4 (THU TU CANH CUNG CHIEU KIM DONG HO
  // LUU Y VAN TOC CAC CANH THAY DOI THEO GIA TRI CAP VAO LA CHINH XAC
  DroneController.DronePWM.setPWM(1000, 1000, 1000, 100); 
}