#define TEST 0

#if defined(TEST) && (TEST == 1)
  #include <ORC_Drone_PWM.h>
  //Khai bao bien DroneController
  ORC_Drone_PWM DronePWM;
  unsigned int pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0;
  int tempValue, input_dutyCycle = 0;

  void setup()
  {
    //Cai dat PWM
    Serial.begin(9600);
    Serial.setTimeout(2);
    DronePWM.PWMSetup();
    /*THROTTLE CALIBRATION*/
    // DronePWM.setPWM(1800,1800,1800,1800);
    // delay(5000);
    // DronePWM.setPWM(0,0,0,0);
    // delay(2000);
  }

  void loop()
  {
    if(Serial.available() > 0){
      tempValue = Serial.parseInt();
      if (tempValue>0)
      {
        input_dutyCycle = tempValue;
        Serial.print("getNewValue at 1: ");
      }
    }
    Serial.println(input_dutyCycle);
    delay(100);
    // D11 -PWM1; D12-PWM2; D6-PWM3; D7-PWM4 (THU TU CANH CUNG CHIEU KIM DONG HO
    // LUU Y VAN TOC CAC CANH THAY DOI THEO GIA TRI CAP VAO LA CHINH XAC
    DronePWM.setPWM(1*(pwm1 + input_dutyCycle), 1*(pwm2 + input_dutyCycle), 1*(pwm3 + input_dutyCycle), 1*(pwm4 + input_dutyCycle)); 
  }

#else
  #include <ORC_Drone_Controller.h>
  //Khai bao bien DroneController
  ORC_Drone_Controller DroneController(1,2);
  unsigned int pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0;
  int tempValue, input_dutyCycle = 0;
  void setup()
  {
    //Cai dat PWM
    Serial.begin(9600);
    Serial.setTimeout(2);
    //Cai dat PWM
    DroneController.DronePWM.PWMSetup();
    /*THROTTLE CALIBRATION*/
    // DroneController.DronePWM.setPWM(1800,1800,1800,1800);
    // delay(5000);
    // DroneController.DronePWM.setPWM(0,0,0,0);
    // delay(2000);
  }
  void loop()
  {
    if(Serial.available() > 0){
      tempValue = Serial.parseInt();
      if (tempValue>0)
      {
        input_dutyCycle = tempValue;
        Serial.print("getNewValue at 0: ");
      }
    }
    Serial.println(input_dutyCycle);
    delay(100);
    // D11 -PWM1; D12-PWM2; D6-PWM3; D7-PWM4 (THU TU CANH CUNG CHIEU KIM DONG HO
    // LUU Y VAN TOC CAC CANH THAY DOI THEO GIA TRI CAP VAO LA CHINH XAC
    DroneController.DronePWM.setPWM(1*(pwm1 + input_dutyCycle), 1*(pwm2 + input_dutyCycle), 1*(pwm3 + input_dutyCycle), 1*(pwm4 + input_dutyCycle)); 
  }

#endif
