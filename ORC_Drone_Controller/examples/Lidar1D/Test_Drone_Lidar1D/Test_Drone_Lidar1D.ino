#include "Arduino.h"

#define LIDARPort Serial3
HardwareSerial LIDARPort(PB11, PB10);

int16_t Lidar1_strength;
#define LIDAR1D_MAX_UNCHANGE 50  //500ms
#define LIDAR1D_GOOD_STANDRANGE 40  //20cm

int16_t lidar1DValue = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  LIDARPort.begin(115200); LIDARPort.setTimeout(2); 
  Lidar1_strength = 0;
  int attempts = 0;
  const int maxAttempts = 50;  // Giới hạn số lần thử

  while (!Lidar1D_read(&lidar1DValue) && attempts < maxAttempts) {
    Serial.println("Dang khoi dong lidar 1D.");
    delay(20);
    attempts++;
  }

  if (attempts == maxAttempts) {
    Serial.println("Khoi dong lidar 1D that bai.");
    // Thực hiện các bước xử lý lỗi tại đây
  }
}

void loop() {
  // Led Bao Hieu Code Da Duoc Nap
  // digitalWrite(PC13, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(100);                      // wait for a second
  // digitalWrite(PC13, LOW);   // turn the LED off by making the voltage LOW
  // delay(100);  

  delay(10);
  if (Lidar1D_read(&lidar1DValue)) {
    Serial.print("Lidar 1D: ");
    Serial.println(lidar1DValue);
  } 
  // else {
  //   Serial.println("Loi du lieu.");
  // }
}

bool Lidar1D_read(int16_t *distance_out){
  uint8_t       TFbuff[9] = {0};
  long          checksum  = 0 ;
  uint32_t serial1Timeout = micros() + 1000; // 1ms timeout
  int16_t distance_in = 0;
  int16_t strength_in = 0;

  if(LIDARPort.available()){  
    TFbuff[0] = LIDARPort.read();
    TFbuff[1] = LIDARPort.read(); 
        
    while( (TFbuff[0] != 0x59) || (TFbuff[1] != 0x59)){
      TFbuff[0] = TFbuff[1];
      TFbuff[1] = LIDARPort.read();
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
        *distance_out = distance_in - 5;
        Lidar1_strength = strength_in;
        return true;    
      }
      else return false;
      
    }else{
        return false;
    }
  }
}
