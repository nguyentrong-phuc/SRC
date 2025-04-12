/*------------------------------------------------------------
  ORC_Drone_PWM.h - Library for manipulting with PWM data.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
  PWM_min = 0; PWM_max = 1800
-------------------------------------------------------------*/

#ifndef ORC_Drone_PWM_h
#define ORC_Drone_PWM_h

#include "Arduino.h"
#include "ORCType.h"

class ORC_Drone_PWM
{
  public:
    ORC_Drone_PWM();
	void PWMSetup();
	void setPWM(unsigned int Pwm1, unsigned int Pwm2, unsigned int Pwm3, unsigned int Pwm4);
	void setPWMFromDrone(orcdrone_data *drone_in, uint16_t activeIn);
	void setPWMMaxMin(unsigned int PWM_min, unsigned int PWM_max);
  private:
    unsigned int _PWM_min, _PWM_max;
	void PwmTimer1Setup(); //CAI DAT TIMMER 1 CHO PWM: OC1A -> D11; OC1B -> D12
	void PwmTimer4Setup(); //CAI DAT TIMMER 4 CHO PWM: OC4A -> D6; OC4B -> D7
	void STMPwmTimer3Setup(); // Channel_1->PA6     CH2->PA7     CH3->PB0      CH4->PB1
};



#endif
