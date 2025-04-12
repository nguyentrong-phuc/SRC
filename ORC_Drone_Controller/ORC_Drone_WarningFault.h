/*------------------------------------------------------------
  ORC_Drone_WarningFault.h - Library for Warning and Fault.
  Created by Dang Xuan Ba, Nov 28, 2024.
  Updated by Dang Xuan Ba, Nov 28, 2024.
  Released into the public domain.
-------------------------------------------------------------*/

#ifndef ORC_Drone_WarningFault_h
#define ORC_Drone_WarningFault_h

#include "Arduino.h"
#include "ORCType.h"

class ORC_Drone_WarningFault
{
  public:
    ORC_Drone_WarningFault(); //Khao khoi tao	
	bool WFCheckandSetError(orcdrone_data *droneData); //Kiem tra canh bao va loi he thong
	bool isWFRequiredError(long in, long maskError);

  private:
    unsigned int test;	
};



#endif