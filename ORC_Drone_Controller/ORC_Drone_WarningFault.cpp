/*------------------------------------------------------------
  ORC_Drone_WarningFault.h - Library for Warning and Fault.
  Created by Dang Xuan Ba, Nov 28, 2024.
  Updated by Dang Xuan Ba, Nov 28, 2024.
  Released into the public domain.
-------------------------------------------------------------*/

#include "Arduino.h"
#include "ORC_Drone_WarningFault.h"
#include "ORCType.h"

/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_WarningFault::ORC_Drone_WarningFault()
{
  test = 0;
}
//----------------------BEGIN OF RF FUNCTIONS--------------------------------------------------------
/*----------------------------------------------------------------------------------------------
 * KIEM TRA LOI HE THONG
------------------------------------------------------------------------------------------------*/
bool ORC_Drone_WarningFault::WFCheckandSetError(orcdrone_data *droneData)
{
	bool result = true;
	/*------------------KIEM TRA LOI GPS----------------*/
	//Kiem tra loi mat GPS
	if (droneData->gpsMode && (droneData->gpsdata.readErrCnt >= GPS_TIME_OUT)){
		droneData->WarningFault = droneData->WarningFault | WF_Error_GPS_Timeout; //Set bit 1 len 1
	}else
		droneData->WarningFault = droneData->WarningFault & (~ WF_Error_GPS_Timeout); //Reset bit 1 xuong 0
	
	//Kiem tra loi chat luong GPS: nosolutionCnt
	//if(droneData->gpsMode && (droneData->gpsdata.pos_cm[3] == GPS_RTK_NOSOLUTION)){
	if(droneData->gpsMode && (droneData->gpsdata.nosolutionCnt >= GPS_RTK_NOSOLUTION_CNT_MAX)){
		droneData->WarningFault = droneData->WarningFault | WF_Error_GPS_NoSolution;
	}else droneData->WarningFault = droneData->WarningFault & (~ WF_Error_GPS_NoSolution);
	
	//Kiem tra loi mat tin hieu RF
	if(droneData->desiredData.RFErrorCnt >= MAX_RF_ERROR){
		droneData->WarningFault = droneData->WarningFault | WF_Error_RF_Timeout;
	}else droneData->WarningFault = droneData->WarningFault & (~ WF_Error_RF_Timeout);
	
	//Kiem tra loi tang offset roll
	if(fabs(droneData->ebimu_data.rollOffset) >= EBI_RollOffset_Thres){
		droneData->WarningFault = droneData->WarningFault | WF_Warning_Atti_RollOffset;
	}else droneData->WarningFault = droneData->WarningFault & (~ WF_Warning_Atti_RollOffset);
	
	//Kiem tra loi tang offset pitch
	if(fabs(droneData->ebimu_data.pitchOffset) >= EBI_PitchOffset_Thres){
		droneData->WarningFault = droneData->WarningFault | WF_Warning_Atti_PitchOffset;
	}else droneData->WarningFault = droneData->WarningFault & (~ WF_Warning_Atti_PitchOffset);
	
	//Kiem tra loi tang Ts
	if(droneData->Ts >= WF_Warning_Ts_LargeThres){
		droneData->WarningFault = droneData->WarningFault | WF_Warning_Ts_Large;
	}else droneData->WarningFault = droneData->WarningFault & (~ WF_Warning_Ts_Large);
	
	return result;
}

/*------------------------------------------
* Kiem tra loi mong muon tim
-------------------------------------------*/
bool ORC_Drone_WarningFault::isWFRequiredError(long in, long maskError){
	return ((in & maskError) != 0);
}




//----------------------END OF RF FUNCTIONS--------------------------------------------------------




