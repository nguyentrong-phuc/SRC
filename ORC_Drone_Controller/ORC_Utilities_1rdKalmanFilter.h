/*
  ORC_Utilities_1rdKalmanFilter.h - Library for process 1st order Kalman filter.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#ifndef ORC_Utilities_1rdKalmanFilter_h
#define ORC_Utilities_1rdKalmanFilter_h

#include "Arduino.h"

class ORC_Utilities_1rdKalmanFilter
{
  public:
    ORC_Utilities_1rdKalmanFilter(float p, float q, float r);
	float update1Value(float measearedValue);
	float update2Value(float vk, float measearedValue, float dt);
	float _r = 1;
  private:
	float _p = 1, _q = 1, _xPre = 0;
};

#endif