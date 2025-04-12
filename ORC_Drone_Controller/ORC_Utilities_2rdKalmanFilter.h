/*
  ORC_Utilities_2rdKalmanFilter.h - Library for process 2nd order Kalman filter.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
*/

#ifndef ORC_Utilities_2rdKalmanFilter_h
#define ORC_Utilities_2rdKalmanFilter_h

#include "Arduino.h"

class ORC_Utilities_2rdKalmanFilter
{
  public:
    ORC_Utilities_2rdKalmanFilter(float *p, float *q, float r);
	float updateValue(float ak, float measX, float dt);


  private:
	float _p[4] = {1,0,0,1}, _q[4] = {1,0,0,1}, _r = 1, _xPre[2] = {0,0};
	//float _p11 = 1, _p12 = 0, _p21 = 0, _p22 = 1, _q11 = 0.1, _q12 = 0, _q21 = 0, _q22 = 0.1, _vk = 0, _xk = 0;
};

#endif