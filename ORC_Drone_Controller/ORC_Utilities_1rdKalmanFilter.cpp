/*------------------------------------------------------------
  ORC_Utilities_1rdKalmanFilter.h - Library for manipulting with PWM data.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
  PWM_min = 0; PWM_max = 1800
-------------------------------------------------------------*/

#include "Arduino.h"
#include "ORC_Utilities_1rdKalmanFilter.h"

/*-------------------------------
* Ham khoi tao
--------------------------------*/
ORC_Utilities_1rdKalmanFilter::ORC_Utilities_1rdKalmanFilter(float p, float q, float r)
{
  //Khoi tao cac he so hoc
  _p = p; 
  _q = q;
  _r = r;
  _xPre = 0;
}

/*------------------
 * BO LOC KALMAN BAC 1, CO THE LOC CHO CAC TIN HIEU vantoc banh xe, gia toc dai, van toc goc
 * x_pre: ngo vao trang thai
 * x_meas: ngo ra do luong duoc (vi du, van toc banh xe do dc
 * q_in: sai so mo hinh
 * r_in: phuong sai do luong ngo ra (tu dieu chinh bang tay)
 * *P_in: bien toan cuc di theo moi gia tri do
--------------------*/
float ORC_Utilities_1rdKalmanFilter::update1Value(float measearedValue){
    //Mô hình: xk_1 = xk + Q; z = xk_1 + R
    float xk, k;
    //Dự báo
    xk = _xPre;
    _p = _p + _q;

    //Hiệu chỉnh (correction)
    k = _p /(_p + _r);
    xk = xk + k*(measearedValue - xk);
    _p = (1 - k)*_p;
	_xPre = xk;
    //xuất ngõ ra    
    return xk;
}
/*------------------
 * BO LOC KALMAN BAC 1 su dung 2 gia tri cap nhat, CO THE LOC CHO CAC TIN HIEU roll pitch yaw
 * x_pre: ngo vao trang thai
 * x_meas: ngo ra do luong duoc (vi du, van toc banh xe do dc
 * q_in: sai so mo hinh
 * r_in: phuong sai do luong ngo ra (tu dieu chinh bang tay)
 * *P_in: bien toan cuc di theo moi gia tri do
--------------------*/
float ORC_Utilities_1rdKalmanFilter::update2Value(float vk, float measearedValue, float dt){
    //Mô hình: xk_1 = xk + v*dt + Q; z = xk_1 + R
    float k;
    //Dự báo
    _xPre = _xPre + vk*dt;
    _p = _p + _q;

    //Hiệu chỉnh (correction)
    k = _p /(_p + _r);
    _xPre = _xPre + k*(measearedValue - _xPre);
    _p = (1 - k)*_p;
	
    //xuất ngõ ra    
    return _xPre;
}


