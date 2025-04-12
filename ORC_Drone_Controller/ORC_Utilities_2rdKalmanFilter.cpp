/*------------------------------------------------------------
  ORC_Utilities_2rdKalmanFilter.h - Library Library for process 2nd order Kalman filter.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
-------------------------------------------------------------*/

#include "Arduino.h"
#include "ORC_Utilities_2rdKalmanFilter.h"

/*-------------------------------
* Ham khoi tao
--------------------------------*/
ORC_Utilities_2rdKalmanFilter::ORC_Utilities_2rdKalmanFilter(float *p, float *q, float r)
{
  //Khoi tao cac he so hoc
  for (int i =0; i <4;i++){
	_p[i] = p[i];
	_q[i] = q[i];
  }
  _r = r;
}

/*------------------
 * BO LOC KALMAN BAC 2, CO THE LOC CHO CAC TIN HIEU VAN TOC VA VI TRI KHI DO DUOC gia toc va vi tri
 * MO HINH: 
 * vk_1 = vk + dt*ak + q1
 * xk_1 = xk + dt*vk + q2
 * zk = xk + r
 * vector trang thai: XK = [vk; xk];
 * ma tran trang thai phik = [1 0; dt 1]; bk = [dt*ak; 0] => Xk_1 = phik*Xk + bk + q
 * ma tran do loi dau ra: hk = [0 1]; =>zk = hk*Xk + r
 * MA TRAN phik 
 * x_pre: ngo vao trang thai
 * x_meas: ngo ra do luong duoc (vi du, van toc banh xe do dc
 * q_in: sai so mo hinh
 * r_in: phuong sai do luong ngo ra (tu dieu chinh bang tay)
 * *P_in: bien toan cuc di theo moi gia tri do
--------------------*/
float ORC_Utilities_2rdKalmanFilter::updateValue(float ak, float measX, float dt){
  float p11 = _p[0], p12 =_p[1], p21 = _p[2], p22 = _p[3];
  //Model: xk_1 = xk + Q; z = xk_1 + R 
  float vk = _xPre[0], xk = _xPre[1], q11 = _q[0], q12 = _q[1], q21 = _q[2], q22 = _q[3];
  float k1, k2;
  //Du bao
  xk = xk + dt*vk; vk = vk + dt*ak;
  p22 = p22 + p12*dt + dt*(p21 + p11*dt) + q22;
  p21 = p21 + p11*dt + q21;
  p12 = p12 + dt*p11 + q12;
  p11 = p11 + q11;

  //Hieu chinh (correction)
  k1 = p12/(p22 + _r);
  k2 = p22/(p22+_r);

  vk = vk + k1*(measX - xk);
  xk = xk + k2*(measX - xk);

  p11 = p11 - p21*k1;
  p12 = p12 - p22*k1;
  p21 = (1-k2)*p21;
  p22 = (1-k2)*p22;
  

  //xuat ngo ra
  _p[0] = p11; _p[1] = p12;_p[2] = p21;_p[3] = p22;
  _xPre[0] = vk; _xPre[1] = xk; 
  return xk;
}


