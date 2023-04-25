#pragma once
#ifndef SPP_H
#define Spp_H

#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include"CONST.h"
#include"Time.h"
#include"Coor.h"
#include"Matrix.h"
#include"Vector.h"
#include"Decode.h"
#include"SatellitePostion.h"

//单点定位结构体
struct rover
{
	GPST               gt;
	XYZ                _XYZpostion;
	XYZ                _deltax;
	BLH                _BLHpostion;
	double             Velocity[3];
	double             RcvClkOft[2];        //0:GSP钟差；1:BDS钟差
	double             RcvClkSft;           //钟速
	double             PDOP,HDOP,VDOP, SigmaPos, SigmaVel;
	double             mxyz[3];
	short              GPSSatNum, BDSSatNum;
	short              AllSatNum;
	bool               IsSuccess;//单点定位是否成功
	rover()
	{
		_deltax.x = 1;
		for (int i = 0; i < 3; i++)
		{
			Velocity[i] = 0.0;
			mxyz[i] = 0.0;
			RcvClkOft[0] = RcvClkOft[1] = RcvClkSft = 0.0;
			PDOP = HDOP = VDOP = SigmaPos = SigmaVel = 999.9;
			GPSSatNum = BDSSatNum = AllSatNum = 0;
			IsSuccess = false;
		}
	}
};

//粗差探测及观测值线性组合
void _LinearCombination(OBS& obs);
//Hopefield模型计算对流层误差
double _Hopefield(double& H, double& E);
//卫星高度角
//double _El(const XYZ& x, BLH& b, double a, double alpha);
double _GetElevationAngle(const XYZ& x, BLH& b, double a, double alpha);
//数组插值
void ArrayInsert(double* array,int n,double value,int No);

void SPP_LS(OBS& obs, EPH* eph, rover& rover);
bool SPP(OBS& obs, EPH* eph, rover& rover);
void SPV(OBS& obs, EPH* eph, rover& rover);


#endif // !SPP_H