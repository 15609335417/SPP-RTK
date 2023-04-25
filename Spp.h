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

//���㶨λ�ṹ��
struct rover
{
	GPST               gt;
	XYZ                _XYZpostion;
	XYZ                _deltax;
	BLH                _BLHpostion;
	double             Velocity[3];
	double             RcvClkOft[2];        //0:GSP�Ӳ1:BDS�Ӳ�
	double             RcvClkSft;           //����
	double             PDOP,HDOP,VDOP, SigmaPos, SigmaVel;
	double             mxyz[3];
	short              GPSSatNum, BDSSatNum;
	short              AllSatNum;
	bool               IsSuccess;//���㶨λ�Ƿ�ɹ�
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

//�ֲ�̽�⼰�۲�ֵ�������
void _LinearCombination(OBS& obs);
//Hopefieldģ�ͼ�����������
double _Hopefield(double& H, double& E);
//���Ǹ߶Ƚ�
//double _El(const XYZ& x, BLH& b, double a, double alpha);
double _GetElevationAngle(const XYZ& x, BLH& b, double a, double alpha);
//�����ֵ
void ArrayInsert(double* array,int n,double value,int No);

void SPP_LS(OBS& obs, EPH* eph, rover& rover);
bool SPP(OBS& obs, EPH* eph, rover& rover);
void SPV(OBS& obs, EPH* eph, rover& rover);


#endif // !SPP_H