#pragma once
#ifndef Kalman_H

#define kalman_H

#include<stdio.h>
#include"RTK.h"


struct _Kalman
{
	//当前时刻
	double X[MaxSatNum * 2 + 3];                                      //状态量
	int dim;                                                          //状态量维度
	int PRNLable[MaxSatNum][4];                                       //卫星PRN号标签 系统 参考星 解算星 频率标签
	int RefPrn[2];                                                    //参考星PRN
	int PRN[MaxSatNum];                                               //参与解算的卫星PRN
	NAVSYS system[MaxSatNum];                                         //参与解算的卫星系统标识
	double XCov[(MaxSatNum * 2 + 3) * (MaxSatNum * 2 + 3)];           //状态向量协方差

	//状态转移矩阵
	double phi[(MaxSatNum * 2 + 3) * (MaxSatNum * 2 + 3)];            //状态转移矩阵

	//上一时刻
	double _X[MaxSatNum * 2 + 3];                                     //状态量
	int _dim;                                                         //状态量维度
	int _PRNLable[MaxSatNum][4];                                      //卫星PRN号标签
	int _RefPrn[2];                                                   //参考星PRN
	int _PRN[MaxSatNum];                                              //参与解算的卫星PRN
	NAVSYS _system[MaxSatNum];                                        //参与解算的卫星系统标识
	double _XCov[(MaxSatNum * 2 + 3) * (MaxSatNum * 2 + 3)];          //状态向量协方差

	_Kalman()
	{
		dim = _dim = 0;
		RefPrn[0] = RefPrn[1] = _RefPrn[0] = _RefPrn[1] = 0;
		for (int i = 0; i < MaxSatNum; i++)
		{
			for (int j = 1; j < 4; j++)
			{
				PRNLable[i][j] = 0;
				_PRNLable[i][j] = 0;
			}
			PRNLable[i][0] = 7;
			_PRNLable[i][0] = 7;
			PRN[i] = _PRN[i] = 0;
			system[i] = Other;
			_system[i] = Other;
		}
		for (int i = 0; i < MaxSatNum * 2 + 3; i++)
		{
			X[i] = _X[i] = 0.0;

		}
		for (int i = 0; i < (MaxSatNum * 2 + 3) * (MaxSatNum * 2 + 3); i++)
		{
			XCov[i] = _XCov[i] = 0.0;
			phi[i] = 0.0;
		}
	}
};

//
bool RTKFloatKalamn(OBS& baseObs, OBS& roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, _Kalman& kalman);
//状态转移矩阵
Matrix _GetPhi(int _dim, int dim, SDEPOCHOBS& SdObs, DDCOBS& ddcobs, _Kalman& kalman);

#endif // !Kalman_H

