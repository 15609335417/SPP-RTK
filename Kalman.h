#pragma once
#ifndef Kalman_H

#define kalman_H

#include<stdio.h>
#include"RTK.h"


struct _Kalman
{
	//��ǰʱ��
	double X[MaxSatNum * 2 + 3];                                      //״̬��
	int dim;                                                          //״̬��ά��
	int PRNLable[MaxSatNum][4];                                       //����PRN�ű�ǩ ϵͳ �ο��� ������ Ƶ�ʱ�ǩ
	int RefPrn[2];                                                    //�ο���PRN
	int PRN[MaxSatNum];                                               //������������PRN
	NAVSYS system[MaxSatNum];                                         //������������ϵͳ��ʶ
	double XCov[(MaxSatNum * 2 + 3) * (MaxSatNum * 2 + 3)];           //״̬����Э����

	//״̬ת�ƾ���
	double phi[(MaxSatNum * 2 + 3) * (MaxSatNum * 2 + 3)];            //״̬ת�ƾ���

	//��һʱ��
	double _X[MaxSatNum * 2 + 3];                                     //״̬��
	int _dim;                                                         //״̬��ά��
	int _PRNLable[MaxSatNum][4];                                      //����PRN�ű�ǩ
	int _RefPrn[2];                                                   //�ο���PRN
	int _PRN[MaxSatNum];                                              //������������PRN
	NAVSYS _system[MaxSatNum];                                        //������������ϵͳ��ʶ
	double _XCov[(MaxSatNum * 2 + 3) * (MaxSatNum * 2 + 3)];          //״̬����Э����

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
//״̬ת�ƾ���
Matrix _GetPhi(int _dim, int dim, SDEPOCHOBS& SdObs, DDCOBS& ddcobs, _Kalman& kalman);

#endif // !Kalman_H

