#pragma once
#ifndef RTK_H

#define RTK_H

#include<stdio.h>
#include"Decode.h"
#include"Spp.h"

#include"Serial.h"
#include"sockets.h"
#include"lambda.h"


//ÿ�����ǵĵ���۲�ֵ����
struct SDSATOBS
{
	unsigned short      PRN;            //PRN
	NAVSYS              system;
	short               Valid;          //��Ч�ԣ�-1����Ч��0����˫Ƶ�۲�ֵ��1��ͨ������̽�⣻2����������3������ͻȻ���֣�
	short               signalStrength; //�ź�ǿ�ȣ�-1������1��ǿ��0��һ��ǿ
	short               half;           //0���а��ܣ�1��û�а���
	double              dP[2], dL[2], f[2];
	short               nBas, nRov;     //�洢����۲�ֵ��Ӧ�Ļ�׼վ������վ����ֵ������
	SDSATOBS()
	{
		PRN = nBas = nRov = half = 0;
		system = Other;
		Valid = signalStrength = -1;
		dP[0] = dP[1] = dL[0] = dL[1] = f[0] = f[1] = 0.0;
	}
};
//ÿ����Ԫ�ĵ���۲�ֵ����
struct SDEPOCHOBS
{
	GPST               gt;
	unsigned short     SatNum;
	SDSATOBS           SdSatObs[MaxSatNum];
	MWGF               mwgf[MaxSatNum];
	SDEPOCHOBS()
	{
		SatNum = 0;
	}
};
//˫��������ݶ���
struct DDCOBS
{
	int                 RefPrn[2], RefPos[2], RefStatus[2];          // �ο������Ǻ���洢λ�ã�0=GPS; 1=BDS;�ο����Ƿ�ѡȡ�ɹ�
	int                 Sats, DDSatNum[2];             // ������˫��ģ����������0=GPS; 1=BDS
	double              FloatAmb[MaxSatNum * 2];       // ˫Ƶģ���ȸ����
	double              FloatAmbCov[MaxSatNum * 2 * MaxSatNum * 2];//�����Э������
	double              FixedAmb[MaxSatNum * 4];       // ����˫Ƶ���Ž�[0,AmbNum]�ʹ��Ž�[AmbNum,2*AmbNum]
	double              ResAmb[2], Ratio;              // LAMBDA������е�ģ���Ȳв�
	float               FixRMS[2];                     // �̶��ⶨλ��rms���
	double              dPos[3];                       // ��������
	bool                bFixed;                        // trueΪ�̶���falseΪδ�̶�
	DDCOBS()
	{
		int i;
		for (i = 0; i < 2; i++)
		{
			DDSatNum[i] = 0;    // ������ϵͳ��˫������
			RefPos[i] = RefPrn[i] = RefStatus[i] = -1;
		}
		Sats = 0;              // ˫����������
		dPos[0] = dPos[1] = dPos[2] = 0.0;
		ResAmb[0] = ResAmb[1] = FixRMS[0] = FixRMS[1] = Ratio = 0.0;
		bFixed = false;
		for (i = 0; i < MaxSatNum * 2; i++)
		{
			FloatAmb[i] = 0.0;
			FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
		}
		for (i = 0; i < MaxSatNum * 2 * MaxSatNum * 2; i++)
		{
			FloatAmbCov[i] = 0.0;
		}
	}
};
//kalman�˲��ṹ��
struct Kalman
{
	GPST gt;                                                          //ʱ����
	double X[MaxSatNum];											  //״̬��
	int dim;                                                          //״̬��ά��
	int PRNLable[MaxSatNum][4];                                       //����PRN�ű�ǩ ϵͳ �ο��� ������ Ƶ�ʱ�ǩ 
	int RefPrn[2];                                                    //�ο���PRN
	int PRN[MaxSatNum];                                               //������������PRN
	NAVSYS system[MaxSatNum];                                         //������������ϵͳ��ʶ
	double XCov[MaxSatNum* MaxSatNum];								  //״̬����Э����
	//bool isInitialize = false;                                        //�Ƿ��ʼ��Ԫ
	//bool isUpSats = false;                                            //�Ƿ���������������������Ҫ��ʼ��
	Kalman()
	{
		dim = 0;
		RefPrn[0] = RefPrn[1] = 0;
		for (int i = 0; i < MaxSatNum; i++)
		{
			X[i] = 0.0;
			PRNLable[i][0] = 7;
			PRNLable[i][1] = PRNLable[i][2] = PRNLable[i][3] = 0;
			PRN[i] = 0;
			system[i] = Other;
			for (int j = 0; j < MaxSatNum; j++)
				XCov[i * MaxSatNum + j] = 0.0;
		}
	}

};

//RTK��λ�����ݶ���
struct RTKData
{
	GPST               gt;
	OBS                BaseObs;
	OBS                RoverObs;
	EPH                Eph[MaxSatNum];
	BASESTATION        basesta;
	SDEPOCHOBS         SDobs;
	DDCOBS             DDObs;
	rover              Baserover;
	rover			   Roverrover;
	Kalman             kalmanLast;
	Kalman             kalmanNow;
	Kalman             kalmanFixed[2];
};



//0.ʱ��ͬ��
short _TimeSynchronization(unsigned char* buffer_base, int& n_base, unsigned char* buffer_rover, int& n_rover,FILE* fp_base, FILE* fp_rover, RTKData& rtk);
//�����봮�ڵ�ʱ��ͬ������
short _TimeSynchronization(unsigned char* buffer_base, int& n_base, unsigned char* buffer_rover, int& n_rover, CSerial& cse, SOCKET& sock, RTKData& rtk);
//�����������ʱ��ͬ������
short _TimeSynchronization(unsigned char* buffer_base, int& n_base, unsigned char* buffer_rover, int& n_rover, SOCKET& baseSock, SOCKET& roverSock, RTKData& rtk);

//1.���ݻ�ȡ
//1:��ȡ���ݳɹ�;0����ȡ���ݲ��ɹ�;-1���ļ���ȡ����
short _GetData(FILE* fp, unsigned char* buffer, int& n, OBS& obs, RTKData& rtk);
//1:��ȡ���ݳɹ�;0����ȡ���ݲ��ɹ�;-1���ļ���ȡ����
short _GetData(CSerial& cse, unsigned char* buffer, int& n, OBS& obs, RTKData& rtk);
//1:��ȡ���ݳɹ�;0����ȡ���ݲ��ɹ�;-1���ļ���ȡ����
short _GetData(SOCKET& sock, unsigned char* buffer, int& n, OBS& obs, RTKData& rtk);

//2.վ�䵥��
void FormSDEpochObs(const OBS& baseObs, const OBS& roverObs, SDEPOCHOBS& SDObs);
//3.�ֲ�̽��
void DetectCycleSilp(SDEPOCHOBS& SDObs);
//4.�ο���ѡȡ
bool DetRefSat(const OBS& baseObs, const OBS& roverObs, SDEPOCHOBS& SDObs, DDCOBS& DDCObs);
//5.��Զ�λ�����
bool RTKFloat(OBS & baseObs,OBS & roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs);
//5.1��վ(����վ)�����ǵļ��ξ���
double distance(XYZ& station, SATPOS* satPostion, int index);
//6.��Զ�λ�̶���
bool RTKFixed(OBS& baseObs, OBS& roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs);


//5.2��Զ�λkalman�˲�
// ��ʼ��
void KalmanInitialize(rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Kalman& kalmanNow);
//
bool RTKFloatKalman(OBS& baseObs, OBS& roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Kalman& kalmanLast, Kalman& kalmanNow);;
// phi����
Matrix _GetPhi(Kalman& kalmanLast, Kalman& kalmanNow);
// Q����
Matrix _GetQ(Kalman& kalmanLast, Kalman& kalmanNow);
// H����
Matrix _GetH(OBS& roverObs, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Matrix& Xk_k1);
//H����̶�
Matrix _GetHFixed(OBS& roverObs, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Matrix& Xk_k1);
// Z����
Matrix _GetZ(OBS& baseObs, OBS& roverObs, BASESTATION& base, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Kalman& kalmanNow);
// R����
Matrix _GetR(int GPSSatNum, int BDSSatNum);
// �̶���R����
Matrix _GetRFixed(int GPSSatNum, int BDSSatNum);
// V����
Matrix _GetV(OBS& baseObs, OBS& roverObs, BASESTATION& base, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Matrix& Xk_k1);
// V����̶���
Matrix _GetVFixed(OBS& baseObs, OBS& roverObs, BASESTATION& base, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Matrix& Xk_k1);

//ʱ��Ԥ��
void EKFPredict(Kalman& kalmanLast, Kalman& kalmanNow, Matrix& Pk_k1, Matrix& Xk_k1);

//�������
void EKFUpdate(OBS& baseObs, OBS& roverObs, BASESTATION& base, SDEPOCHOBS& SDObs, DDCOBS& DDCObs,
	Kalman & kalmanNow, Matrix & Pk_k1, Matrix & Xk_k1, Matrix & Pk, Matrix & Xk);

//�̶���
bool RTKFixedKalman(OBS& baseObs, OBS& roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Kalman& kalmanLast);
#endif // !RTK_H
