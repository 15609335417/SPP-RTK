#pragma once
#ifndef Decode_H

#define Decode_H
#define POLYCRC32 0xEDB88320u 
#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include"CONST.h"
#include"Time.h"
#include"Coor.h"
#include"Config.h"


//��������ṹ��
//����ϵͳö��
enum NAVSYS
{
	GPS = 0, GLONASS, SBAS, Galileo, BDS, QZSS, NavIC, Other
};

//�м���:����λ�ã��Ӳ����
struct SATPOS
{
	GPST gt;//ʱ�� 
	unsigned short PRN ;//����PRN
	NAVSYS system ;
	unsigned short SatNum ;//����õ������Ǹ���
	unsigned short GPSSatNum ;
	unsigned short BDSSatNum ;
	XYZ XYZPos;//λ��XYZ
	ENU ENUPos;
	XYZ XYZVel;//�ٶ�XYZ
	double clock ;//�����Ӳ�
	double clockVelocity ;//����
	double El ;//������Բ�վ�ĸ߶Ƚ�(��)
	double Trop ;//���������
	double PIF ;//�����
	double TGD1, TGD2 ;
	short Status ;//����״̬��1 ���ã����� ������
	SATPOS()
	{
		PRN = SatNum = GPSSatNum = BDSSatNum = 0;
		system = Other;
		clock = clockVelocity=El = PIF= Trop = TGD1 = TGD2 = 0.0;
		Status = 0;
	}
};
//�������йصĹ۲�ֵ
struct SatObs 
{
	unsigned short PRN = 0;//����PRN
	NAVSYS system = Other;//����ϵͳ GPS������ ٤���ԣ�GLONASS��QZSS
	double f[2];//˫ƵƵ��
	double C[2];//α��:L1C/A L2P B1I B3I
	double L[2];//��λ����
	double D[2];//�����գ�Hz
	float CNR[2];//�����
	float PsrSigma[2];//α��۲�ֵ��׼��
	float AdrSigma[2];//�ز���λ��׼��
	float locktime[2];//����ʱ��
	short PhaseLockFlag[2];//��λ������־��0��δ������1������
	short ParityKnownFlag[2];//��żУ����֪��־��0��δ֪��1����֪
	short CodeLockedFlag[2];//����������־��0��δ������1������
	short CarrierPhaseMeasurement[2];//�ز���λ������0��û�мӰ��ܣ�1�����˰���
	SatObs()
	{
		for (int i = 0; i < 2; i++)
		{
			f[i] = C[i] = L[i] = D[i] = CNR[i] = PsrSigma[i] = AdrSigma[i] = 0.0;
			locktime[i] = 0.0;
			PhaseLockFlag[i] = ParityKnownFlag[i] = CodeLockedFlag[i] = CarrierPhaseMeasurement[i] = 0;
		}
	}
};
//MW GF���
struct MWGF
{
	GPST gt;
	NAVSYS system = GPS;
	unsigned short PRN = 0;
	double Lmw = 0.0;
	double _Lmw = 0.0;
	double Lgf = 0.0;
	int n = 1;
};
//�۲�ֵ
struct OBS
{
	GPST gt;//�۲�ʱ��
	unsigned short SatNum = 0;//��������
	unsigned short ObsNum = 0;//�۲�ֵ�ܸ���
	unsigned short GPSNum = 0;//GPS���Ǹ���
	unsigned short BDSNum = 0;//BDS���Ǹ���
	SatObs satobs[MaxSatNum];//����48��GPS32
	SATPOS satpos[MaxSatNum];//�м���
	MWGF outer[MaxSatNum];//�ϸ���Ԫ��Lmw�ȵ�
};
//��������
struct EPH
{
	GPST gt;
	unsigned short PRN = 0;
	unsigned long Health = 1;//0��������1��������
	NAVSYS system = GPS;//����ϵͳ
	double toc = 0.0;//�Ӳ���ϲο�ʱ��
	double a[3];//�Ӳ������
	unsigned long IODE[2];//��������ʱ��
	GPST toe;//�����ϲο�ʱ��
	double TGD[2];//Ⱥ�ӳ�
	//������
	double M0 = 0.0;//ƽ�����
	double A = 0.0;//������
	double e = 0.0;//ƫ����
	double i0 = 0.0;//������
	double omega = 0.0;//���ص�Ǿ�
	double OMEGA0 = 0.0;//������ྭ-GAST(t0)
	//���㶯����
	double Deltan = 0.0;//ƽ�����ٶȵĸ���ֵ
	double HatOMEGA = 0.0;//������ྭ�仯��
	double Hati = 0.0;//�����Ǳ仯��
	double Cuc = 0.0;//�������u�����Ҹ�����
	double Cus = 0.0;//�������u�����Ҹ�����
	double Cic = 0.0;//������i�����Ҹ�����
	double Cis = 0.0;//������i�����Ҹ�����
	double Crc = 0.0;//���������ĵľ���r�����Ҹ�����
	double Crs = 0.0;//���������ĵľ���r�����Ҹ�����
	EPH()
	{
		a[0] = a[1] = a[2] = 0.0;
		IODE[0] = IODE[1] = 0;
		TGD[0] = TGD[1] = 0.0;
		PRN = 0;
		Health = 1;//0��������1��������
		system = Other;//����ϵͳ
		toc = 0.0;//�Ӳ���ϲο�ʱ��
		GPST toe;//�����ϲο�ʱ��
		//������
		M0 = 0.0;//ƽ�����
		A = 0.0;//������
		e = 0.0;//ƫ����
		i0 = 0.0;//������
		omega = 0.0;//���ص�Ǿ�
		OMEGA0 = 0.0;//������ྭ-GAST(t0)
		//���㶯����
		Deltan = 0.0;//ƽ�����ٶȵĸ���ֵ
		HatOMEGA = 0.0;//������ྭ�仯��
		Hati = 0.0;//�����Ǳ仯��
		Cuc = 0.0;//�������u�����Ҹ�����
		Cus = 0.0;//�������u�����Ҹ�����
		Cic = 0.0;//������i�����Ҹ�����
		Cis = 0.0;//������i�����Ҹ�����
		Crc = 0.0;//���������ĵľ���r�����Ҹ�����
		Crs = 0.0;//���������ĵľ���r�����Ҹ�����
	}
};
//BESTPOS
struct BASESTATION
{
	GPST gt;//
	BLH BLHBESTPOS;//λ��
	XYZ XYZBESTPOS;//λ��
	float PosAccuracy[3];//λ�þ���
	BASESTATION()
	{
		for (int i = 0; i < 3; i++)
			PosAccuracy[i] = 0;

	}
};


//���ݽ���
unsigned int crc32(const unsigned char* buff, int len);//CRCУ�麯��

double D8(unsigned char* p);
unsigned short US2(unsigned char* p);
unsigned int UI4(unsigned char* p);
unsigned long UL4(unsigned char* p);
long L4(unsigned char* p);
short S2(unsigned char* p);
float F4(unsigned char* p);

bool _DecodeNovOem7(unsigned char* buffer, int& n, OBS& obs, EPH* eph, BASESTATION& basesta);
void _DecodeEphG(unsigned char* buffer, int i, EPH* eph);
void _DecodeEphC(unsigned char* buffer, int i, EPH* eph);
void DecodeBase(unsigned char* buffer, int i, BASESTATION& base);
void DecodeRange(unsigned char* buffer, int i, OBS& obs);

#endif // !Decode_H
