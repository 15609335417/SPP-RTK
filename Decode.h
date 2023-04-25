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


//解码所需结构体
//导航系统枚举
enum NAVSYS
{
	GPS = 0, GLONASS, SBAS, Galileo, BDS, QZSS, NavIC, Other
};

//中间结果:卫星位置，钟差，钟速
struct SATPOS
{
	GPST gt;//时间 
	unsigned short PRN ;//卫星PRN
	NAVSYS system ;
	unsigned short SatNum ;//解算得到的卫星个数
	unsigned short GPSSatNum ;
	unsigned short BDSSatNum ;
	XYZ XYZPos;//位置XYZ
	ENU ENUPos;
	XYZ XYZVel;//速度XYZ
	double clock ;//卫星钟差
	double clockVelocity ;//钟速
	double El ;//卫星相对测站的高度角(°)
	double Trop ;//对流层改正
	double PIF ;//电离层
	double TGD1, TGD2 ;
	short Status ;//卫星状态：1 可用，其他 不可用
	SATPOS()
	{
		PRN = SatNum = GPSSatNum = BDSSatNum = 0;
		system = Other;
		clock = clockVelocity=El = PIF= Trop = TGD1 = TGD2 = 0.0;
		Status = 0;
	}
};
//与卫星有关的观测值
struct SatObs 
{
	unsigned short PRN = 0;//卫星PRN
	NAVSYS system = Other;//导航系统 GPS，北斗 伽利略，GLONASS，QZSS
	double f[2];//双频频率
	double C[2];//伪距:L1C/A L2P B1I B3I
	double L[2];//相位：周
	double D[2];//多普勒：Hz
	float CNR[2];//载噪比
	float PsrSigma[2];//伪距观测值标准差
	float AdrSigma[2];//载波相位标准差
	float locktime[2];//锁定时长
	short PhaseLockFlag[2];//相位锁定标志：0：未锁定，1：锁定
	short ParityKnownFlag[2];//奇偶校验已知标志：0：未知，1：已知
	short CodeLockedFlag[2];//代码锁定标志：0：未锁定，1：锁定
	short CarrierPhaseMeasurement[2];//载波相位测量：0：没有加半周，1：加了半周
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
//MW GF组合
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
//观测值
struct OBS
{
	GPST gt;//观测时间
	unsigned short SatNum = 0;//卫星总数
	unsigned short ObsNum = 0;//观测值总个数
	unsigned short GPSNum = 0;//GPS卫星个数
	unsigned short BDSNum = 0;//BDS卫星个数
	SatObs satobs[MaxSatNum];//北斗48，GPS32
	SATPOS satpos[MaxSatNum];//中间结果
	MWGF outer[MaxSatNum];//上个历元的Lmw等等
};
//卫星星历
struct EPH
{
	GPST gt;
	unsigned short PRN = 0;
	unsigned long Health = 1;//0：健康；1：不健康
	NAVSYS system = GPS;//导航系统
	double toc = 0.0;//钟差拟合参考时刻
	double a[3];//钟差改正数
	unsigned long IODE[2];//星历发布时间
	GPST toe;//轨道拟合参考时刻
	double TGD[2];//群延迟
	//六根数
	double M0 = 0.0;//平近点角
	double A = 0.0;//长半轴
	double e = 0.0;//偏心率
	double i0 = 0.0;//轨道倾角
	double omega = 0.0;//近地点角距
	double OMEGA0 = 0.0;//升交点赤经-GAST(t0)
	//九摄动参数
	double Deltan = 0.0;//平均角速度的改正值
	double HatOMEGA = 0.0;//升交点赤经变化率
	double Hati = 0.0;//轨道倾角变化率
	double Cuc = 0.0;//升交距角u的余弦改正项
	double Cus = 0.0;//升交距角u的正弦改正项
	double Cic = 0.0;//轨道倾角i的余弦改正项
	double Cis = 0.0;//轨道倾角i的正弦改正项
	double Crc = 0.0;//卫星至地心的距离r的余弦改正项
	double Crs = 0.0;//卫星至地心的距离r的正弦改正项
	EPH()
	{
		a[0] = a[1] = a[2] = 0.0;
		IODE[0] = IODE[1] = 0;
		TGD[0] = TGD[1] = 0.0;
		PRN = 0;
		Health = 1;//0：健康；1：不健康
		system = Other;//导航系统
		toc = 0.0;//钟差拟合参考时刻
		GPST toe;//轨道拟合参考时刻
		//六根数
		M0 = 0.0;//平近点角
		A = 0.0;//长半轴
		e = 0.0;//偏心率
		i0 = 0.0;//轨道倾角
		omega = 0.0;//近地点角距
		OMEGA0 = 0.0;//升交点赤经-GAST(t0)
		//九摄动参数
		Deltan = 0.0;//平均角速度的改正值
		HatOMEGA = 0.0;//升交点赤经变化率
		Hati = 0.0;//轨道倾角变化率
		Cuc = 0.0;//升交距角u的余弦改正项
		Cus = 0.0;//升交距角u的正弦改正项
		Cic = 0.0;//轨道倾角i的余弦改正项
		Cis = 0.0;//轨道倾角i的正弦改正项
		Crc = 0.0;//卫星至地心的距离r的余弦改正项
		Crs = 0.0;//卫星至地心的距离r的正弦改正项
	}
};
//BESTPOS
struct BASESTATION
{
	GPST gt;//
	BLH BLHBESTPOS;//位置
	XYZ XYZBESTPOS;//位置
	float PosAccuracy[3];//位置精度
	BASESTATION()
	{
		for (int i = 0; i < 3; i++)
			PosAccuracy[i] = 0;

	}
};


//数据解码
unsigned int crc32(const unsigned char* buff, int len);//CRC校验函数

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
