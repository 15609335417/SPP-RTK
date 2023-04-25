#pragma once
#ifndef RTK_H

#define RTK_H

#include<stdio.h>
#include"Decode.h"
#include"Spp.h"

#include"Serial.h"
#include"sockets.h"
#include"lambda.h"


//每颗卫星的单差观测值数据
struct SDSATOBS
{
	unsigned short      PRN;            //PRN
	NAVSYS              system;
	short               Valid;          //有效性：-1：无效；0：有双频观测值；1：通过周跳探测；2：有周跳；3：卫星突然出现；
	short               signalStrength; //信号强度：-1：弱；1：强；0：一般强
	short               half;           //0：有半周；1：没有半周
	double              dP[2], dL[2], f[2];
	short               nBas, nRov;     //存储单差观测值对应的基准站和流动站的数值索引号
	SDSATOBS()
	{
		PRN = nBas = nRov = half = 0;
		system = Other;
		Valid = signalStrength = -1;
		dP[0] = dP[1] = dL[0] = dL[1] = f[0] = f[1] = 0.0;
	}
};
//每个历元的单差观测值定义
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
//双差相关数据定义
struct DDCOBS
{
	int                 RefPrn[2], RefPos[2], RefStatus[2];          // 参考星卫星号与存储位置，0=GPS; 1=BDS;参考星是否选取成功
	int                 Sats, DDSatNum[2];             // 待估的双差模糊度数量，0=GPS; 1=BDS
	double              FloatAmb[MaxSatNum * 2];       // 双频模糊度浮点解
	double              FloatAmbCov[MaxSatNum * 2 * MaxSatNum * 2];//浮点解协方差阵
	double              FixedAmb[MaxSatNum * 4];       // 包括双频最优解[0,AmbNum]和次优解[AmbNum,2*AmbNum]
	double              ResAmb[2], Ratio;              // LAMBDA浮点解中的模糊度残差
	float               FixRMS[2];                     // 固定解定位中rms误差
	double              dPos[3];                       // 基线向量
	bool                bFixed;                        // true为固定，false为未固定
	DDCOBS()
	{
		int i;
		for (i = 0; i < 2; i++)
		{
			DDSatNum[i] = 0;    // 各卫星系统的双差数量
			RefPos[i] = RefPrn[i] = RefStatus[i] = -1;
		}
		Sats = 0;              // 双差卫星总数
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
//kalman滤波结构体
struct Kalman
{
	GPST gt;                                                          //时间标记
	double X[MaxSatNum];											  //状态量
	int dim;                                                          //状态量维度
	int PRNLable[MaxSatNum][4];                                       //卫星PRN号标签 系统 参考星 解算星 频率标签 
	int RefPrn[2];                                                    //参考星PRN
	int PRN[MaxSatNum];                                               //参与解算的卫星PRN
	NAVSYS system[MaxSatNum];                                         //参与解算的卫星系统标识
	double XCov[MaxSatNum* MaxSatNum];								  //状态向量协方差
	//bool isInitialize = false;                                        //是否初始历元
	//bool isUpSats = false;                                            //是否有卫星升起，若升起，则需要初始化
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

//RTK定位的数据定义
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



//0.时间同步
short _TimeSynchronization(unsigned char* buffer_base, int& n_base, unsigned char* buffer_rover, int& n_rover,FILE* fp_base, FILE* fp_rover, RTKData& rtk);
//网络与串口的时间同步函数
short _TimeSynchronization(unsigned char* buffer_base, int& n_base, unsigned char* buffer_rover, int& n_rover, CSerial& cse, SOCKET& sock, RTKData& rtk);
//网络与网络的时间同步函数
short _TimeSynchronization(unsigned char* buffer_base, int& n_base, unsigned char* buffer_rover, int& n_rover, SOCKET& baseSock, SOCKET& roverSock, RTKData& rtk);

//1.数据获取
//1:获取数据成功;0：获取数据不成功;-1：文件读取结束
short _GetData(FILE* fp, unsigned char* buffer, int& n, OBS& obs, RTKData& rtk);
//1:获取数据成功;0：获取数据不成功;-1：文件读取结束
short _GetData(CSerial& cse, unsigned char* buffer, int& n, OBS& obs, RTKData& rtk);
//1:获取数据成功;0：获取数据不成功;-1：文件读取结束
short _GetData(SOCKET& sock, unsigned char* buffer, int& n, OBS& obs, RTKData& rtk);

//2.站间单差
void FormSDEpochObs(const OBS& baseObs, const OBS& roverObs, SDEPOCHOBS& SDObs);
//3.粗差探测
void DetectCycleSilp(SDEPOCHOBS& SDObs);
//4.参考星选取
bool DetRefSat(const OBS& baseObs, const OBS& roverObs, SDEPOCHOBS& SDObs, DDCOBS& DDCObs);
//5.相对定位浮点解
bool RTKFloat(OBS & baseObs,OBS & roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs);
//5.1基站(流动站)到卫星的几何距离
double distance(XYZ& station, SATPOS* satPostion, int index);
//6.相对定位固定解
bool RTKFixed(OBS& baseObs, OBS& roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs);


//5.2相对定位kalman滤波
// 初始化
void KalmanInitialize(rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Kalman& kalmanNow);
//
bool RTKFloatKalman(OBS& baseObs, OBS& roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Kalman& kalmanLast, Kalman& kalmanNow);;
// phi矩阵
Matrix _GetPhi(Kalman& kalmanLast, Kalman& kalmanNow);
// Q矩阵
Matrix _GetQ(Kalman& kalmanLast, Kalman& kalmanNow);
// H矩阵
Matrix _GetH(OBS& roverObs, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Matrix& Xk_k1);
//H矩阵固定
Matrix _GetHFixed(OBS& roverObs, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Matrix& Xk_k1);
// Z矩阵
Matrix _GetZ(OBS& baseObs, OBS& roverObs, BASESTATION& base, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Kalman& kalmanNow);
// R矩阵
Matrix _GetR(int GPSSatNum, int BDSSatNum);
// 固定解R矩阵
Matrix _GetRFixed(int GPSSatNum, int BDSSatNum);
// V矩阵
Matrix _GetV(OBS& baseObs, OBS& roverObs, BASESTATION& base, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Matrix& Xk_k1);
// V矩阵固定解
Matrix _GetVFixed(OBS& baseObs, OBS& roverObs, BASESTATION& base, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Matrix& Xk_k1);

//时间预测
void EKFPredict(Kalman& kalmanLast, Kalman& kalmanNow, Matrix& Pk_k1, Matrix& Xk_k1);

//量测更新
void EKFUpdate(OBS& baseObs, OBS& roverObs, BASESTATION& base, SDEPOCHOBS& SDObs, DDCOBS& DDCObs,
	Kalman & kalmanNow, Matrix & Pk_k1, Matrix & Xk_k1, Matrix & Pk, Matrix & Xk);

//固定解
bool RTKFixedKalman(OBS& baseObs, OBS& roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Kalman& kalmanLast);
#endif // !RTK_H
