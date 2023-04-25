#include"ErrorCorrection.h"
//注
//MWGF outline[obs.SatNum]
//粗差探测之前是上个历元数据
//粗差探测之后是本历元数据

/// <summary>
/// 计算双频数据有效性，计算双频伪距相位Lmw，Lgf组合
/// 周跳与粗差探测，伪距IF组合
/// </summary>
/// <param name="obs">双频伪距相位观测值</param>
void _LinearCombination(OBS& obs)
{
	MWGF outer[MaxSatNum];//本历元的MW GF 组合
	for (int i = 0; i < obs.SatNum; i++)
	{
		//先把所有卫星设为状态不好
		obs.satpos[i].Status = 0;
		outer[i].system = obs.satpos[i].system;
		outer[i].PRN = obs.satpos[i].PRN;
		
		memcpy(&outer[i].gt, &obs.gt, sizeof(GPST));
		//double 类型小于EPS
		//无双频观测值的直接为0
		if ((fabs(obs.satobs[i].C[0]) < EPS || fabs(obs.satobs[i].C[1]) < EPS)) {
			outer[i]._Lmw = 0;
			outer[i].n = 1;//本时刻为第一个历元
			continue;
		}
		if ((fabs(obs.satobs[i].L[0]) < EPS || fabs(obs.satobs[i].L[1]) < EPS)) {
			outer[i]._Lmw = 0;
			outer[i].n = 1;//本时刻为第一个历元
			continue;
		}
		//计算组合Lmw，Lgf
		outer[i].Lmw = 1 / (obs.satobs[i].f[0] - obs.satobs[i].f[1]) * (Cspeed * obs.satobs[i].L[0] - Cspeed * obs.satobs[i].L[1])
			- 1.0 / (obs.satobs[i].f[0] + obs.satobs[i].f[1]) * (obs.satobs[i].f[0] * obs.satobs[i].C[0] + obs.satobs[i].f[1] * obs.satobs[i].C[1]);		
		outer[i].Lgf= Cspeed / obs.satobs[i].f[0] * obs.satobs[i].L[0] - Cspeed / obs.satobs[i].f[1] * obs.satobs[i].L[1];
		
		//突然出现的问题的解决
		bool Status = false;
		//找上一时刻的PRN
		for (int j = 0; j <MaxSatNum; j++)
		{
			if (obs.outer[j].PRN == 0)continue;
			if (outer[i].system != obs.outer[j].system)continue;
			if (outer[i].PRN != obs.outer[j].PRN)continue;
			//找到j了 j 上一时刻
			Status = true;
			//前后历元的_Lmw Lgf作差
			double dGF = outer[i].Lgf - obs.outer[j].Lgf;
			double dMW = outer[i].Lmw - obs.outer[j]._Lmw;
			//检查是否超限
			if (fabs(obs.outer[j]._Lmw)>EPS&& fabs(dMW) <= MaxDeltaLmw && fabs(dGF) <= MaxDeltaLgf)
			{
				//标记为可用
				obs.satpos[i].Status = 1;
				obs.satpos[i].PIF = (pow(obs.satobs[i].f[0], 2) * obs.satobs[i].C[0] -
					pow(obs.satobs[i].f[1], 2) * obs.satobs[i].C[1]) / (pow(obs.satobs[i].f[0], 2) -
						pow(obs.satobs[i].f[1], 2));
			}
			else
			{
				//粗差出现即标记为2，令本时刻的平滑值为计算所得Lmw
				outer[i]._Lmw = outer[i].Lmw ;
				outer[i].n = 1;
				obs.satpos[i].Status = 2;//粗差
				break;
			}
			//计算本历元的_Lmw[1];
			outer[i]._Lmw = (double)(obs.outer[j].n - 1) / (double)obs.outer[j].n * obs.outer[j]._Lmw + 1.0 / (double)obs.outer[j].n * outer[i].Lmw;
			outer[i].n = obs.outer[j].n + 1;
			break;
		}	
		if (!Status) 
		{
			obs.satpos[i].Status = -1; 
			outer[i]._Lmw =outer[i].Lmw;
			outer[i].n = 1;//本时刻为第一个历元
		}
	}
	memcpy(obs.outer,outer,MaxSatNum*sizeof(MWGF));	
}

/// <summary>
/// 对流层改正——Hopefield模型
/// </summary>
/// <param name="H">测站高</param>
/// <param name="E">卫星高度角</param>
/// <returns>对流层改正数</returns>
double _Hopefield(double& H, double& E)
{
	if (fabs(H) >= 10000) return 0;
	double trop;
	//1.对流层改正
	double H0 = 0.0;
	double T0 = 15 + 273.16;
	double P0 = 1013.25;
	double RH0 = 0.5;
	//Hopefield模型
	double RH = RH0 * exp(-0.0006396 * (H - H0));
	double P = P0 * pow(1 - 0.0000226 * (H - H0), 5.225);
	double T = T0 - 0.0065 * (H - H0);
	double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
	double hw = 11000;
	double hd = 40136 + 148.72 * (T0 - 273.16);
	double Kw = 155.2e-7 * 4810 / (pow(T, 2)) * e * (hw - H);
	double Kd = 155.2e-7 * P / T * (hd - H);
	trop = Kd / sin(deg2rad(sqrt(pow(E, 2) + 6.25))) + Kw / sin(deg2rad(sqrt(pow(E, 2) + 2.25)));
	return trop;
}

/// <summary>
/// 地球自转改正
/// </summary>
/// <param name="x">卫星位置或速度</param>
/// <param name="omega_e"></param>
/// <param name="dt"></param>
void _RotationCorrection(XYZ& x, double &omega_e, double &dt)
{
	double arrayXk[3] = { x.x ,x.y ,x.z };
	x.x = arrayXk[0] * cos(omega_e * dt) + arrayXk[1] * sin(omega_e * dt);
	x.y = -arrayXk[0] * sin(omega_e * dt) + arrayXk[1] * cos(omega_e * dt);
	x.z = arrayXk[2];
}
