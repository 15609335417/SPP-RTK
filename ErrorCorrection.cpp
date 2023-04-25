#include"ErrorCorrection.h"
//ע
//MWGF outline[obs.SatNum]
//�ֲ�̽��֮ǰ���ϸ���Ԫ����
//�ֲ�̽��֮���Ǳ���Ԫ����

/// <summary>
/// ����˫Ƶ������Ч�ԣ�����˫Ƶα����λLmw��Lgf���
/// ������ֲ�̽�⣬α��IF���
/// </summary>
/// <param name="obs">˫Ƶα����λ�۲�ֵ</param>
void _LinearCombination(OBS& obs)
{
	MWGF outer[MaxSatNum];//����Ԫ��MW GF ���
	for (int i = 0; i < obs.SatNum; i++)
	{
		//�Ȱ�����������Ϊ״̬����
		obs.satpos[i].Status = 0;
		outer[i].system = obs.satpos[i].system;
		outer[i].PRN = obs.satpos[i].PRN;
		
		memcpy(&outer[i].gt, &obs.gt, sizeof(GPST));
		//double ����С��EPS
		//��˫Ƶ�۲�ֵ��ֱ��Ϊ0
		if ((fabs(obs.satobs[i].C[0]) < EPS || fabs(obs.satobs[i].C[1]) < EPS)) {
			outer[i]._Lmw = 0;
			outer[i].n = 1;//��ʱ��Ϊ��һ����Ԫ
			continue;
		}
		if ((fabs(obs.satobs[i].L[0]) < EPS || fabs(obs.satobs[i].L[1]) < EPS)) {
			outer[i]._Lmw = 0;
			outer[i].n = 1;//��ʱ��Ϊ��һ����Ԫ
			continue;
		}
		//�������Lmw��Lgf
		outer[i].Lmw = 1 / (obs.satobs[i].f[0] - obs.satobs[i].f[1]) * (Cspeed * obs.satobs[i].L[0] - Cspeed * obs.satobs[i].L[1])
			- 1.0 / (obs.satobs[i].f[0] + obs.satobs[i].f[1]) * (obs.satobs[i].f[0] * obs.satobs[i].C[0] + obs.satobs[i].f[1] * obs.satobs[i].C[1]);		
		outer[i].Lgf= Cspeed / obs.satobs[i].f[0] * obs.satobs[i].L[0] - Cspeed / obs.satobs[i].f[1] * obs.satobs[i].L[1];
		
		//ͻȻ���ֵ�����Ľ��
		bool Status = false;
		//����һʱ�̵�PRN
		for (int j = 0; j <MaxSatNum; j++)
		{
			if (obs.outer[j].PRN == 0)continue;
			if (outer[i].system != obs.outer[j].system)continue;
			if (outer[i].PRN != obs.outer[j].PRN)continue;
			//�ҵ�j�� j ��һʱ��
			Status = true;
			//ǰ����Ԫ��_Lmw Lgf����
			double dGF = outer[i].Lgf - obs.outer[j].Lgf;
			double dMW = outer[i].Lmw - obs.outer[j]._Lmw;
			//����Ƿ���
			if (fabs(obs.outer[j]._Lmw)>EPS&& fabs(dMW) <= MaxDeltaLmw && fabs(dGF) <= MaxDeltaLgf)
			{
				//���Ϊ����
				obs.satpos[i].Status = 1;
				obs.satpos[i].PIF = (pow(obs.satobs[i].f[0], 2) * obs.satobs[i].C[0] -
					pow(obs.satobs[i].f[1], 2) * obs.satobs[i].C[1]) / (pow(obs.satobs[i].f[0], 2) -
						pow(obs.satobs[i].f[1], 2));
			}
			else
			{
				//�ֲ���ּ����Ϊ2���ʱ�̵�ƽ��ֵΪ��������Lmw
				outer[i]._Lmw = outer[i].Lmw ;
				outer[i].n = 1;
				obs.satpos[i].Status = 2;//�ֲ�
				break;
			}
			//���㱾��Ԫ��_Lmw[1];
			outer[i]._Lmw = (double)(obs.outer[j].n - 1) / (double)obs.outer[j].n * obs.outer[j]._Lmw + 1.0 / (double)obs.outer[j].n * outer[i].Lmw;
			outer[i].n = obs.outer[j].n + 1;
			break;
		}	
		if (!Status) 
		{
			obs.satpos[i].Status = -1; 
			outer[i]._Lmw =outer[i].Lmw;
			outer[i].n = 1;//��ʱ��Ϊ��һ����Ԫ
		}
	}
	memcpy(obs.outer,outer,MaxSatNum*sizeof(MWGF));	
}

/// <summary>
/// �������������Hopefieldģ��
/// </summary>
/// <param name="H">��վ��</param>
/// <param name="E">���Ǹ߶Ƚ�</param>
/// <returns>�����������</returns>
double _Hopefield(double& H, double& E)
{
	if (fabs(H) >= 10000) return 0;
	double trop;
	//1.���������
	double H0 = 0.0;
	double T0 = 15 + 273.16;
	double P0 = 1013.25;
	double RH0 = 0.5;
	//Hopefieldģ��
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
/// ������ת����
/// </summary>
/// <param name="x">����λ�û��ٶ�</param>
/// <param name="omega_e"></param>
/// <param name="dt"></param>
void _RotationCorrection(XYZ& x, double &omega_e, double &dt)
{
	double arrayXk[3] = { x.x ,x.y ,x.z };
	x.x = arrayXk[0] * cos(omega_e * dt) + arrayXk[1] * sin(omega_e * dt);
	x.y = -arrayXk[0] * sin(omega_e * dt) + arrayXk[1] * cos(omega_e * dt);
	x.z = arrayXk[2];
}
