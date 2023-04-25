#include"Spp.h"

/// <summary>
/// 计算双频数据有效性，计算双频伪距相位Lmw，Lgf组合
/// 周跳与粗差探测，伪距IF组合
/// //MWGF outline[obs.SatNum]
/// 粗差探测之前是上个历元数据
/// 粗差探测之后是本历元数据
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
		outer[i].Lmw = 1 / (obs.satobs[i].f[0] - obs.satobs[i].f[1]) * (obs.satobs[i].f[0] * obs.satobs[i].L[0] - obs.satobs[i].f[1] * obs.satobs[i].L[1] )
			- 1.0 / (obs.satobs[i].f[0] + obs.satobs[i].f[1]) * (obs.satobs[i].f[0] * obs.satobs[i].C[0] + obs.satobs[i].f[1] * obs.satobs[i].C[1]);
		outer[i].Lgf = obs.satobs[i].L[0] - obs.satobs[i].L[1];

		//突然出现的问题的解决
		bool Status = false;
		//找上一时刻的PRN
		for (int j = 0; j < MaxSatNum; j++)
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
			if (fabs(obs.outer[j]._Lmw) > EPS && fabs(dMW) <= MaxDeltaLmw && fabs(dGF) <= MaxDeltaLgf)
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
				outer[i]._Lmw = outer[i].Lmw;
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
			outer[i]._Lmw = outer[i].Lmw;
			outer[i].n = 1;//本时刻为第一个历元
		}
	}
	memcpy(obs.outer, outer, MaxSatNum * sizeof(MWGF));
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
/// 获取卫星高度角
/// </summary>
/// <param name="x">卫星位置</param>
/// <param name="b">测站位置</param>
/// <returns>卫星高度角</returns>
double _GetElevationAngle(const XYZ& x, BLH& b, double a, double alpha)
{
	ENU e;
	XYZ2ENU(x, b, e, a, alpha);
	return rad2deg(acos(sqrt((e.n * e.n + e.e * e.e) / (e.n * e.n + e.e * e.e + e.u * e.u))));
}


//数组插值
//需数组长度够长
void ArrayInsert(double* array, int n, double value, int No)
{
	if (n >= No)
		return;
	for (int i = No-1; i >n; i--)
	{
		array[i] = array[i - 1];
	}
	array[n] = value;
}

/// <summary>
/// 最小二乘
/// </summary>
void SPP_LS(OBS& obs, EPH* eph, rover& rover)
{
	int rows = 0;//B矩阵的行数
	int cols = 5;//B矩阵的列数
	rover.GPSSatNum = rover.BDSSatNum = 0;
	double* arrayB = new double[obs.SatNum * cols]; int B_index = 0;
	double* arrayw = new double[obs.SatNum];
	double** arrayP = new double* [obs.SatNum];
	//P初始化
	for (int i = 0; i < obs.SatNum; i++)
	{
		arrayP[i] = new double[obs.SatNum];
		for (int j = 0; j < obs.SatNum; j++)
		{
			arrayP[i][j] = 0.0;
		}
	}
	for (int i = 0; i < obs.SatNum; i++)
	{
		if (obs.satpos[i].Status != 1)continue;
		if (Norm(obs.satpos[i].XYZPos) < EPS) continue;
		double p0 = Norm(obs.satpos[i].XYZPos, rover._XYZpostion);
		arrayB[B_index] = (rover._XYZpostion.x - obs.satpos[i].XYZPos.x) / p0; B_index++;
		arrayB[B_index] = (rover._XYZpostion.y - obs.satpos[i].XYZPos.y) / p0; B_index++;
		arrayB[B_index] = (rover._XYZpostion.z - obs.satpos[i].XYZPos.z) / p0; B_index++;
		arrayB[B_index] = 1; B_index++;
		if (obs.satpos[i].system == GPS)
		{
			arrayw[rows] = obs.satpos[i].PIF - (p0 + obs.satpos[i].Trop - Cspeed * obs.satpos[i].clock + rover.RcvClkOft[0]);
			rover.GPSSatNum++;
		}
		else if (obs.satpos[i].system == BDS)
		{
			arrayw[rows] = obs.satpos[i].PIF - (Cspeed * pow(B1I, 2) * eph[obs.satobs[i].PRN - 1 + MaxGPSSatNum].TGD[0]) / (pow(B1I, 2) - pow(B3I, 2))
			- (p0 + obs.satpos[i].Trop - Cspeed * obs.satpos[i].clock + rover.RcvClkOft[1]);//这个没错，不准改
			rover.BDSSatNum++;
		}
		arrayP[rows][rows] = pow(cfg.pseudorangenoise, 2);
		rows++;
	}
	//确认B的维度
	//重构B矩阵
	rover.AllSatNum = obs.SatNum;
	if (rover.GPSSatNum * rover.BDSSatNum == 0)
		cols = 4;
	else 
	{
		cols = 5;
		for (int i = 0; i < rover.GPSSatNum; i++)
		{
			ArrayInsert(arrayB, i * cols + 4, 0, obs.SatNum * cols);
		}
		for (int i = 0; i < rover.BDSSatNum ; i++)
		{
			ArrayInsert(arrayB, (rover.GPSSatNum - 1) * cols + 4 + i * cols + 4, 0, obs.SatNum * cols);
		}
	}
	if (rows < cols)
	{
		delete[] arrayB;
		delete[] arrayw;
		for (int i = 0; i < obs.SatNum; i++)
		{
			delete arrayP[i];
		}
		delete[] arrayP;
		return;
	}
	else;
	double* _arrayP = new double[rows * rows];
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < rows; j++)
		{
			_arrayP[i * rows + j] = arrayP[i][j];
		}
	}
	Matrix B(rows, cols); B = arrayB; //B.Show();
	Matrix w(rows, 1); w = arrayw;
	Matrix v(rows, 1);
	Matrix P(rows, 1.0);
	Matrix x(cols, 1);
	Matrix Q(cols, cols);
	Matrix D(cols, cols);
	x = (B.Transpose() * P * B).Inverse() * B.Transpose() * P * w;
	v = B * x - w;
	rover.SigmaPos = sqrt((v.Transpose() * P * v)(0, 0) / (double)(rows - cols));
	Q = (B.Transpose() * P * B).Inverse();
	D = Q * rover.SigmaPos;
	rover.PDOP = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));

	//XYZ
	rover._XYZpostion.x += x(0, 0);
	rover._XYZpostion.y += x(1, 0);
	rover._XYZpostion.z += x(2, 0);

	//dxyz
	rover._deltax.x = x(0, 0);
	rover._deltax.y = x(1, 0);
	rover._deltax.z = x(2, 0);

	//钟差
	rover.RcvClkOft[0] += x(3, 0);
	if (cols > 4) rover.RcvClkOft[1] += x(4, 0);

	//
	rover.gt = obs.gt;
	XYZ2BLH(rover._XYZpostion, rover._BLHpostion, a_GPS, alpha_GPS);

	double arrayH[9] = { -sin(deg2rad(rover._BLHpostion.b)) * cos(deg2rad(rover._BLHpostion.l)),-sin(deg2rad(rover._BLHpostion.b)) * sin(deg2rad(rover._BLHpostion.l)),cos(deg2rad(rover._BLHpostion.b)),
		-sin(deg2rad(rover._BLHpostion.l)),cos(deg2rad(rover._BLHpostion.l)),0,
		cos(deg2rad(rover._BLHpostion.b)) * cos(deg2rad(rover._BLHpostion.l)),cos(deg2rad(rover._BLHpostion.b)) * sin(deg2rad(rover._BLHpostion.l)),sin(deg2rad(rover._BLHpostion.b)) };
	Matrix H(3, 3); H = arrayH;
	double arrayQ[9] = { Q(0,0),Q(0,1) ,Q(0,2) ,Q(1,0), Q(1,1), Q(1,2) ,Q(2,0) ,Q(2,1),Q(2,2) };
	Matrix q(3, 3); q = arrayQ;
	Matrix _Q(3, 3); _Q = H * q * H.Transpose();
	rover.HDOP = sqrt(_Q(0, 0) + _Q(1, 1));
	rover.VDOP = sqrt(_Q(2, 2));
	delete[] arrayB;
	delete[] arrayw;
	for (int i = 0; i < obs.SatNum; i++)
	{
		delete arrayP[i];
	}
	delete[] arrayP;
	delete[] _arrayP;
}

/// <summary>
/// SPP单点定位
/// </summary>
bool SPP(OBS& obs, EPH* eph, rover& rover)
{

	/*设定初始位置posresult.XYZPos.x,y,z=0*/
/*设置循环次数*/
	int count = 0;
	/*初始参数设置*/
	double a = 0, alpha = 0, GM = 0, omega_e = 0;
	rover.gt = obs.gt;
	while (Norm(rover._deltax) > 1.0e-6)
	{
		/*计算卫星位置,速度，钟差，钟速*/
		//_SatPos(ephG, ephC, obs);
		_SatPos(eph, obs);
		for (int t = 0; t < obs.SatNum; t++)
		{
			/*状态不好的卫星不参与解算*///卫星位置解算失败也不参与解算
			//if (obs.satpos[t].Status == 0)continue;
			if (obs.satpos[t].Status != 1)continue;
			/*地球自转改正*/
			_ParameterSetting(obs.satpos[t].system, a, alpha, GM, omega_e);
			double rou = Norm(obs.satpos[t].XYZPos, rover._XYZpostion);
			double dt = rou / Cspeed;
			_RotationCorrection(obs.satpos[t].XYZPos, omega_e, dt);
			_RotationCorrection(obs.satpos[t].XYZVel, omega_e, dt);
			/*计算卫星高度角*/
			obs.satpos[t].El = _GetElevationAngle(obs.satpos[t].XYZPos, rover._BLHpostion, a, alpha);
			//obs.satpos[t].El = _El(obs.satpos[t].XYZPos, rover.PostionBLH, a, alpha);
			if (obs.satpos[t].El >= cfg.eleuationmask && obs.satpos[t].El <= 89 && obs.satpos[t].Status != 0) obs.satpos[t].Status = 1;
			else if (!(obs.satpos[t].El >= cfg.eleuationmask && obs.satpos[t].El <= 89) && obs.satpos[t].Status != 0)obs.satpos[t].Status = 3;//高度角不正常的卫星
			/*计算对流层延迟*/
			obs.satpos[t].Trop = _Hopefield(rover._BLHpostion.h, obs.satpos[t].El);
		}
		/*SPP*/
		SPP_LS(obs, eph, rover);
		count++;
		if (count > 10)
			break;
	}
	if (rover.GPSSatNum > 0 || rover.BDSSatNum > 0)
	{
		rover.IsSuccess = 1;
		return true;	
	}
	else return false;

}

/// <summary>
/// SPV 单点测速
/// </summary>
void SPV(OBS& obs, EPH* eph, rover& rover)
{
	/*如果星历存在即计算*/
	bool Status = false;
	for (int i = 0; i < MaxGPSSatNum + MaxBDSSatNum; i++)
	{
		if (eph[i].PRN != 0) {
			Status = true; break;
		}
	}
	if (!Status)return;
	int rows = 0;
	double* arrayB = new double[obs.SatNum * 4];
	double* arrayw = new double[obs.SatNum];
	for (int i = 0; i < obs.SatNum; i++)
	{
		if (obs.satpos[i].Status != 1)continue;
		double P = Norm(obs.satpos[i].XYZPos, rover._XYZpostion);
		double rouDOT = ((obs.satpos[i].XYZPos.x - rover._XYZpostion.x) * obs.satpos[i].XYZVel.x +
			(obs.satpos[i].XYZPos.y - rover._XYZpostion.y) * obs.satpos[i].XYZVel.y +
			(obs.satpos[i].XYZPos.z - rover._XYZpostion.z) * obs.satpos[i].XYZVel.z) / P;
		arrayB[rows * 4] = (rover._XYZpostion.x - obs.satpos[i].XYZPos.x) / P;
		arrayB[rows * 4 + 1] = (rover._XYZpostion.y - obs.satpos[i].XYZPos.y) / P;
		arrayB[rows * 4 + 2] = (rover._XYZpostion.z - obs.satpos[i].XYZPos.z) / P;
		arrayB[rows * 4 + 3] = 1;
		arrayw[rows] = obs.satobs[i].D[0] - (rouDOT - Cspeed * obs.satpos[i].clockVelocity);
		rows++;
	}
	if (rows < 4) {
		delete[] arrayB;
		delete[] arrayw;
		return;
	}
	Matrix B(rows, 4); B = arrayB;
	Matrix w(rows, 1); w = arrayw;
	Matrix P(rows, 1.0);
	Matrix x(rows, 1);
	x = (B.Transpose() * P * B).Inverse() * B.Transpose() * P * w;
	Matrix v(rows, 1);
	v = B * x - w;
	double sigma = sqrt((v.Transpose() * P * v)(0, 0) / (double)(rows - 4));
	Matrix Q(rows, rows);
	Q = (B.Transpose() * P * B).Inverse();
	Matrix D(rows, rows);
	D = Q * sigma;
	rover.Velocity[0] = x(0, 0);
	rover.Velocity[1] = x(1, 0);
	rover.Velocity[2] = x(2, 0);
	rover.RcvClkSft = x(3, 0);
	rover.SigmaVel = sigma;
	//postion.VDOT = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));

	delete[] arrayB;
	delete[] arrayw;
}