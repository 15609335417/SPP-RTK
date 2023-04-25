#include"SatellitePostion.h"
/// <summary>
/// 计算卫星位置，速度，钟差，钟速
/// </summary>
/// <param name="eph">星历</param>
/// <param name="obs">观测值中含有中间结果结构体</param>
void _SatPos(const EPH* eph, OBS& obs)
{
	//地球椭球参数
	//为了之后进行参数设置
	double GM = GM_GPS;//地球引力常量
	double omega_e = omega_e_GPS;//地球自转角速度
	double a = a_GPS;//椭球长半轴
	double alpha = alpha_GPS;//椭球扁率
	//数据是否过期
	bool datavalidity = false;
	int j = 0;
	double tk = 0;
 	for (int i = 0; i < obs.SatNum; i++)//OBS
	{
		//在粗差探测之后，没有粗差的卫星状态设置为1，再次进行卫星位置解算，解算成功仍是1,失败则赋0
		/*检查卫星观测值状态*/
		if (obs.satpos[i].Status == 0|| obs.satpos[i].Status ==2|| obs.satpos[i].Status == -1)continue;//状态不好的不参与解算
		/*另设变量j=obs.satpos[i].PRN - 1;*/
		if (obs.satpos[i].system == GPS)
			j = obs.satpos[i].PRN - 1;
		else if (obs.satpos[i].system == BDS)
			j = obs.satpos[i].PRN - 1 + MaxGPSSatNum;
		/*判断PRN是否合格*/
		if (eph[j].PRN <1||eph[j].PRN>63) {
			obs.satpos[i].Status = 0; continue;
		}
		/*判断卫星是否健康*/
		if (eph[j].Health != 0) { obs.satpos[i].Status = 0; continue; }//星历不健康以及没有的按不健康算
		/*计算卫星信号发射的时刻，最初始的卫星钟差为零*/
		GPST Ttr;
		Ttr.Week = obs.gt.Week;
		Ttr.SecOfWeek = obs.gt.SecOfWeek - obs.satobs[i].C[0] / Cspeed - obs.satpos[i].clock;
		//参数设置
		_ParameterSetting(eph[j].system, a, alpha, GM, omega_e);
		//星历是否过期 //如果过期，tk=0;
		tk = _DataValidity(eph[j], Ttr);
		if (!(fabs(tk) > EPS)) { obs.satpos[i].Status = 0; continue; }
		/*首次计算时需再次利用前面所得Ttr计算得到卫星钟差继而再次计算得到卫星信号发射时刻的GPST*/
		if (fabs(obs.satpos[i].clock) < EPS)
		{
			/*首次计算时需利用利用上面的Ttr计算卫星钟差*/
			_SatClk(eph[j], obs.satpos[i], Ttr, tk, GM, omega_e);
			/*再次计算卫星信号发射的时刻*/
			Ttr.SecOfWeek = obs.gt.SecOfWeek - obs.satobs[i].C[0] / Cspeed - obs.satpos[i].clock;
			tk = _DataValidity(eph[j], Ttr);
			if (!(fabs(tk) > EPS)) { obs.satpos[i].Status = 0; continue; }
		}
		//计算卫星位置和速度
		_SatPosVel(eph[j], obs.satpos[i], Ttr, tk, GM, omega_e);
		//计算钟差和钟速
		_SatClk(eph[j], obs.satpos[i], Ttr, tk, GM, omega_e);
		//经过上述计算的状态设置为1
		obs.satpos[i].Status = 1;
	}
}

/// <summary>
/// 数据是否过期
/// </summary>
/// <param name="eph">星历</param>
/// <param name="gt">卫星发射时间</param>
/// <returns>tk</returns>
double _DataValidity(const EPH& eph, const GPST &gt)
{
	double tk = 0;
	tk = (gt.Week - eph.toe.Week) * 604800.0+gt.SecOfWeek - eph.toe.SecOfWeek;
	if (eph.system == BDS)
		tk = tk - 14;
	if (tk > 302400)
		tk -= 604800;
	else if (tk < -302400)
		tk += 604800;
	//有效性检测
	if (eph.system == GPS)
	{
		if (fabs(tk) > 7500)
			tk =0;
	}
	else if (eph.system == BDS)
		if (fabs(tk) > 3900)
			tk = 0;
	return tk;
}
/// <summary>
/// 参数设置
/// </summary>
/// <param name="system">卫星系统</param>
/// <param name="a">椭球长半轴</param>
/// <param name="alpha">椭球扁率</param>
/// <param name="GM">地球引力常数</param>
/// <param name="omega_e">地球自转角速度</param>
void _ParameterSetting(const NAVSYS& system, double& a, double& alpha, double& GM, double& omega_e)
{
	if (system == GPS)
	{
		a = a_GPS;
		GM = GM_GPS;
		omega_e = omega_e_GPS;
		alpha = alpha_GPS;
	}
	else if (system == BDS)
	{
		a = a_BDS;
		GM = GM_BDS;
		omega_e = omega_e_BDS;
		alpha = alpha_BDS;
	}
}
/// <summary>
/// 卫星位置速度
/// </summary>
/// <param name="eph">星历</param>
/// <param name="satpos">中间结构体</param>
/// <param name="gt">卫星信号发射时刻</param>
/// <param name="tk"></param>
/// <param name="GM"></param>
/// <param name="omega_e"></param>
void _SatPosVel(const EPH& eph, SATPOS& satpos, GPST &gt, double &tk, double& GM, double &omega_e)
{
	//1.计算卫星运动的平均角速度n
	double n0 = sqrt(GM / (eph.A * eph.A * eph.A));
	double n = n0 + eph.Deltan;
	//2.计算观测瞬间卫星的平近点角M
	double M = eph.M0 + n * tk;
	//3.计算偏近点角E
	double E0 = M;
	double E = 0.0;
	double deltaE = E0;
	while (fabs(deltaE) > EPS)
	{
		E = M + eph.e * sin(E0);
		deltaE = E - E0;
		E0 = E;
	}
	//4.计算真近点角f  范围[-pi,pi]
	double f = atan2(sqrt(1 - eph.e * eph.e) * sin(E), (cos(E) - eph.e));
	//5.计算升交角距u'
	double _u = eph.omega + f;
	//6.计算摄动该正项
	double du = eph.Cuc * cos(2 * _u) + eph.Cus * sin(2 * _u);
	double dr = eph.Crc * cos(2 * _u) + eph.Crs * sin(2 * _u);
	double di = eph.Cic * cos(2 * _u) + eph.Cis * sin(2 * _u);
	//7.对u',r',i0进行改正
	double u = _u + du;
	double r = eph.A * (1 - eph.e * cos(E)) + dr;
	double I = eph.i0 + di + eph.Hati * tk;
	//8.计算卫星在轨道面坐标系中的坐标
	double x = r * cos(u);
	double y = r * sin(u);
	//9. 10.
	double L = 0.0;
	double PosGK[3] = { 0.0,0.0,0.0 };
	if (eph.system == GPS)
	{
		//9. 
		L = eph.OMEGA0 + eph.HatOMEGA * tk - omega_e * gt.SecOfWeek;
		//10.
		satpos.XYZPos.x = x * cos(L) - y * cos(I) * sin(L);
		satpos.XYZPos.y = x * sin(L) + y * cos(I) * cos(L);
		satpos.XYZPos.z = y * sin(I);
		
	}
	else if (eph.system == BDS)
	{
		//GEO轨道： 1 2 3 4 5 59 60 61 62 63
		if (eph.PRN <= 5 || (eph.PRN >= 59 && eph.PRN <= 63))
		{
			//9.
			L = eph.OMEGA0 + eph.HatOMEGA * tk - omega_e * eph.toe.SecOfWeek;
			//10.
			PosGK[0] = x * cos(L) - y * cos(I) * sin(L);
			PosGK[1] = x * sin(L) + y * cos(I) * cos(L);
			PosGK[2] = y * sin(I);
			double Rx[9] = { 1,0,0,0,cos(deg2rad(-5)),sin(deg2rad(-5)) ,0,-sin(deg2rad(-5)) ,cos(deg2rad(-5)) };
			double Rz[9] = { cos(tk * omega_e) ,sin(tk * omega_e) ,0,-sin(tk * omega_e) ,cos(tk * omega_e) ,0,0,0,1 };
			Matrix GK(3, 1); GK = PosGK;
			Matrix RX(3, 3); RX = Rx;
			Matrix RZ(3, 3); RZ = Rz;
			Matrix PosK(3, 1); PosK = RZ * RX * GK;
			satpos.XYZPos.x = PosK.get(0, 0);
			satpos.XYZPos.y = PosK.get(1, 0);
			satpos.XYZPos.z = PosK.get(2, 0);
			
		}
		else
		{
			L = eph.OMEGA0 + (eph.HatOMEGA - omega_e) * tk - omega_e * eph.toe.SecOfWeek;
			satpos.XYZPos.x = x * cos(L) - y * cos(I) * sin(L);
			satpos.XYZPos.y = x * sin(L) + y * cos(I) * cos(L);
			satpos.XYZPos.z = y * sin(I);
			
		}
	}
	double EDOT = n / (1 - eph.e * cos(E));
	double fDOT = EDOT * sqrt(1 - eph.e * eph.e) / (1 - eph.e * cos(E));
	double iDOT= eph.Hati + 2 * fDOT * (eph.Cis * cos(2 * _u) - eph.Cic * sin(2 * _u));
	double uDOT=fDOT+2*fDOT* (eph.Cus * cos(2 * _u) - eph.Cuc * sin(2 * _u));
	double rDOT=eph.e* eph.A* EDOT* sin(E) + 2 * fDOT * (eph.Crs * cos(2 * _u) - eph.Crc * sin(2 * _u));
	double XDOT = rDOT * cos(u) - r * uDOT * sin(u);
	double YDOT = rDOT * sin(u) + r * uDOT * cos(u);


	if (eph.system == BDS && (eph.PRN <= 5 || (eph.PRN >= 59 && eph.PRN <= 63)))
	{
		double LDOT = eph.HatOMEGA;
		double XGK = PosGK[0];
		double YGK = PosGK[1];
		double ZGK = PosGK[2];
		double XGKDOT = cos(L) * (XDOT - y * cos(I) * LDOT) + sin(L) * (-x * LDOT - YDOT * cos(I) + y * sin(I) * iDOT);
		double YGKDOT = sin(L) * (XDOT - y * cos(I) * LDOT) + cos(L) * (x * LDOT + YDOT * cos(I) - y * sin(I) * iDOT);
		double ZGKDOT = YDOT * sin(I) + y * cos(I) * iDOT;
		satpos.XYZVel.x = omega_e * (-XGK * sin(omega_e * tk) + YGK * cos(omega_e * tk) * cos(deg2rad(-5)) + ZGK * cos(omega_e * tk) * sin(deg2rad(-5))) + XGKDOT * cos(omega_e * tk) + sin(omega_e * tk) * (YGKDOT * cos(deg2rad(-5)) + ZGKDOT * sin(deg2rad(-5)));
		satpos.XYZVel.y = omega_e * (-XGK * cos(omega_e * tk) - YGK * sin(omega_e * tk) * cos(deg2rad(-5)) - ZGK * sin(omega_e * tk) * sin(deg2rad(-5))) - XGKDOT * sin(omega_e * tk) + cos(omega_e * tk) * (YGKDOT * cos(deg2rad(-5)) + ZGKDOT * sin(deg2rad(-5)));
		satpos.XYZVel.z = -YGKDOT * sin(deg2rad(-5)) + ZGKDOT * cos(deg2rad(-5));
	}
	else if (eph.system == GPS || (eph.system == BDS && (eph.PRN > 5 && eph.PRN < 59)))  //GPS和北斗MEO和IGSO
	{
		double LDOT = eph.HatOMEGA - omega_e;
		satpos.XYZVel.x = -x * LDOT * sin(L) + XDOT * cos(L) - YDOT * sin(L) * cos(I) - y * (LDOT * cos(L) * cos(I) - iDOT * sin(L) * sin(I));
		satpos.XYZVel.y = x * LDOT * cos(L) + XDOT * sin(L) + YDOT * cos(L) * cos(I) - y * (LDOT * sin(L) * cos(I) + iDOT * cos(L) * sin(I));
		satpos.XYZVel.z = YDOT * sin(I) + y * iDOT * cos(I);
	}
	else;

}
/// <summary>
/// 卫星钟差钟速
/// </summary>
/// <param name="eph">星历</param>
/// <param name="satpos">中间结果</param>
/// <param name="gt">卫星信号发射时刻</param>
/// <param name="tk"></param>
/// <param name="GM"></param>
/// <param name="omega_e"></param>
void _SatClk(const EPH& eph, SATPOS& satpos, GPST &gt, double &tk, double& GM, double &omega_e)
{
	double c = 2.99792458e8;//光速
	//1.计算卫星运动的平均角速度n
	double n0 = sqrt(GM / (eph.A * eph.A * eph.A));
	double n = n0 + eph.Deltan;
	//2.计算观测瞬间卫星的平近点角M
	double M = eph.M0 + n * tk;
	//3.计算偏近点角E
	double E0 = M;
	double E = 0.0;
	double deltaE = E0;
	while (fabs(deltaE) > EPS)
	{
		E = M + eph.e * sin(E0);
		deltaE = E - E0;
		E0 = E;
	}
	//钟差
	double F = -2 * sqrt(GM) / (c * c);
	double dtr = F * eph.e * sqrt(eph.A) * sin(E);
	satpos.clock = eph.a[0] + eph.a[1] * ((tk + eph.toe.SecOfWeek) - eph.toc) + eph.a[2] * ((tk + eph.toe.SecOfWeek) - eph.toc) * ((tk + eph.toe.SecOfWeek) - eph.toc) + dtr;
	//钟速
	double EDOT = n / (1 - eph.e * cos(E));
	satpos.clockVelocity = eph.a[1] + 2 * eph.a[2] * tk + F * eph.e * sqrt(eph.A) * cos(E) * EDOT;

}

/// <summary>
/// 地球自转改正
/// </summary>
/// <param name="x"></param>
/// <param name="omega_e"></param>
/// <param name="dt"></param>
void _RotationCorrection(XYZ& x, double& omega_e, double& dt)
{
	double arrayXk[3] = { x.x ,x.y ,x.z };
	x.x = arrayXk[0] * cos(omega_e * dt) + arrayXk[1] * sin(omega_e * dt);
	x.y = -arrayXk[0] * sin(omega_e * dt) + arrayXk[1] * cos(omega_e * dt);
	x.z = arrayXk[2];
}