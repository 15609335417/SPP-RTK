#include"SatellitePostion.h"
/// <summary>
/// ��������λ�ã��ٶȣ��Ӳ����
/// </summary>
/// <param name="eph">����</param>
/// <param name="obs">�۲�ֵ�к����м����ṹ��</param>
void _SatPos(const EPH* eph, OBS& obs)
{
	//�����������
	//Ϊ��֮����в�������
	double GM = GM_GPS;//������������
	double omega_e = omega_e_GPS;//������ת���ٶ�
	double a = a_GPS;//���򳤰���
	double alpha = alpha_GPS;//�������
	//�����Ƿ����
	bool datavalidity = false;
	int j = 0;
	double tk = 0;
 	for (int i = 0; i < obs.SatNum; i++)//OBS
	{
		//�ڴֲ�̽��֮��û�дֲ������״̬����Ϊ1���ٴν�������λ�ý��㣬����ɹ�����1,ʧ����0
		/*������ǹ۲�ֵ״̬*/
		if (obs.satpos[i].Status == 0|| obs.satpos[i].Status ==2|| obs.satpos[i].Status == -1)continue;//״̬���õĲ��������
		/*�������j=obs.satpos[i].PRN - 1;*/
		if (obs.satpos[i].system == GPS)
			j = obs.satpos[i].PRN - 1;
		else if (obs.satpos[i].system == BDS)
			j = obs.satpos[i].PRN - 1 + MaxGPSSatNum;
		/*�ж�PRN�Ƿ�ϸ�*/
		if (eph[j].PRN <1||eph[j].PRN>63) {
			obs.satpos[i].Status = 0; continue;
		}
		/*�ж������Ƿ񽡿�*/
		if (eph[j].Health != 0) { obs.satpos[i].Status = 0; continue; }//�����������Լ�û�еİ���������
		/*���������źŷ����ʱ�̣����ʼ�������Ӳ�Ϊ��*/
		GPST Ttr;
		Ttr.Week = obs.gt.Week;
		Ttr.SecOfWeek = obs.gt.SecOfWeek - obs.satobs[i].C[0] / Cspeed - obs.satpos[i].clock;
		//��������
		_ParameterSetting(eph[j].system, a, alpha, GM, omega_e);
		//�����Ƿ���� //������ڣ�tk=0;
		tk = _DataValidity(eph[j], Ttr);
		if (!(fabs(tk) > EPS)) { obs.satpos[i].Status = 0; continue; }
		/*�״μ���ʱ���ٴ�����ǰ������Ttr����õ������Ӳ�̶��ٴμ���õ������źŷ���ʱ�̵�GPST*/
		if (fabs(obs.satpos[i].clock) < EPS)
		{
			/*�״μ���ʱ���������������Ttr���������Ӳ�*/
			_SatClk(eph[j], obs.satpos[i], Ttr, tk, GM, omega_e);
			/*�ٴμ��������źŷ����ʱ��*/
			Ttr.SecOfWeek = obs.gt.SecOfWeek - obs.satobs[i].C[0] / Cspeed - obs.satpos[i].clock;
			tk = _DataValidity(eph[j], Ttr);
			if (!(fabs(tk) > EPS)) { obs.satpos[i].Status = 0; continue; }
		}
		//��������λ�ú��ٶ�
		_SatPosVel(eph[j], obs.satpos[i], Ttr, tk, GM, omega_e);
		//�����Ӳ������
		_SatClk(eph[j], obs.satpos[i], Ttr, tk, GM, omega_e);
		//�������������״̬����Ϊ1
		obs.satpos[i].Status = 1;
	}
}

/// <summary>
/// �����Ƿ����
/// </summary>
/// <param name="eph">����</param>
/// <param name="gt">���Ƿ���ʱ��</param>
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
	//��Ч�Լ��
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
/// ��������
/// </summary>
/// <param name="system">����ϵͳ</param>
/// <param name="a">���򳤰���</param>
/// <param name="alpha">�������</param>
/// <param name="GM">������������</param>
/// <param name="omega_e">������ת���ٶ�</param>
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
/// ����λ���ٶ�
/// </summary>
/// <param name="eph">����</param>
/// <param name="satpos">�м�ṹ��</param>
/// <param name="gt">�����źŷ���ʱ��</param>
/// <param name="tk"></param>
/// <param name="GM"></param>
/// <param name="omega_e"></param>
void _SatPosVel(const EPH& eph, SATPOS& satpos, GPST &gt, double &tk, double& GM, double &omega_e)
{
	//1.���������˶���ƽ�����ٶ�n
	double n0 = sqrt(GM / (eph.A * eph.A * eph.A));
	double n = n0 + eph.Deltan;
	//2.����۲�˲�����ǵ�ƽ�����M
	double M = eph.M0 + n * tk;
	//3.����ƫ�����E
	double E0 = M;
	double E = 0.0;
	double deltaE = E0;
	while (fabs(deltaE) > EPS)
	{
		E = M + eph.e * sin(E0);
		deltaE = E - E0;
		E0 = E;
	}
	//4.����������f  ��Χ[-pi,pi]
	double f = atan2(sqrt(1 - eph.e * eph.e) * sin(E), (cos(E) - eph.e));
	//5.���������Ǿ�u'
	double _u = eph.omega + f;
	//6.�����㶯������
	double du = eph.Cuc * cos(2 * _u) + eph.Cus * sin(2 * _u);
	double dr = eph.Crc * cos(2 * _u) + eph.Crs * sin(2 * _u);
	double di = eph.Cic * cos(2 * _u) + eph.Cis * sin(2 * _u);
	//7.��u',r',i0���и���
	double u = _u + du;
	double r = eph.A * (1 - eph.e * cos(E)) + dr;
	double I = eph.i0 + di + eph.Hati * tk;
	//8.���������ڹ��������ϵ�е�����
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
		//GEO����� 1 2 3 4 5 59 60 61 62 63
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
	else if (eph.system == GPS || (eph.system == BDS && (eph.PRN > 5 && eph.PRN < 59)))  //GPS�ͱ���MEO��IGSO
	{
		double LDOT = eph.HatOMEGA - omega_e;
		satpos.XYZVel.x = -x * LDOT * sin(L) + XDOT * cos(L) - YDOT * sin(L) * cos(I) - y * (LDOT * cos(L) * cos(I) - iDOT * sin(L) * sin(I));
		satpos.XYZVel.y = x * LDOT * cos(L) + XDOT * sin(L) + YDOT * cos(L) * cos(I) - y * (LDOT * sin(L) * cos(I) + iDOT * cos(L) * sin(I));
		satpos.XYZVel.z = YDOT * sin(I) + y * iDOT * cos(I);
	}
	else;

}
/// <summary>
/// �����Ӳ�����
/// </summary>
/// <param name="eph">����</param>
/// <param name="satpos">�м���</param>
/// <param name="gt">�����źŷ���ʱ��</param>
/// <param name="tk"></param>
/// <param name="GM"></param>
/// <param name="omega_e"></param>
void _SatClk(const EPH& eph, SATPOS& satpos, GPST &gt, double &tk, double& GM, double &omega_e)
{
	double c = 2.99792458e8;//����
	//1.���������˶���ƽ�����ٶ�n
	double n0 = sqrt(GM / (eph.A * eph.A * eph.A));
	double n = n0 + eph.Deltan;
	//2.����۲�˲�����ǵ�ƽ�����M
	double M = eph.M0 + n * tk;
	//3.����ƫ�����E
	double E0 = M;
	double E = 0.0;
	double deltaE = E0;
	while (fabs(deltaE) > EPS)
	{
		E = M + eph.e * sin(E0);
		deltaE = E - E0;
		E0 = E;
	}
	//�Ӳ�
	double F = -2 * sqrt(GM) / (c * c);
	double dtr = F * eph.e * sqrt(eph.A) * sin(E);
	satpos.clock = eph.a[0] + eph.a[1] * ((tk + eph.toe.SecOfWeek) - eph.toc) + eph.a[2] * ((tk + eph.toe.SecOfWeek) - eph.toc) * ((tk + eph.toe.SecOfWeek) - eph.toc) + dtr;
	//����
	double EDOT = n / (1 - eph.e * cos(E));
	satpos.clockVelocity = eph.a[1] + 2 * eph.a[2] * tk + F * eph.e * sqrt(eph.A) * cos(E) * EDOT;

}

/// <summary>
/// ������ת����
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