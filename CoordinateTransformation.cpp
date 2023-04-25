#include"Coor.h"
using namespace std;
/// <summary>
/// 角度转弧度
/// </summary>
/// <param name="deg">角度</param>
/// <returns>弧度</returns>
double deg2rad(double deg)
{
	return deg* PI / 180.0;
}
/// <summary>
/// 弧度转角度
/// </summary>
/// <param name="rad">弧度</param>
/// <returns>角度</returns>
double rad2deg(double rad)
{
	return rad * 180.0 / PI;
}


/// <summary>
/// XYZ->BLH
/// </summary>
/// <param name="x">XYZ</param>
/// <param name="b">BLH</param>
/// <param name="a">椭球长半轴</param>
/// <param name="alpha">椭球扁率</param>
void XYZ2BLH(const XYZ& x, BLH& b, double a, double alpha)
{
	double e2 = alpha * 2 - alpha * alpha;
	//计算b.l
	if (x.x == 0 && x.y > 0)
		b.l = 90;
	else if(x.x == 0 && x.y < 0)
		b.l = - 90;
	else if (x.x < 0 && x.y >= 0)
	{
		b.l = atan(x.y / x.x);
		b.l = rad2deg(b.l);
		b.l += 180;
	}
	else if (x.x < 0 && x.y <= 0)
	{
		b.l = atan(x.y / x.x);
		b.l = rad2deg(b.l);
		b.l -= 180;
	}
	else
	{
		b.l = atan2(x.y, x.x);
		b.l = rad2deg(b.l);
	}

	//b.b
	double B0= atan(x.z / sqrt(x.x * x.x + x.y * x.y)+1);
	double N = 0;
	double B1 = atan(x.z / sqrt(x.x * x.x + x.y * x.y));
	while (fabs(B0 - B1) > EPS)
	{
		B0 = B1;
		N = a / sqrt((1 - e2 * sin(B0) * sin(B0)));
		b.h = (x.z / sin(B0)) - N * (1 - e2);
		B1 = atan((x.z + N * e2 * sin(B0)) / sqrt(x.x * x.x + x.y * x.y));
	}
	b.b = B1;
	b.b = rad2deg(b.b);
	
}

/// <summary>
/// BLH->XYZ
/// </summary>
/// <param name="b">BLH坐标</param>
/// <param name="x">XYZ坐标</param>
/// <param name="a">椭球长半轴</param>
/// <param name="alpha">椭球扁率</param>
void BLH2XYZ(const BLH& b, XYZ& x, double a, double alpha)
{
	double e2 = alpha * 2 - alpha * alpha;
	double B = deg2rad(b.b);
	double L = deg2rad(b.l);
	double N = a / sqrt(1 - e2  * sin(B) * sin(B));
	x.x = (N + b.h) * cos(B) * cos(L);
	x.y = (N + b.h) * cos(B) * sin(L);
	x.z = (N * (1 - e2) + b.h) * sin(B);
}

/// <summary>
/// XYZ->ENU
/// </summary>
/// <param name="x">XYZ坐标</param>
/// <param name="b">站心的BLH坐标</param>
/// <param name="e">ENU坐标</param>
/// <param name="a">椭球长半轴</param>
/// <param name="alpha">椭球扁率</param>
void XYZ2ENU(const XYZ& x, const BLH& b, ENU& e, double a, double alpha)
{
	XYZ x0 ;
	BLH2XYZ(b, x0, a, alpha);
	e.e = -(x.x - x0.x) * sin(deg2rad(b.l)) + (x.y - x0.y) * cos(deg2rad(b.l));
	e.n = -(x.x - x0.x) * sin(deg2rad(b.b)) * cos(deg2rad(b.l)) - (x.y - x0.y) * sin(deg2rad(b.b)) * sin(deg2rad(b.l)) + (x.z - x0.z) * cos(deg2rad(b.b));
	e.u = (x.x - x0.x) * cos(deg2rad(b.b)) * cos(deg2rad(b.l)) + (x.y - x0.y) * cos(deg2rad(b.b)) * sin(deg2rad(b.l)) + (x.z - x0.z) * sin(deg2rad(b.b));

}
/// <summary>
/// ENU->XYZ
/// </summary>
/// <param name="e">ENU坐标</param>
/// <param name="b">站心的BLH坐标</param>
/// <param name="x">XYZ坐标</param>
/// <param name="a">椭球长半轴</param>
/// <param name="alpha">椭球扁率</param>
void ENU2XYZ(const ENU& e, const BLH& b, XYZ& x, double a, double alpha)
{
	XYZ x0;
	BLH2XYZ(b, x0, a, alpha);

	x.x = -e.e * sin(deg2rad(b.l)) - e.n * sin(deg2rad(b.b)) * cos(deg2rad(b.l)) + e.u * cos(deg2rad(b.b)) * cos(deg2rad(b.l)) + x0.x;
	x.y = e.e * cos(deg2rad(b.l)) - e.n * sin(deg2rad(b.b)) * sin(deg2rad(b.l)) + e.u * cos(deg2rad(b.b)) * sin(deg2rad(b.l)) + x0.y;
	x.z = e.n * cos(deg2rad(b.b)) + e.u * sin(deg2rad(b.b)) + x0.z;

}
