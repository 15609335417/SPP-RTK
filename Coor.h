#pragma once
#ifndef Coor_H

#define Coor_H
#define PI 3.141592653589793
#define EPS 1e-10

#include<iostream>
#include<stdio.h>
#include<cmath>

// 坐标系统
struct XYZ
{
	double x;
	double y;
	double z;
	XYZ()
	{
		x = y = z = 0.0;
	}
};
struct BLH
{
	double b;//deg
	double l;//deg
	double h;
	BLH()
	{
		b = l = h = 0.0;
	}

};
struct ENU
{
	double e;
	double n;
	double u;
	ENU()
	{
		e = n = u = 0.0;
	}
};
struct NED
{
	double n;
	double e;
	double d;
	NED()
	{
		e = n = d = 0.0;
	}
};
//定义联合
union _blh
{
	BLH blh;
	double arrayblh[3];
	_blh()
	{
		for (int i = 0; i < 3; i++)
		{
			arrayblh[i] = 0.0;
		}
	}
	_blh(double a,double b,double c)
	{
		arrayblh[0] = a; arrayblh[1] = b; arrayblh[2] = c;
	}
};

union _xyz
{
	XYZ xyz;
	double arrayxyz[3];
	_xyz()
	{
		for (int i = 0; i < 3; i++)
		{
			arrayxyz[i] = 0.0;
		}
	}
	_xyz(double a, double b, double c)
	{
		arrayxyz[0] = a; arrayxyz[1] = b; arrayxyz[2] = c;
	}
};

union _enu
{
	ENU enu;
	double arrayenu[3];
	_enu()
	{
		for (int i = 0; i < 3; i++)
		{
			arrayenu[i] = 0.0;
		}
	}
	_enu(double a, double b, double c)
	{
		arrayenu[0] = a; arrayenu[1] = b; arrayenu[2] = c;
	}
};
union _ned
{
	NED ned;
	double arrayned[3];
	_ned()
	{
		for (int i = 0; i < 3; i++)
		{
			arrayned[i] = 0.0;
		}
	}
	_ned(double a, double b, double c)
	{
		arrayned[0] = a; arrayned[1] = b; arrayned[2] = c;
	}
};

double deg2rad(double deg);
double rad2deg(double rad);
void XYZ2BLH(const XYZ& x, BLH& b, double a, double alpha);
void BLH2XYZ(const BLH& b, XYZ& x, double a, double alpha);
void XYZ2ENU(const XYZ& x, const BLH& b, ENU& e, double a, double alpha);
void ENU2XYZ(const ENU& e, const BLH& b, XYZ& x, double a, double alpha);

inline double Norm(const XYZ& x, const XYZ& x1)
{
	return sqrt(pow(x.x - x1.x, 2) + pow(x.y - x1.y, 2) + pow(x.z - x1.z, 2));
}
inline double Norm(const XYZ& x)
{
	return sqrt(pow(x.x, 2) + pow(x.y, 2) + pow(x.z, 2));
}
//使用联合，但没有debug
inline double Norm(const _xyz& x, const _xyz& x1)
{
	double sum = 0.0;
	for (int i = 0; i < 3; i++)
	{
		sum += pow(x.arrayxyz[i] - x1.arrayxyz[i], 2);
	}
	return sum;
}
inline double Norm(const _xyz& x)
{
	double sum = 0.0;
	for (int i = 0; i < 3; i++)
	{
		sum += pow(x.arrayxyz[i], 2);
	}
	return sum;
}

#endif // !Coor_H
