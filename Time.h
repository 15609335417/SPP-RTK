#pragma once
#ifndef Time_H

#define Time_H

#include<iostream>
#include<cmath>
//通用时
struct CommonTime
{
	short Year;
	short Month;
	short Day;
	short Hour;
	short Minute;
	double Second;
	CommonTime()
	{
		 Year=0;
		 Month=0;
		 Day=0;
		 Hour=0;
		 Minute=0;
		 Second = 0.0;
	}
};
//简化儒略日
struct MjdTime
{
	int Days;
	double FracDays;
	MjdTime()
	{
		Days = 0;
		FracDays = 0.0;
	}

};
//GPS时
struct GPST
{
	unsigned short Week;
	double SecOfWeek;
	GPST()
	{
		Week = 0;
		SecOfWeek=0.0;
	}
};


//时间转换函数

void CommonTime2MjdTime(const CommonTime& ct, MjdTime& mjd);
void MjdTime2CommonTime(const MjdTime& mjd, CommonTime& ct);
void MjdTime2GPST(const MjdTime& mjd, GPST& gt);
void GPST2MjdTime(const GPST& gt, MjdTime& mjd);
void CommonTime2GPST(const CommonTime& ct, GPST& gt);
void GPST2CommonTime(const GPST& gt, CommonTime& ct);

#endif // !Time_H
