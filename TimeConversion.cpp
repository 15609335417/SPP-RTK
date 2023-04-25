#include"Time.h"

using namespace std;
/// <summary>
/// 公历转简化儒略日
/// </summary>
/// <param name="ct">公历</param>
/// <param name="mjd">简化儒略日</param>
void CommonTime2MjdTime(const CommonTime& ct, MjdTime& mjd)
{
	short m = ct.Month;
	short y = ct.Year+4800;
    short d = (short)(ct.Day + (ct.Hour * 3600 + ct.Minute * 60 + ct.Second) / 86400);
    if (m <= 2)
    {
        m = m + 12;
        y = y - 1;
    }
    int e = (int)(floor(30.6 * (m + 1)));
    int a = (int)floor(y / 100);
    int b=0;
    if (y == 1582)
    {
        if (ct.Month < 10)
            b = -38;
        else if (ct.Month == 10)
        {
            if (d < 15)
                b = -38;
        }
    }
    else if(y<1582)
    {
        b = -38;
    }
    else
    {
        b =(int) floor((a / 4.0) - a);
    }
    int c = (int)floor(365.25 * y);
    double jd = b + c + e + d - 32167.5;
    double MJD= jd - 2400000.5;
    mjd.Days = (int)floor(MJD);
    mjd.FracDays = (ct.Hour+ct.Minute/60.0+ct.Second/3600.0)/24.0;
	
}
/// <summary>
/// 简化儒略日转换为公历
/// </summary>
/// <param name="mjd">简化儒略日</param>
/// <param name="ct">公历</param>
void MjdTime2CommonTime(const MjdTime& mjd, CommonTime& ct)
{
    double jd = mjd.Days + mjd.FracDays + 2400000.5;
    short BC = 0;
    double j0 = 0.0;
    double dd = 0;
    short n1 = 0;
    short n2 = 0;
    short n3 = 0;
    short year0=0;
    if (jd < 1721423.5)
    {
        BC = 1;
    }
    else
        BC = 0;

    if (jd < 2299160.5) 
    {
        j0 = floor(jd + 0.5);
        dd = jd + 0.5 - j0;
    }
    else
    {
        n1 = (short)floor((jd - 2342031.5) / 36524.25 / 4) + 1;
        n2 = (short)floor((jd - 2378555.5) / 36524.25 / 4) + 1;
        n3 = (short)floor((jd - 2415079.5) / 36524.25 / 4) + 1;
        j0 = n1 + n2 + n3 + jd + 10;
        dd = j0 + 0.5 - floor(j0 + 0.5);
        j0 = floor(j0 + 0.5);
    
    }   
    j0 = j0 + 32083;
    year0 = (short)ceil(j0 / 365.25) - 1;
    ct.Year = year0 - 4800;
    ct.Day = (short)(j0 - floor(year0 * 365.25));
    ct.Month = (short)floor((ct.Day - 0.6) / 30.6) + 3;
    ct.Day = ct.Day - (int)(round((ct.Month - 3) * 30.6));
    if (ct.Month > 12)
    {
        ct.Month = ct.Month - 12;
        ct.Year = ct.Year + 1;
    }
    ct.Year = ct.Year - BC;
    ct.Second = round(dd * 86400);
    ct.Hour = (short)floor(ct.Second / 3600);
    ct.Second = ct.Second - ct.Hour * 3600;
    ct.Minute = (short)floor(ct.Second / 60);
    ct.Second = ct.Second - ct.Minute * 60;
}
/// <summary>
/// 简化儒略日转GPST
/// </summary>
/// <param name="mjd">简化儒略日</param>
/// <param name="gt">GPST</param>
void MjdTime2GPST(const MjdTime& mjd, GPST& gt)
{
    gt.Week = (short)floor((mjd.Days + mjd.FracDays - 44244) / 7.0);
    gt.SecOfWeek = (mjd.Days  - 44244 - gt.Week * 7+ mjd.FracDays) * 86400;
}
/// <summary>
/// GPST转简化儒略日
/// </summary>
/// <param name="gt">GPST</param>
/// <param name="mjd">简化儒略日</param>
void GPST2MjdTime(const GPST& gt, MjdTime& mjd)
{
    mjd.Days = (int)floor(44244 + gt.Week * 7 + gt.SecOfWeek / 86400.0);
    mjd.FracDays = 44244 + gt.Week * 7 + gt.SecOfWeek / 86400.0 - mjd.Days;
}
/// <summary>
/// 公历转GPST
/// </summary>
/// <param name="ct"></param>
/// <param name="gt"></param>
void CommonTime2GPST(const CommonTime& ct, GPST& gt)
{
    MjdTime mjd;
    CommonTime2MjdTime(ct, mjd);
    MjdTime2GPST(mjd, gt);
}
/// <summary>
/// GPST转公历
/// </summary>
/// <param name="gt">GPST</param>
/// <param name="ct">公历</param>
void GPST2CommonTime(const GPST& gt, CommonTime& ct)
{
    MjdTime mjd;
    GPST2MjdTime(gt, mjd);
    MjdTime2CommonTime(mjd, ct);
}