#include "TimeConversion.h"
/*时间转换算法*/
void CommonTimeToMJD(const CommonTime* ct, MJDTime* mjd)
{
	short y = 0;
	short m = 0;
	if (ct->month <= 2)
	{
		y = ct->year - 1;
		m = ct->month + 12;
	}
	else
	{
		y = ct->year;
		m = ct->month;
	}
	//计算儒略历
	double JD = int(365.25 * y) + int(30.6001 * (m + 1)) + ct->day
		+ (ct->hour) / 24.0 + (ct->minute) / 60.0 / 24.0 + 
		(ct->second) / 3600.0 / 24.0 + 1720981.5;
	//计算简化儒略历
	mjd->days = int(JD - 2400000.5);
	mjd->fracday =  (ct->hour) / 24.0 + (ct->minute) / 60.0 / 24.0 + 
		(ct->second) / 3600.0 / 24.0;
}

void MJDToGPS(const MJDTime* mjd, GPSTime* gps)
{
	gps->week = int((mjd->days + mjd->fracday - 44244) / 7);
	gps->secofweek = (mjd->days + mjd->fracday - 44244 - gps->week * 7) * 86400;
}

void CommonTimeToGPS(const CommonTime* ct, GPSTime* gps)
{
	short y = 0;
	short m = 0;
	if (ct->month <= 2)
	{
		y = ct->year - 1;
		m = ct->month + 12;
	}
	else
	{
		y = ct->year;
		m = ct->month;
	}
	//计算儒略历
	double JD = int(365.25 * y) + int(30.6001 * (m + 1)) + ct->day + (ct->hour) / 24.0 + (ct->minute) / 60.0 / 24.0 + (ct->second) / 3600.0 / 24.0 + 1720981.5;
	//计算简化儒略历
	double MJD_days= int(JD - 2400000.5);
	double MJD_frac = (ct->hour) / 24.0 + (ct->minute) / 60.0 / 24.0 + (ct->second) / 3600.0 / 24.0;
	//将简化儒略历转换到GPS时
	gps->week = int((MJD_days+MJD_frac - 44244) / 7);
	gps->secofweek = (MJD_days + MJD_frac - 44244 - gps->week * 7) * 86400;
}
void MJDToCommonTime(const MJDTime* mjd,  CommonTime* ct)
{
	double JD = mjd->days + mjd->fracday + 2400000.5;
	double frac = mjd->fracday;//计算日的小数部分
	double a = int(JD + 0.5);//整数部分
	double b = a + 1537;
	double c = int((b - 122.1) / 365.25);
	double d = int(365.25 * c);
	double e = int((b - d) / 30.6001);
	ct->day = b - d - int(30.6001 * e);//日
	ct->month = e - 1 - 12 * int(e / 14);//月
	ct->year = c - 4715 - int((7 + ct->month) / 10);//年
	ct->hour = int(frac * 24);
	ct->minute = int((frac * 24 - ct->hour) * 60);
	ct->second = ((frac * 24 - ct->hour) * 60 - ct->minute) * 60;
}
void GPSToMJD(const GPSTime* gps, MJDTime* mjd)//GPS时转换到简化儒略历
{
	mjd->days = 44244 + gps->week * 7 + int(gps->secofweek / 86400.0);
	mjd->fracday = gps->secofweek / 86400.0 - int(gps->secofweek / 86400.0);
}

void GPSToCommonTime(const GPSTime* gps, CommonTime* ct)//GPS时转换到通用时
{
	double MJD = 44244 + gps->week * 7 + gps->secofweek / 86400.0;
	double JD = MJD + 2400000.5;
	double frac = gps->secofweek / 86400.0 - int(gps->secofweek / 86400.0);//计算日的小数部分
	double a = int(JD + 0.5);//整数部分
	double b = a + 1537;
	double c = int((b - 122.1) / 365.25);
	double d = int(365.25 * c);
	double e = int((b - d) / 30.6001);
	ct->day = b - d - int(30.6001 * e);//日
	ct->month = e - 1 - 12 * int(e / 14);//月
	ct->year = c - 4715 - int((7 + ct->month) / 10);//年
	ct->hour = int(frac * 24);
	ct->minute = int((frac * 24 - ct->hour) * 60);
	ct->second = ((frac * 24 - ct->hour) * 60 - ct->minute) * 60;
}