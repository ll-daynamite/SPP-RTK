#pragma once
#include<iostream>
using namespace std;
/*通用时间定义*/
struct CommonTime
{
	short year;
	unsigned short month;
	unsigned short day;
	unsigned short hour;
	unsigned short minute;
	double second;
};

/*简化儒略历*/
struct MJDTime
{
	int days;
	double fracday;

	MJDTime()
	{
		days = 0;
		fracday = 0.0;
	}
};

/*GPS时间定义*/
struct GPSTime
{
	unsigned short week;
	double secofweek;

	GPSTime()
	{
		week = 0;
		secofweek = 0.0;
	}
};
/*时间转换算法*/
void CommonTimeToMJD(const CommonTime* ct, MJDTime* mjd);//通用时转换到简化儒略历
void MJDToGPS(const MJDTime* mjd, GPSTime* gps);//简化儒略历转换到GPS时
void CommonTimeToGPS(const CommonTime* ct, GPSTime* gps);//通用时转换到GPS时
void MJDToCommonTime(const MJDTime* mjd, CommonTime* ct);//简化儒略历转换到通用时
void GPSToMJD(const GPSTime* gps, MJDTime* mjd);//GPS时转换到儒略历
void GPSToCommonTime(const GPSTime* gps, CommonTime* ct);//GPS时转换到通用时



