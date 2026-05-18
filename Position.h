#pragma once
#include<cmath>
#include"Decode.h"
//#define pi 3.1415926535898
#include"CoordinateTransformation.h"
#define C_Light 299792458.0      
#define GPS_GM 3.986005e+14
#define GPS_EarthRotation 7.2921151467e-5
#define BDS_GM 3.986004418e+14
#define BDS_EarthRotation 7.2921150e-5
using namespace std;


bool CompSatClkOff(const int prn, const GNSSSys Sys, const GPSTime* t,
	GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATPVT* Mid);//计算钟差和钟速
int CompGPSSatPVT(const int prn, const GPSTime* t, const GPSEPHREC* Eph, SATPVT* Mid);
int CompBDSSatPVT(const int prn, const GPSTime* t, const GPSEPHREC* Eph, SATPVT* Mid);
int CompEarthRotationCorr(GNSSSys sys, double user[], SATPVT* Mid);//计算地球自转改正
