#pragma once
#include"Decode.h"
#include"Position.h"
#include<Eigen/Dense>
#include<iomanip>
using namespace Eigen;
void ComputeSatPVTAtSignalTrans(const EPOCHOBSDATA*
	Epk, GPSEPHREC* Eph, GPSEPHREC* BDSEph, double UserPos[3]);//计算信号发射时刻的卫星位置
bool SPP(
	EPOCHOBSDATA* Epoch,
	GPSEPHREC* GPSEph,
	GPSEPHREC* BDSEph,
	POSRES* Res,
	bool writeObservationLog = true);//单点定位
bool SPV(EPOCHOBSDATA* Epoch,POSRES* Res);//单点测速
