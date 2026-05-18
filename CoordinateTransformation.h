#pragma once
#include<iostream>
#include<Eigen/Dense>
#define my_pi 3.14159265358979
#define R_WGS84  6378137.0
#define E_WGS84 0.081819190842552335
#define R_CGCS2000 6378137.0
#define E_CGCS2000  0.081819191042810976
using namespace std;
using namespace Eigen;
/*笛卡尔坐标系*/
struct XYZ
{
	double X;//单位是m
	double Y;
	double Z;
};
/*大地坐标系*/
struct BLH
{
	double latitude;//纬度,单位是弧度
	double longitude;//经度,单位是弧度
	double height;//大地高
};
/*测站地平坐标*/
struct ENU
{
	double dE;//单位是m
	double dN;
	double dU;
};

//坐标转换算法
void BLHToXYZ(const BLH* blh, XYZ* xyz, double R, const double E);//大地坐标系转换到笛卡尔坐标系
void XYZToBLH(const XYZ* xyz, BLH* blh, double R, double E);//笛卡尔坐标系转换到大地坐标系
MatrixXd BLHToNEUMat(const BLH* blh);//计算测站地平坐标转换矩阵
void BLHToENU(const BLH* blh, ENU* enu);//大地坐标转测站坐标
VectorXd CompSatEIAz(const XYZ* xyz, const double Xs[], const MatrixXd matrixR);//计算卫星的高度角和方位角，Xs是卫星坐标的地心地固坐标，xyz是求解的测站坐标
VectorXd CompEnudPos(const BLH* std, const BLH* xyz, const MatrixXd matrixR);//测站地平系的定位误差