#include "CoordinateTransformation.h"
void BLHToXYZ(const BLH* blh, XYZ* xyz, double R, const double E)//大地坐标系转换到笛卡尔坐标系
{
	double N = R / sqrt(1 - E * E * sin(blh->latitude ) * sin(blh->latitude ));
	xyz->X = (N + blh->height) * cos(blh->latitude) * cos(blh->longitude);
	xyz->Y = (N + blh->height) * cos(blh->latitude) * sin(blh->longitude);
	xyz->Z = (N * (1 - E * E) + blh->height) * sin(blh->latitude );
}
void XYZToBLH(const XYZ* xyz, BLH* blh, double R, double E)//笛卡尔坐标系转换到大地坐标系,返回角度值
{
	blh->longitude = atan2(xyz->Y, xyz->X);
	double delta_Z1 = E * E * xyz->Z;
	double delta_Z2 = 0;
	double sinB, N=0;
	int count = 0;
	while (abs(delta_Z1 - delta_Z2) > 0.00000001&&count<=15)
	{
		delta_Z2 = delta_Z1;
		sinB = (xyz->Z + delta_Z2) / sqrt(pow(xyz->X, 2) + pow(xyz->Y, 2) + pow(xyz->Z + delta_Z2, 2));
		N = R / sqrt(1 - E * E * sinB * sinB);
		delta_Z1 = N * E * E * sinB;//更新delta_Z的值
		count++;
	}
	blh->latitude = atan2(xyz->Z + delta_Z1, sqrt(pow(xyz->X, 2) + pow(xyz->Y, 2)));
	blh->height = sqrt(pow(xyz->X, 2) + pow(xyz->Y, 2) + pow(xyz->Z + delta_Z1, 2)) - N;
}

MatrixXd BLHToNEUMat(const BLH* blh)//计算测站地平坐标转换矩阵
{
	MatrixXd R(3, 3);
	R << -sin(blh->longitude), cos(blh->longitude), 0, -sin(blh->latitude) * cos(blh->longitude),
		-sin(blh->latitude) * sin(blh->longitude), cos(blh->latitude), cos(blh->latitude)* cos(blh->longitude),
		cos(blh->latitude)* sin(blh->longitude), sin(blh->latitude);
	return R;
}

void BLHToENU(const BLH* blh, ENU* enu)//大地坐标转测站坐标
{
	MatrixXd R(3, 3);
	VectorXd Blh(3);
	VectorXd Enu(3);
	R = BLHToNEUMat(blh);
	Blh << blh->latitude, blh->longitude, blh->height;
	Enu = R * Blh;
	enu->dE = Enu[0];
	enu->dN = Enu[1];
	enu->dU = Enu[2];
}

VectorXd CompEnudPos(const BLH*std, const BLH*xyz,const MatrixXd matrixR)//测站地平系的定位误差，X0是测站精确坐标，xyz是解算坐标
{
	VectorXd delta_co(3);
	VectorXd denu(3);
	delta_co << xyz->latitude- std->latitude, xyz->longitude - std->longitude, xyz->height - std->height;
	denu =matrixR * delta_co;
	return denu;
}

VectorXd CompSatEIAz(const XYZ*xyz, const double Xs[],const MatrixXd matrixR)//计算卫星的高度角和方位角，Xs是卫星坐标的地心地固坐标，xyz是求解的测站坐标
{
	VectorXd Sateiaz(2);//储存卫星的高度角和方位角，单位是弧度
	VectorXd denu(3);
	VectorXd delta(3);
	double sateiaz[2];
	delta << Xs[0] - xyz->X, Xs[1] -xyz->Y, Xs[2] - xyz->Z;
	denu = matrixR * delta;
	Sateiaz << atan(denu[2] / sqrt(denu[0] * denu[0] + denu[1] * denu[1])), atan2(denu[0], denu[1]);
	return Sateiaz;
}