#include "Position.h"
#include<Eigen/Dense>
using namespace Eigen;

bool CompSatClkOff(const int Prn, const GNSSSys Sys, const GPSTime* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATPVT* Mid)
{
	double dt, LimT = 7500.0;
	GPSTime CurT;
	GPSEPHREC* EPH;

	CurT = *t;
	if (Sys == GPS) EPH = GPSEph + Prn - 1;
	else if (Sys == BDS) {
		EPH = BDSEph + Prn - 1;
		CurT.week -= 1356;
		CurT.secofweek -= 14;
		LimT = 3900.0;
	}
	else return false;

	dt = (CurT.week - EPH->TOC.week) * 604800.0 + CurT.secofweek - EPH->TOC.secofweek;
	if (fabs(dt) > LimT || EPH->SVHealth != 0) return false;

	//计算卫星钟差和钟速
	Mid->SatClkOft = EPH->ClkBias + EPH->ClkDrift * dt + EPH->ClkDriftRate * pow(dt, 2);
	Mid->SatClkSft = EPH->ClkDrift + 2 * EPH->ClkDriftRate * ((t->week - EPH->TOC.week) * 86400 + t->secofweek - EPH->TOC.secofweek);
	Mid->Valid = true;
	return true;
}


int CompGPSSatPVT(const int prn, const GPSTime* t, const GPSEPHREC* Ephp, SATPVT* Mid)
{
	const GPSEPHREC* Eph = Ephp;
	if (!Eph->SVHealth)
	{
		/*计算卫星坐标*/
		//1.计算轨道长半轴
		double A = pow(Eph->SqrtA, 2);
		//2.计算平均运动角速度
		double n0 = sqrt(GPS_GM / pow(A, 3));
		//3.计算相对星历参考历元的时间
		double tk = (t->week - Eph->TOE.week) * 604800.0 + (t->secofweek - Eph->TOE.secofweek);
		//4.对平均角速度进行改正
		double n = n0 + Eph->DeltaN;
		//5.计算平近点角
		double Mk = Eph->M0 + n * tk;
		//6.迭代计算偏近点角
		double Ek = 0;
		//double Ek_mid = Mk;
		int count = 0;//迭代次数
		//while (abs(Ek_mid - Ek) > 1e-12&&count<=10)
		//{
		//	Ek = Mk + Eph->e * sin(Ek_mid);
		//	//更新迭代中间值
		//	Ek_mid = Ek;
		//	count++;
		//}
		double Ek_prev = Mk;
		Ek = Mk + Eph->e * sin(Ek_prev);
		while (abs(Ek - Ek_prev) > 1e-12 && count <= 10) 
		{
			Ek_prev = Ek;
			Ek = Mk + Eph->e * sin(Ek_prev);
			count++;
		}
		//7.计算真近点角
		double vk = atan2((sqrt(1 - pow(Eph->e, 2)) * sin(Ek)) / (1 - Eph->e * cos(Ek)), (cos(Ek) - Eph->e) / (1 - Eph->e * cos(Ek)));
		//8.计算升交角距
		double fie_k = vk + Eph->omega;
		//9.计算二阶调和改正数
		double delta_uk = Eph->Cus * sin(2 * fie_k) + Eph->Cuc * cos(2 * fie_k);
		double delta_rk = Eph->Crs * sin(2 * fie_k) + Eph->Crc * cos(2 * fie_k);
		double delta_ik = Eph->Cis * sin(2 * fie_k) + Eph->Cic * cos(2 * fie_k);
		//10.计算经过改正的升交角距
		double uk = fie_k + delta_uk;
		//11.计算经过改正的向径
		double rk = A * (1 - Eph->e * cos(Ek)) + delta_rk;
		//12.计算经过改正的轨道倾角
		double ik = Eph->i0 + delta_ik + tk * Eph->idot;
		//13.计算卫星在轨道平面上的位置
		double xk_ = rk * cos(uk);
		double yk_ = rk * sin(uk);
		//14.计算改正后的升交点经度
		double omega_k = Eph->OMEGA + (Eph->OMEGADot - GPS_EarthRotation) * tk - GPS_EarthRotation * Eph->TOE.secofweek;
		//15.计算地固坐标系下卫星的位置
		double xk = xk_ * cos(omega_k) - yk_ * cos(ik) * sin(omega_k);
		double yk = xk_ * sin(omega_k) + yk_* cos(ik) * cos(omega_k);
		double zk = yk_ * sin(ik);
		Mid->SatPos[0] = xk;
		Mid->SatPos[1] = yk;
		Mid->SatPos[2] = zk;

		/*钟差精确计算*/
		double tr = -2 * sqrt(GPS_GM) * Eph->e * Eph->SqrtA * sin(Ek) / pow(C_Light, 2);
		Mid->SatClkOft += tr;

		/*卫星运动速度计算*/
		double Ek_dot = n / (1 - Eph->e * cos(Ek));
		double fie_k_dot = sqrt(1 - pow(Eph->e, 2)) * Ek_dot / (1 - Eph->e * cos(Ek));
		double uk_dot = 2 * (Eph->Cus * cos(2 * fie_k) - Eph->Cuc * sin(2 * fie_k)) * fie_k_dot + fie_k_dot;
		double rk_dot = A * Eph->e * sin(Ek) * Ek_dot + 2 * (Eph->Crs * cos(2 * fie_k) - Eph->Crc * sin(2 * fie_k)) * fie_k_dot;
		double Ik_dot = Eph->idot + 2 * (Eph->Cis * cos(2 * fie_k) - Eph->Cic * sin(2 * fie_k)) * fie_k_dot;
		double omega_k_dot = Eph->OMEGADot - GPS_EarthRotation;
		MatrixXd R_dot(3, 4);
		R_dot << cos(omega_k), -1 * sin(omega_k) * cos(ik), -1 * (xk_ * sin(omega_k) + yk_ * cos(omega_k) * cos(ik)), yk_* sin(omega_k)* sin(ik),
			sin(omega_k), cos(omega_k)* cos(ik), (xk_ * cos(omega_k) - yk_ * sin(omega_k) * cos(ik)), -1 * yk_ * cos(omega_k) * sin(ik),
			0, sin(ik), 0, yk_*cos(ik);
		double xk_dot = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
		double yk_dot = rk_dot * sin(uk) + rk * uk_dot * cos(uk);
		MatrixXd mid(4, 1);
		MatrixXd vel(3, 1);
		mid << xk_dot, yk_dot, omega_k_dot, Ik_dot;
		vel = R_dot * mid;
		Mid->SatVel[0] = vel(0,0);
		Mid->SatVel[1] = vel(1,0);
		Mid->SatVel[2] = vel(2,0);

		/*卫星钟速改正*/
		double tr_dot = -2 * sqrt(GPS_GM) * Eph->e * Eph->SqrtA * cos(Ek) * Ek_dot / pow(C_Light, 2);
		Mid->SatClkSft += tr_dot;

		/*硬件延迟*/
		Mid->Tgd1 = Eph->TGD1;
		Mid->Tgd2 = Eph->TGD2;
		Mid->URA = Eph->URA;
		Mid->prn = Eph->PRN;
		/*解算成功*/
		Mid->Valid = true;
		return 1;
	}
	else
	{
		Mid->Valid = false;
		return 0;
	}
}

int CompBDSSatPVT(const int prn, const GPSTime* t, const GPSEPHREC* Ephp, SATPVT* Mid)
{
	const GPSEPHREC* Eph = Ephp;
	if (!Eph->SVHealth)
	{
		/*计算卫星坐标*/
		//1.计算轨道长半轴
		double A = pow(Eph->SqrtA, 2);
		//2.计算平均运动角速度
		double n0 = sqrt(BDS_GM / pow(A, 3));
		//3.计算相对星历参考历元的时间
		double tk = (t->week - 1356 - Eph->TOE.week) * 604800.0 + (t->secofweek - 14 - Eph->TOE.secofweek);
		//4.对平均角速度进行改正
		double n = n0 + Eph->DeltaN;
		//5.计算平近点角
		double Mk = Eph->M0 + n * tk;
		//6.迭代计算偏近点角
		double Ek = 0;
		double Ek_mid = Mk;
		int count = 0;//迭代次数
		while (abs(Ek_mid - Ek) > 1e-12 && count <= 100)
		{
			//更新迭代中间值
			Ek_mid = Ek;
			Ek = Mk + Eph->e * sin(Ek_mid);			
			count++;
		}
		//7.计算真近点角
		double vk = atan2((sqrt(1 - pow(Eph->e, 2)) * sin(Ek)) , (cos(Ek) - Eph->e));
		//8.计算升交角距
		double fie_k = vk + Eph->omega;
		//9.计算二阶调和改正数
		double delta_uk = Eph->Cus * sin(2 * fie_k) + Eph->Cuc * cos(2 * fie_k);
		double delta_rk = Eph->Crs * sin(2 * fie_k) + Eph->Crc * cos(2 * fie_k);
		double delta_ik = Eph->Cis * sin(2 * fie_k) + Eph->Cic * cos(2 * fie_k);
		//10.计算经过改正的升交角距
		double uk = fie_k + delta_uk;
		//11.计算经过改正的向径
		double rk = A * (1 - Eph->e * cos(Ek)) + delta_rk;
		//12.计算经过改正的轨道倾角
		double ik = Eph->i0 + delta_ik + tk * Eph->idot;
		//13.计算卫星在轨道平面上的位置
		double xk_ = rk * cos(uk);
		double l = rk * sin(uk);
		double yk_ = rk * sin(uk);

		/*根据卫星类型分别计算*/
		if (Eph->i0 < (30 * my_pi / 180.0))//GEO卫星
		{
			//计算历元升交点经度
			double omega_k = Eph->OMEGA + Eph->OMEGADot * tk - BDS_EarthRotation * (Eph->TOE.secofweek);

			//计算GEO卫星在自定义坐标系中的坐标
			double X_GK = xk_ * cos(omega_k) - yk_ * cos(ik) * sin(omega_k);
			double Y_GK = xk_ * sin(omega_k) + yk_ * cos(ik) * cos(omega_k);
			double Z_GK = yk_ * sin(ik);
			MatrixXd GK(3, 1);
			GK << X_GK, Y_GK, Z_GK;
			//计算GEO卫星在BDCS坐标系中的坐标
			MatrixXd Rz(3, 3), Rx(3, 3);
			double fiez = BDS_EarthRotation * tk;
			double fiex = -5 * my_pi / 180.0;
			Rx << 1, 0, 0, 0, cos(fiex), sin(fiex), 0, -1 * sin(fiex), cos(fiex);
			Rz << cos(fiez), sin(fiez), 0, -1 * sin(fiez), cos(fiez), 0, 0, 0, 1;
			MatrixXd K(3, 1);
			K = Rz * Rx * GK;
			Mid->SatPos[0] = K(0, 0);
			Mid->SatPos[1] = K(1, 0);
			Mid->SatPos[2] = K(2, 0);

			//计算GEO卫星速度
			//计算历元升交点经度变化率（惯性系）
			double omega_k_dot = Eph->OMEGADot;

			//计算导数
			double Ek_dot = n / (1 - Eph->e * cos(Ek));
			double fie_k_dot = sqrt(1 - pow(Eph->e, 2)) * Ek_dot / (1 - Eph->e * cos(Ek));
			double uk_dot = 2 * (Eph->Cus * cos(2 * fie_k) - Eph->Cuc * sin(2 * fie_k)) * fie_k_dot + fie_k_dot;
			double rk_dot = A * Eph->e * sin(Ek) * Ek_dot + 2 * (Eph->Crs * cos(2 * fie_k) - Eph->Crc * sin(2 * fie_k)) * fie_k_dot;
			double Ik_dot = Eph->idot + 2 * (Eph->Cis * cos(2 * fie_k) - Eph->Cic * sin(2 * fie_k)) * fie_k_dot;
			double xk_dot = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
			double yk_dot = rk_dot * sin(uk) + rk * uk_dot * cos(uk);
			MatrixXd Rz_dot(3, 3), vel(3, 1), RGK_dot(3, 1);
			Rz_dot << -sin(fiez)*BDS_EarthRotation, cos(fiez)* BDS_EarthRotation, 0, -cos(fiez)* BDS_EarthRotation, -sin(fiez)* BDS_EarthRotation, 0, 0, 0, 0;

			double X_GK_dot = xk_dot * cos(omega_k) - omega_k_dot * xk_ * sin(omega_k) - yk_dot * cos(ik) * sin(omega_k)
				+ Ik_dot * yk_ * sin(ik) * sin(omega_k) - omega_k_dot * yk_ * cos(ik) * cos(omega_k);
			double Y_GK_dot = xk_dot * sin(omega_k) + omega_k_dot * xk_ * cos(omega_k) + yk_dot * cos(ik) * cos(omega_k)
				- Ik_dot * yk_ * sin(ik) * cos(omega_k) - omega_k_dot * yk_ * cos(ik) * sin(omega_k);
			double Z_GK_dot = yk_dot * sin(ik) + Ik_dot * yk_ * cos(ik);
			RGK_dot << X_GK_dot, Y_GK_dot, Z_GK_dot;
			//计算速度
			vel = Rz * Rx * RGK_dot + Rz_dot * Rx * GK;
			Mid->SatVel[0] = vel(0,0);
			Mid->SatVel[1] = vel(1,0);
			Mid->SatVel[2] = vel(2,0);
			/*钟差精确计算*/
			double tr = -2 * sqrt(BDS_GM) * Eph->e * Eph->SqrtA * sin(Ek) / pow(C_Light, 2);
			Mid->SatClkOft += tr;
			/*卫星钟速改正*/
			double tr_dot = -2 * sqrt(BDS_GM) * Eph->e * Eph->SqrtA * cos(Ek) * Ek_dot / pow(C_Light, 2);
			Mid->SatClkSft += tr_dot;

			/*硬件延迟*/
			Mid->Tgd1 = Eph->TGD1;
			Mid->Tgd2 = Eph->TGD2;
			Mid->URA = Eph->URA;
			Mid->prn = Eph->PRN;
			//解算成功
			Mid->Valid = true;
			return 1;
		}
		else
		{
			//计算历元升交点经度（地固系）
			double omega_k = Eph->OMEGA + (Eph->OMEGADot - BDS_EarthRotation) * tk - BDS_EarthRotation * (Eph->TOE.secofweek);
			//计算MEO/IGSO卫星在BDCS坐标系中的坐标
			Mid->SatPos[0] = xk_ * cos(omega_k) - yk_ * cos(ik) * sin(omega_k);
			Mid->SatPos[1] = xk_ * sin(omega_k) + yk_ * cos(ik) * cos(omega_k);
			Mid->SatPos[2] = yk_ * sin(ik);

			//计算卫星速度
			double Ek_dot = n / (1 - Eph->e * cos(Ek));
			double fie_k_dot = sqrt(1 - pow(Eph->e, 2)) * Ek_dot / (1 - Eph->e * cos(Ek));
			double uk_dot = 2 * (Eph->Cus * cos(2 * fie_k) - Eph->Cuc * sin(2 * fie_k)) * fie_k_dot + fie_k_dot;
			double rk_dot = A * Eph->e * sin(Ek) * Ek_dot + 2 * (Eph->Crs * cos(2 * fie_k) - Eph->Crc * sin(2 * fie_k)) * fie_k_dot;
			double Ik_dot = Eph->idot + 2 * (Eph->Cis * cos(2 * fie_k) - Eph->Cic * sin(2 * fie_k)) * fie_k_dot;
			double omega_k_dot = Eph->OMEGADot - BDS_EarthRotation;
			MatrixXd R_dot(3, 4);
			R_dot << cos(omega_k), -sin(omega_k) * cos(ik), -1.0 * (xk_ * sin(omega_k) + yk_ * cos(omega_k) * cos(ik)), yk_* sin(omega_k)* sin(ik),
				sin(omega_k), cos(omega_k)* cos(ik), (xk_ * cos(omega_k) - yk_ * sin(omega_k) * cos(ik)), -yk_ * cos(omega_k) * sin(ik),
				0, sin(ik), 0, yk_* cos(ik);
			double xk_dot = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
			double yk_dot = rk_dot * sin(uk) + rk * uk_dot * cos(uk);
			MatrixXd mid(4, 1);
			MatrixXd vel(3, 1);
			mid << xk_dot, yk_dot, omega_k_dot, Ik_dot;
			vel = R_dot * mid;
			Mid->SatVel[0] = vel(0,0);
			Mid->SatVel[1] = vel(1,0);
			Mid->SatVel[2] = vel(2,0);

			/*钟差精确计算*/
			double tr = -2 * sqrt(BDS_GM) * Eph->e * Eph->SqrtA * sin(Ek) / pow(C_Light, 2);
			Mid->SatClkOft += tr;
			/*卫星钟速改正*/
			double tr_dot = -2 * sqrt(BDS_GM) * Eph->e * Eph->SqrtA * cos(Ek) * Ek_dot / pow(C_Light, 2);
			Mid->SatClkSft += tr_dot;

			/*硬件延迟*/
			Mid->Tgd1 = Eph->TGD1;
			Mid->Tgd2 = Eph->TGD2;
			Mid->URA = Eph->URA;
			Mid->prn = Eph->PRN;
			//解算成功
			Mid->Valid = true;
			return 1;
		}
	}
	else
	{
		Mid->Valid = false;
		return 0;
	}
}

int CompEarthRotationCorr(GNSSSys sys, double user[], SATPVT* Mid)//计算地球自转改正
{
	double delta_t, rou, alpha;
	MatrixXd Rz(3,3), xk(3, 1), xk_dot(3, 1), X(3, 1), X_dot(3, 1);
	rou = sqrt(pow((Mid->SatPos[0] - user[0]), 2) + pow((Mid->SatPos[1] - user[1]), 2) + pow((Mid->SatPos[2] - user[2]), 2));
	delta_t = rou / C_Light;
	if (sys == 1)
	{
		alpha = GPS_EarthRotation * delta_t;
	}
	else if (sys == 2)
	{
		alpha = BDS_EarthRotation * delta_t;
	}
	else
	{
		return 0;
	}
	Rz << cos(alpha), sin(alpha), 0, -sin(alpha), cos(alpha), 0, 0, 0, 1;
	xk << Mid->SatPos[0], Mid->SatPos[1], Mid->SatPos[2];
	xk_dot << Mid->SatVel[0], Mid->SatVel[1], Mid->SatVel[2];
	X = Rz * xk;
	X_dot = Rz * xk_dot;

	Mid->SatPos[0] = X(0,0);
	Mid->SatPos[1] = X(1,0);
	Mid->SatPos[2] = X(2,0);

	Mid->SatVel[0] = X_dot(0,0);
	Mid->SatVel[1] = X_dot(1,0);
	Mid->SatVel[2] = X_dot(2,0);

	return 1;
}