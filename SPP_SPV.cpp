#include "SPP_SPV.h"
#include<fstream>
#include"ErrorCorrection.h"
void ComputeSatPVTAtSignalTrans( EPOCHOBSDATA*
	Epk, GPSEPHREC* Eph, GPSEPHREC* BDSEph, double UserPos[3])//计算信号发射时刻的卫星位置
{
	for (int i = 0; i < Epk->Satnum; i++)
	{
		memset(Epk->SatPVT + i, 0, sizeof(SATPVT));
		GPSEPHREC* eph = NULL;
		if (Epk->SatObs[i].system == GPS)
		{
			eph = Eph + Epk->SatObs[i].Prn - 1;
		}
		else if (Epk->SatObs[i].system == BDS)
		{
			eph = BDSEph + Epk->SatObs[i].Prn - 1;
		}
		//初始化
		Epk->SatPVT[i].SatClkOft = 0.0;
		Epk->SatPVT[i].SatClkSft = 0.0;
		//计算信号发射时刻
		GPSTime tr;
		tr.week = Epk->Time.week;
		tr.secofweek = Epk->Time.secofweek - Epk->SatObs[i].P[0] / C_Light - Epk->SatPVT[i].SatClkOft;

		//计算钟差、钟速
		CompSatClkOff(Epk->SatObs[i].Prn, Epk->SatObs[i].system, &Epk->Time, Eph, BDSEph, &(Epk->SatPVT[i]));
		if (!Epk->SatPVT[i].Valid) continue;

		//再次计算信号发射时刻
		tr.secofweek = Epk->Time.secofweek - Epk->SatObs[i].P[0] / C_Light -Epk->SatPVT[i].SatClkOft;

		//计算卫星的位置，钟差，速度
		if (Epk->SatObs[i].system == GPS)
		{
			CompGPSSatPVT(Epk->SatObs[i].Prn, &tr, eph, &Epk->SatPVT[i]);
		}
		else if (Epk->SatObs[i].system == BDS)
		{
			CompBDSSatPVT(Epk->SatObs[i].Prn, &tr, eph, &Epk->SatPVT[i]);
		}
		//对卫星位置进行地球自转改正
		CompEarthRotationCorr(Epk->SatObs[i].system, UserPos, &Epk->SatPVT[i]);
	}

}

bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, POSRES* Res)//单点定位
{
	MatrixXd X0(5, 1), X(5, 1),W,N,v,P;
	VectorXd sateaiz(2);
	double userpos[3];
	double a1= 0,a2=0;//接收机钟差
	double TGD, dt;
	int index = 0, num_GPS = 0, num_BDS = 0, valid_count = 0;//有效卫星数
	MatrixXd B(Epoch->Satnum, 5), w(Epoch->Satnum, 1);
	MatrixXd D_delta = MatrixXd::Zero(Epoch->Satnum, Epoch->Satnum);
	BLH blh;
	XYZ xyz, xyz2;

	//初始化矩阵
	X0 << 0, 0, 0, 0,0;
	X << 0.0, 0.0, 0.0, 0.0, 0.0;
	do {
		for (int i = 0; i < 3; i++)
		{
			userpos[i] = X0(i,0);//接收机坐标
		}
		xyz2.X = X0(0,0);
		xyz2.Y = X0(1,0);
		xyz2.Z = X0(2,0);
		index = 0, num_GPS = 0, num_BDS = 0; 
		//2.计算信号发射时刻的卫星位置和钟差，计算地球自转改正和对流层延迟等
		ComputeSatPVTAtSignalTrans(Epoch, GPSEph, BDSEph, userpos);
		for (int i = 0; i < Epoch->Satnum; i++)
		{
			if (!Epoch->SatObs[i].Valid || !Epoch->SatPVT[i].Valid) continue;;
			xyz.X = Epoch->SatPVT[i].SatPos[0];
			xyz.Y = Epoch->SatPVT[i].SatPos[1];
			xyz.Z = Epoch->SatPVT[i].SatPos[2];
			a1 = 0.0, a2 = 0.0;
			if (Epoch->SatObs[i].system == BDS)
			{
				num_BDS++;
				a2 = 1.0;
				dt = X0(4, 0);
				TGD = pow(FG1_BDS, 2) * Epoch->SatPVT[i].Tgd1 / (pow(FG1_BDS, 2) - pow(FG3_BDS, 2));
				XYZToBLH(&xyz2, &blh, R_CGCS2000, E_CGCS2000);
				D_delta(index, index) = BDSEph->URA;
			}
			else if (Epoch->SatObs[i].system == GPS)
			{
				num_GPS++;
				a1 = 1.0;
				dt = X0(3, 0);
				TGD = 0.0;
				XYZToBLH(&xyz2, &blh, R_WGS84, E_WGS84);
				D_delta(index, index) = GPSEph->URA;
			}
			sateaiz = CompSatEIAz(&xyz2, Epoch->SatPVT[i].SatPos, BLHToNEUMat(&blh));
			Epoch->SatPVT[i].TropCorr = hopfield(blh.height, sateaiz(0));
			Epoch->SatPVT[i].Elevation = sateaiz(0);
			Epoch->SatPVT[i].Azimuth = sateaiz(1);
			double rou = sqrt(pow(Epoch->SatPVT[i].SatPos[0] - X0(0, 0), 2) + pow(Epoch->SatPVT[i].SatPos[1] - X0(1, 0), 2) + pow(Epoch->SatPVT[i].SatPos[2] - X0(2, 0), 2));
			//3.对所有卫星的观测数据进行线性化。卫星位置计算失败、观测数据不完整或者有粗差，不参与定位计算；
			// 以初始位置为参考，对观测方程线性化，计算系数矩阵和残差向量，统计参与定位的各系统卫星数和所有卫星数
			w(index, 0) = Epoch->ComObs[i].PIF - (rou + dt - C_Light * Epoch->SatPVT[i].SatClkOft + Epoch->SatPVT[i].TropCorr + TGD * C_Light);
			B(index, 0) = (X0(0,0) - Epoch->SatPVT[i].SatPos[0]) / rou;
			B(index, 1) = (X0(1,0) - Epoch->SatPVT[i].SatPos[1]) / rou;
			B(index, 2) = (X0(2,0) - Epoch->SatPVT[i].SatPos[2]) / rou;
			B(index, 3) = a1;
			B(index, 4) = a2;
			index++;
		}

		//4.卫星总数是否大于未知参数的数量，如果卫星数不足，直接返回定位失败
		if (index < 5) return false;
		//5.若GPS或BDS卫星数量为0，重构法方程的矩阵
		//计算法方程矩阵
		MatrixXd B_temp, w_temp,D_temp;
		B_temp = B.topRows(index);
		w_temp = w.topRows(index);
		D_temp = D_delta.topLeftCorner(index,index);
		B = B_temp;
		w = w_temp;
		D_delta = D_temp;
		P = MatrixXd::Identity(index, index);
		N = B.transpose() * P * B;
		W = B.transpose() * P * w;

		//对矩阵重构
		if (!num_GPS && !num_BDS) return false;
		if (!num_GPS)//只有北斗卫星
		{
			MatrixXd Nx(4, 4);
			Nx << N.topLeftCorner(3, 3), N.topRightCorner(3, 1), N.bottomLeftCorner(1, 3), N.bottomRightCorner(1, 1);
			MatrixXd Wx(4, 1);
			Wx << W(0, 0), W(1, 0), W(2, 0), W(4, 0);
			N = Nx;
			W = Wx;
		}
		else if (!num_BDS)//只有GPS卫星
		{
			MatrixXd Nx(4, 4);
			Nx << N.topLeftCorner(4, 4);
			MatrixXd Wx(4, 1);
			Wx << W(0, 0), W(1, 0), W(2, 0), W(3, 0);
			N = Nx;
			W = Wx;
		}
		//6.最小二乘求解
		X = N.inverse() *W;
		for (int i = 0; i < 3; i++)
		{
			X0(i, 0) += X(i, 0);
		}
		//7.检查改正数是否收敛，若没有收敛则更新初始位置迭代计算
		if (num_GPS > 0 && num_BDS > 0)
		{
			X0(3, 0) += X(3, 0);
			X0(4, 0) += X(4, 0);
		}
		else
		{
			if (num_GPS > 0) X0(3,0) += X(3,0);
			else X0(4,0) += X(3,0);
		}
	} while (X.norm() > 1e-4);
	//8.定位精度评价
	if (!num_BDS)//单GPS
	{
		MatrixXd X_mid(5,1);
		X_mid << X(0, 0), X(1, 0), X(2, 0), X(3, 0), 0;
		X = X_mid;
	}	
	else if (!num_GPS)//单BDS
	{
		MatrixXd X_mid(5, 1);
		X_mid << X(0, 0), X(1, 0), X(2, 0), 0, X(3, 0);
		X = X_mid;
	}
	v = B * X - w;
	Res->Time.week = Epoch->Time.week;
	Res->Time.secofweek = Epoch->Time.secofweek;
	Res->validcount = index;
	for (int i = 0; i < 3; i++)
	{
		Res->Pos[i] = X0(i,0);
	}
	Res->PDOP = sqrt(N.inverse()(0, 0)+ N.inverse()(1, 1)+ N.inverse()(2, 2));
	double vPv = (v.transpose() * P * v)(0,0);
	Res->SigmaPos = sqrt(vPv/ (index -X.rows()));

	Res->GPS_clk = X0(3);
	Res->BDS_clk = X0(4);
	//cout << fixed << setprecision(3);
	//cout<< Res->Time.week << " " << Res->Time.secofweek<<" "<<endl;
	//cout << "GPS钟差：" << Res->GPS_clk << " " << "BDS钟差：" << Res->BDS_clk << " " << "有效卫星数：" << Res->validcount << endl;
	//cout<< "X:" << Res->Pos[0] << " " << " Y:" << Res->Pos[1] << " " << "Z:" << Res->Pos[2] << " " << "sigamaPos: " << Res->SigmaPos << " " << "PDOP:" << Res->PDOP << endl;
	if (FILEMODE == 1)
	{
		ofstream outfile1("OBS.txt",ios::app);
		outfile1 << fixed << setprecision(3);
		outfile1 << "# " << Epoch->Time.week << " " << Epoch->Time.secofweek << " " << Res->validcount << endl;
		for (int i = 0; i < Epoch->Satnum; i++)
		{
			string prnhead;
			if (!Epoch->SatObs[i].Valid || !Epoch->SatPVT[i].Valid) continue;
			if (Epoch->SatObs[i].system == BDS) prnhead = "C";
			else prnhead = "G";
			outfile1 << prnhead << Epoch->SatPVT[i].prn << " ";
			outfile1 << Epoch->SatPVT[i].SatPos[0] << " " << Epoch->SatPVT[i].SatPos[1] << " " << Epoch->SatPVT[i].SatPos[2] << " ";
			outfile1 << sqrt(pow(Epoch->SatPVT[i].SatPos[0] - Res->Pos[0], 2) + pow(Epoch->SatPVT[i].SatPos[1] - Res->Pos[1], 2) + pow(Epoch->SatPVT[i].SatPos[2] - Res->Pos[2], 2))<<" ";
			outfile1 << Epoch->SatPVT[i].URA << endl;
		}
	}
	return true;
}

bool SPV(EPOCHOBSDATA* Epoch, POSRES* Res)//单点测速
{
	int validSatCount = 0; // 实际有效卫星计数
	for (int i = 0; i < Epoch->Satnum; i++) 
	{
		if (!Epoch->SatObs[i].Valid || !Epoch->SatPVT[i].Valid) continue;
		validSatCount++;
	}
	Res->validcount = validSatCount;
	if (validSatCount < 4) return false;
	Vector4d X(0, 0, 0, 0);
	MatrixXd B(validSatCount, 4), P = MatrixXd::Identity(validSatCount, validSatCount);
	VectorXd w(validSatCount);
	int idx = 0;//有效卫星的索引
	for (int i = 0; i < Epoch->Satnum; i++)
	{
		if (!Epoch->SatObs[i].Valid || !Epoch->SatPVT[i].Valid) continue;
		// 计算接收机到卫星的几何距离
		double dx = Epoch->SatPVT[i].SatPos[0] - Res->Pos[0];
		double dy = Epoch->SatPVT[i].SatPos[1] - Res->Pos[1];
		double dz = Epoch->SatPVT[i].SatPos[2] - Res->Pos[2];
		double rou = sqrt(dx * dx + dy * dy + dz * dz);

		double rou_dot = (dx * Epoch->SatPVT[i].SatVel[0] +dy * Epoch->SatPVT[i].SatVel[1] +dz * Epoch->SatPVT[i].SatVel[2]) / rou;
		w(idx) = Epoch->SatObs[i].D[0] - (rou_dot - C_Light * Epoch->SatPVT[i].SatClkSft);
		B(idx, 0) = dx / rou;
		B(idx, 1) = dy / rou;
		B(idx, 2) = dz / rou;
		B(idx, 3) = 1.0;
		idx++;
	}
	X = (B.transpose() * P * B).inverse() * (B.transpose() * P * w);
	for (int i = 0; i < 3; i++)
	{
		Res->Vel[i] = X(i);
	}
	Res->SigmaVel = sqrt((B * X - w).transpose() * P * (B * X - w)) / sqrt(idx - 4);
	Res->dclk = X(3);
	//cout << "VX: " << Res->Vel[0] <<" " << "VY: " << Res->Vel[1] <<" " << "VZ: " << Res->Vel[2] <<" " << "SigmaVel: " << Res->SigmaVel << endl;
	return true;
}
