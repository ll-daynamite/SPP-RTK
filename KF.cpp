#pragma once

#include"RTK.h"

/*
@brief 获取当前卫星（非参考星）在双差对中的索引
*/
int RTKEKF::getAmbIdx(vector<pair<int, int>> pai, int prn, int sys)
{
	for (int i = 0; i < pai.size(); ++i)
	{
		if (pai[i].first == sys && pai[i].second == prn) return i;
	}
	return -1;

}

/*
@brief EKF初始化
*/
void RTKEKF::initEKF(rtkdata& rtkdata)
{
	// 计算GPS和BDS双差卫星数
	int ddbdsnum = 0, ddgpsnum = 0;
	for (int i = 0; i < rtkdata.SdObs.SatNum; i++)
	{
		if (rtkdata.SdObs.SdSatObs[i].System == BDS && rtkdata.SdObs.SdSatObs[i].prn != rtkdata.DDObs.RefPrn[1] && rtkdata.SdObs.SdSatObs[i].Valid)
		{
			ddbdsnum++;
		}
		if (rtkdata.SdObs.SdSatObs[i].System == GPS && rtkdata.SdObs.SdSatObs[i].prn != rtkdata.DDObs.RefPrn[0] && rtkdata.SdObs.SdSatObs[i].Valid)
		{
			ddgpsnum++;
		}
	}
	rtkdata.DDObs.Sats = ddgpsnum + ddbdsnum;
	rtkdata.DDObs.DDSatNum[0] = ddgpsnum;
	rtkdata.DDObs.DDSatNum[1] = ddbdsnum;

	int num_amb = (rtkdata.DDObs.DDSatNum[0] + rtkdata.DDObs.DDSatNum[1]) * 2;
	current_state.X.resize(NUM_OF_STATE_OF_NOAMU + num_amb);
	current_state.P = MatrixXd::Zero(NUM_OF_STATE_OF_NOAMU + num_amb, NUM_OF_STATE_OF_NOAMU + num_amb);

	// 位置和协方差
	for (int i = 0; i < 3; i++)
	{
		current_state.X(i) = rtkdata.roverpos.Pos[i];
	}
	current_state.P.block(0, 0, 3, 3) = Matrix3d::Identity() * 10.0;

	// 速度和协方差
	for (int i = 3; i < 6; i++)
	{
		current_state.X(i) = rtkdata.roverpos.Vel[i-3];
	}
	current_state.P.block(3, 3, 3, 3) = Matrix3d::Identity() * 1.0;

	// 模糊度和协方差
	current_state.X.tail(num_amb).setZero();
	for (int i = 0; i < num_amb; ++i) {
		current_state.P.diagonal()(NUM_OF_STATE_OF_NOAMU + i) = 10000.0;
	}
	ref_prn[0] = rtkdata.DDObs.RefPrn[0], ref_prn[1] = rtkdata.DDObs.RefPrn[1];
	sys_prn_pair.clear();
	for (int i = 0; i < rtkdata.SdObs.SatNum; i++)
	{
		int n = (rtkdata.SdObs.SdSatObs[i].System == GPS) ? 0 : 1;
		if (i == rtkdata.DDObs.RefPos[n] || !rtkdata.SdObs.SdSatObs[i].Valid)continue;//跳过参考星和不可用卫星
		sys_prn_pair.push_back(make_pair(n, rtkdata.SdObs.SdSatObs[i].prn));
	}
}

/*
@brief 一步预测
*/
void RTKEKF::predict(rtkdata& rtkdata)
{
	int num_amb = (rtkdata.DDObs.DDSatNum[0] + rtkdata.DDObs.DDSatNum[1]) * 2;
	VectorXd new_x(NUM_OF_STATE_OF_NOAMU + num_amb);
	MatrixXd new_P = MatrixXd::Zero(NUM_OF_STATE_OF_NOAMU + num_amb, NUM_OF_STATE_OF_NOAMU + num_amb);
	MatrixXd FAI = MatrixXd::Zero(NUM_OF_STATE_OF_NOAMU + num_amb, NUM_OF_STATE_OF_NOAMU + sys_prn_pair.size() * 2); // 状态转移矩阵

	double sigma_pos = 0.1, sigma_vel = rtkdata.roverpos.SigmaVel;
	if (sigma_vel < 0.01) sigma_vel = 0.1; // 最小噪声保证
	Q = MatrixXd::Zero(NUM_OF_STATE_OF_NOAMU + num_amb, NUM_OF_STATE_OF_NOAMU + num_amb);
	// 位置噪声
	Q.block(0, 0, 3, 3) = Matrix3d::Identity() * sigma_pos * sigma_pos;
	// 速度噪声
	Q.block(3, 3, 3, 3) = Matrix3d::Identity() * sigma_vel * sigma_vel;
	// 根据上个历元调整模糊度固定噪声
	for (int i = 0; i < num_amb; i++) 
	{
		bool is_fixed = rtkdata.DDObs.bFixed;
		double sigma_amb;

		// 自适应噪声策略
		if (is_fixed) sigma_amb = 1e-6;   // 已固定
		else sigma_amb = 1e-3;

		Q(NUM_OF_STATE_OF_NOAMU + i, NUM_OF_STATE_OF_NOAMU + i) = sigma_amb * sigma_amb;
	}

	// 当前历元
	int new_ref_prn[2] = { rtkdata.DDObs.RefPrn[0],rtkdata.DDObs.RefPrn[1] };
	vector<pair<int, int>> new_sys_prn_pair;
	for (int i = 0; i < rtkdata.SdObs.SatNum; ++i)
	{
		int n = (rtkdata.SdObs.SdSatObs[i].System == GPS) ? 0 : 1;
		if (i == rtkdata.DDObs.RefPos[n] || !rtkdata.SdObs.SdSatObs[i].Valid) continue;
		new_sys_prn_pair.push_back(make_pair(n, rtkdata.SdObs.SdSatObs[i].prn));
	}

	// 位置和速度不变
	FAI.block(0, 0, NUM_OF_STATE_OF_NOAMU, NUM_OF_STATE_OF_NOAMU) = MatrixXd::Identity(NUM_OF_STATE_OF_NOAMU, NUM_OF_STATE_OF_NOAMU);
	FAI.block(0, 3, 3, 3) = Matrix3d::Identity() * 1.0;

	// 模糊度
	vector<bool> need2init(num_amb, false);
	for (int i = 0; i < new_sys_prn_pair.size(); i++)
	{
		int n = new_sys_prn_pair[i].first, prn = new_sys_prn_pair[i].second;
		int new_rows = NUM_OF_STATE_OF_NOAMU + 2 * i;
		int ref_idx = getAmbIdx(sys_prn_pair, new_ref_prn[n], n);
		int idx = getAmbIdx(sys_prn_pair, prn, n);
		// 参考星相同
		if (new_ref_prn[n] == this->ref_prn[n])
		{
			if (idx != -1)  // 卫星存在于上历元
			{
				FAI(new_rows + 0, NUM_OF_STATE_OF_NOAMU + 2 * idx + 0) = 1.0;
				FAI(new_rows + 1, NUM_OF_STATE_OF_NOAMU + 2 * idx + 1) = 1.0;
			}
			else   // 卫星新出现
			{
				need2init[2 * i] = true; need2init[2 * i + 1] = true;
			}
		}
		else if (ref_idx != -1 && idx != -1)  // 参考星不同，但是当前历元的参考星存在于上历元
		{
			FAI(new_rows + 0, NUM_OF_STATE_OF_NOAMU + 2 * idx + 0) = 1.0;
			FAI(new_rows + 0, NUM_OF_STATE_OF_NOAMU + 2 * ref_idx + 0) = -1.0;
			FAI(new_rows + 1, NUM_OF_STATE_OF_NOAMU + 2 * idx + 1) = 1.0;
			FAI(new_rows + 1, NUM_OF_STATE_OF_NOAMU + 2 * ref_idx + 1) = -1.0;
		}
		else// 其他情况，重新进行初始化
		{
			need2init[2 * i] = true; need2init[2 * i + 1] = true;
		}
	}
	new_x = FAI * current_state.X;
	new_P = FAI * current_state.P * FAI.transpose() + Q;

	// 初始化无法进行转移的模糊度
	for (int i = 0; i < num_amb; i += 2)
	{
		if (need2init[i])
		{
			new_x(NUM_OF_STATE_OF_NOAMU + i + 0) = 0.0;
			new_P(NUM_OF_STATE_OF_NOAMU + i + 0, NUM_OF_STATE_OF_NOAMU + i + 0) = 10000.0;
			new_x(NUM_OF_STATE_OF_NOAMU + i + 1) = 0.0;
			new_P(NUM_OF_STATE_OF_NOAMU + i + 1, NUM_OF_STATE_OF_NOAMU + i + 1) = 10000.0;
		}
	}

	//rtkdata.DDObs.FixedAmb.resize(num_amb, 2);
	current_state.X = new_x;
	current_state.P = new_P;
	ref_prn[0] = new_ref_prn[0], ref_prn[1] = new_ref_prn[1];
	sys_prn_pair = new_sys_prn_pair;

}

/*
@brief 测量更新
*/
void RTKEKF::update(rtkdata& rtkdata)
{
	int sum_num = rtkdata.DDObs.DDSatNum[0] + rtkdata.DDObs.DDSatNum[1];
	current_obs.Z = VectorXd::Zero(sum_num * 4), current_obs.R = MatrixXd::Zero(sum_num * 4, sum_num * 4), current_obs.H = MatrixXd::Zero(sum_num * 4, NUM_OF_STATE_OF_NOAMU + sum_num * 2);
	VectorXd Z_pred = VectorXd::Zero(sum_num * 4);

	// 构造观测值方差阵和基站到卫星的距离
	// 初始化基站和流动站位置
	MatrixXd Basepos(3, 1), Roverpos(3, 1);
	double norm_base = sqrt(pow(rtkdata.basepos.BestPos[0], 2) + pow(rtkdata.basepos.BestPos[1], 2) + pow(rtkdata.basepos.BestPos[2], 2));
	for (int i = 0; i < 3; i++)
	{
		if (norm_base == 0.0)
		{
			Basepos(i, 0) = rtkdata.basepos.Pos[i];
		}
		else
		{
			Basepos(i, 0) = rtkdata.basepos.BestPos[i];
		}
		Roverpos(i, 0) = rtkdata.roverpos.Pos[i];
	}

	VectorXd Pbase2sat(sum_num);
	double Psigma = 0.3, Lsigma = 0.01;
	int rows = 0;
	for (int i = 0; i < rtkdata.SdObs.SatNum; i++)
	{
		int n = (rtkdata.SdObs.SdSatObs[i].System == GPS) ? 0 : 1;
		int ref_prn = rtkdata.DDObs.RefPrn[n], ref_idx = rtkdata.DDObs.RefPos[n];
		int rover_idx = rtkdata.SdObs.SdSatObs[i].nRov, base_idx = rtkdata.SdObs.SdSatObs[i].nBas;
		GNSSSys sys = rtkdata.SdObs.SdSatObs[i].System;
		// 跳过参考星/伪距和载波未通过周跳检测/卫星星历不正常/
		if (i == ref_idx || !rtkdata.SdObs.SdSatObs[i].Valid) continue;
		current_obs.Z(rows * 4 + 0) = rtkdata.SdObs.SdSatObs[i].dP[0] - rtkdata.SdObs.SdSatObs[ref_idx].dP[0];
		current_obs.Z(rows * 4 + 1) = rtkdata.SdObs.SdSatObs[i].dP[1] - rtkdata.SdObs.SdSatObs[ref_idx].dP[1];
		current_obs.Z(rows * 4 + 2) = rtkdata.SdObs.SdSatObs[i].dL[0] - rtkdata.SdObs.SdSatObs[ref_idx].dL[0];
		current_obs.Z(rows * 4 + 3) = rtkdata.SdObs.SdSatObs[i].dL[1] - rtkdata.SdObs.SdSatObs[ref_idx].dL[1];
		// 基站到所有非参考卫星的距离
		Pbase2sat(rows, 0) = sqrt(
			pow(Basepos(0) - rtkdata.base_obs.SatPVT[base_idx].SatPos[0], 2) +
			pow(Basepos(1) - rtkdata.base_obs.SatPVT[base_idx].SatPos[1], 2) +
			pow(Basepos(2) - rtkdata.base_obs.SatPVT[base_idx].SatPos[2], 2)
		);
		// 权阵初始化
		int cols = 0;  // 里层有效卫星数
		for (int j = 0; j < rtkdata.SdObs.SatNum; j++)
		{
			int rover_idx_tmp = rtkdata.SdObs.SdSatObs[j].nRov, base_idx_tmp = rtkdata.SdObs.SdSatObs[j].nBas;
			if (j == rtkdata.DDObs.RefPos[0] || j == rtkdata.DDObs.RefPos[1] || !rtkdata.SdObs.SdSatObs[j].Valid) continue;
			if (sys != rtkdata.SdObs.SdSatObs[j].System) {
				cols++; continue;   // 卫星有效但系统不同
			}

			int ndd = (sys == GPS) ? rtkdata.DDObs.DDSatNum[0] : rtkdata.DDObs.DDSatNum[1];
			double a = (rows == cols) ? 4 : 2;
			current_obs.R(rows * 4 + 0, cols * 4 + 0) = a * (Psigma * Psigma); 
			current_obs.R(rows * 4 + 1, cols * 4 + 1) = a * (Psigma * Psigma);
			current_obs.R(rows * 4 + 2, cols * 4 + 2) = a * (Lsigma * Lsigma);
			current_obs.R(rows * 4 + 3, cols * 4 + 3) = a * (Lsigma * Lsigma);
			cols++;
		}
		rows++;
	}
	Vector3d baseref_sat_PVT_GPS, baseref_sat_PVT_BDS, roverref_sat_PVT_GPS, roverref_sat_PVT_BDS;
	// 获取设计矩阵
	int ref_in_base[2] = { rtkdata.SdObs.SdSatObs[rtkdata.DDObs.RefPos[0]].nBas,
								 rtkdata.SdObs.SdSatObs[rtkdata.DDObs.RefPos[1]].nBas };
	for (int i = 0; i < 3; i++)
	{
		baseref_sat_PVT_GPS(i, 0) = rtkdata.base_obs.SatPVT[ref_in_base[0]].SatPos[i];
		baseref_sat_PVT_BDS(i, 0) = rtkdata.base_obs.SatPVT[ref_in_base[1]].SatPos[i];
	}

	double Pbase2sat_ref[2] = {
		(Basepos - baseref_sat_PVT_GPS).norm(),
		(Basepos - baseref_sat_PVT_BDS).norm()
	};
	int ref_in_rover[2] = { rtkdata.SdObs.SdSatObs[rtkdata.DDObs.RefPos[0]].nRov,
							rtkdata.SdObs.SdSatObs[rtkdata.DDObs.RefPos[1]].nRov };

	VectorXd Prover2sat(sum_num);
	for (int i = 0; i < 3; i++)
	{
		roverref_sat_PVT_GPS(i, 0) = rtkdata.rover_obs.SatPVT[ref_in_rover[0]].SatPos[i];
		roverref_sat_PVT_BDS(i, 0) = rtkdata.rover_obs.SatPVT[ref_in_rover[1]].SatPos[i];
	}

	double Prover2sat_ref[2] = {
		(current_state.X.head(3) - roverref_sat_PVT_GPS).norm(),
		(current_state.X.head(3) - roverref_sat_PVT_BDS).norm()
	};
	rows = 0;
	for (int i = 0; i < rtkdata.SdObs.SatNum; i++)
	{
		int n = (rtkdata.SdObs.SdSatObs[i].System == GPS) ? 0 : 1;
		int ref_prn = rtkdata.DDObs.RefPrn[n], ref_idx = rtkdata.DDObs.RefPos[n];
		int rover_idx = rtkdata.SdObs.SdSatObs[i].nRov, base_idx = rtkdata.SdObs.SdSatObs[i].nBas;

		if (i == ref_idx || !rtkdata.SdObs.SdSatObs[i].Valid) continue;

		Prover2sat(rows) = sqrt(pow(current_state.X(0) - rtkdata.rover_obs.SatPVT[rover_idx].SatPos[0], 2) +
			pow(current_state.X(1) - rtkdata.rover_obs.SatPVT[rover_idx].SatPos[1], 2) +
			pow(current_state.X(2) - rtkdata.rover_obs.SatPVT[rover_idx].SatPos[2], 2));

		double lamuda1 = 0, lamuda2 = 0;
		if (rtkdata.SdObs.SdSatObs[i].System == GPS) { lamuda1 = WL1_GPS; lamuda2 = WL2_GPS; }
		else { lamuda1 = WL1_BDS; lamuda2 = WL3_BDS; }
		// 设计矩阵
		double l_ = (current_state.X(0) - rtkdata.rover_obs.SatPVT[rover_idx].SatPos[0]) / Prover2sat(rows)
			- (current_state.X(0) - rtkdata.rover_obs.SatPVT[ref_in_rover[n]].SatPos[0]) / Prover2sat_ref[n];
		double m_ = (current_state.X(1) - rtkdata.rover_obs.SatPVT[rover_idx].SatPos[1]) / Prover2sat(rows)
			- (current_state.X(1) - rtkdata.rover_obs.SatPVT[ref_in_rover[n]].SatPos[1]) / Prover2sat_ref[n];
		double n_ = (current_state.X(2) - rtkdata.rover_obs.SatPVT[rover_idx].SatPos[2]) / Prover2sat(rows)
			- (current_state.X(2) - rtkdata.rover_obs.SatPVT[ref_in_rover[n]].SatPos[2]) / Prover2sat_ref[n];

		for (int j = 0; j < 4; j++)
		{
			current_obs.H(4 * rows + j, 0) = l_; current_obs.H(4 * rows + j, 1) = m_; current_obs.H(4 * rows + j, 2) = n_;
		}
		Z_pred(4 * rows + 0) = Prover2sat(rows) - Pbase2sat(rows) - Prover2sat_ref[n] + Pbase2sat_ref[n];
		Z_pred(4 * rows + 1) = Prover2sat(rows) - Pbase2sat(rows) - Prover2sat_ref[n] + Pbase2sat_ref[n];
		Z_pred(4 * rows + 2) = Prover2sat(rows) - Pbase2sat(rows) - Prover2sat_ref[n] + Pbase2sat_ref[n] + lamuda1 * current_state.X(NUM_OF_STATE_OF_NOAMU + 2 * rows + 0);
		Z_pred(4 * rows + 3) = Prover2sat(rows) - Pbase2sat(rows) - Prover2sat_ref[n] + Pbase2sat_ref[n] + lamuda2 * current_state.X(NUM_OF_STATE_OF_NOAMU + 2 * rows + 1);
		current_obs.H(rows * 4 + 2, NUM_OF_STATE_OF_NOAMU + rows * 2 + 0) = lamuda1;
		current_obs.H(rows * 4 + 3, NUM_OF_STATE_OF_NOAMU + rows * 2 + 1) = lamuda2;
		rows++;
	}

	// 增益矩阵
	Kk = current_state.P * current_obs.H.transpose() * (current_obs.H * current_state.P * current_obs.H.transpose() + current_obs.R).inverse();
	// 新息
	Vkk1 = current_obs.Z - Z_pred;

	// 状态滤波和滤波方差
	current_state.X = current_state.X + Kk * Vkk1;
	current_state.P = (MatrixXd::Identity(current_state.P.rows(), current_state.P.cols()) - Kk * current_obs.H) * current_state.P * (MatrixXd::Identity(current_state.P.rows(), current_state.P.cols()) - Kk * current_obs.H).transpose() + Kk * current_obs.R * Kk.transpose();

	//模糊度固定
	rtkdata.DDObs.bFixed = false;
	rtkdata.DDObs.Ratio = 0.0;

	int n =sum_num * 2;  // 模糊度参数个数
	int noamb_param_num = 6;// 非模糊度参数个数（位置速度参数）
	int total_params = 6+n;
	//计算协方差矩阵
	MatrixXd float_Qxx;
	float_Qxx = current_state.P;
	// 检查是否有足够的模糊度进行固定
	if (n > 0 && float_Qxx.rows() >= total_params)
	{
		// 浮点模糊度和协方差
		VectorXd float_ambiguities = current_state.X.segment(noamb_param_num, n);
		VectorXd fixed_ambiguities(n);
		MatrixXd Q_ambiguities = float_Qxx.block(noamb_param_num, noamb_param_num, n, n);
		VectorXd X_float = current_state.X.head(3);
		MatrixXd Q_pos_amb = float_Qxx.topRightCorner(noamb_param_num-3, n);

		// 调用LAMBDA算法
		double* a_ptr = float_ambiguities.data();
		double* Q_ptr = Q_ambiguities.data();
		double* s_ptr = rtkdata.DDObs.ResAmb;

		if (lambda(n, rtkdata.DDObs.m, a_ptr, Q_ptr, rtkdata.DDObs.FixedAmb, rtkdata.DDObs.ResAmb) == 0)
		{
			// 模糊度确认
			rtkdata.DDObs.Ratio = rtkdata.DDObs.ResAmb[1] / rtkdata.DDObs.ResAmb[0];
			if (rtkdata.DDObs.Ratio >= 3.0)
			{
				// 固定成功，计算固定解
				rtkdata.DDObs.bFixed = true;
				for (int i = 0; i < float_ambiguities.size(); i++)
				{
					fixed_ambiguities(i) = rtkdata.DDObs.FixedAmb[i];
				}
				VectorXd X_fixed = X_float - Q_pos_amb * Q_ambiguities.inverse() * (float_ambiguities - fixed_ambiguities);

				// 更新流动站位置为固定解
				current_state.X.head(3) = X_fixed;

				current_state.P.block(0, 0, noamb_param_num, noamb_param_num) -= float_Qxx.topRightCorner(noamb_param_num, n) * Q_ambiguities.inverse() * float_Qxx.topRightCorner(noamb_param_num, n).transpose();
				for (int k = 0; k < n; k++)
				{
					int idx = noamb_param_num + k;
					current_state.X(idx) = round(fixed_ambiguities(k));//模糊度转换成整数
					current_state.P(idx, idx) = 1e-10;
					for (int j = 0; j < current_state.P.cols(); j++)
					{
						if (j != idx)
						{
							current_state.P(idx, j) = 0.0;
							current_state.P(j, idx) = 0.0;
						}
					}
				}
			}

		}

		if (!rtkdata.DDObs.bFixed)
		{
			// 使用浮点解
			current_state.X.head(3) = X_float;
		}
	}
	else
	{
		// 没有足够卫星进行模糊度固定，使用浮点解
		cout << "卫星数量不足" << endl;
	}
}

void RTKEKF::EKF(rtkdata&data)
{
	initEKF(data);
	predict(data);
	update(data);
}