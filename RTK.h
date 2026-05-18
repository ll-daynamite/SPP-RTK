#pragma once
#include"Decode.h"
#include"sockets.h"
#include<vector>
#include<fstream>
/*每颗卫星的单差观测数据定义*/
struct SDSATOBS
{
	short prn;
	GNSSSys System;
	bool Valid;
	double dP[2], dL[2];
	short nBas, nRov;//储存单差观测值对应的基准站和流动站的数值索引号

	SDSATOBS()
	{
		prn = 0;
		nBas = -1;
		nRov = -1;
		System = UNKS;
		dP[0] = dP[1] = dL[0] = dL[1] = 0.0;
		Valid =0;
	}
};
/*每个历元的单差观测数据定义*/
struct SDEPOCHOBS
{
	GPSTime Time;
	short SatNum;
	SDSATOBS SdSatObs[MAXCHANNUM];
	MWGF SdCObs[MAXCHANNUM];
	SDEPOCHOBS()
	{
		SatNum=0;
	}
};
/*双差相关的数据定义*/
struct DDCOBS
{
	int RefPrn[2], RefPos[2];//参考星卫星号与索引号，0=GPS；1=BDS
	int Sats, DDSatNum[2];//待估的双差模糊度数量，0=GPS；1=BDS
	double FixedAmb[MAXCHANNUM * 4];//包括双频最优解[0，AmbNum]和次优解[AmbNum,2*AmbNum]
	double ResAmb[2], Ratio;//LAMBDA浮点解中的模糊度残差
	int m=2;//需要返回的待选解数量
	float FixRMS[2];//固定解定位中rms误差
	double dPos[3];//基线向量
	bool bFixed;//true为固定，false为未固定
	DDCOBS()
	{
		int i;
		for (i = 0; i < 2; i++) {
			DDSatNum[i] = 0;    // 各卫星系统的双差数量
			RefPos[i] = RefPrn[i] = -1;
		}
		Sats = 0;              // 双差卫星总数
		dPos[0] = dPos[1] = dPos[2] = 0.0;
		ResAmb[0] = ResAmb[1] = FixRMS[0] = FixRMS[1] = Ratio = 0.0;
		bFixed = false;
		for (i = 0; i < MAXCHANNUM * 2; i++)
		{
			FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
		}
	}
};
/*RTK定位数据定义*/
struct rtkdata
{
	EPOCHOBSDATA rover_obs, base_obs;
	GPSEPHREC geph[MAXGPSNUM], beph[MAXBDSNUM];
	SDEPOCHOBS SdObs;
	DDCOBS DDObs;
	POSRES basepos,roverpos;
};

int RTKTimeSyn_socket(SOCKET rover_sock, SOCKET base_sock, rtkdata& rtkdata, FILE* roverFile, FILE* baseFile);
//int RTKTimeSyn_socket(SOCKET rover_sock, SOCKET base_sock, rtkdata* rtkdata);
int TimeSynch(FILE* roverfile, FILE* basefile, rtkdata*data);//时间同步函数
void FormSDEpochObs(EPOCHOBSDATA *EpkB,EPOCHOBSDATA *EpkR,SDEPOCHOBS *SDObs);//站间单差观测值函数
void DetectCycleSlip(EPOCHOBSDATA* EpkB, EPOCHOBSDATA* EpkR, SDEPOCHOBS* Obs);
void Detect_abnormal( EPOCHOBSDATA* Epk);//判断数据中是否有半周标记和周跳
void DetRefSat(const EPOCHOBSDATA* EpkB, const EPOCHOBSDATA* EpkR, SDEPOCHOBS* SDObs, DDCOBS* DDObs);
int RTKPostioning(const EPOCHOBSDATA* epkA, const EPOCHOBSDATA* epkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs,POSRES* base,POSRES* rover);
void detect_epk(EPOCHOBSDATA* rover_obs, EPOCHOBSDATA* base_obs, SDEPOCHOBS* sd, DDCOBS* dd_related);

int MatrixInv(int n, double a[], double b[]);
void MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[]);
int LD(int n, const double* Q, double* L, double* D);
void gauss(int n, double* L, double* Z, int i, int j);
void perm(int n, double* L, double* D, int j, double del, double* Z);
void reduction(int n, double* L, double* D, double* Z);
int search(int n, int m, const double* L, const double* D, const double* zs, double* zn, double* s);
int lambda(int n, int m, const double* a, const double* Q, double* F, double* s);

struct STATE
{
	VectorXd X;
	MatrixXd P;
	STATE()
	{
		X.setZero();
	}
};
struct Obser
{
	VectorXd Z;   // 观测值
	MatrixXd R;   // 观测方差
	MatrixXd H;   // 设计矩阵
};
class RTKEKF
{
private:
	static const int NUM_OF_STATE_OF_NOAMU = 6;   // 非模糊度参数数量
	STATE current_state;   // 当前状态及其协方差
	Obser current_obs;

	MatrixXd Q;         // 过程噪声
	MatrixXd Kk;		// 增益矩阵
	VectorXd Vkk1;		// 新息序列
	VectorXd dX;		// 对各状态的增益

	int ref_prn[2];
	vector<pair<int, int>> sys_prn_pair;  // (卫星系统,卫星prn)(0:GPS/1:BDS)

	bool isInit;
	void initEKF(rtkdata& rtkdata);//初始化
	void predict(rtkdata& rtkdata);//一步预测
	void update(rtkdata& rtkdata);//测量更新
	int getAmbIdx(vector<pair<int, int>> pai, int prn, int sys);
public:
	RTKEKF() { isInit = false; };
	void EKF(rtkdata& rtkdata);
	Vector3d getPos() { return current_state.X.head(3); };
};

void output_save(rtkdata data, Vector3d kf, string filename);

