#include "ErrorCorrection.h"
double hopfield(const double H, const double Elev)//对流层改正函数 输入的高度角为弧度
{
	if (H > 18000) return 0;
	//标准气象元素
	double H0 = 0.0;//海平面 m
	double T0 = 288.76;//温度 K
	double p0 = 1013.25;//气压 mbar
	double RH0 = 0.5;//相对湿度
	
	//高度角弧度转角度
	double Elev_ = Elev * 180.0 / my_pi;
	//按照模型计算改正数
	double RH = RH0 * exp(-0.0006396 * (H - H0));
	double p = p0 * pow((1 - 0.0000226 * (H - H0)), 5.225);
	double T = T0 - 0.0065 * (H - H0);
	double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * pow(T, 2));
	double hw = 11000;
	double hd = 40136 + 148.72 * (T0 - 273.16);
	double Kw = (155.2e-7) * 4810 * e * (hw - H) / pow(T, 2);
	double Kd = (155.2e-7) * p * (hd - H) / T;
	double Trop = Kd / sin(sqrt(Elev_ * Elev_ + 6.25) * my_pi / 180.0) + Kw / sin(sqrt(Elev_ * Elev_ + 2.25) * my_pi / 180.0);
	return Trop;
}

void DetectOutlier(EPOCHOBSDATA* Obs)
{
	MWGF CurComObs[MAXCHANNUM];
	double w1=0, w2=0;
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		//1. 检查该卫星的双频伪距和相位数据是否有效和完整，若不全或为0，将Valid标记为false，continue
		if (fabs(Obs->SatObs[i].P[0]) <1e-5 || fabs(Obs->SatObs[i].P[1]) <1e-5 ||
			fabs(Obs->SatObs[i].L[0]) <1e-5 || fabs(Obs->SatObs[i].L[1])<1e-5)
		{
			Obs->SatObs[i].Valid = false;
			continue;
		}
		CurComObs[i].Prn = Obs->SatObs[i].Prn;
		CurComObs[i].Sys = Obs->SatObs[i].system;
		//2. 计算当前历元该卫星的GF和MW组合值
		if (CurComObs[i].Sys == GPS)
		{
			w1 = FG1_GPS;
			w2 = FG2_GPS;
		}
		else if (CurComObs[i].Sys == BDS)
		{
			w1 = FG1_BDS;
			w2 = FG3_BDS;
		}
		CurComObs[i].GF = Obs->SatObs[i].L[0] - Obs->SatObs[i].L[1];
		CurComObs[i].MW = (w1 * Obs->SatObs[i].L[0] - w2 * Obs->SatObs[i].L[1]) / (w1 - w2) -
			(w1 * Obs->SatObs[i].P[0] + w2 * Obs->SatObs[i].P[1]) / (w1 + w2);
		//初始化平滑参数
		CurComObs[i].n = 1;
		//3. 从上个历元的MWGF数据中查找该卫星的GF和MW组合值
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (CurComObs[i].Prn == Obs->ComObs[j].Prn && CurComObs[i].Sys == Obs->ComObs[j].Sys)
			{
				//4. 计算当前历元该卫星GF与上一历元对应GF的差值dGF
				//5. 计算当前历元该卫星MW与上一历元对应MW平滑值的差值dMW
				double dGF = CurComObs[i].GF - Obs->ComObs[j].GF;
				double dMW = CurComObs[i].MW - Obs->ComObs[j].MW;
				//6. 检查dGF和dMW是否超限，限差阈值建议为5cm和3m。若超限，标记为粗差，将Valid标记为false ，若不超限，标记为可用将Valid标记为true，并计算该卫星的MW平滑值
				if (fabs(dGF) < 0.05 && fabs(dMW) < 3)
				{
					Obs->SatObs[i].Valid = true;
					CurComObs[i].MW = (Obs->ComObs[j].n * Obs->ComObs[j].MW + CurComObs[i].MW) / (Obs->ComObs[j].n + 1);
					CurComObs[i].n = Obs->ComObs[j].n + 1;
				}
				else
				{
					Obs->SatObs[i].Valid = false;
				}
				break;
			}
		}
		//7. 对于可用的观测数据，计算伪距的IF组合观测值，用于SPP
		if (Obs->SatObs[i].Valid)
		{
			CurComObs[i].PIF = (w1 * w1 * Obs->SatObs[i].P[0] - w2 * w2 * Obs->SatObs[i].P[1]) / (w1 * w1 - w2 * w2);
		}
	}
	//8. 所有卫星循环计算完成之后，将CurComObs内存拷贝到ComObs，即函数运行结束后， ComObs保存了当前历元的GF和MW平滑值。
	memcpy(Obs->ComObs, CurComObs, MAXCHANNUM * sizeof(MWGF));
}

