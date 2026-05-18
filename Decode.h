#pragma once
#include "TimeConversion.h"
#include"CoordinateTransformation.h"
using namespace std;
#define MAXCHANNUM 36
#define MAXRAWLEN 40960
#define GPST_BDT 14
#define MAXBDSNUM 63
#define MAXGPSNUM 32
#define POLYCRC32 0xEDB88320u

#define C_Light 299792458.0      
#define  FG1_GPS  1575.42E6             /* L1信号频率 */
#define  FG2_GPS  1227.60E6             /* L2信号频率 */

#define  WL1_GPS  (C_Light/FG1_GPS)
#define  WL2_GPS  (C_Light/FG2_GPS)
#define  FG1_BDS  1561.098E6               /* B1信号的基准频率 */
#define  FG3_BDS  1268.520E6               /* B3信号的基准频率 */
#define  WL1_BDS  (C_Light/FG1_BDS)
#define  WL3_BDS  (C_Light/FG3_BDS)       // 波长

#define MINELEV    0*3.1415926535/180                       //最小高度角阈值

#define FILEMODE 1//1：文件模式 2：实时模式
#define positioning 1//定位模式参数，0代表SPP；1代表RTK

/*导航卫星系统定义*/
enum GNSSSys {UNKS=0,GPS,BDS,GLONASS,GALILEO,QZSS};

/*每颗卫星位置、速度和钟差等中间计算结果*/
struct SATPVT
{
	double SatPos[3], SatVel[3];//卫星的位置和速度
	double SatClkOft, SatClkSft;//卫星的钟差和钟速
	double Elevation, Azimuth;//卫星的高度角和方位角
	double TropCorr;//对流层延迟改正
	double Tgd1, Tgd2;
	bool Valid;//True表示计算成功，false表示没有星历或者星历过期
	double URA;
	int prn;
	SATPVT()
	{
		SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
		SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
		Elevation = 3.1415926535 / 2.0;
		SatClkOft = SatClkSft = 0.0;
		Tgd1 = Tgd2 = TropCorr = 0.0;
		Valid = false;
	}
};

struct MWGF
{
	short Prn;
	GNSSSys Sys;
	double MW;
	double GF;
	double PIF;	//消电离层伪距组合
	int n;	//平滑计数

	MWGF()
	{
		Prn = n = 0;
		Sys = UNKS;
		MW = GF = PIF = 0.0;
	}
};


/*每颗卫星的观测数据定义*/
struct SATOBSDATA
{
	short Prn;
	GNSSSys system;
	double P[2], L[2], D[2];//分别储存相位观测值、伪距观测值、多普勒观测值
	double cn0[2], LockTime[2];
	unsigned char half[2];
	bool Valid;
	bool cycleslip[2];
	SATOBSDATA()
	{
		Prn = 0;
		system = UNKS;
		for (int i = 0; i < 2; i++)
		{
			P[i] = L[i] = D[i] = 0.0;
			cycleslip[i] = false;


		}
		Valid = false;
	}
};

/*每个历元的观测数据定义*/
struct EPOCHOBSDATA
{
	GPSTime Time;
	short Satnum;
	SATOBSDATA SatObs[MAXCHANNUM];
	SATPVT SatPVT[MAXCHANNUM];
	MWGF ComObs[MAXCHANNUM];
	EPOCHOBSDATA()
	{
		Satnum = 0;
	}
};
/*卫星星历结构体*/
struct GPSEPHREC
{
	short PRN;
	GNSSSys system;
	GPSTime TOC, TOE;
	double ClkBias, ClkDrift, ClkDriftRate;
	double IODE, IODC;
	double SqrtA, M0, e, OMEGA, i0, omega;
	double Crs, Crc, Cuc, Cus, Cic, Cis;
	double DeltaN, OMEGADot, idot;
	int SVHealth;
	double TGD1, TGD2;
	double URA; //用户测距精度

	GPSEPHREC()
	{
		PRN = SVHealth = 0;
		system = UNKS;
		ClkBias = ClkDrift = ClkDriftRate = IODE = IODC = TGD1 = TGD2 = 0.0;
		SqrtA = e = M0 = OMEGA = omega = i0 = DeltaN = OMEGADot = idot = 0.0;
		Crs = Cuc = Cus = Cis = Cic = Crc = 0.0;
	}
};

/*每个历元的定位结果结构体定义*/
struct POSRES
{
	GPSTime Time;
	double Pos[3], Vel[3],BestPos[3];
	double PDOP, SigmaPos, SigmaVel;
	int SatNum;
	double dclk, GPS_clk, BDS_clk;//钟差
	double validcount;//有效卫星数
	bool Valid;//false=没有星历或者星历过期

	POSRES()
	{
		for (int i = 0; i < 3; i++)
		{
			Pos[i] = 0.0;
			Vel[i] = 0.0;
			BestPos[i] = 0.0;
		}
		SatNum = 0;
		PDOP = SigmaVel = SigmaPos = 0.0;
		Valid = false;
	}
};
/*定义消息头*/
struct OEM7_msg_header
{
	unsigned char sync1;			// 0xAA
	unsigned char sync2;			// 0x44
	unsigned char sync3;			// 0x12
	unsigned char header_len;		// 消息头长度
	unsigned short message_id;		// 消息ID
	unsigned char message_type;     // 消息类型
	unsigned char poer_address;
	unsigned short message_len;		// 消息长度
	unsigned short sequence;
	unsigned char idle_time;
	unsigned char time_status;
	unsigned short gps_week;		// GPS周
	unsigned long  gps_ms;            // GPS毫秒数
	unsigned long  receiver_status;
	unsigned short reserved;
	unsigned short receiver_sw_ver;
};
int read(const char* filename, const char* out_file = "");//读取二进制文件，并输出到一个文件中
//NovAtel OEM7数据解码函数
int DecodeNovOem7Dat(unsigned char buff[], int& Len, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[],POSRES* pos);
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBSDATA* obs);
void decode_gpsephem(unsigned char* buff, GPSEPHREC geph[]);
void decode_bdsephem(unsigned char* buff, GPSEPHREC beph[]);
void decode_psrpos(unsigned char* buff, POSRES* pos);
unsigned int crc32(const unsigned char* buff, int len);

//读取数据
double d8(const unsigned char* p);//读取八字节double类型数据
float d4(const unsigned char* data);//读取4字节double类型数据
