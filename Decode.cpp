#include "Decode.h"


/*解算八字节数据*/
double d8(unsigned char* p)
{
	double r = 0.0;
	memcpy(&r, p, 8);
	return r;
}

/*解算四字节数据*/
float d4(unsigned char* data)
{
	float value;
	memcpy(&value, data, 4);
	return value;
}

unsigned long l4(unsigned char* data)
{
	unsigned long value;
	memcpy(&value, data, 4);
	return value;
}

/*计算CRC校验码*/
unsigned int crc32(const unsigned char* buff, int len)
{
	int i, j;
	unsigned int crc = 0;
	for (i = 0; i < len; i++)
	{
		crc ^= buff[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 1)
			{
				crc = (crc >> 1) ^ POLYCRC32;
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	return crc;
}

/*获取定位精确结果*/
void decode_psrpos(unsigned char* buff, POSRES* pos)
{
	BLH blh;
	XYZ xyz;
	blh.latitude= d8(buff+36)*my_pi/180.0;
	blh.longitude = d8(buff + 44) * my_pi / 180.0;
	blh.height= d8(buff + 52);
	BLHToXYZ(&blh, &xyz, R_CGCS2000, E_CGCS2000);
	pos->BestPos[0] = xyz.X;
	pos->BestPos[1] =xyz.Y;
	pos->BestPos[2] =xyz.Z;//坐标系为XYZ
}


/*解码OEM7格式数据，返回是否解码成功*/
int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[],POSRES*pos)
{
	int i = 0;
	int flag=0;
	int status;//读取观测值的状态
	while (i <= len-3)
	{
		if ((buff[0 + i] == 0xAA) && (buff[1 + i] == 0x44) && (buff[2 + i] == 0x12))
		{
			//找到同步字符

			if (i + 28 > len)
			{
				break;
			}
			//获取读取数据位置的指针
			unsigned char* ptr = buff + i;
			
			//读取消息头
			OEM7_msg_header head;
			//直接拷贝消息头
			memcpy(&head, ptr, sizeof(OEM7_msg_header));
			if (i + 28 + head.message_len + 4 > len)
			{
				break;
			}

			//CRC校验
			unsigned long crc = *(unsigned long*)(ptr + 28 + head.message_len);
			if (crc32(ptr, 28 + head.message_len) != crc)
			{
				i = i + 3;
				continue;
			}
			//判断消息ID
			status = 0;
			switch (head.message_id)
			{
			case 43:      //观测值
				flag=decode_rangeb_oem7(ptr, obs);
				status = 1;
				break;
			case 7:    //GPS星历
				decode_gpsephem(ptr, geph);
				break;
			case 42:
				decode_psrpos(ptr, pos);
				break;
			case 1696:
				decode_bdsephem(ptr, beph);
				break;
			default:
				break;
			}
			i = i + 28 + head.message_len + 4;
			if (flag == 1&&FILEMODE==1&&status==1) break;
		}
		else 
		{
			i++;
		}
	}

	//处理剩余字节
	memcpy(buff + 0, buff + i, len - i);
	len = len - i;
	return 1;
}

/*解码观测数据，返回是否解码成功*/
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBSDATA* obs)
{
	int i, j, n, k=0, ObsNum, Freq;
	unsigned short Prn;
	GNSSSys sys;
	double w1;
	int PhaseLockFlag, CodeLockedFlag, ParityFlag, SatSystem, SigType;
	unsigned int ChanStatus;
	unsigned char* p = buff + 28;//数据指针位置
	
	//1.从消息头中解码得到观测时刻，该时刻为接收机表面时，用GPSTIME结构体表示
	memcpy(&(obs->Time.week), buff + 14, 2);
	obs->Time.secofweek = l4(buff + 16) * 1e-3;
	//2.解码得到观测值数量，即所有卫星所有信号观测值的总数
	//观测卫星数量
	memcpy(&ObsNum, p, 4);
	memset(obs->SatObs, 0, MAXCHANNUM * sizeof(SATOBSDATA));
	//3.对所有信号观测值进行循环解码
	for (i = 0, p += 4; i < ObsNum; i++, p += 44)
	{
		//1.解码得到跟踪状态标记
		memcpy(&ChanStatus, p + 40, 4);
		ParityFlag = (ChanStatus >> 11) & 0x01;
		PhaseLockFlag = (ChanStatus >> 10) & 0x01;
		CodeLockedFlag = (ChanStatus >> 12) & 0x01;
		SatSystem = (ChanStatus >> 16) & 0x07;
		SigType = (ChanStatus >> 21) & 0x1F;
		//3.只读取GPS和BDS卫星
		//4.只读取L1 C/A、L2P(Y)，B1I和B3I
		if (SatSystem == 0) {
			sys = GPS;
			if (SigType == 0) {
				Freq = 0; w1 = WL1_GPS;
			}
			else if (SigType == 9) { Freq = 1; w1 = WL2_GPS; }
			else continue;
		}
		else if (SatSystem == 4) {
			sys = BDS;
			if (SigType == 0 || SigType == 4) {
				Freq = 0; w1 = WL1_BDS;
			}
			else if (SigType == 2 || SigType == 6) { Freq = 1; w1 = WL3_BDS; }
			else continue;
		}
		else continue;

		//4.解码得到卫星号prn以及系统号，在当前观测值结构体中搜索，
		//找到相同卫星，就将解码的观测值填充到该卫星对应的数组中；
		//如果当前已解码的卫星数据中没有发现，则填充到现有数据的末尾
		memcpy(&Prn, p, 2);
		for (j = 0; j < MAXCHANNUM; j++)
		{
			if (obs->SatObs[j].system == sys && obs->SatObs[j].Prn == Prn)
			{
				n = j;
				break;
			}
			else if (obs->SatObs[j].Prn == 0)
			{
				k = n = j;
				break;//找到第一个未储存数据的数组索引
			}
		}
		obs->SatObs[n].Prn = Prn;
		obs->SatObs[n].system = sys;
		obs->SatObs[n].P[Freq] = CodeLockedFlag == 1 ? d8(p + 4) : 0.0;
		obs->SatObs[n].L[Freq] = -w1 * (PhaseLockFlag == 1 ? d8(p + 16) : 0.0);
		obs->SatObs[n].D[Freq] = -w1 * d4(p + 28);
		obs->SatObs[n].cn0[Freq] = d4(p + 32);
		obs->SatObs[n].LockTime[Freq] = d4(p + 36);
		obs->SatObs[n].half[Freq] = ParityFlag;
	}
	obs->Satnum = k + 1;
	if (obs->Satnum == 0) return 0;
	return 1;
}

/*解码北斗星历，卫星是否健康*/
void decode_bdsephem(unsigned char* buff, GPSEPHREC beph[])
{
	int prn;
	unsigned char* p = buff + 28;//数据开始位置的指针
	GPSEPHREC* eph;
	memcpy(&prn, p, 4);
	eph = beph + prn - 1;
	if (prn < 1||prn >= MAXBDSNUM)
	{
		return;
	}
	memcpy(&eph->SVHealth, p + 16, 4);
	if (!eph->SVHealth)
	{
		eph->PRN = prn;
		eph->system = BDS;
		memcpy(&eph->TOC.week, p + 4, 4);
		memcpy(&eph->TOE.week, p + 4, 4);
		eph->URA = d8(p + 8)*d8(p+8);
		eph->TGD1 = d8(p + 20);
		eph->TGD2 = d8(p + 28);
		eph->TOC.secofweek=l4(p + 40);
		eph->ClkBias=d8(p+44);
		eph->ClkDrift = d8(p + 52);
		eph->ClkDriftRate = d8(p + 60);
		//memcpy(&eph->IODE, p + 68, 4);
		eph->IODE = l4(p + 68);
		//memcpy(&eph->IODC, p + 36, 4);
		eph->IODC = l4(p + 36);
		eph->TOE.secofweek = l4(p + 72);
		eph->SqrtA = d8(p + 76);
		eph->e = d8(p + 84);
		eph->omega = d8(p + 92);
		eph->DeltaN = d8(p + 100);
		eph->M0 = d8(p + 108);
		eph->OMEGA = d8(p + 116);
		eph->OMEGADot = d8(p + 124);
		eph->i0 = d8(p + 132);
		eph->idot = d8(p + 140);
		eph->Cuc = d8(p + 148);
		eph->Cus = d8(p + 156);
		eph->Crc = d8(p + 164);
		eph->Crs = d8(p + 172);
		eph->Cic = d8(p + 180);
		eph->Cis = d8(p + 188);
	}
	else
	{
		return;
	}

}


void decode_gpsephem(unsigned char* buff, GPSEPHREC geph[])
{
	int prn;
	unsigned char* p = buff + 28;//数据开始位置的指针
	GPSEPHREC* eph;
	memcpy(&prn, p, 4);
	if (prn < 1 || prn >= MAXGPSNUM)
	{
		return;
	}
	eph = geph + prn - 1;
	eph->PRN = prn;
	eph->system = GPS;
	//memcpy(&eph->IODE, p + 16, 4);
	eph->IODE = l4(p + 16);
	//memcpy(&eph->IODC, p + 160, 4);
	eph->IODC = l4(p + 160);
	memcpy(&eph->TOC.week, p + 24, 4);
	memcpy(&eph->TOE.week, p + 24, 4);
	eph->TOE.secofweek = d8(p + 32);
	eph->SqrtA = sqrt(d8(p + 40));
	eph->DeltaN = d8(p + 48);
	eph->M0 = d8(p + 56);
	eph->e = d8(p + 64);
	eph->omega = d8(p + 72);
	eph->Cuc = d8(p + 80);
	eph->Cus = d8(p + 88);
	eph->Crc = d8(p + 96);
	eph->Crs = d8(p + 104);
	eph->Cic = d8(p + 112);
	eph->Cis = d8(p + 120);
	eph->i0 = d8(p + 128);
	eph->idot = d8(p + 136);
	eph->OMEGA = d8(p + 144);
	eph->OMEGADot = d8(p + 152);
	eph->TOC.secofweek = d8(p + 164);
	eph->TGD1 = d8(p + 172);
	eph->TGD2 = 0.0;
	eph->ClkBias = d8(p + 180);
	eph->ClkDrift = d8(p + 188);
	eph->ClkDriftRate = d8(p + 196);
	eph->URA = d8(p + 216);


}

/*读取二进制文件*/
int read(const char* filename, const char* outfile)
{
	unsigned char buff[MAXRAWLEN];//缓存区
	FILE* file = NULL;
	errno_t err = fopen_s(&file, filename, "rb");
	EPOCHOBSDATA obs;
	POSRES pos;
	GPSEPHREC geph[MAXGPSNUM], beph[MAXBDSNUM];
	if (err != 0)
	{
		perror("文件打开失败");
		return -1;
	}
	int len = 0;
	int lenr;
	while (!feof(file))
	{
		lenr = fread(buff+len, sizeof(unsigned char), MAXRAWLEN - len, file);
		if (lenr < MAXRAWLEN-len)
		{
			break;
		}
		len += lenr;
		DecodeNovOem7Dat(buff, len, &obs, geph, beph,&pos);
	}

	fclose(file);
	return 1;
}
