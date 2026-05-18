#include"RTK.h"
#include"sockets.h"
#include<iomanip>
using namespace std;
/*int RTKTimeSyn_socket(SOCKET rover_sock, SOCKET base_sock, rtkdata* data)//时间同步函数
{
    double dt;
    static int lenD_base = 0, lenD_rover = 0;//剩余长度
    static unsigned char buff_base[MAXRAWLEN], buff_rover[MAXRAWLEN];//最大缓冲区

    // 读取流动站数据
    int lenr_rover = recv(rover_sock, (char*)(buff_rover + lenD_rover), MAXRAWLEN - lenD_rover, 0);
    if (lenr_rover <= 0) return -1; // Socket连接断开

    lenD_rover += lenr_rover;

    // 解码流动站数据
    if (DecodeNovOem7Dat(buff_rover, lenD_rover, &data->rover_obs, data->geph, data->beph, &data->bestpos) != 1)
        return 0; // 尚未解出完整历元

    // 比较流动站观测时刻和当前基站观测时刻
    dt = (data->rover_obs.Time.week - data->base_obs.Time.week) * 604800 +
        data->rover_obs.Time.secofweek - data->base_obs.Time.secofweek;

    if (fabs(dt) < 1) return 1; // 时间同步成功

    // 若不在限差范围内，读取基站数据
    int lenr_base = recv(base_sock, (char*)(buff_base + lenD_base), MAXRAWLEN - lenD_base, 0);
    if (lenr_base <= 0) return -1; // Socket连接断开

    lenD_base += lenr_base;

    // 解码基站数据
    if (DecodeNovOem7Dat(buff_base, lenD_base, &data->base_obs, data->geph, data->beph, &data->bestpos) == 1)
    {
        dt = (data->rover_obs.Time.week - data->base_obs.Time.week) * 604800 +
            data->rover_obs.Time.secofweek - data->base_obs.Time.secofweek;

        if (fabs(dt) < 1)
            return 1; // 同步成功
        else if (dt < 0)
            return 0; // 流动站时间较旧，需要等待新的流动站数据
    }

    return 0; // 未同步
}*/
int RTKTimeSyn_socket(SOCKET rover_sock, SOCKET base_sock, rtkdata& rtkdata, FILE* roverFile, FILE* baseFile)
{
    static unsigned char rover_buff[MAXRAWLEN] = { 0 };
    static unsigned char base_buff[MAXRAWLEN] = { 0 };
    static int Rremaining = 0, Bremaining = 0;
    const double TIMELIMIT = 0.5;
    double time_diff = 0.0;

    // 读取流动站数据
    unsigned char tmpR[MAXRAWLEN];
    int lenR = recv(rover_sock, (char*)tmpR, MAXRAWLEN, 0);
    if (lenR <= 0) return -1;

    // 保存流动站原始数据
    if (roverFile) {
        fwrite(tmpR, 1, lenR, roverFile);
        fflush(roverFile);
    }

    if (Rremaining + lenR > MAXRAWLEN) Rremaining = 0;
    memcpy(rover_buff + Rremaining, tmpR, lenR);
    Rremaining += lenR;

    // 解流动站数据
    if (DecodeNovOem7Dat(rover_buff, Rremaining, &rtkdata.rover_obs, rtkdata.geph, rtkdata.beph, &rtkdata.roverpos) != 1)
        return 0;  // 尚未解出完整历元

    // 计算时间差
    time_diff = rtkdata.rover_obs.Time.secofweek - rtkdata.base_obs.Time.secofweek +
        (rtkdata.rover_obs.Time.week - rtkdata.base_obs.Time.week) * 604800;
    if (fabs(time_diff) < TIMELIMIT) return 1;

    // 若时间未对齐，持续从基站读数据直到同步
    unsigned char tmpB[MAXRAWLEN];
    int lenB = recv(base_sock, (char*)tmpB, MAXRAWLEN, 0);
    if (lenB > 0) {
        // 保存基准站原始数据
        if (baseFile) {
            fwrite(tmpB, 1, lenB, baseFile);
            fflush(baseFile);
        }

        if (Bremaining + lenB > MAXRAWLEN) Bremaining = 0;
        memcpy(base_buff + Bremaining, tmpB, lenB);
        Bremaining += lenB;
    }

    if (DecodeNovOem7Dat(base_buff, Bremaining, &rtkdata.base_obs, rtkdata.geph, rtkdata.beph, &rtkdata.basepos) == 1)
    {
        time_diff = rtkdata.rover_obs.Time.secofweek - rtkdata.base_obs.Time.secofweek +
            (rtkdata.rover_obs.Time.week - rtkdata.base_obs.Time.week) * 604800;

        if (fabs(time_diff) < TIMELIMIT)
            return 1;  // 同步成功
        else if (time_diff < 0)
            return 0;  // 基站时间较新
    }

    return 0;
}
int TimeSynch(FILE* roverfile, FILE* basefile, rtkdata*data)//时间同步函数
{
    double dt;
    int lenr_base=0, lenr_rover=0;//实际接收数据长度
    static int lenD_base = 0, lenD_rover = 0;//剩余长度
    static unsigned char buff_base[MAXRAWLEN], buff_rover[MAXRAWLEN];//最大缓冲区
    //读取流动站数据
    while (!feof(roverfile))
    {
        if ((lenr_rover = fread(buff_rover + lenD_rover, sizeof(unsigned char), MAXRAWLEN - lenD_rover, roverfile)) < MAXRAWLEN - lenD_rover)return -1;
        lenD_rover += lenr_rover;
        if (DecodeNovOem7Dat(buff_rover, lenD_rover, &data->rover_obs, data->geph, data->beph, &data->roverpos) == 1)break;
    }
    //比较流动站观测时刻和当前基站观测时刻
    dt = (data->rover_obs.Time.week - data->base_obs.Time.week) * 604800 + data->rover_obs.Time.secofweek - data->base_obs.Time.secofweek;
    if (fabs(dt) < 0.5)return 1;
    //若不在限差范围内
    while (!feof(basefile))
    {
        if ((lenr_base = fread(buff_base + lenD_base, sizeof(unsigned char), MAXRAWLEN - lenD_base, basefile)) < MAXRAWLEN - lenD_base)return -1;
        lenD_base += lenr_base;
        if (DecodeNovOem7Dat(buff_base, lenD_base, &data->base_obs, data->geph, data->beph, &data->basepos) == 1)
        {
            dt = (data->rover_obs.Time.week - data->base_obs.Time.week) * 604800 + data->rover_obs.Time.secofweek - data->base_obs.Time.secofweek;
            if (fabs(dt) <0.5)return 1;
            else if (dt < 0)return 0;
            else;
        }
    }
}
void Detect_abnormal( EPOCHOBSDATA* Epk)
{
    //检查卫星号和系统号是否正常，卫星观测值是否完整，如果不正常，continue；
    for (int i = 0; i < Epk->Satnum; i++)
    {
        if (Epk->SatObs[i].Prn == 0 || Epk->SatObs[i].system == UNKS) Epk->SatObs[i].Valid = false;
        else if (fabs(Epk->SatObs[i].P[0]) < 1e-5 || fabs(Epk->SatObs[i].P[1]) < 1e-5 ||
            fabs(Epk->SatObs[i].L[0]) < 1e-5 || fabs(Epk->SatObs[i].L[1]) < 1e-5)
        {
            Epk->SatObs[i].Valid = false;
        }
        else
        {
            Epk->SatObs[i].Valid = true;
        }
    }
}
void FormSDEpochObs(EPOCHOBSDATA* EpkB, EPOCHOBSDATA* EpkR, SDEPOCHOBS* SDObs)
{
    // 1. 参数有效性检查
    if (!EpkB || !EpkR || !SDObs) {
        return;
    }

    // 2. 数组边界检查
    if (!SDObs->SdSatObs) {
        return;
    }

    // 重置计数器
    SDObs->SatNum = 0;

    for (int i = 0; i < EpkR->Satnum; i++)
    {
        if (!EpkR->SatObs[i].Valid) continue;

        for (int j = 0; j < EpkB->Satnum; j++)
        {
            if (!EpkB->SatObs[j].Valid) continue;

            // 对同类型和同频率的观测值求差
            if (EpkR->SatObs[i].Prn == EpkB->SatObs[j].Prn &&
                EpkR->SatObs[i].system == EpkB->SatObs[j].system)
            {
                // 按照第一段代码逻辑添加有效性检查
                // 检查锁定时长等观测数据质量指标
                if (EpkR->SatObs[i].LockTime[0] < 6 || EpkR->SatObs[i].LockTime[1] < 6 ||
                    EpkB->SatObs[j].LockTime[0] < 6 || EpkB->SatObs[j].LockTime[1] < 6) {
                    continue; // 锁定时长不满足要求，跳过
                }



                // 计算单差观测值
                for (int k = 0; k < 2; k++)
                {
                    SDObs->SdSatObs[SDObs->SatNum].dP[k] = EpkR->SatObs[i].P[k] - EpkB->SatObs[j].P[k];
                    SDObs->SdSatObs[SDObs->SatNum].dL[k] = EpkR->SatObs[i].L[k] - EpkB->SatObs[j].L[k];
                }

                // 保存卫星信息
                SDObs->SdSatObs[SDObs->SatNum].prn = EpkR->SatObs[i].Prn;
                SDObs->SdSatObs[SDObs->SatNum].System = EpkR->SatObs[i].system;
                SDObs->SdSatObs[SDObs->SatNum].nBas = j;
                SDObs->SdSatObs[SDObs->SatNum].nRov = i;

                

                // 增加计数
                SDObs->SatNum++;

                break;
            }
        }
    }
    // 设置时间
    SDObs->Time = EpkR->Time;
}
    

void DetectCycleSlip(EPOCHOBSDATA* EpkB, EPOCHOBSDATA* EpkR, SDEPOCHOBS* Obs)//周跳探测
{
    MWGF CurComObs[MAXCHANNUM]; // 初始化数组
    double w1 = 0, w2 = 0;
    static EPOCHOBSDATA pre_base , pre_rover ;
    static int first_run = 1;
    
    // 第一次运行时初始化前历元数据
    if (first_run) {
        memcpy(&pre_base, EpkB, sizeof(EPOCHOBSDATA));
        memcpy(&pre_rover, EpkR, sizeof(EPOCHOBSDATA));
        first_run = 0;
        return; // 第一次运行不进行周跳探测
    }

    for (int i = 0; i < Obs->SatNum; i++)
    {
        int base_ind = Obs->SdSatObs[i].nBas;
        int rover_ind = Obs->SdSatObs[i].nRov;


        //相位锁定时间检查
        bool lock_time_valid = true;
        for (int bn = 0; bn < pre_base.Satnum; bn++)
        {
            if (pre_base.SatObs[bn].Prn == EpkB->SatObs[base_ind].Prn &&
                pre_base.SatObs[bn].system == EpkB->SatObs[base_ind].system)
            {
                if (EpkB->SatObs[base_ind].LockTime[0] < pre_base.SatObs[bn].LockTime[0] ||
                    EpkB->SatObs[base_ind].LockTime[1] < pre_base.SatObs[bn].LockTime[1])
                {
                    lock_time_valid = false;
                    break;
                }
            }
        }

        for (int rn = 0; rn < pre_rover.Satnum; rn++)
        {
            if (pre_rover.SatObs[rn].Prn == EpkR->SatObs[rover_ind].Prn &&
                pre_rover.SatObs[rn].system == EpkR->SatObs[rover_ind].system)
            {
                if (EpkR->SatObs[rover_ind].LockTime[0] < pre_rover.SatObs[rn].LockTime[0] ||
                    EpkR->SatObs[rover_ind].LockTime[1] < pre_rover.SatObs[rn].LockTime[1])
                {
                    lock_time_valid = false;
                    break;
                }
            }
        }

        if (!lock_time_valid) {
            Obs->SdSatObs[i].Valid = false;
            continue;
        }

        // 设置当前组合观测值的基本信息
        CurComObs[i].Prn = Obs->SdSatObs[i].prn;
        CurComObs[i].Sys = Obs->SdSatObs[i].System;
        CurComObs[i].n = 1; // 默认值

        // 计算当前历元该卫星的GF和MW组合值
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

        CurComObs[i].GF = Obs->SdSatObs[i].dL[0] - Obs->SdSatObs[i].dL[1];
        CurComObs[i].MW = (w1 * Obs->SdSatObs[i].dL[0] - w2 * Obs->SdSatObs[i].dL[1]) / (w1 - w2) -
            (w1 * Obs->SdSatObs[i].dP[0] + w2 * Obs->SdSatObs[i].dP[1]) / (w1 + w2);

        // 查找上一历元的对应卫星
        bool found_previous = false;
        for (int j = 0; j < MAXCHANNUM; j++)
        {
            if (CurComObs[i].Prn == Obs->SdCObs[j].Prn && CurComObs[i].Sys == Obs->SdCObs[j].Sys)
            {
                found_previous = true;
                // 计算当前历元该卫星GF与上一历元对应GF的差值dGF
                // 计算当前历元该卫星MW与上一历元对应MW平滑值的差值dMW
                double dGF = CurComObs[i].GF - Obs->SdCObs[j].GF;
                double dMW = CurComObs[i].MW - Obs->SdCObs[j].MW;

                // 检查dGF和dMW是否超限
                if (fabs(dGF) < 0.05 && fabs(dMW) < 3)
                {
                    // 正常卫星，保持Valid为true
                    Obs->SdSatObs[i].Valid = true;
                    // 计算MW平滑值
                    CurComObs[i].MW = (Obs->SdCObs[j].n * Obs->SdCObs[j].MW + CurComObs[i].MW) / (Obs->SdCObs[j].n + 1);
                    CurComObs[i].n = Obs->SdCObs[j].n + 1;
                }
                else
                {
                    // 周跳或粗差
                    Obs->SdSatObs[i].Valid = false;
                }
                break;
            }
        }
        //半周标记和卫星星历检查
        if (!EpkB->SatObs[base_ind].half[0] || !EpkB->SatObs[base_ind].half[1] ||
            !EpkR->SatObs[rover_ind].half[0] || !EpkR->SatObs[rover_ind].half[1] ||
            !EpkB->SatObs[base_ind].Valid || !EpkR->SatObs[rover_ind].Valid)
        {
            Obs->SdSatObs[i].Valid = false;
            continue;
        }
        // 如果没有找到上一历元的对应卫星，认为是新卫星，默认为有效
        if (!found_previous) {
            Obs->SdSatObs[i].Valid = true;
        }
    }

    // 更新组合观测值历史
    memcpy(Obs->SdCObs, CurComObs, MAXCHANNUM * sizeof(MWGF));

    // 更新前历元数据
    memcpy(&pre_base, EpkB, sizeof(EPOCHOBSDATA));
    memcpy(&pre_rover, EpkR, sizeof(EPOCHOBSDATA));
}
//卫星星历检查
void detect_epk(EPOCHOBSDATA* rover_obs, EPOCHOBSDATA* base_obs, SDEPOCHOBS* sd, DDCOBS* dd_related)
{
    for (int i = 0; i < sd->SatNum; i++)
    {
        int rover_idx = sd->SdSatObs[i].nRov, base_idx = sd->SdSatObs[i].nBas;
        int n = (sd->SdSatObs[i].System == GPS) ? 0 : 1;
        int ref_idx = dd_related->RefPos[n];
        // 跳过参考星/周跳的卫星
        if (i == ref_idx || !sd->SdSatObs[i].Valid) continue;
        // 卫星星历不正常
        if (!rover_obs->SatPVT[rover_idx].Valid || !base_obs->SatPVT[base_idx].Valid)
        {
            sd->SdSatObs[i].Valid = false;
            continue;
        }
        //剔除高度角低的卫星
        if (rover_obs->SatPVT[rover_idx].Elevation < MINELEV)
        {
            sd->SdSatObs[i].Valid = false;
        }
        if (base_obs->SatPVT[base_idx].Elevation < MINELEV)
        {
            sd->SdSatObs[i].Valid = false;
        }
    }
}
void DetRefSat(const EPOCHOBSDATA* epkA, const EPOCHOBSDATA* epkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs)
{
    int i, j, n;
    double Sum[2] = { 0.0 }, MaxSum[2] = { 0.0 };
    int RefPrn[2] = { -1 }, RefIndex[2] = { -1 };

    for (int i = 0; i < SDObs->SatNum; i++)
    {
        if (!SDObs->SdSatObs[i].Valid || !epkA->SatPVT[SDObs->SdSatObs[i].nBas].Valid || !epkB->SatPVT[SDObs->SdSatObs[i].nRov].Valid) continue;
        if (epkA->SatObs[SDObs->SdSatObs[i].nBas].LockTime[0] < 6 || epkA->SatObs[SDObs->SdSatObs[i].nBas].LockTime[1] < 6 ) continue;

        n = SDObs->SdSatObs[i].System == GPS ? 0 : 1;
        Sum[n] = epkB->SatPVT[SDObs->SdSatObs[i].nRov].Elevation + epkA->SatObs[SDObs->SdSatObs[i].nBas].cn0[0]+epkA->SatObs[SDObs->SdSatObs[i].nBas].cn0[1]+
                 epkB->SatObs[SDObs->SdSatObs[i].nRov].cn0[0]+ epkB->SatObs[SDObs->SdSatObs[i].nRov].cn0[1];//质量评估指标

        if (Sum[n] > MaxSum[n]) {
            MaxSum[n] = Sum[n];
            RefPrn[n] = SDObs->SdSatObs[i].prn;
            RefIndex[n] = i;
        }
    }

    for (i = 0; i < 2; i++) {
        if (MaxSum[i] < 150.0) DDObs->RefPos[i] = -1;
        else
        {
            DDObs->RefPos[i] = RefIndex[i];//参考星索引
            DDObs->RefPrn[i] = RefPrn[i];//参考星prn号
        }
    }
}
/*void DetRefSat(const EPOCHOBSDATA* epkA, const EPOCHOBSDATA* epkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs)
{
    // 初始化参考星信息
    DDObs->RefPos[0] = -1;  // GPS参考星索引
    DDObs->RefPos[1] = -1;  // BDS参考星索引
    DDObs->RefPrn[0] = -1;  // GPS参考星PRN
    DDObs->RefPrn[1] = -1;  // BDS参考星PRN

    double MaxScore[2] = { 0.0 };
    int RefIndex[2] = { -1, -1 };

    for (int i = 0; i < SDObs->SatNum; i++)
    {
        // 有效性检查
        if (!SDObs->SdSatObs[i].Valid) continue;

        int base_idx = SDObs->SdSatObs[i].nBas;
        int rover_idx = SDObs->SdSatObs[i].nRov;

        // 检查索引有效性
        if (base_idx < 0 || base_idx >= epkA->Satnum ||
            rover_idx < 0 || rover_idx >= epkB->Satnum) continue;

        // 检查PVT有效性
        if (!epkA->SatPVT[base_idx].Valid || !epkB->SatPVT[rover_idx].Valid) continue;

        // 检查锁定时长（基站和流动站都要检查）
        if (epkA->SatObs[base_idx].LockTime[0] < 6 ||
            epkA->SatObs[base_idx].LockTime[1] < 6 ||
            epkB->SatObs[rover_idx].LockTime[0] < 6 ||
            epkB->SatObs[rover_idx].LockTime[1] < 6) continue;

        // 确定系统类型
        int sys = (SDObs->SdSatObs[i].System == GPS) ? 0 : 1;

        // 改进的评分标准
        double elevation_score = epkB->SatPVT[rover_idx].Elevation;  // 高度角（度）
        double cn0_score = (epkA->SatObs[base_idx].cn0[0] +
            epkA->SatObs[base_idx].cn0[1] +
            epkB->SatObs[rover_idx].cn0[0] +
            epkB->SatObs[rover_idx].cn0[1]) / 4.0;  // 平均信噪比

        // 综合评分（可以根据需要调整权重）
        double total_score = elevation_score * 2.0 + cn0_score;  // 高度角权重更高

        // 选择评分最高的作为参考星
        if (total_score > MaxScore[sys]) {
            MaxScore[sys] = total_score;
            RefIndex[sys] = i;
            DDObs->RefPrn[sys] = SDObs->SdSatObs[i].prn;
        }
    }

    // 设置参考星位置索引
    for (int i = 0; i < 2; i++) {
        if (MaxScore[i] >30.0) {  
            DDObs->RefPos[i] = RefIndex[i];
        }
        else {
            DDObs->RefPos[i] = -1;
            DDObs->RefPrn[i] = -1;
        }
    }
}*/
int RTKPostioning(const EPOCHOBSDATA* epkA, const EPOCHOBSDATA* epkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* base, POSRES* rover)
{
    int ddgpsnum = 0, ddbdsnum = 0;

    // 初始化基站和流动站位置
    MatrixXd Basepos(3, 1), Roverpos(3, 1);
    double norm_base = sqrt(pow(base->BestPos[0], 2)+ pow(base->BestPos[1], 2)+ pow(base->BestPos[2], 2));
    for (int i = 0; i < 3; i++)
    {
        if (norm_base == 0.0)
        {
            Basepos(i, 0) = base->Pos[i];
        }
        else 
        {
            Basepos(i, 0) = base->BestPos[i];
        }
        Roverpos(i, 0) = rover->Pos[i];    
    }
    // 检查参考星索引有效性
    if (DDObs->RefPos[0] < 0 || DDObs->RefPos[0] >= SDObs->SatNum ||
        DDObs->RefPos[1] < 0 || DDObs->RefPos[1] >= SDObs->SatNum) {
        return 0;
    }
    // 计算GPS和BDS双差卫星数
    for (int i = 0; i < SDObs->SatNum; i++)
    {
        if (SDObs->SdSatObs[i].System == BDS && SDObs->SdSatObs[i].prn != DDObs->RefPrn[1]&&SDObs->SdSatObs[i].Valid)
        {
            ddbdsnum++;
        }
        if (SDObs->SdSatObs[i].System == GPS && SDObs->SdSatObs[i].prn != DDObs->RefPrn[0] && SDObs->SdSatObs[i].Valid)
        {
            ddgpsnum++;
        }
    }
    DDObs->Sats = ddgpsnum + ddbdsnum;
    DDObs->DDSatNum[0] = ddgpsnum;
    DDObs->DDSatNum[1] = ddbdsnum;

    // 初始化矩阵
    int total_params = 3 + (DDObs->Sats) * 2;
    int total_obs = 4 * (DDObs->Sats);

    MatrixXd B(total_obs, total_params);
    MatrixXd W(total_obs, 1);
    VectorXd X(total_params), delta_X(total_params), N(total_obs);
    MatrixXd P = MatrixXd::Zero(total_obs, total_obs);
    MatrixXd distance_base2sat(DDObs->Sats, 1);
    MatrixXd baseref_sat_PVT_GPS(3, 1), baseref_sat_PVT_BDS(3, 1), roverref_sat_PVT_BDS(3, 1), roverref_sat_PVT_GPS(3, 1);

    // 定义参数
    double sigma_p = 0.3, sigma_l = 0.01;
    X.head(3) = Roverpos;
    int curr_id = 0;

    // 获取参考星索引
    int ref_base[2] = { SDObs->SdSatObs[DDObs->RefPos[0]].nBas, SDObs->SdSatObs[DDObs->RefPos[1]].nBas };
    // 检查参考星索引是否在有效范围内
    if (ref_base[0] < 0 || ref_base[0] >= epkA->Satnum ||
        ref_base[1] < 0 || ref_base[1] >= epkA->Satnum) {
        return 0;
    }
    for (int i = 0; i < 3; i++)
    {
        baseref_sat_PVT_GPS(i, 0) = epkA->SatPVT[ref_base[0]].SatPos[i]; 
        baseref_sat_PVT_BDS(i, 0) = epkA->SatPVT[ref_base[1]].SatPos[i];  
    }

    double distance_base2ref[2] = {
        (Basepos - baseref_sat_PVT_GPS).norm(),
        (Basepos - baseref_sat_PVT_BDS).norm()
    };

    // 构建设计矩阵和权阵
    for (int i = 0; i < SDObs->SatNum; i++)
    {
        int n = -1;
        if (SDObs->SdSatObs[i].System == GPS) n = 0;
        else if (SDObs->SdSatObs[i].System == BDS) n = 1;
        else continue; // 跳过其他系统

        int ref_idx = DDObs->RefPos[n];
        int base_idx = SDObs->SdSatObs[i].nBas;

        // 跳过参考星和不可用的卫星
        if (i == ref_idx || !SDObs->SdSatObs[i].Valid) continue;

        // 双差观测值
        N(curr_id * 4 + 0) = SDObs->SdSatObs[i].dP[0] - SDObs->SdSatObs[ref_idx].dP[0];
        N(curr_id * 4 + 1) = SDObs->SdSatObs[i].dP[1] - SDObs->SdSatObs[ref_idx].dP[1];
        N(curr_id * 4 + 2) = SDObs->SdSatObs[i].dL[0] - SDObs->SdSatObs[ref_idx].dL[0];
        N(curr_id * 4 + 3) = SDObs->SdSatObs[i].dL[1] - SDObs->SdSatObs[ref_idx].dL[1];

        // 计算基站到非参考卫星的距离
        distance_base2sat(curr_id,0) = sqrt(
            pow(Basepos(0) - epkA->SatPVT[base_idx].SatPos[0], 2) +
            pow(Basepos(1) - epkA->SatPVT[base_idx].SatPos[1], 2) +
            pow(Basepos(2) - epkA->SatPVT[base_idx].SatPos[2], 2)
        );

        double f1 = 0, f2 = 0;
        if (SDObs->SdSatObs[i].System == GPS) {
            f1 = WL1_GPS; f2 = WL2_GPS;
        }
        else {
            f1 = WL1_BDS; f2 = WL3_BDS;
        }

        // 初始化模糊度参数
        X(3 + curr_id * 2 + 0) = (N(curr_id * 4 + 2) - N(curr_id * 4 + 0)) / f1;
        X(3 + curr_id * 2 + 1) = (N(curr_id * 4 + 3) - N(curr_id * 4 + 1)) / f2;

        // 初始化权阵
        int tmp_idx = 0;
        for (int j = 0; j < SDObs->SatNum; j++)
        {
            if (j == DDObs->RefPos[0] || j == DDObs->RefPos[1] || !SDObs->SdSatObs[j].Valid) continue;

            if (SDObs->SdSatObs[i].System != SDObs->SdSatObs[j].System)
            {
                tmp_idx++;
                continue;
            }

            int ddnum = (SDObs->SdSatObs[i].System == GPS) ? ddgpsnum : ddbdsnum;
            if (curr_id == tmp_idx)
            {
                P(curr_id * 4 + 0, tmp_idx * 4 + 0) = ddnum / (pow(sigma_p,2) * (ddnum + 1));
                P(curr_id * 4 + 1, tmp_idx * 4 + 1) = ddnum / (pow(sigma_p, 2) * (ddnum + 1));
                P(curr_id * 4 + 2, tmp_idx * 4 + 2) = ddnum / (pow(sigma_l, 2) * (ddnum + 1));
                P(curr_id * 4 + 3, tmp_idx * 4 + 3) = ddnum / (pow(sigma_l, 2) * (ddnum + 1));
            }
            else
            {
                P(curr_id * 4 + 0, tmp_idx * 4 + 0) = -1 / (pow(sigma_p, 2) * (ddnum + 1));
                P(curr_id * 4 + 1, tmp_idx * 4 + 1) = -1 / (pow(sigma_p, 2) * (ddnum + 1));
                P(curr_id * 4 + 2, tmp_idx * 4 + 2) = -1 / (pow(sigma_l, 2) * (ddnum + 1));
                P(curr_id * 4 + 3, tmp_idx * 4 + 3) = -1 / (pow(sigma_l, 2) * (ddnum + 1));
            }
            tmp_idx++;
        }
        curr_id++;
    }

    // 最小二乘迭代求解浮点解
    int count = 0, rows = 0;
    int ref_rover[2] = { SDObs->SdSatObs[DDObs->RefPos[0]].nRov, SDObs->SdSatObs[DDObs->RefPos[1]].nRov };

    MatrixXd float_Qxx; // 浮点解协方差矩阵
    // 检查流动站参考星索引有效性
    if (ref_rover[0] < 0 || ref_rover[0] >= epkB->Satnum ||
        ref_rover[1] < 0 || ref_rover[1] >= epkB->Satnum) {
        return 0;
    }
    do{

        VectorXd distance_rover2sat(DDObs->Sats);
        B.setZero();
        W.setZero();
        rows = 0;

        for (int i = 0; i < 3; i++)
        {
            roverref_sat_PVT_GPS(i, 0) = epkB->SatPVT[ref_rover[0]].SatPos[i];
            roverref_sat_PVT_BDS(i, 0) = epkB->SatPVT[ref_rover[1]].SatPos[i];
        }

        double distance_rover2sat_ref[2] = {
            (X.head(3) - roverref_sat_PVT_GPS).norm(),
            (X.head(3) - roverref_sat_PVT_BDS).norm()
        };

        for (int i = 0; i < SDObs->SatNum; i++)
        {
            int n = -1;
            if (SDObs->SdSatObs[i].System == GPS) n = 0;
            else if (SDObs->SdSatObs[i].System == BDS) n = 1;
            else continue;

            int ref_idx = DDObs->RefPos[n];
            int rover_idx = SDObs->SdSatObs[i].nRov;
            int base_idx = SDObs->SdSatObs[i].nBas;
            // 跳过参考星和不可用卫星
            if (i == ref_idx || !SDObs->SdSatObs[i].Valid) continue;
            // 流动站到非参考星的距离计算
            distance_rover2sat(rows,0) = sqrt(
                pow(X(0) - epkB->SatPVT[rover_idx].SatPos[0], 2) +
                pow(X(1) - epkB->SatPVT[rover_idx].SatPos[1], 2) +
                pow(X(2) - epkB->SatPVT[rover_idx].SatPos[2], 2)
            );

            double f1 = 0, f2 = 0;
            if (SDObs->SdSatObs[i].System == GPS) {
                f1 = WL1_GPS; f2 = WL2_GPS;
            }
            else {
                f1 = WL1_BDS; f2 = WL3_BDS;
            }
            double range_dd = (distance_rover2sat(rows) - distance_rover2sat_ref[n])
                - (distance_base2sat(rows) - distance_base2ref[n]); 
            W(4 * rows + 0) = N(4 * rows + 0) - range_dd;
            W(4 * rows + 1) = N(4 * rows + 1) - range_dd;
            W(4 * rows + 2) = N(4 * rows + 2) - range_dd - f1 * X(3 + rows * 2 + 0);
            W(4 * rows + 3) = N(4 * rows + 3) - range_dd - f2 * X(3 + rows * 2 + 1);
            // 观测方程线性化
            // 设计矩阵
            double l = (X(0) - epkB->SatPVT[rover_idx].SatPos[0]) / distance_rover2sat(rows) -
                (X(0) - epkB->SatPVT[ref_rover[n]].SatPos[0]) / distance_rover2sat_ref[n];
            double m = (X(1) - epkB->SatPVT[rover_idx].SatPos[1]) / distance_rover2sat(rows) -
                (X(1) - epkB->SatPVT[ref_rover[n]].SatPos[1]) / distance_rover2sat_ref[n];
            double n_val = (X(2) - epkB->SatPVT[rover_idx].SatPos[2]) / distance_rover2sat(rows) -
                (X(2) - epkB->SatPVT[ref_rover[n]].SatPos[2]) / distance_rover2sat_ref[n];

            for (int j = 0; j < 4; j++)
            {
                B(4 * rows + j, 0) = l;
                B(4 * rows + j, 1) = m;
                B(4 * rows + j, 2) = n_val;
            }
            B(rows * 4 + 2, 3 + rows * 2 + 0) = f1;
            B(rows * 4 + 3, 3 + rows * 2 + 1) = f2;
            rows++;
        }
        // 观测方程数不足
        if (rows * 4 < total_params || SDObs->SatNum < 4) break;

        // 计算法方程和协方差矩阵
        MatrixXd N_mat = B.transpose() * P * B;
        float_Qxx = N_mat.inverse();  // 保存协方差矩阵用于模糊度固定

        delta_X = float_Qxx * B.transpose() * P * W;

        if (isnan(delta_X(0)))
        {
            cout << "nan detected in delta_X" << endl;
            break;
        }
        X += delta_X;
        count++;
    } while (delta_X.norm() > 0.001 && count < 100);

    // 保存浮点解
    rover->Pos[0] = X(0);
    rover->Pos[1] = X(1);
    rover->Pos[2] = X(2);

    // LAMBDA模糊度固定
    DDObs->bFixed = false;
    DDObs->Ratio = 0.0;

    int n = (ddgpsnum + ddbdsnum) * 2;  // 模糊度参数个数
    int noamb_param_num = 3;             // 非模糊度参数个数（位置参数）

    // 检查是否有足够的模糊度进行固定
    if (n > 0 && float_Qxx.rows() >= total_params)
    {
        // 浮点模糊度和协方差
        VectorXd float_ambiguities = X.segment(noamb_param_num, n);
        VectorXd fixed_ambiguities(n);
        MatrixXd Q_ambiguities = float_Qxx.block(noamb_param_num, noamb_param_num, n, n);
        VectorXd X_float = X.head(3);
        MatrixXd Q_pos_amb = float_Qxx.topRightCorner(noamb_param_num, n);

        // 调用LAMBDA算法
        double* a_ptr = float_ambiguities.data();
        double* Q_ptr = Q_ambiguities.data();
        double* s_ptr = DDObs->ResAmb;

        if (lambda(n, DDObs->m, a_ptr, Q_ptr, DDObs->FixedAmb, DDObs->ResAmb) == 0)
        {
            // 模糊度确认
            DDObs->Ratio = DDObs->ResAmb[1] / DDObs->ResAmb[0];
            if (DDObs->Ratio >= 3.0)
            {
                // 固定成功，计算固定解
                DDObs->bFixed = true;
                for (int i = 0; i < float_ambiguities.size(); i++)
                {
                    fixed_ambiguities(i) = DDObs->FixedAmb[i];
                }
                VectorXd X_fixed = X_float - Q_pos_amb * Q_ambiguities.inverse() * (float_ambiguities - fixed_ambiguities);


                // 更新流动站位置为固定解
                rover->Pos[0] = X_fixed(0);
                rover->Pos[1] = X_fixed(1);
                rover->Pos[2] = X_fixed(2);
                cout << "基线向量：" << endl;
                for (int i = 0; i < 3; i++)
                {
                    cout << fixed << setprecision(4);
                    cout << rover->Pos[i] - Basepos(i) << endl;
                }

                cout << "Fixed solution successful, Ratio: " << DDObs->Ratio << endl;
                return 2;  // 返回固定解状态
            }
        }

        if (!DDObs->bFixed)
        {
            cout << "Fixed solution failed, using float solution" << endl;
            // 使用浮点解
            rover->Pos[0] = X(0);
            rover->Pos[1] = X(1);
            rover->Pos[2] = X(2);

            return 1;  // 返回浮点解状态
        }
    }
    else
    {
        // 没有足够卫星进行模糊度固定，使用浮点解
        rover->Pos[0] = X(0);
        rover->Pos[1] = X(1);
        rover->Pos[2] = X(2);
        return 1;  // 返回浮点解状态
    }



    return 0;  // 失败
}
