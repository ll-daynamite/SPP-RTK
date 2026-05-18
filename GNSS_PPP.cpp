// GNSS_PPP.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include<string>
#include"CoordinateTransformation.h"
#include"TimeConversion.h"
#include"ErrorCorrection.h"
#include"Decode.h"
#include"Position.h"
#include"SPP_SPV.h"
#include"RTK.h"
int main()
{
	if (positioning == 0)//SPP模式
	{
		const char* filename = "raw_data_20250606_221850.bin";
		unsigned char buff[MAXRAWLEN];//缓存区
		FILE* file = NULL;
		errno_t err = fopen_s(&file, filename, "rb");
		EPOCHOBSDATA obs;
		POSRES pos, std_pos;
		GPSEPHREC geph[MAXGPSNUM], beph[MAXBDSNUM];
		XYZ res_xyz, std_xyz;
		BLH res_blh, std_blh;
		VectorXd enu_error(3);
		ofstream outfile("spp.txt");
		if (FILEMODE == 1)
		{
			if (err != 0)
			{
				perror("文件打开失败");
				return -1;
			}
			int len = 0;
			int lenr;
			while (!feof(file))
			{
				lenr = fread(buff + len, sizeof(unsigned char), MAXRAWLEN - len, file);
				if (lenr < MAXRAWLEN - len)
				{
					break;
				}
				len += lenr;
				int flag_code = 0;
				flag_code = DecodeNovOem7Dat(buff, len, &obs, geph, beph, &std_pos);
				if (flag_code == 1)
				{
					DetectOutlier(&obs);
					SPP(&obs, geph, beph, &pos);
					SPV(&obs, &pos);
					outfile << fixed << setprecision(4);
					outfile << pos.Time.week << " " << pos.Time.secofweek << " ";
					outfile << pos.Pos[0] << " " << pos.Pos[1] << " " << pos.Pos[2] << " " << pos.GPS_clk << " " << pos.BDS_clk << endl;
				}

			}

			fclose(file);

		}
		else if (FILEMODE == 2)
		{
			// 创建带时间戳的输出文件名
			char binFilename[100];
			time_t now = time(0);
			struct tm tstruct;
			localtime_s(&tstruct, &now);
			strftime(binFilename, sizeof(binFilename), "raw_data_%Y%m%d_%H%M%S.bin", &tstruct);
			FILE* binFile = NULL;
			errno_t binErr = fopen_s(&binFile, binFilename, "wb");
			if (binErr != 0) {
				printf("无法创建原始数据文件！\n");
				// 继续运行但不保存数据
			}
			else {
				printf("正在保存原始数据到: %s\n", binFilename);
			}
			SOCKET NetGps;
			int lenD = 0;
			int lenR;
			if (OpenSocket(NetGps, "47.114.134.129", 7190) == false)
			{
				printf("This ip & port was not opened.\n");
				if (binFile) fclose(binFile);
				return 0;
			}
			while (true)
			{
				Sleep(980);
				unsigned char Buff[MAXRAWLEN] = {};//临时缓存区
				if ((lenR = recv(NetGps, (char*)Buff, MAXRAWLEN, 0)) > 0)
				{
					// 保存接收到的原始数据
					if (binFile)
					{
						fwrite(Buff, 1, lenR, binFile);
						fflush(binFile);  // 确保数据立即写入磁盘
					}
					memcpy(buff + lenD, Buff, lenR);
					lenD += lenR;
					int flag_code = DecodeNovOem7Dat(buff, lenD, &obs, geph, beph, &std_pos);
					if (flag_code == 1)
					{
						DetectOutlier(&obs);
						SPP(&obs, geph, beph, &pos);
						res_xyz.X = pos.Pos[0];
						res_xyz.Y = pos.Pos[1];
						res_xyz.Z = pos.Pos[2];
						std_blh.latitude = std_pos.Pos[0];
						std_blh.longitude = std_pos.Pos[1];
						std_blh.height = std_pos.Pos[2];
						XYZToBLH(&res_xyz, &res_blh, R_WGS84, E_WGS84);
						enu_error = CompEnudPos(&std_blh, &res_blh, BLHToNEUMat(&std_blh));
						//cout << "站心定位误差：" << "E: " << enu_error(0) << "N: " << enu_error(1) << "U: " << enu_error(2) << endl;
						SPV(&obs, &pos);

					}

				}
			}

		}

	}
    else if (positioning == 1)//RTK模式
    {
        const char* PPP_filename1 = "E:/GNSS_PPP/20h-short-baseline-data/oem719-202510311730-rover.bin";
        const char* PPP_filename2 = "E:/GNSS_PPP/20h-short-baseline-data/oem719-202510311730-base.bin";
        const char* KF_filename1 = "E:/GNSS_PPP/20h-short-baseline-data/oem719-202510311730-rover2.bin";
        const char*KF_filename2 = "E:/GNSS_PPP/20h-short-baseline-data/oem719-202510311730-base2.bin";
        unsigned char buff_base[MAXRAWLEN], buff_rover[MAXRAWLEN];//缓存区
        // 打开文件
        FILE* file_rover = NULL;
        FILE* file_base = NULL;
        FILE* kf_file_rover = NULL;
        FILE* kf_file_base = NULL;
        errno_t err_rover = fopen_s(&file_rover, PPP_filename1, "rb");
        errno_t err_base = fopen_s(&file_base, PPP_filename2, "rb");
        errno_t kf_err_rover = fopen_s(&kf_file_rover, KF_filename1, "rb");
        errno_t kf_err_base = fopen_s(&kf_file_base, KF_filename2, "rb");
        POSRES base_pos, rover_pos;
        rtkdata data;
        rtkdata kf_data;
        RTKEKF kf;
        Vector3d filter_X;
        ofstream file("rtkout.csv");
        ofstream kffile("kfout.csv");
        if (!file.is_open()) {
            cerr << "无法创建文件: "  << endl;
            return -1;
        }
        file << "week,sec,X,Y,Z,fixed,ratio" << endl;
        kffile << "week,sec,X,Y,Z,fixed,ratio" << endl;
        if (FILEMODE == 1)
        {
            if (err_rover != 0 || err_base != 0||kf_err_base!=0||kf_err_rover!=0) {
                if (file_rover) fclose(file_rover);
                if (file_base) fclose(file_base);
                if (kf_file_base) fclose(kf_file_base);
                if (kf_file_base) fclose(kf_file_base);
                perror("文件打开失败");
                return -1;
            }

            while (1)
            {
                int synch = 0;
                synch = TimeSynch(file_rover, file_base, &data);
                if (synch == 1)
                {
                    DetectOutlier(&(data.rover_obs));
                    DetectOutlier(&(data.base_obs));
                    SPP(&data.rover_obs, data.geph, data.beph, &data.roverpos);
                    SPP(&data.base_obs, data.geph, data.beph, &data.basepos);
                    SPV(&data.rover_obs, &data.roverpos);
                    Detect_abnormal(&(data.rover_obs));
                    Detect_abnormal(&(data.base_obs));
                    FormSDEpochObs(&(data.base_obs), &(data.rover_obs), &(data.SdObs));
                    DetectCycleSlip(&(data.base_obs), &(data.rover_obs), &(data.SdObs));
                    DetRefSat(&(data.base_obs), &(data.rover_obs), &(data.SdObs), &(data.DDObs));
                    detect_epk(&(data.rover_obs), &(data.base_obs), &(data.SdObs), &(data.DDObs));
                    cout << data.roverpos.Time.week << " " << data.roverpos.Time.secofweek << endl;
                    if (RTKPostioning(&(data.base_obs), &(data.rover_obs), &(data.SdObs), &(data.DDObs), &data.basepos, &data.roverpos) == 2)
                    {
                        cout << fixed << setprecision(4);
                        cout << "流动站PPP结果：" << endl;
                        cout << data.roverpos.Pos[0] << " " << data.roverpos.Pos[1] << " " << data.roverpos.Pos[2] << endl;
                    }
                    else if (RTKPostioning(&(data.base_obs), &(data.rover_obs), &(data.SdObs), &(data.DDObs), &data.basepos, &data.roverpos) == 1)
                    {
                        cout << fixed << setprecision(4);
                        cout << "固定失败，输出浮点解：" << endl;
                        cout << data.roverpos.Pos[0] << " " << data.roverpos.Pos[1] << " " << data.roverpos.Pos[2] << endl;
                    }
                }
                else if (synch == 0) continue;
                else break;
                file << fixed << setprecision(4);
                file << data.roverpos.Time.week << ","
                    << data.roverpos.Time.secofweek << ","
                    << data.roverpos.Pos[0] << ","
                    << data.roverpos.Pos[1] << ","
                    << data.roverpos.Pos[2] << ","
                    << data.DDObs.bFixed << ","
                    << data.DDObs.Ratio << endl;
            }
            fclose(file_rover);
            fclose(file_base);
            while (1)
            {
                int kf_synch = 0;
                kf_synch = TimeSynch(kf_file_rover, kf_file_base, &kf_data);
                if (kf_synch == 1)
                {
                    DetectOutlier(&(kf_data.rover_obs));
                    DetectOutlier(&(kf_data.base_obs));
                    SPP(&kf_data.rover_obs, kf_data.geph, kf_data.beph, &kf_data.roverpos);
                    SPP(&kf_data.base_obs, kf_data.geph, kf_data.beph, &kf_data.basepos);
                    SPV(&kf_data.rover_obs, &kf_data.roverpos);
                    SPV(&kf_data.base_obs, &kf_data.basepos);
                    Detect_abnormal(&(kf_data.rover_obs));
                    Detect_abnormal(&(kf_data.base_obs));
                    FormSDEpochObs(&(kf_data.base_obs), &(kf_data.rover_obs), &(kf_data.SdObs));
                    DetectCycleSlip(&(kf_data.base_obs), &(kf_data.rover_obs), &(kf_data.SdObs));
                    DetRefSat(&(kf_data.base_obs), &(kf_data.rover_obs), &(kf_data.SdObs), &(kf_data.DDObs));
                    detect_epk(&(kf_data.rover_obs), &(kf_data.base_obs), &(kf_data.SdObs), &(kf_data.DDObs));
                    // 检查参考星索引有效性
                    if (kf_data.DDObs.RefPos[0] < 0 || kf_data.DDObs.RefPos[0] >= kf_data.SdObs.SatNum ||
                        kf_data.DDObs.RefPos[1] < 0 || kf_data.DDObs.RefPos[1] >= kf_data.SdObs.SatNum) {
                        continue;
                    }
                    if (kf_data.DDObs.RefPos[0] < 0 || kf_data.DDObs.RefPos[0] >= kf_data.SdObs.SatNum ||
                        kf_data.DDObs.RefPos[1] < 0 || kf_data.DDObs.RefPos[1] >= kf_data.SdObs.SatNum) {
                        continue;
                    }
                    kf.EKF(kf_data);
                    filter_X = kf.getPos();
                    if (kf_data.DDObs.bFixed)
                    {
                        cout << fixed << setprecision(4);
                        cout << "流动站滤波结果：" << endl;
                        cout << filter_X(0) << " " << filter_X(1) << " " << filter_X(2) << endl;
                    }
                    else
                    {
                        cout << fixed << setprecision(4);
                        cout << "流动站滤波浮点解：" << endl;
                        cout << filter_X(0) << " " << filter_X(1) << " " << filter_X(2) << endl;
                    }
                    kffile << fixed << setprecision(4);
                    kffile<<kf_data.roverpos.Time.week<<","
                        <<kf_data.roverpos.Time.secofweek<<","
                        <<filter_X(0) << ","
                        << filter_X(1) << ","
                        << filter_X(2) << ","
                        << kf_data.DDObs.bFixed << ","
                        << kf_data.DDObs.Ratio << endl;

                }
                else if (kf_synch == 0) continue;
                else break;

            }
            fclose(kf_file_rover);
            fclose(kf_file_base);
        }
        else if (FILEMODE == 2)
        {
            const char* rover_ip = "8.148.22.229";  // 流动站IP
            const char* base_ip = "47.114.134.129";   // 基站IP
            //const char* base_ip = "8.148.22.229";   // 基站IP
            int rover_port = 5002;                   // 流动站端口
            int base_port = 7190;                    // 基站端口
            // 创建带时间戳的输出文件名
            char roverBinFilename[100], baseBinFilename[100];
            time_t now = time(0);
            struct tm tstruct;
            localtime_s(&tstruct, &now);

            strftime(roverBinFilename, sizeof(roverBinFilename), "rover_raw_data_%Y%m%d_%H%M%S.bin", &tstruct);
            strftime(baseBinFilename, sizeof(baseBinFilename), "base_raw_data_%Y%m%d_%H%M%S.bin", &tstruct);

            FILE* roverBinFile = NULL;
            FILE* baseBinFile = NULL;

            errno_t roverBinErr = fopen_s(&roverBinFile, roverBinFilename, "wb");
            errno_t baseBinErr = fopen_s(&baseBinFile, baseBinFilename, "wb");

            if (roverBinErr != 0) {
                printf("无法创建流动站原始数据文件！\n");
            }
            else {
                printf("正在保存流动站原始数据到: %s\n", roverBinFilename);
            }

            if (baseBinErr != 0) {
                printf("无法创建基准站原始数据文件！\n");
            }
            else {
                printf("正在保存基准站原始数据到: %s\n", baseBinFilename);
            }
            // 打开两个Socket连接
            SOCKET NetGps_rover, NetGps_base;

            if (!OpenSocket(NetGps_base, base_ip, base_port)) {
                cout << "基站Socket连接失败" << endl;
                return -1;
            }
            if (!OpenSocket(NetGps_rover, rover_ip, rover_port)) {
                cout << "流动站Socket连接失败" << endl;
                closesocket(NetGps_base);
                return -1;
            }

            cout << "Socket RTK启动成功，等待数据流..." << endl;

            while (true)
            {

                Sleep(500);  // 适当延时

                // 使用RTKTimeSyn_socket进行时间同步
                short flag = RTKTimeSyn_socket(NetGps_rover, NetGps_base, data,roverBinFile,baseBinFile);
                if (flag == -1)
                {
                    printf("数据流断开，退出.\n");
                    break;
                }
                else if (flag == 0)
                    continue; // 未同步

                printf("-------------------------------------\n");
                printf("时间同步成功，开始RTK解算...\n");

                // 同步成功后执行RTK解算流程
                DetectOutlier(&(data.rover_obs));
                DetectOutlier(&(data.base_obs));

                // 单点定位
                cout << "流动站单点定位结果" << endl;
                bool rover_spp = SPP(&data.rover_obs, data.geph, data.beph, &data.roverpos);
                cout << "基准站单点定位结果" << endl;
                bool base_spp = SPP(&data.base_obs, data.geph, data.beph, &data.basepos);
                if (rover_spp &&base_spp )
                {
                    // 单点测速
                    SPV(&data.rover_obs, &data.roverpos);

                    // 构建站间卫星观测数据
                    FormSDEpochObs(&(data.base_obs), &(data.rover_obs), &(data.SdObs));

                    // 探测周跳
                    DetectCycleSlip(&(data.base_obs), &(data.rover_obs), &(data.SdObs));

                    // 探测参考卫星、检查星历
                    DetRefSat(&(data.base_obs), &(data.rover_obs), &(data.SdObs), &(data.DDObs));
                    detect_epk(&data.rover_obs, &data.base_obs, &data.SdObs, &data.DDObs);
                    // 执行RTK定位
                    int rtk_result = RTKPostioning(&(data.base_obs), &(data.rover_obs), &(data.SdObs), &(data.DDObs), &data.basepos, &data.roverpos);
                    kf.EKF(data);
                    filter_X = kf.getPos();
                    cout << fixed << setprecision(4);
                    cout << "流动站滤波结果：" << endl;
                    cout << filter_X(0) << " " << filter_X(1) << " " << filter_X(2) << endl;
                    if (rtk_result == 2)
                    {
                        cout << fixed << setprecision(4);
                        cout << "固定解成功！流动站位置：" << endl;
                        cout << data.roverpos.Pos[0] << " " << data.roverpos.Pos[1] << " " << data.roverpos.Pos[2] << endl;
                    }
                    else if (rtk_result == 1)
                    {
                        cout << fixed << setprecision(4);
                        cout << "固定失败，输出浮点解：" << endl;
                        cout << data.roverpos.Pos[0] << " " << data.roverpos.Pos[1] << " " << data.roverpos.Pos[2] << endl;
                    }
                    else
                    {
                        cout << "RTK解算失败" << endl;
                    }
                }
                else
                {
                    cout << "单点定位失败" << endl;
                }
            }

            // 关闭文件和Socket连接
            if (roverBinFile) fclose(roverBinFile);
            if (baseBinFile) fclose(baseBinFile);
            closesocket(NetGps_rover);
            closesocket(NetGps_base);
        }
    }
	system("pause");
	return 0;
}
