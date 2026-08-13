#include "RtkWorkflow.h"

#include "ErrorCorrection.h"
#include "RTK.h"
#include "SPP_SPV.h"
#include "WorkflowSupport.h"

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace
{
struct RtkEpochOptions
{
    bool computeBaseVelocity = false;
    bool detectAbnormalObservations = true;
    bool requireSuccessfulSpp = false;
    bool writeObservationLog = true;
};

bool PrepareRtkEpoch(rtkdata& data, const RtkEpochOptions& options)
{
    DetectOutlier(&data.rover_obs);
    DetectOutlier(&data.base_obs);

    const bool roverSpp = SPP(
        &data.rover_obs,
        data.geph,
        data.beph,
        &data.roverpos,
        options.writeObservationLog);
    const bool baseSpp = SPP(
        &data.base_obs,
        data.geph,
        data.beph,
        &data.basepos,
        options.writeObservationLog);

    if (options.requireSuccessfulSpp && (!roverSpp || !baseSpp))
        return false;

    SPV(&data.rover_obs, &data.roverpos);
    if (options.computeBaseVelocity)
        SPV(&data.base_obs, &data.basepos);

    if (options.detectAbnormalObservations)
    {
        Detect_abnormal(&data.rover_obs);
        Detect_abnormal(&data.base_obs);
    }

    FormSDEpochObs(&data.base_obs, &data.rover_obs, &data.SdObs);
    DetectCycleSlip(&data.base_obs, &data.rover_obs, &data.SdObs);
    DetRefSat(&data.base_obs, &data.rover_obs, &data.SdObs, &data.DDObs);
    detect_epk(&data.rover_obs, &data.base_obs, &data.SdObs, &data.DDObs);
    return true;
}

bool HasValidReferenceSatellites(const rtkdata& data)
{
    return data.DDObs.RefPos[0] >= 0 &&
           data.DDObs.RefPos[0] < data.SdObs.SatNum &&
           data.DDObs.RefPos[1] >= 0 &&
           data.DDObs.RefPos[1] < data.SdObs.SatNum;
}

int SolveRtkEpoch(rtkdata& data)
{
    return RTKPostioning(
        &data.base_obs,
        &data.rover_obs,
        &data.SdObs,
        &data.DDObs,
        &data.basepos,
        &data.roverpos);
}

void PrintRtkResult(const rtkdata& data, int result)
{
    std::cout << std::fixed << std::setprecision(4);
    if (result == 2)
        std::cout << "固定解成功！流动站位置：" << std::endl;
    else if (result == 1)
        std::cout << "固定失败，输出浮点解：" << std::endl;
    else
    {
        std::cout << "RTK 解算失败" << std::endl;
        return;
    }

    std::cout << data.roverpos.Pos[0] << ' '
              << data.roverpos.Pos[1] << ' '
              << data.roverpos.Pos[2] << std::endl;
}

void WriteRtkCsvRow(std::ostream& output, const rtkdata& data)
{
    output << std::fixed << std::setprecision(4)
           << data.roverpos.Time.week << ','
           << data.roverpos.Time.secofweek << ','
           << data.roverpos.Pos[0] << ','
           << data.roverpos.Pos[1] << ','
           << data.roverpos.Pos[2] << ','
           << data.DDObs.bFixed << ','
           << data.DDObs.Ratio << std::endl;
}

void WriteFilterCsvRow(
    std::ostream& output,
    const rtkdata& data,
    const Vector3d& filteredPosition)
{
    output << std::fixed << std::setprecision(4)
           << data.roverpos.Time.week << ','
           << data.roverpos.Time.secofweek << ','
           << filteredPosition(0) << ','
           << filteredPosition(1) << ','
           << filteredPosition(2) << ','
           << data.DDObs.bFixed << ','
           << data.DDObs.Ratio << std::endl;
}

void PrintFilterResult(const rtkdata& data, const Vector3d& position)
{
    std::cout << std::fixed << std::setprecision(4)
              << (data.DDObs.bFixed ? "流动站滤波结果：" : "流动站滤波浮点解：")
              << std::endl
              << position(0) << ' ' << position(1) << ' ' << position(2)
              << std::endl;
}

} // namespace

int RunRtkFile(const RtkOptions& options)
{
    FileHandle roverInput = OpenFile(options.roverFile, "rb");
    FileHandle baseInput = OpenFile(options.baseFile, "rb");
    FileHandle filterRoverInput = OpenFile(options.filterRoverFile, "rb");
    FileHandle filterBaseInput = OpenFile(options.filterBaseFile, "rb");
    if (!roverInput || !baseInput || !filterRoverInput || !filterBaseInput)
    {
        std::cerr << "无法打开一个或多个 RTK 输入文件，请检查命令行路径" << std::endl;
        return -1;
    }

    std::ofstream positionOutput(options.positionOutputFile);
    std::ofstream filterOutput(options.filterOutputFile);
    if (!positionOutput || !filterOutput)
    {
        std::cerr << "无法创建 RTK 输出文件" << std::endl;
        return -1;
    }
    positionOutput << "week,sec,X,Y,Z,fixed,ratio" << std::endl;
    filterOutput << "week,sec,X,Y,Z,fixed,ratio" << std::endl;

    rtkdata positionData;
    const RtkEpochOptions positionEpochOptions = {
        false,
        true,
        false,
        true};

    while (true)
    {
        const int synchronization =
            TimeSynch(roverInput.get(), baseInput.get(), &positionData);
        if (synchronization == 0)
            continue;
        if (synchronization != 1)
            break;

        PrepareRtkEpoch(positionData, positionEpochOptions);
        std::cout << positionData.roverpos.Time.week << ' '
                  << positionData.roverpos.Time.secofweek << std::endl;

        // The original main could call RTKPostioning twice for a float solution.
        // Compute it once and reuse the result without changing the algorithm itself.
        const int result = SolveRtkEpoch(positionData);
        PrintRtkResult(positionData, result);
        WriteRtkCsvRow(positionOutput, positionData);
    }

    rtkdata filterData;
    RTKEKF filter;
    const RtkEpochOptions filterEpochOptions = {
        true,
        true,
        false,
        true};

    while (true)
    {
        const int synchronization =
            TimeSynch(filterRoverInput.get(), filterBaseInput.get(), &filterData);
        if (synchronization == 0)
            continue;
        if (synchronization != 1)
            break;

        PrepareRtkEpoch(filterData, filterEpochOptions);
        if (!HasValidReferenceSatellites(filterData))
            continue;

        filter.EKF(filterData);
        const Vector3d filteredPosition = filter.getPos();
        PrintFilterResult(filterData, filteredPosition);
        WriteFilterCsvRow(filterOutput, filterData, filteredPosition);
    }

    return 0;
}

int RunRtkRealtime(const RtkOptions& options)
{
    const std::string roverRawFilename =
        MakeTimestampedFilename("rover_raw_data_%Y%m%d_%H%M%S.bin");
    const std::string baseRawFilename =
        MakeTimestampedFilename("base_raw_data_%Y%m%d_%H%M%S.bin");
    FileHandle roverRawOutput = OpenFile(roverRawFilename, "wb");
    FileHandle baseRawOutput = OpenFile(baseRawFilename, "wb");

    if (roverRawOutput)
        std::cout << "正在保存流动站原始数据到: " << roverRawFilename << std::endl;
    else
        std::cerr << "无法创建流动站原始数据文件" << std::endl;
    if (baseRawOutput)
        std::cout << "正在保存基准站原始数据到: " << baseRawFilename << std::endl;
    else
        std::cerr << "无法创建基准站原始数据文件" << std::endl;

    SocketHandle baseSocket;
    SocketHandle roverSocket;
    if (!baseSocket.Open(options.baseEndpoint))
    {
        std::cerr << "基站 Socket 连接失败" << std::endl;
        return -1;
    }
    if (!roverSocket.Open(options.roverEndpoint))
    {
        std::cerr << "流动站 Socket 连接失败" << std::endl;
        return -1;
    }

    std::cout << "Socket RTK 启动成功，等待数据流..." << std::endl;

    rtkdata data;
    RTKEKF filter;
    const RtkEpochOptions epochOptions = {
        false,
        false,
        true,
        false};

    while (true)
    {
        Sleep(500);
        const int synchronization = RTKTimeSyn_socket(
            roverSocket.Get(),
            baseSocket.Get(),
            data,
            roverRawOutput.get(),
            baseRawOutput.get());
        if (synchronization < 0)
        {
            std::cout << "数据流断开，退出" << std::endl;
            break;
        }
        if (synchronization == 0)
            continue;

        std::cout << "-------------------------------------" << std::endl
                  << "时间同步成功，开始 RTK 解算..." << std::endl;
        if (!PrepareRtkEpoch(data, epochOptions))
        {
            std::cout << "单点定位失败" << std::endl;
            continue;
        }

        const int result = SolveRtkEpoch(data);
        filter.EKF(data);
        const Vector3d filteredPosition = filter.getPos();
        PrintFilterResult(data, filteredPosition);
        PrintRtkResult(data, result);
    }

    return 0;
}
