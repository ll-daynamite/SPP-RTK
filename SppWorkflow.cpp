#include "SppWorkflow.h"

#include "CoordinateTransformation.h"
#include "Decode.h"
#include "ErrorCorrection.h"
#include "SPP_SPV.h"
#include "WorkflowSupport.h"

#include <array>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace
{
void ProcessSppEpoch(
    EPOCHOBSDATA& observations,
    GPSEPHREC gpsEphemeris[],
    GPSEPHREC bdsEphemeris[],
    POSRES& position,
    bool writeObservationLog)
{
    DetectOutlier(&observations);
    SPP(&observations, gpsEphemeris, bdsEphemeris, &position, writeObservationLog);
    SPV(&observations, &position);
}

void WriteSppResult(std::ostream& output, const POSRES& position)
{
    output << std::fixed << std::setprecision(4)
           << position.Time.week << ' ' << position.Time.secofweek << ' '
           << position.Pos[0] << ' ' << position.Pos[1] << ' ' << position.Pos[2] << ' '
           << position.GPS_clk << ' ' << position.BDS_clk << std::endl;
}

} // namespace

int RunSppFile(const SppOptions& options)
{
    FileHandle input = OpenFile(options.inputFile, "rb");
    if (!input)
    {
        std::cerr << "无法打开 SPP 输入文件: " << options.inputFile << std::endl;
        return -1;
    }

    std::ofstream output(options.outputFile);
    if (!output)
    {
        std::cerr << "无法创建 SPP 输出文件: " << options.outputFile << std::endl;
        return -1;
    }

    std::array<unsigned char, MAXRAWLEN> buffer = {};
    int bufferedLength = 0;
    EPOCHOBSDATA observations;
    POSRES position;
    POSRES referencePosition;
    GPSEPHREC gpsEphemeris[MAXGPSNUM];
    GPSEPHREC bdsEphemeris[MAXBDSNUM];

    while (!std::feof(input.get()))
    {
        const int available = MAXRAWLEN - bufferedLength;
        const int bytesRead = static_cast<int>(
            std::fread(buffer.data() + bufferedLength, sizeof(unsigned char), available, input.get()));
        if (bytesRead < available)
            break;

        bufferedLength += bytesRead;
        if (DecodeNovOem7Dat(
                buffer.data(),
                bufferedLength,
                &observations,
                gpsEphemeris,
                bdsEphemeris,
                &referencePosition,
                true) == 1)
        {
            ProcessSppEpoch(observations, gpsEphemeris, bdsEphemeris, position, true);
            WriteSppResult(output, position);
        }
    }

    return 0;
}

int RunSppRealtime(const SppOptions& options)
{
    const std::string rawFilename =
        MakeTimestampedFilename("raw_data_%Y%m%d_%H%M%S.bin");
    FileHandle rawOutput = OpenFile(rawFilename, "wb");
    if (rawOutput)
        std::cout << "正在保存原始数据到: " << rawFilename << std::endl;
    else
        std::cerr << "无法创建原始数据文件，程序将继续运行但不保存数据" << std::endl;

    SocketHandle network;
    if (!network.Open(options.realtimeEndpoint))
    {
        std::cerr << "无法连接 SPP 数据源: "
                  << options.realtimeEndpoint.address << ':'
                  << options.realtimeEndpoint.port << std::endl;
        return -1;
    }

    std::array<unsigned char, MAXRAWLEN> decodeBuffer = {};
    int bufferedLength = 0;
    EPOCHOBSDATA observations;
    POSRES position;
    POSRES referencePosition;
    GPSEPHREC gpsEphemeris[MAXGPSNUM];
    GPSEPHREC bdsEphemeris[MAXBDSNUM];

    while (true)
    {
        Sleep(980);
        std::array<unsigned char, MAXRAWLEN> receiveBuffer = {};
        const int bytesRead = recv(
            network.Get(),
            reinterpret_cast<char*>(receiveBuffer.data()),
            MAXRAWLEN,
            0);
        if (bytesRead <= 0)
            continue;

        if (rawOutput)
        {
            std::fwrite(receiveBuffer.data(), 1, bytesRead, rawOutput.get());
            std::fflush(rawOutput.get());
        }

        if (bufferedLength + bytesRead > MAXRAWLEN)
            bufferedLength = 0;
        std::memcpy(decodeBuffer.data() + bufferedLength, receiveBuffer.data(), bytesRead);
        bufferedLength += bytesRead;

        if (DecodeNovOem7Dat(
                decodeBuffer.data(),
                bufferedLength,
                &observations,
                gpsEphemeris,
                bdsEphemeris,
                &referencePosition,
                false) != 1)
        {
            continue;
        }

        ProcessSppEpoch(observations, gpsEphemeris, bdsEphemeris, position, false);

        XYZ computedXyz = {position.Pos[0], position.Pos[1], position.Pos[2]};
        BLH computedBlh;
        BLH referenceBlh = {
            referencePosition.Pos[0],
            referencePosition.Pos[1],
            referencePosition.Pos[2]};
        XYZToBLH(&computedXyz, &computedBlh, R_WGS84, E_WGS84);

        // Keep the original ENU error calculation available to debuggers/callers.
        const VectorXd enuError = CompEnudPos(
            &referenceBlh,
            &computedBlh,
            BLHToNEUMat(&referenceBlh));
        (void)enuError;
    }
}
