#include "ApplicationConfig.h"

#include <limits>
#include <ostream>
#include <stdexcept>

namespace
{
bool ReadOptionValue(
    int argc,
    char* argv[],
    int& index,
    const std::string& option,
    std::string& value,
    std::string& errorMessage)
{
    if (index + 1 >= argc)
    {
        errorMessage = "选项缺少参数: " + option;
        return false;
    }

    value = argv[++index];
    return true;
}

bool ParsePort(
    const std::string& text,
    const std::string& option,
    unsigned short& port,
    std::string& errorMessage)
{
    try
    {
        std::size_t parsedLength = 0;
        const unsigned long value = std::stoul(text, &parsedLength);
        if (parsedLength != text.size() || value == 0 ||
            value > std::numeric_limits<unsigned short>::max())
        {
            throw std::out_of_range("port");
        }

        port = static_cast<unsigned short>(value);
        return true;
    }
    catch (const std::exception&)
    {
        errorMessage = option + " 必须是 1-65535 之间的端口号";
        return false;
    }
}
} // namespace

ApplicationConfig MakeDefaultConfig()
{
    ApplicationConfig config;

    // These defaults preserve the values used by the original main function.
    config.positioningMode = PositioningMode::Rtk;
    config.inputMode = InputMode::File;

    config.spp.inputFile = "raw_data_20250606_221850.bin";
    config.spp.outputFile = "spp.txt";
    config.spp.realtimeEndpoint = {"47.114.134.129", 7190};

    config.rtk.roverFile =
        "E:/GNSS_PPP/20h-short-baseline-data/oem719-202510311730-rover.bin";
    config.rtk.baseFile =
        "E:/GNSS_PPP/20h-short-baseline-data/oem719-202510311730-base.bin";
    config.rtk.filterRoverFile =
        "E:/GNSS_PPP/20h-short-baseline-data/oem719-202510311730-rover2.bin";
    config.rtk.filterBaseFile =
        "E:/GNSS_PPP/20h-short-baseline-data/oem719-202510311730-base2.bin";
    config.rtk.positionOutputFile = "rtkout.csv";
    config.rtk.filterOutputFile = "kfout.csv";
    config.rtk.roverEndpoint = {"8.148.22.229", 5002};
    config.rtk.baseEndpoint = {"47.114.134.129", 7190};

    return config;
}

CommandLineResult ParseCommandLine(
    int argc,
    char* argv[],
    ApplicationConfig& config,
    std::string& errorMessage)
{
    for (int index = 1; index < argc; ++index)
    {
        const std::string option = argv[index];
        std::string value;

        if (option == "--help" || option == "-h")
        {
            return CommandLineResult::HelpRequested;
        }
        if (option == "--positioning")
        {
            if (!ReadOptionValue(argc, argv, index, option, value, errorMessage))
                return CommandLineResult::Error;

            if (value == "spp")
                config.positioningMode = PositioningMode::Spp;
            else if (value == "rtk")
                config.positioningMode = PositioningMode::Rtk;
            else
            {
                errorMessage = "--positioning 仅支持 spp 或 rtk";
                return CommandLineResult::Error;
            }
        }
        else if (option == "--input")
        {
            if (!ReadOptionValue(argc, argv, index, option, value, errorMessage))
                return CommandLineResult::Error;

            if (value == "file")
                config.inputMode = InputMode::File;
            else if (value == "realtime")
                config.inputMode = InputMode::Realtime;
            else
            {
                errorMessage = "--input 仅支持 file 或 realtime";
                return CommandLineResult::Error;
            }
        }
        else if (option == "--spp-file")
        {
            if (!ReadOptionValue(argc, argv, index, option, config.spp.inputFile, errorMessage))
                return CommandLineResult::Error;
        }
        else if (option == "--spp-output")
        {
            if (!ReadOptionValue(argc, argv, index, option, config.spp.outputFile, errorMessage))
                return CommandLineResult::Error;
        }
        else if (option == "--rover-file")
        {
            if (!ReadOptionValue(argc, argv, index, option, config.rtk.roverFile, errorMessage))
                return CommandLineResult::Error;
        }
        else if (option == "--base-file")
        {
            if (!ReadOptionValue(argc, argv, index, option, config.rtk.baseFile, errorMessage))
                return CommandLineResult::Error;
        }
        else if (option == "--kf-rover-file")
        {
            if (!ReadOptionValue(argc, argv, index, option, config.rtk.filterRoverFile, errorMessage))
                return CommandLineResult::Error;
        }
        else if (option == "--kf-base-file")
        {
            if (!ReadOptionValue(argc, argv, index, option, config.rtk.filterBaseFile, errorMessage))
                return CommandLineResult::Error;
        }
        else if (option == "--rtk-output")
        {
            if (!ReadOptionValue(argc, argv, index, option, config.rtk.positionOutputFile, errorMessage))
                return CommandLineResult::Error;
        }
        else if (option == "--kf-output")
        {
            if (!ReadOptionValue(argc, argv, index, option, config.rtk.filterOutputFile, errorMessage))
                return CommandLineResult::Error;
        }
        else if (option == "--rover-ip")
        {
            if (!ReadOptionValue(argc, argv, index, option, config.rtk.roverEndpoint.address, errorMessage))
                return CommandLineResult::Error;
        }
        else if (option == "--base-ip")
        {
            if (!ReadOptionValue(argc, argv, index, option, config.rtk.baseEndpoint.address, errorMessage))
                return CommandLineResult::Error;
        }
        else if (option == "--spp-ip")
        {
            if (!ReadOptionValue(argc, argv, index, option, config.spp.realtimeEndpoint.address, errorMessage))
                return CommandLineResult::Error;
        }
        else if (option == "--rover-port" || option == "--base-port" || option == "--spp-port")
        {
            if (!ReadOptionValue(argc, argv, index, option, value, errorMessage))
                return CommandLineResult::Error;

            unsigned short* port = &config.spp.realtimeEndpoint.port;
            if (option == "--rover-port")
                port = &config.rtk.roverEndpoint.port;
            else if (option == "--base-port")
                port = &config.rtk.baseEndpoint.port;

            if (!ParsePort(value, option, *port, errorMessage))
                return CommandLineResult::Error;
        }
        else
        {
            errorMessage = "未知选项: " + option;
            return CommandLineResult::Error;
        }
    }

    return CommandLineResult::Success;
}

void PrintUsage(std::ostream& output, const char* executableName)
{
    output
        << "用法: " << executableName << " [选项]\n\n"
        << "运行模式:\n"
        << "  --positioning spp|rtk       定位模式（默认 rtk）\n"
        << "  --input file|realtime      输入模式（默认 file）\n\n"
        << "SPP 文件模式:\n"
        << "  --spp-file PATH            OEM7 原始数据文件\n"
        << "  --spp-output PATH          SPP 结果文件\n\n"
        << "RTK 文件模式:\n"
        << "  --rover-file PATH          流动站文件\n"
        << "  --base-file PATH           基准站文件\n"
        << "  --kf-rover-file PATH       EKF 流动站文件\n"
        << "  --kf-base-file PATH        EKF 基准站文件\n"
        << "  --rtk-output PATH          RTK 结果 CSV\n"
        << "  --kf-output PATH           EKF 结果 CSV\n\n"
        << "实时模式:\n"
        << "  --spp-ip IP --spp-port N\n"
        << "  --rover-ip IP --rover-port N\n"
        << "  --base-ip IP --base-port N\n\n"
        << "其他:\n"
        << "  -h, --help                 显示本帮助\n";
}
