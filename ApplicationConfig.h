#pragma once

#include <iosfwd>
#include <string>

enum class PositioningMode
{
    Spp,
    Rtk
};

enum class InputMode
{
    File,
    Realtime
};

struct NetworkEndpoint
{
    std::string address;
    unsigned short port = 0;
};

struct SppOptions
{
    std::string inputFile;
    std::string outputFile;
    NetworkEndpoint realtimeEndpoint;
};

struct RtkOptions
{
    std::string roverFile;
    std::string baseFile;
    std::string filterRoverFile;
    std::string filterBaseFile;
    std::string positionOutputFile;
    std::string filterOutputFile;
    NetworkEndpoint roverEndpoint;
    NetworkEndpoint baseEndpoint;
};

struct ApplicationConfig
{
    PositioningMode positioningMode = PositioningMode::Rtk;
    InputMode inputMode = InputMode::File;
    SppOptions spp;
    RtkOptions rtk;
};

enum class CommandLineResult
{
    Success,
    HelpRequested,
    Error
};

ApplicationConfig MakeDefaultConfig();
CommandLineResult ParseCommandLine(
    int argc,
    char* argv[],
    ApplicationConfig& config,
    std::string& errorMessage);
void PrintUsage(std::ostream& output, const char* executableName);
