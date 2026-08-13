#include "Application.h"
#include "ApplicationConfig.h"

#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    ApplicationConfig config = MakeDefaultConfig();
    std::string errorMessage;
    const CommandLineResult parseResult =
        ParseCommandLine(argc, argv, config, errorMessage);

    if (parseResult == CommandLineResult::HelpRequested)
    {
        PrintUsage(std::cout, argv[0]);
        return 0;
    }
    if (parseResult == CommandLineResult::Error)
    {
        std::cerr << errorMessage << "\n\n";
        PrintUsage(std::cerr, argv[0]);
        return 2;
    }

    return RunApplication(config);
}
