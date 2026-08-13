#include "Application.h"

#include "RtkWorkflow.h"
#include "SppWorkflow.h"

int RunApplication(const ApplicationConfig& config)
{
    if (config.positioningMode == PositioningMode::Spp)
    {
        return config.inputMode == InputMode::File
            ? RunSppFile(config.spp)
            : RunSppRealtime(config.spp);
    }

    return config.inputMode == InputMode::File
        ? RunRtkFile(config.rtk)
        : RunRtkRealtime(config.rtk);
}
