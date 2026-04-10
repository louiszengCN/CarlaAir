// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "AirSim.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Modules/ModuleInterface.h"
#if WITH_EDITOR
#include <compiler/disable-ue4-macros.h>
#include <carla/Exception.h>
#include <compiler/enable-ue4-macros.h>
#include <exception>
#endif

class FAirSim : public IModuleInterface
{
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};

IMPLEMENT_MODULE(FAirSim, AirSim)

void FAirSim::StartupModule()
{
    //plugin startup
    UE_LOG(LogTemp, Log, TEXT("StartupModule: AirSim plugin"));
}

void FAirSim::ShutdownModule()
{
    //plugin shutdown
}

#if WITH_EDITOR
namespace carla {

void throw_exception(const std::exception &e)
{
    UE_LOG(LogTemp, Fatal, TEXT("Carla exception forwarded via AirSim: %s"), UTF8_TO_TCHAR(e.what()));
    std::terminate();
}

} // namespace carla
#endif
