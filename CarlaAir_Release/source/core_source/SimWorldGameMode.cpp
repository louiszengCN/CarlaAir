// SimWorldGameMode.cpp — Unified CARLA + AirSim GameMode
// All methods ported from SimHUD.cpp and AirSimGameMode.cpp with minimal changes.
// v0.1.2: Added built-in FPS drone control with background thread.

#include "SimWorldGameMode.h"
#include "UObject/ConstructorHelpers.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Misc/FileHelper.h"
#include "IImageWrapperModule.h"

#include "Vehicles/Multirotor/SimModeWorldMultiRotor.h"
#include "Vehicles/Multirotor/MultirotorPawnSimApi.h"
#include "Vehicles/Car/SimModeCar.h"
#include "Vehicles/ComputerVision/SimModeComputerVision.h"

#include "common/AirSimSettings.hpp"
#include "common/Common.hpp"
#include "AirBlueprintLib.h"
#include "Carla/Game/CarlaHUD.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Weather/Weather.h"
#include "Carla/Actor/CarlaActorFactory.h"
#include "Carla/Actor/CarlaActorFactoryBlueprint.h"
#include "Carla/Sensor/SensorFactory.h"
#include "Carla/Actor/StaticMeshFactory.h"
#include "Carla/Trigger/TriggerFactory.h"
#include "Carla/Actor/UtilActorFactory.h"
#include "Carla/AI/AIControllerFactory.h"

#include "GameFramework/SpectatorPawn.h"
#include "Kismet/GameplayStatics.h"

#include <stdexcept>
#include <cmath>

// ---------- AirSim logger (separate instance for SimWorldGameMode) ----------

class ASimWorldUnrealLog : public msr::airlib::Utils::Logger
{
public:
    virtual void log(int level, const std::string& message) override
    {
        size_t tab_pos;
        static const std::string delim = ":\t";
        if ((tab_pos = message.find(delim)) != std::string::npos) {
            UAirBlueprintLib::LogMessageString(message.substr(0, tab_pos),
                                               message.substr(tab_pos + delim.size(), std::string::npos),
                                               LogDebugLevel::Informational);
            return;
        }

        if (level == msr::airlib::Utils::kLogLevelError) {
            UE_LOG(LogTemp, Error, TEXT("%s"), *FString(message.c_str()));
        }
        else if (level == msr::airlib::Utils::kLogLevelWarn) {
            UE_LOG(LogTemp, Warning, TEXT("%s"), *FString(message.c_str()));
        }
        else {
            UE_LOG(LogTemp, Log, TEXT("%s"), *FString(message.c_str()));
        }

        msr::airlib::Utils::Logger::log(level, message);
    }
};

static ASimWorldUnrealLog GlobalSimWorldLog;

// ========================== FDroneControlWorker ==========================
// Background thread: reads shared velocity/yaw, calls blocking moveByVelocity()

class FDroneControlWorker : public FRunnable
{
public:
    FDroneControlWorker(ASimModeBase* SimMode, FCriticalSection* Lock,
                        FVector* VelocityNED, float* Yaw, bool* ShouldHover, bool* DroneReady)
        : SimMode_(SimMode), Lock_(Lock), VelocityNED_(VelocityNED),
          Yaw_(Yaw), bShouldHover_(ShouldHover), bDroneReady_(DroneReady)
    {
    }

    virtual bool Init() override
    {
        bRunning_ = true;
        return true;
    }

    virtual uint32 Run() override
    {
        using namespace msr::airlib;

        // Get the multirotor API
        auto* PawnApi = SimMode_ ? SimMode_->getVehicleSimApi() : nullptr;
        if (!PawnApi) {
            UE_LOG(LogTemp, Error, TEXT("FDroneControlWorker: No vehicle sim API!"));
            return 1;
        }

        auto* MultiPawnApi = static_cast<MultirotorPawnSimApi*>(PawnApi);
        MultirotorApiBase* DroneApi = MultiPawnApi->getVehicleApi();
        if (!DroneApi) {
            UE_LOG(LogTemp, Error, TEXT("FDroneControlWorker: No multirotor API!"));
            return 1;
        }

        // Enable API control and arm
        DroneApi->enableApiControl(true);
        DroneApi->armDisarm(true);

        // Takeoff
        UE_LOG(LogTemp, Log, TEXT("FDroneControlWorker: Taking off..."));
        DroneApi->takeoff(15.0f);
        UE_LOG(LogTemp, Log, TEXT("FDroneControlWorker: Takeoff complete, FPS control active."));

        *bDroneReady_ = true;
        bool bWasHovering = false;

        // Main control loop (~20Hz, each moveByVelocity blocks for ~duration)
        while (bRunning_)
        {
            FVector Vel;
            float YawCmd;
            bool Hover;

            // Read shared state
            {
                FScopeLock ScopeLock(Lock_);
                Vel = *VelocityNED_;
                YawCmd = *Yaw_;
                Hover = *bShouldHover_;
            }

            if (Hover)
            {
                if (!bWasHovering)
                {
                    DroneApi->hover();
                    bWasHovering = true;
                }
                // Idle when hovering — sleep a bit to avoid busy-wait
                FPlatformProcess::Sleep(0.05f);
            }
            else
            {
                bWasHovering = false;
                YawMode yaw_mode(false, YawCmd);  // is_rate=false, absolute yaw angle
                DroneApi->moveByVelocity(
                    Vel.X, Vel.Y, Vel.Z,
                    0.1f,  // duration: short so we can update often
                    DrivetrainType::MaxDegreeOfFreedom,
                    yaw_mode);
                // moveByVelocity blocks for ~100ms (the duration), then we loop
            }
        }

        // Land on shutdown
        UE_LOG(LogTemp, Log, TEXT("FDroneControlWorker: Landing..."));
        DroneApi->hover();
        DroneApi->land(10.0f);
        DroneApi->armDisarm(false);
        DroneApi->enableApiControl(false);

        return 0;
    }

    virtual void Stop() override
    {
        bRunning_ = false;
    }

    virtual void Exit() override
    {
    }

private:
    ASimModeBase* SimMode_ = nullptr;
    FCriticalSection* Lock_ = nullptr;
    FVector* VelocityNED_ = nullptr;
    float* Yaw_ = nullptr;
    bool* bShouldHover_ = nullptr;
    bool* bDroneReady_ = nullptr;
    volatile bool bRunning_ = false;
};

// ========================== Constructor ==========================

ASimWorldGameMode::ASimWorldGameMode(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)  // CARLA creates Episode, Recorder, TaggerDelegate, etc.
{
    // AirSim manages its own vehicles; CARLA spawns via ActorFactories.
    // Setting DefaultPawnClass = nullptr prevents UE4 from auto-spawning a pawn
    // that would break AirSim drone control.
    // We manually create a spectator pawn in BeginPlay() for CARLA compatibility.
    DefaultPawnClass = nullptr;

    // Keep HUDClass = ACarlaHUD from parent (AirSim UI uses UMG widget instead)
    // HUDClass is already set to ACarlaHUD::StaticClass() by ACarlaGameModeBase constructor.

    // --- CARLA Blueprint properties (normally set in CarlaGameMode blueprint) ---

    // Weather blueprint class
    static ConstructorHelpers::FClassFinder<AWeather> weather_class(
        TEXT("/Game/Carla/Blueprints/Weather/BP_Weather"));
    if (weather_class.Succeeded()) {
        WeatherClass = weather_class.Class;
    }

    // Actor Factories — C++ factories
    ActorFactories.Add(ASensorFactory::StaticClass());
    ActorFactories.Add(AStaticMeshFactory::StaticClass());
    ActorFactories.Add(ATriggerFactory::StaticClass());
    ActorFactories.Add(AUtilActorFactory::StaticClass());
    ActorFactories.Add(AAIControllerFactory::StaticClass());

    // Actor Factories — Blueprint factories (vehicles, walkers, props)
    static ConstructorHelpers::FClassFinder<ACarlaActorFactoryBlueprint> vehicle_factory(
        TEXT("/Game/Carla/Blueprints/Vehicles/VehicleFactory"));
    if (vehicle_factory.Succeeded())
        ActorFactories.Add(vehicle_factory.Class);

    static ConstructorHelpers::FClassFinder<ACarlaActorFactoryBlueprint> walker_factory(
        TEXT("/Game/Carla/Blueprints/Walkers/WalkerFactory"));
    if (walker_factory.Succeeded())
        ActorFactories.Add(walker_factory.Class);

    static ConstructorHelpers::FClassFinder<ACarlaActorFactoryBlueprint> prop_factory(
        TEXT("/Game/Carla/Blueprints/Props/PropFactory"));
    if (prop_factory.Succeeded())
        ActorFactories.Add(prop_factory.Class);

    // --- AirSim setup ---

    // Initialize AirSim logger
    common_utils::Utils::getSetLogger(&GlobalSimWorldLog);

    // Pre-load ImageWrapper module (must be done on main thread)
    static IImageWrapperModule& ImageWrapperModule =
        FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));

    // Load AirSim HUD widget blueprint
    static ConstructorHelpers::FClassFinder<UUserWidget> hud_widget_class(
        TEXT("WidgetBlueprint'/AirSim/Blueprints/BP_SimHUDWidget'"));
    WidgetClass_ = hud_widget_class.Succeeded() ? hud_widget_class.Class : nullptr;
}

// ========================== BeginPlay ==========================

void ASimWorldGameMode::BeginPlay()
{
    // Full CARLA initialization: Episode, Weather, Traffic, Recorder, etc.
    Super::BeginPlay();

    // Manually create a spectator pawn for CARLA API compatibility.
    // DefaultPawnClass is nullptr (required for AirSim drone control), so
    // Episode->InitializeAtBeginPlay() could not find a spectator.
    // We create one now and register it with CARLA's Episode.
    // IMPORTANT: Do NOT possess the pawn — possessing it breaks AirSim's input handling.
    {
        auto* World = GetWorld();
        if (World)
        {
            FActorSpawnParameters SpawnParams;
            SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            auto* SpectatorPawn = World->SpawnActor<ASpectatorPawn>(
                ASpectatorPawn::StaticClass(), FTransform::Identity, SpawnParams);
            if (SpectatorPawn)
            {
                CachedSpectator_ = SpectatorPawn;

                // Register with CARLA Episode (get via GameInstance for non-const access)
                auto* GI = Cast<UCarlaGameInstance>(GetGameInstance());
                auto* Episode = GI ? GI->GetCarlaEpisode() : nullptr;
                if (Episode)
                {
                    Episode->Spectator = SpectatorPawn;
                    FActorDescription Description;
                    Description.Id = TEXT("spectator");
                    Description.Class = SpectatorPawn->GetClass();
                    Episode->ActorDispatcher->RegisterActor(*SpectatorPawn, Description);
                    UE_LOG(LogTemp, Log, TEXT("SimWorldGameMode: Spectator pawn created and registered."));
                }
            }
        }
    }

    UE_LOG(LogTemp, Log, TEXT("SimWorldGameMode: CARLA BeginPlay complete, starting AirSim bootstrap..."));

    try {
        UAirBlueprintLib::OnBeginPlay();

        // NO loadLevel() — map already loaded by CARLA/UE4. AirSim's level_name setting is ignored.

        InitializeAirSimSettings();
        SetUnrealEngineSettings();
        CreateSimMode();
        CreateAirSimWidget();
        SetupAirSimInputBindings();

        if (SimMode_)
            SimMode_->startApiServer();

        UE_LOG(LogTemp, Log, TEXT("SimWorldGameMode: AirSim bootstrap complete. API server started."));

        // Setup FPS drone control (must be after SimMode creation)
        SetupFPSControl();
    }
    catch (std::exception& ex) {
        UAirBlueprintLib::LogMessageString("Error at AirSim startup: ", ex.what(), LogDebugLevel::Failure);
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok,
            std::string("Error at AirSim startup: ") + ex.what(), "Error");
    }
}

// ========================== Tick ==========================

void ASimWorldGameMode::Tick(float DeltaSeconds)
{
    // CARLA: Recorder tick
    Super::Tick(DeltaSeconds);

    // AirSim: Update debug report widget
    if (SimMode_ && SimMode_->EnableReport && Widget_)
        Widget_->updateDebugReport(SimMode_->getDebugReport());

    // FPS drone control: read input, update shared state
    if (bFPSControlActive_)
    {
        UpdateFPSControl(DeltaSeconds);
        UpdateCameraFollow();
    }
}

// ========================== EndPlay ==========================

void ASimWorldGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // Stop drone control thread
    if (DroneWorker_)
    {
        DroneWorker_->Stop();
    }
    if (DroneThread_)
    {
        DroneThread_->WaitForCompletion();
        delete DroneThread_;
        DroneThread_ = nullptr;
    }
    if (DroneWorker_)
    {
        delete DroneWorker_;
        DroneWorker_ = nullptr;
    }
    bFPSControlActive_ = false;
    bDroneReady_ = false;

    // Stop AirSim API server
    if (SimMode_)
        SimMode_->stopApiServer();

    // Destroy AirSim widget
    if (Widget_) {
        Widget_->Destruct();
        Widget_ = nullptr;
    }

    // Destroy SimMode actor
    if (SimMode_) {
        SimMode_->Destroy();
        SimMode_ = nullptr;
    }

    UAirBlueprintLib::OnEndPlay();

    // CARLA cleanup: Episode->EndPlay(), GameInstance->NotifyEndEpisode(), etc.
    Super::EndPlay(EndPlayReason);
}

// ========================== FPS Drone Control ==========================

void ASimWorldGameMode::SetupFPSControl()
{
    if (!SimMode_) {
        UE_LOG(LogTemp, Warning, TEXT("SetupFPSControl: No SimMode, skipping FPS control."));
        return;
    }

    // Only enable for multirotor mode
    std::string simmode_name = AirSimSettings::singleton().simmode_name;
    if (simmode_name != AirSimSettings::kSimModeTypeMultirotor) {
        UE_LOG(LogTemp, Log, TEXT("SetupFPSControl: SimMode is not Multirotor, skipping FPS control."));
        return;
    }

    APlayerController* PC = GetWorld()->GetFirstPlayerController();
    if (!PC) {
        UE_LOG(LogTemp, Warning, TEXT("SetupFPSControl: No PlayerController!"));
        return;
    }

    // Set mouse capture for FPS control
    PC->bShowMouseCursor = false;
    FInputModeGameOnly InputMode;
    PC->SetInputMode(InputMode);

    // Set view target to spectator
    if (CachedSpectator_)
        PC->SetViewTarget(CachedSpectator_);

    // Build weather presets
    {
        // ClearNoon
        FWeatherParameters w;
        w.Cloudiness = 10.0f; w.SunAltitudeAngle = 75.0f; w.SunAzimuthAngle = 0.0f;
        WeatherPresets_.Add(w);
    }
    {
        // Cloudy
        FWeatherParameters w;
        w.Cloudiness = 80.0f; w.SunAltitudeAngle = 50.0f;
        WeatherPresets_.Add(w);
    }
    {
        // Rain
        FWeatherParameters w;
        w.Cloudiness = 90.0f; w.Precipitation = 80.0f; w.PrecipitationDeposits = 60.0f;
        w.Wetness = 80.0f; w.SunAltitudeAngle = 40.0f;
        WeatherPresets_.Add(w);
    }
    {
        // Sunset
        FWeatherParameters w;
        w.Cloudiness = 30.0f; w.SunAltitudeAngle = 5.0f; w.SunAzimuthAngle = 270.0f;
        WeatherPresets_.Add(w);
    }
    {
        // Night
        FWeatherParameters w;
        w.Cloudiness = 20.0f; w.SunAltitudeAngle = -80.0f;
        WeatherPresets_.Add(w);
    }
    {
        // DustStorm
        FWeatherParameters w;
        w.Cloudiness = 60.0f; w.DustStorm = 80.0f; w.FogDensity = 30.0f;
        w.WindIntensity = 80.0f; w.SunAltitudeAngle = 40.0f;
        WeatherPresets_.Add(w);
    }

    // Build map list (filter to Town* only)
    TArray<FString> AllMaps = UCarlaStatics::GetAllMapNames();
    for (const FString& Map : AllMaps)
    {
        if (Map.StartsWith(TEXT("Town")))
            AvailableMaps_.Add(Map);
    }
    // Find current map index
    FString CurrentMap = GetWorld()->GetMapName();
    CurrentMap = CurrentMap.Replace(TEXT("UEDPIE_0_"), TEXT(""));  // Strip editor prefix
    for (int32 i = 0; i < AvailableMaps_.Num(); ++i)
    {
        if (CurrentMap.Contains(AvailableMaps_[i]))
        {
            MapIndex_ = i;
            break;
        }
    }

    // Bind FPS control keys
    UAirBlueprintLib::BindActionToKey("InputEventToggleView", EKeys::V, this, &ASimWorldGameMode::InputEventToggleView);
    UAirBlueprintLib::BindActionToKey("InputEventNextWeather", EKeys::N, this, &ASimWorldGameMode::InputEventNextWeather);
    UAirBlueprintLib::BindActionToKey("InputEventNextMap", EKeys::M, this, &ASimWorldGameMode::InputEventNextMap);
    UAirBlueprintLib::BindActionToKey("InputEventToggleMouseCapture", EKeys::Tab, this, &ASimWorldGameMode::InputEventToggleMouseCapture);
    UAirBlueprintLib::BindActionToKey("InputEventSpeedUp", EKeys::Equals, this, &ASimWorldGameMode::InputEventSpeedUp);
    UAirBlueprintLib::BindActionToKey("InputEventSpeedDown", EKeys::Hyphen, this, &ASimWorldGameMode::InputEventSpeedDown);

    // Start drone control thread
    DroneWorker_ = new FDroneControlWorker(
        SimMode_, &DroneControlLock_, &DesiredVelocityNED_, &DesiredYaw_,
        &bShouldHover_, &bDroneReady_);
    DroneThread_ = FRunnableThread::Create(DroneWorker_, TEXT("DroneControlThread"));

    bFPSControlActive_ = true;
    UE_LOG(LogTemp, Log, TEXT("SetupFPSControl: FPS drone control active. WASD=Move, Mouse=Look, V=View, N=Weather, M=Map"));
}

void ASimWorldGameMode::UpdateFPSControl(float DeltaSeconds)
{
    APlayerController* PC = GetWorld()->GetFirstPlayerController();
    if (!PC || !bDroneReady_)
        return;

    // ---- Mouse look ----
    if (bMouseCaptured_)
    {
        float MouseX = 0.0f, MouseY = 0.0f;
        PC->GetInputMouseDelta(MouseX, MouseY);

        FPSYaw_ += MouseX * 2.0f;        // sensitivity multiplier
        FPSPitch_ -= MouseY * 2.0f;
        FPSPitch_ = FMath::Clamp(FPSPitch_, -89.0f, 89.0f);

        // Normalize yaw to [0, 360)
        while (FPSYaw_ >= 360.0f) FPSYaw_ -= 360.0f;
        while (FPSYaw_ < 0.0f)   FPSYaw_ += 360.0f;
    }

    // ---- Keyboard movement -> NED velocity ----
    float Fwd = 0.0f, Strafe = 0.0f, Vert = 0.0f;
    if (PC->IsInputKeyDown(EKeys::W)) Fwd += 1.0f;
    if (PC->IsInputKeyDown(EKeys::S)) Fwd -= 1.0f;
    if (PC->IsInputKeyDown(EKeys::D)) Strafe += 1.0f;
    if (PC->IsInputKeyDown(EKeys::A)) Strafe -= 1.0f;
    if (PC->IsInputKeyDown(EKeys::Q) || PC->IsInputKeyDown(EKeys::SpaceBar)) Vert += 1.0f;
    if (PC->IsInputKeyDown(EKeys::E) || PC->IsInputKeyDown(EKeys::LeftShift)) Vert -= 1.0f;

    bool bHasInput = (FMath::Abs(Fwd) > 0.01f || FMath::Abs(Strafe) > 0.01f || FMath::Abs(Vert) > 0.01f);

    // Decompose by yaw into world-frame NED
    // AirSim NED: X=North, Y=East, Z=Down
    float YawRad = FMath::DegreesToRadians(FPSYaw_);
    float VxNED = (Fwd * FMath::Cos(YawRad) - Strafe * FMath::Sin(YawRad)) * DroneSpeed_;
    float VyNED = (Fwd * FMath::Sin(YawRad) + Strafe * FMath::Cos(YawRad)) * DroneSpeed_;
    float VzNED = -Vert * DroneSpeed_;  // NED: negative Z = up

    // Write to shared state
    {
        FScopeLock ScopeLock(&DroneControlLock_);
        DesiredVelocityNED_ = FVector(VxNED, VyNED, VzNED);
        DesiredYaw_ = FPSYaw_;
        bShouldHover_ = !bHasInput;
    }
}

void ASimWorldGameMode::UpdateCameraFollow()
{
    if (!SimMode_ || !CachedSpectator_ || !bDroneReady_)
        return;

    auto* PawnApi = SimMode_->getVehicleSimApi();
    if (!PawnApi)
        return;

    FVector DronePos = PawnApi->getUUPosition();
    FRotator DroneRot = PawnApi->getUUOrientation();

    FRotator CamRot(FPSPitch_, FPSYaw_, 0.0f);

    if (bFirstPersonView_)
    {
        // 1st person: camera at drone position
        CachedSpectator_->SetActorLocation(DronePos);
        CachedSpectator_->SetActorRotation(CamRot);
    }
    else
    {
        // 3rd person: 5m behind + 2m above, looking at drone
        FVector Offset = CamRot.Vector() * -500.0f;  // 5m behind (UE4 cm)
        Offset.Z += 200.0f;  // 2m above
        FVector CamPos = DronePos + Offset;
        FRotator LookAt = (DronePos - CamPos).Rotation();

        CachedSpectator_->SetActorLocation(CamPos);
        CachedSpectator_->SetActorRotation(LookAt);
    }
}

// ========================== FPS Input Handlers ==========================

void ASimWorldGameMode::InputEventToggleView()
{
    bFirstPersonView_ = !bFirstPersonView_;
    UE_LOG(LogTemp, Log, TEXT("FPS View: %s"),
           bFirstPersonView_ ? TEXT("1st Person") : TEXT("3rd Person"));
}

void ASimWorldGameMode::InputEventNextWeather()
{
    if (WeatherPresets_.Num() == 0) return;

    WeatherIndex_ = (WeatherIndex_ + 1) % WeatherPresets_.Num();
    const FWeatherParameters& Preset = WeatherPresets_[WeatherIndex_];

    auto* Episode = UCarlaStatics::GetCurrentEpisode(GetWorld());
    if (Episode)
    {
        AWeather* Weather = Episode->GetWeather();
        if (Weather)
        {
            Weather->ApplyWeather(Preset);
        }
    }

    static const TCHAR* PresetNames[] = {
        TEXT("ClearNoon"), TEXT("Cloudy"), TEXT("Rain"),
        TEXT("Sunset"), TEXT("Night"), TEXT("DustStorm")
    };
    UE_LOG(LogTemp, Log, TEXT("Weather: %s"), PresetNames[WeatherIndex_]);
}

void ASimWorldGameMode::InputEventNextMap()
{
    if (AvailableMaps_.Num() == 0) return;

    MapIndex_ = (MapIndex_ + 1) % AvailableMaps_.Num();
    FString NextMap = AvailableMaps_[MapIndex_];

    UE_LOG(LogTemp, Log, TEXT("Loading map: %s"), *NextMap);

    auto* Episode = UCarlaStatics::GetCurrentEpisode(GetWorld());
    if (Episode)
    {
        Episode->LoadNewEpisode(NextMap, true);
    }
}

void ASimWorldGameMode::InputEventToggleMouseCapture()
{
    APlayerController* PC = GetWorld()->GetFirstPlayerController();
    if (!PC) return;

    bMouseCaptured_ = !bMouseCaptured_;
    if (bMouseCaptured_)
    {
        PC->bShowMouseCursor = false;
        FInputModeGameOnly InputMode;
        PC->SetInputMode(InputMode);
    }
    else
    {
        PC->bShowMouseCursor = true;
        FInputModeGameAndUI InputMode;
        InputMode.SetHideCursorDuringCapture(false);
        PC->SetInputMode(InputMode);
    }
    UE_LOG(LogTemp, Log, TEXT("Mouse capture: %s"),
           bMouseCaptured_ ? TEXT("ON") : TEXT("OFF"));
}

void ASimWorldGameMode::InputEventSpeedUp()
{
    DroneSpeed_ = FMath::Min(30.0f, DroneSpeed_ + 2.0f);
    UE_LOG(LogTemp, Log, TEXT("Drone speed: %.0f m/s"), DroneSpeed_);
}

void ASimWorldGameMode::InputEventSpeedDown()
{
    DroneSpeed_ = FMath::Max(2.0f, DroneSpeed_ - 2.0f);
    UE_LOG(LogTemp, Log, TEXT("Drone speed: %.0f m/s"), DroneSpeed_);
}

// ========================== AirSim Settings ==========================

void ASimWorldGameMode::InitializeAirSimSettings()
{
    std::string settingsText;
    if (GetSettingsText(settingsText))
        AirSimSettings::initializeSettings(settingsText);
    else
        AirSimSettings::createDefaultSettingsFile();

    AirSimSettings::singleton().load(std::bind(&ASimWorldGameMode::GetSimModeFromUser, this));

    for (const auto& warning : AirSimSettings::singleton().warning_messages) {
        UAirBlueprintLib::LogMessageString(warning, "", LogDebugLevel::Failure);
    }
    for (const auto& error : AirSimSettings::singleton().error_messages) {
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok, error, "settings.json");
    }
}

void ASimWorldGameMode::SetUnrealEngineSettings()
{
    // Disable motion blur for clean capture images
    GetWorld()->GetGameViewport()->GetEngineShowFlags()->SetMotionBlur(false);

    // Enable custom stencil (required for AirSim segmentation)
    static const auto custom_depth_var = IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth"));
    custom_depth_var->Set(3);
    UKismetSystemLibrary::ExecuteConsoleCommand(GetWorld(), FString("r.CustomDepth 3"));

    // Increase render fence timeout for large environments with stencil IDs
    static const auto render_timeout_var = IConsoleManager::Get().FindConsoleVariable(TEXT("g.TimeoutForBlockOnRenderFence"));
    render_timeout_var->Set(300000);
}

// ========================== SimMode Creation ==========================

void ASimWorldGameMode::CreateSimMode()
{
    std::string simmode_name = AirSimSettings::singleton().simmode_name;

    FActorSpawnParameters simmode_spawn_params;
    simmode_spawn_params.SpawnCollisionHandlingOverride =
        ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    // Spawn at origin — used for global NED transforms
    if (simmode_name == AirSimSettings::kSimModeTypeMultirotor)
        SimMode_ = GetWorld()->SpawnActor<ASimModeWorldMultiRotor>(
            FVector::ZeroVector, FRotator::ZeroRotator, simmode_spawn_params);
    else if (simmode_name == AirSimSettings::kSimModeTypeCar)
        SimMode_ = GetWorld()->SpawnActor<ASimModeCar>(
            FVector::ZeroVector, FRotator::ZeroRotator, simmode_spawn_params);
    else if (simmode_name == AirSimSettings::kSimModeTypeComputerVision)
        SimMode_ = GetWorld()->SpawnActor<ASimModeComputerVision>(
            FVector::ZeroVector, FRotator::ZeroRotator, simmode_spawn_params);
    else {
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok,
            std::string("SimMode is not valid: ") + simmode_name, "Error");
        UAirBlueprintLib::LogMessageString("SimMode is not valid: ", simmode_name, LogDebugLevel::Failure);
    }
}

// ========================== Widget / HUD ==========================

void ASimWorldGameMode::CreateAirSimWidget()
{
    if (WidgetClass_ != nullptr) {
        APlayerController* player_controller = GetWorld()->GetFirstPlayerController();
        auto* pawn = player_controller->GetPawn();
        if (pawn) {
            std::string pawn_name = std::string(TCHAR_TO_ANSI(*pawn->GetName()));
            Utils::log(pawn_name);
        }
        else {
            UAirBlueprintLib::ShowMessage(EAppMsgType::Ok,
                std::string("There were no compatible vehicles created for current SimMode! Check your settings.json."),
                "Error");
            UAirBlueprintLib::LogMessage(
                TEXT("There were no compatible vehicles created for current SimMode! Check your settings.json."),
                TEXT(""), LogDebugLevel::Failure);
        }

        Widget_ = CreateWidget<USimHUDWidget>(player_controller, WidgetClass_);
    }
    else {
        Widget_ = nullptr;
        UAirBlueprintLib::LogMessage(
            TEXT("Cannot instantiate BP_SimHUDWidget blueprint!"), TEXT(""), LogDebugLevel::Failure);
    }

    InitializeSubWindows();

    if (Widget_) {
        Widget_->AddToViewport();
        Widget_->initializeForPlay();
        if (SimMode_)
            Widget_->setReportVisible(SimMode_->EnableReport);
        Widget_->setOnToggleRecordingHandler(std::bind(&ASimWorldGameMode::ToggleRecordHandler, this));
        Widget_->setRecordButtonVisibility(AirSimSettings::singleton().is_record_ui_visible);
        UpdateWidgetSubwindowVisibility();
    }
}

void ASimWorldGameMode::InitializeSubWindows()
{
    if (!SimMode_)
        return;

    auto default_vehicle_sim_api = SimMode_->getVehicleSimApi();

    if (default_vehicle_sim_api) {
        auto camera_count = default_vehicle_sim_api->getCameraCount();

        if (camera_count > 0) {
            SubwindowCameras_[0] = default_vehicle_sim_api->getCamera("");
            SubwindowCameras_[1] = default_vehicle_sim_api->getCamera("");
            SubwindowCameras_[2] = default_vehicle_sim_api->getCamera("");
        }
        else
            SubwindowCameras_[0] = SubwindowCameras_[1] = SubwindowCameras_[2] = nullptr;
    }

    for (const auto& setting : GetSubWindowSettings()) {
        APIPCamera* camera = SimMode_->getCamera(
            msr::airlib::CameraDetails(setting.camera_name, setting.vehicle_name, setting.external));
        if (camera)
            SubwindowCameras_[setting.window_index] = camera;
        else
            UAirBlueprintLib::LogMessageString("Invalid Camera settings in <SubWindows> element",
                                               std::to_string(setting.window_index),
                                               LogDebugLevel::Failure);
    }
}

// ========================== Input Bindings ==========================

void ASimWorldGameMode::SetupAirSimInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("inputEventToggleRecording", EKeys::R, this, &ASimWorldGameMode::InputEventToggleRecording);
    UAirBlueprintLib::BindActionToKey("InputEventToggleReport", EKeys::Semicolon, this, &ASimWorldGameMode::InputEventToggleReport);
    UAirBlueprintLib::BindActionToKey("InputEventToggleHelp", EKeys::F1, this, &ASimWorldGameMode::InputEventToggleHelp);
    UAirBlueprintLib::BindActionToKey("InputEventToggleTrace", EKeys::T, this, &ASimWorldGameMode::InputEventToggleTrace);

    UAirBlueprintLib::BindActionToKey("InputEventToggleSubwindow0", EKeys::One, this, &ASimWorldGameMode::InputEventToggleSubwindow0);
    UAirBlueprintLib::BindActionToKey("InputEventToggleSubwindow1", EKeys::Two, this, &ASimWorldGameMode::InputEventToggleSubwindow1);
    UAirBlueprintLib::BindActionToKey("InputEventToggleSubwindow2", EKeys::Three, this, &ASimWorldGameMode::InputEventToggleSubwindow2);
    UAirBlueprintLib::BindActionToKey("InputEventToggleAll", EKeys::Zero, this, &ASimWorldGameMode::InputEventToggleAll);
}

// ========================== Input Handlers ==========================

void ASimWorldGameMode::ToggleRecordHandler()
{
    if (SimMode_)
        SimMode_->toggleRecording();
}

void ASimWorldGameMode::InputEventToggleRecording()
{
    ToggleRecordHandler();
}

void ASimWorldGameMode::InputEventToggleReport()
{
    if (SimMode_ && Widget_) {
        SimMode_->EnableReport = !SimMode_->EnableReport;
        Widget_->setReportVisible(SimMode_->EnableReport);
    }
}

void ASimWorldGameMode::InputEventToggleHelp()
{
    if (Widget_)
        Widget_->toggleHelpVisibility();
}

void ASimWorldGameMode::InputEventToggleTrace()
{
    if (SimMode_)
        SimMode_->toggleTraceAll();
}

void ASimWorldGameMode::UpdateWidgetSubwindowVisibility()
{
    if (!Widget_)
        return;

    for (int window_index = 0; window_index < AirSimSettings::kSubwindowCount; ++window_index) {
        APIPCamera* camera = SubwindowCameras_[window_index];
        ImageType camera_type = GetSubWindowSettings().at(window_index).image_type;

        bool is_visible = GetSubWindowSettings().at(window_index).visible && camera != nullptr;

        if (camera != nullptr) {
            camera->setCameraTypeEnabled(camera_type, is_visible);
            camera->setCameraTypeUpdate(camera_type, false);
        }

        Widget_->setSubwindowVisibility(window_index,
                                        is_visible,
                                        is_visible ? camera->getRenderTarget(camera_type, false) : nullptr);
    }
}

bool ASimWorldGameMode::IsWidgetSubwindowVisible(int window_index)
{
    return Widget_ ? Widget_->getSubwindowVisibility(window_index) != 0 : false;
}

void ASimWorldGameMode::ToggleSubwindowVisibility(int window_index)
{
    GetSubWindowSettings().at(window_index).visible = !GetSubWindowSettings().at(window_index).visible;
    UpdateWidgetSubwindowVisibility();
}

void ASimWorldGameMode::InputEventToggleSubwindow0()
{
    ToggleSubwindowVisibility(0);
}

void ASimWorldGameMode::InputEventToggleSubwindow1()
{
    ToggleSubwindowVisibility(1);
}

void ASimWorldGameMode::InputEventToggleSubwindow2()
{
    ToggleSubwindowVisibility(2);
}

void ASimWorldGameMode::InputEventToggleAll()
{
    GetSubWindowSettings().at(0).visible = !GetSubWindowSettings().at(0).visible;
    GetSubWindowSettings().at(1).visible = GetSubWindowSettings().at(2).visible = GetSubWindowSettings().at(0).visible;
    UpdateWidgetSubwindowVisibility();
}

// ========================== Settings Helpers ==========================

const std::vector<ASimWorldGameMode::AirSimSettings::SubwindowSetting>&
ASimWorldGameMode::GetSubWindowSettings() const
{
    return AirSimSettings::singleton().subwindow_settings;
}

std::vector<ASimWorldGameMode::AirSimSettings::SubwindowSetting>&
ASimWorldGameMode::GetSubWindowSettings()
{
    return AirSimSettings::singleton().subwindow_settings;
}

std::string ASimWorldGameMode::GetSimModeFromUser()
{
    if (EAppReturnType::No == UAirBlueprintLib::ShowMessage(EAppMsgType::YesNo,
            "Would you like to use car simulation? Choose no to use quadrotor simulation.",
            "Choose Vehicle")) {
        return AirSimSettings::kSimModeTypeMultirotor;
    }
    else
        return AirSimSettings::kSimModeTypeCar;
}

FString ASimWorldGameMode::GetLaunchPath(const std::string& filename)
{
    FString launch_rel_path = FPaths::LaunchDir();
    FString abs_path = FPaths::ConvertRelativePathToFull(launch_rel_path);
    return FPaths::Combine(abs_path, FString(filename.c_str()));
}

bool ASimWorldGameMode::GetSettingsText(std::string& settingsText)
{
    return (GetSettingsTextFromCommandLine(settingsText) ||
            ReadSettingsTextFromFile(FString(msr::airlib::Settings::getExecutableFullPath("settings.json").c_str()), settingsText) ||
            ReadSettingsTextFromFile(GetLaunchPath("settings.json"), settingsText) ||
            ReadSettingsTextFromFile(FString(msr::airlib::Settings::Settings::getUserDirectoryFullPath("settings.json").c_str()), settingsText));
}

bool ASimWorldGameMode::GetSettingsTextFromCommandLine(std::string& settingsText)
{
    bool found = false;
    FString settingsTextFString;
    const TCHAR* commandLineArgs = FCommandLine::Get();

    if (FParse::Param(commandLineArgs, TEXT("-settings"))) {
        FString commandLineArgsFString = FString(commandLineArgs);
        int idx = commandLineArgsFString.Find(TEXT("-settings"));
        FString settingsJsonFString = commandLineArgsFString.RightChop(idx + 10);

        if (ReadSettingsTextFromFile(settingsJsonFString.TrimQuotes(), settingsText)) {
            return true;
        }

        if (FParse::QuotedString(*settingsJsonFString, settingsTextFString)) {
            settingsText = std::string(TCHAR_TO_UTF8(*settingsTextFString));
            found = true;
        }
    }

    return found;
}

bool ASimWorldGameMode::ReadSettingsTextFromFile(const FString& settingsFilepath, std::string& settingsText)
{
    bool found = FPaths::FileExists(settingsFilepath);
    if (found) {
        FString settingsTextFStr;
        bool readSuccessful = FFileHelper::LoadFileToString(settingsTextFStr, *settingsFilepath);
        if (readSuccessful) {
            UAirBlueprintLib::LogMessageString("Loaded settings from ",
                TCHAR_TO_UTF8(*settingsFilepath), LogDebugLevel::Informational);
            settingsText = TCHAR_TO_UTF8(*settingsTextFStr);
        }
        else {
            UAirBlueprintLib::LogMessageString("Cannot read file ",
                TCHAR_TO_UTF8(*settingsFilepath), LogDebugLevel::Failure);
            throw std::runtime_error("Cannot read settings file.");
        }
    }
    return found;
}
