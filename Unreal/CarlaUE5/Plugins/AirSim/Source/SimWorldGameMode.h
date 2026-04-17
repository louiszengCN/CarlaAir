// SimWorldGameMode.h — Unified CARLA + AirSim GameMode
// Inherits from ACarlaGameModeBase (all CARLA features) and bootstraps
// AirSim's SimMode as a regular actor, avoiding the GameMode slot conflict.

#pragma once

#include "CoreMinimal.h"
#include "Carla/Game/CarlaGameModeBase.h"
#include "Carla/Weather/WeatherParameters.h"
#include "SimHUD/SimHUDWidget.h"
#include "SimMode/SimModeBase.h"
#include "PIPCamera.h"
#include "common/AirSimSettings.hpp"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "Widgets/SCompoundWidget.h"
#include "SimWorldGameMode.generated.h"

// Forward declarations
class FDroneControlWorker;

UCLASS()
class AIRSIM_API ASimWorldGameMode : public ACarlaGameModeBase
{
    GENERATED_BODY()

public:
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
    typedef msr::airlib::AirSimSettings AirSimSettings;

    ASimWorldGameMode(const FObjectInitializer& ObjectInitializer);

    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    // Input event handlers (ported from SimHUD)
    void InputEventToggleRecording();
    void InputEventToggleReport();
    void InputEventToggleHelp();
    void InputEventToggleTrace();
    void InputEventToggleSubwindow0();
    void InputEventToggleSubwindow1();
    void InputEventToggleSubwindow2();
    void InputEventToggleAll();

    // FPS drone control input handlers
    void InputEventNextWeather();
    void InputEventToggleMouseCapture();
    void InputEventSpeedUp();
    void InputEventSpeedDown();
    void InputEventToggleHelpOverlay();
    void InputEventTogglePhysicsMode();

protected:
    void SetupAirSimInputBindings();
    void ToggleRecordHandler();
    void UpdateWidgetSubwindowVisibility();
    bool IsWidgetSubwindowVisible(int window_index);
    void ToggleSubwindowVisibility(int window_index);

    // FPS drone control
    void SetupFPSControl();
    void UpdateFPSControl(float DeltaSeconds);
    void UpdateCameraFollow();
    void DrawHelpOverlay();
    void CreateHelpOverlayWidget();
    void ShowHelpOverlay(bool bShow);
    void UpdateHelpOverlayText();

private:
    void InitializeAirSimSettings();
    void SetUnrealEngineSettings();
    void CreateSimMode();
    void CreateAirSimWidget();
    void InitializeSubWindows();

    bool GetSettingsText(std::string& settingsText);
    bool GetSettingsTextFromCommandLine(std::string& settingsText);
    bool ReadSettingsTextFromFile(const FString& fileName, std::string& settingsText);
    std::string GetSimModeFromUser();

    static FString GetLaunchPath(const std::string& filename);

    const std::vector<AirSimSettings::SubwindowSetting>& GetSubWindowSettings() const;
    std::vector<AirSimSettings::SubwindowSetting>& GetSubWindowSettings();

private:
    typedef common_utils::Utils Utils;
    UClass* WidgetClass_;

    UPROPERTY()
    USimHUDWidget* Widget_;

    UPROPERTY()
    ASimModeBase* SimMode_;

    APIPCamera* SubwindowCameras_[AirSimSettings::kSubwindowCount];

    // ---- FPS Drone Control State ----

    // Drone speed & orientation
    float DroneSpeed_ = 8.0f;           // m/s, adjustable with +/-
    float FPSYaw_ = 0.0f;              // accumulated yaw (degrees)
    float FPSPitch_ = 0.0f;            // camera pitch (degrees), clamped ±89
    float CameraFollowYaw_ = 0.0f;     // smoothed camera yaw for 3rd person
    bool bMouseCaptured_ = true;        // mouse capture state
    bool bMouseInitialized_ = false;    // first mouse position read done
    bool bFPSControlActive_ = false;    // FPS control initialized?
    bool bYawInitialized_ = false;      // FPSYaw_ synced from drone actual yaw?

    // Cached spectator
    UPROPERTY()
    APawn* CachedSpectator_ = nullptr;

    // Background thread for drone commands
    FDroneControlWorker* DroneWorker_ = nullptr;
    FRunnableThread* DroneThread_ = nullptr;

    // Shared state between game thread and drone control thread
    FCriticalSection DroneControlLock_;
    FVector DesiredVelocityNED_ = FVector::ZeroVector;  // NED velocity
    float DesiredYaw_ = 0.0f;
    bool bShouldHover_ = true;     // true when no movement input
    bool bDroneReady_ = false;     // set by worker after takeoff

    // Weather presets
    TArray<FWeatherParameters> WeatherPresets_;
    int32 WeatherIndex_ = 0;

    // One-time drone registration with CARLA ActorDispatcher
    bool bDroneRegistered_ = false;

    // Help overlay & physics mode (v0.1.5)
    bool bShowHelp_ = false;
    bool bPhysicsCollision_ = true;  // default: physics mode ON

    // Slate help overlay widget (Apple-style design)
    TSharedPtr<SVerticalBox> HelpOverlayContainer_;
    TSharedPtr<STextBlock> HelpTitleBlock_;
    TSharedPtr<STextBlock> HelpSubtitleBlock_;
    TSharedPtr<STextBlock> HelpContentBlock_;
    TSharedPtr<STextBlock> HelpStatusBlock_;
    TSharedPtr<SWidget> HelpOverlayWrapper_;
};
