// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/PixelReader.h"
#include "Carla/Sensor/Sensor.h"
// UE5: USceneCaptureComponent2D_CARLA removed; use standard component instead
#include "Components/SceneCaptureComponent2D.h"
#include "Carla/Sensor/ImageUtil.h"

#include "Async/Async.h"
// UE5: GBufferView.h removed — GBuffer API no longer available via this header

#include <type_traits>

#include "SceneCaptureSensor.generated.h"



class UDrawFrustumComponent;
class UStaticMeshComponent;
class UTextureRenderTarget2D;



struct FCameraGBufferUint8
{
  /// Prevent this sensor to be spawned by users.
  using not_spawnable = void;

  void SetDataStream(FDataStream InStream)
  {
    Stream = std::move(InStream);
  }

  /// Replace the Stream associated with this sensor.
  void SetStream(FDataMultiStream InStream)
  {
    Stream = std::move(InStream);
  }
  /// Return the token that allows subscribing to this sensor's stream.
  auto GetToken() const
  {
    bIsUsed = true;
    return Stream.GetToken();
  }

  /// Dummy. Required for compatibility with other sensors only.
  FTransform GetActorTransform() const
  {
    return {};
  }
  /// Return the FDataStream associated with this sensor.
  ///
  /// You need to provide a reference to self, this is necessary for template
  /// deduction.
  template <typename SensorT>
  FAsyncDataStream GetDataStream(const SensorT &Self)
  {
    return Stream.MakeAsyncDataStream(Self, Self.GetEpisode().GetElapsedGameTime());
  }

  mutable bool bIsUsed = false;
  FDataStream Stream;
};



struct FCameraGBufferFloat
{
  /// Prevent this sensor to be spawned by users.
  using not_spawnable = void;

  void SetDataStream(FDataStream InStream)
  {
    Stream = std::move(InStream);
  }

  /// Replace the Stream associated with this sensor.
  void SetStream(FDataMultiStream InStream)
  {
    Stream = std::move(InStream);
  }
  /// Return the token that allows subscribing to this sensor's stream.
  auto GetToken() const
  {
    bIsUsed = true;
    return Stream.GetToken();
  }
  /// Dummy. Required for compatibility with other sensors only.
  FTransform GetActorTransform() const
  {
    return {};
  }
  /// Return the FDataStream associated with this sensor.
  ///
  /// You need to provide a reference to self, this is necessary for template
  /// deduction.
  template <typename SensorT>
  FAsyncDataStream GetDataStream(const SensorT &Self)
  {
    return Stream.MakeAsyncDataStream(Self, Self.GetEpisode().GetElapsedGameTime());
  }

  mutable bool bIsUsed = false;
  FDataStream Stream;
};






/// Base class for sensors using a USceneCaptureComponent2D for rendering the
/// scene. This class does not capture data, use
/// `FPixelReader::SendPixelsInRenderThread(*this)` in derived classes.
///
/// To access the USceneCaptureComponent2D override the
/// SetUpSceneCaptureComponent function.
///
/// @warning All the setters should be called before BeginPlay.
UCLASS(Abstract)
class CARLA_API ASceneCaptureSensor : public ASensor
{
  GENERATED_BODY()

  friend class ACarlaGameModeBase;
  friend class FPixelReader;
  friend class FPixelReader2;

public:

  ASceneCaptureSensor(const FObjectInitializer &ObjectInitializer);

  void Set(const FActorDescription &ActorDescription) override;

  void SetImageSize(uint32 Width, uint32 Height);

  uint32 GetImageWidth() const
  {
    return ImageWidth;
  }

  uint32 GetImageHeight() const
  {
    return ImageHeight;
  }

  UFUNCTION(BlueprintCallable)
  void EnablePostProcessingEffects(bool Enable = true)
  {
    bEnablePostProcessingEffects = Enable;
  }

  UFUNCTION(BlueprintCallable)
  bool ArePostProcessingEffectsEnabled() const
  {
    return bEnablePostProcessingEffects;
  }

  UFUNCTION(BlueprintCallable)
  void Enable16BitFormat(bool Enable = false)
  {
    bEnable16BitFormat = Enable;
  }

  UFUNCTION(BlueprintCallable)
  bool Is16BitFormatEnabled() const
  {
    return bEnable16BitFormat;
  }

  UFUNCTION(BlueprintCallable)
  void SetFOVAngle(float FOVAngle);

  UFUNCTION(BlueprintCallable)
  float GetFOVAngle() const;

  UFUNCTION(BlueprintCallable)
  void SetTargetGamma(float InTargetGamma)
  {
    TargetGamma = InTargetGamma;
  }

  UFUNCTION(BlueprintCallable)
  float GetTargetGamma() const
  {
    return TargetGamma;
  }

  UFUNCTION(BlueprintCallable)
  void SetExposureMethod(EAutoExposureMethod Method);

  UFUNCTION(BlueprintCallable)
  EAutoExposureMethod GetExposureMethod() const;

  UFUNCTION(BlueprintCallable)
  void SetExposureCompensation(float Compensation);

  UFUNCTION(BlueprintCallable)
  float GetExposureCompensation() const;

  UFUNCTION(BlueprintCallable)
  void SetShutterSpeed(float Speed);

  UFUNCTION(BlueprintCallable)
  float GetShutterSpeed() const;

  UFUNCTION(BlueprintCallable)
  void SetISO(float ISO);

  UFUNCTION(BlueprintCallable)
  float GetISO() const;

  UFUNCTION(BlueprintCallable)
  void SetAperture(float Aperture);

  UFUNCTION(BlueprintCallable)
  float GetAperture() const;

  UFUNCTION(BlueprintCallable)
  void SetFocalDistance(float Distance);

  UFUNCTION(BlueprintCallable)
  float GetFocalDistance() const;

  UFUNCTION(BlueprintCallable)
  void SetDepthBlurAmount(float Amount);

  UFUNCTION(BlueprintCallable)
  float GetDepthBlurAmount() const;

  UFUNCTION(BlueprintCallable)
  void SetDepthBlurRadius(float Radius);

  UFUNCTION(BlueprintCallable)
  float GetDepthBlurRadius() const;

  UFUNCTION(BlueprintCallable)
  void SetBladeCount(int Count);

  UFUNCTION(BlueprintCallable)
  int GetBladeCount() const;

  UFUNCTION(BlueprintCallable)
  void SetDepthOfFieldMinFstop(float MinFstop);

  UFUNCTION(BlueprintCallable)
  float GetDepthOfFieldMinFstop() const;

  UFUNCTION(BlueprintCallable)
  void SetFilmSlope(float Slope);

  UFUNCTION(BlueprintCallable)
  float GetFilmSlope() const;

  UFUNCTION(BlueprintCallable)
  void SetFilmToe(float Toe);

  UFUNCTION(BlueprintCallable)
  float GetFilmToe() const;

  UFUNCTION(BlueprintCallable)
  void SetFilmShoulder(float Shoulder);

  UFUNCTION(BlueprintCallable)
  float GetFilmShoulder() const;

  UFUNCTION(BlueprintCallable)
  void SetFilmBlackClip(float BlackClip);

  UFUNCTION(BlueprintCallable)
  float GetFilmBlackClip() const;

  UFUNCTION(BlueprintCallable)
  void SetFilmWhiteClip(float WhiteClip);

  UFUNCTION(BlueprintCallable)
  float GetFilmWhiteClip() const;

  UFUNCTION(BlueprintCallable)
  void SetExposureMinBrightness(float Brightness);

  UFUNCTION(BlueprintCallable)
  float GetExposureMinBrightness() const;

  UFUNCTION(BlueprintCallable)
  void SetExposureMaxBrightness(float Brightness);

  UFUNCTION(BlueprintCallable)
  float GetExposureMaxBrightness() const;

  UFUNCTION(BlueprintCallable)
  void SetExposureSpeedDown(float Speed);

  UFUNCTION(BlueprintCallable)
  float GetExposureSpeedDown() const;

  UFUNCTION(BlueprintCallable)
  void SetExposureSpeedUp(float Speed);

  UFUNCTION(BlueprintCallable)
  float GetExposureSpeedUp() const;

  UFUNCTION(BlueprintCallable)
  void SetExposureCalibrationConstant(float Constant);

  UFUNCTION(BlueprintCallable)
  float GetExposureCalibrationConstant() const;

  UFUNCTION(BlueprintCallable)
  void SetMotionBlurIntensity(float Intensity);

  UFUNCTION(BlueprintCallable)
  float GetMotionBlurIntensity() const;

  UFUNCTION(BlueprintCallable)
  void SetMotionBlurMaxDistortion(float MaxDistortion);

  UFUNCTION(BlueprintCallable)
  float GetMotionBlurMaxDistortion() const;

  UFUNCTION(BlueprintCallable)
  void SetMotionBlurMinObjectScreenSize(float ScreenSize);

  UFUNCTION(BlueprintCallable)
  float GetMotionBlurMinObjectScreenSize() const;

  UFUNCTION(BlueprintCallable)
  void SetLensFlareIntensity(float Intensity);

  UFUNCTION(BlueprintCallable)
  float GetLensFlareIntensity() const;

  UFUNCTION(BlueprintCallable)
  void SetBloomIntensity(float Intensity);

  UFUNCTION(BlueprintCallable)
  float GetBloomIntensity() const;

  UFUNCTION(BlueprintCallable)
  void SetWhiteTemp(float Temp);

  UFUNCTION(BlueprintCallable)
  float GetWhiteTemp() const;

  UFUNCTION(BlueprintCallable)
  void SetWhiteTint(float Tint);

  UFUNCTION(BlueprintCallable)
  float GetWhiteTint() const;

  UFUNCTION(BlueprintCallable)
  void SetChromAberrIntensity(float Intensity);

  UFUNCTION(BlueprintCallable)
  float GetChromAberrIntensity() const;

  UFUNCTION(BlueprintCallable)
  void SetChromAberrOffset(float ChromAberrOffset);

  UFUNCTION(BlueprintCallable)
  float GetChromAberrOffset() const;

  /// Use for debugging purposes only.
  UFUNCTION(BlueprintCallable)
  bool ReadPixels(TArray<FColor> &BitMap) const
  {
    check(CaptureRenderTarget != nullptr);
    return FPixelReader::WritePixelsToArray(*CaptureRenderTarget, BitMap);
  }

  /// Use for debugging purposes only.
  UFUNCTION(BlueprintCallable)
  void SaveCaptureToDisk(const FString &FilePath) const
  {
    check(CaptureRenderTarget != nullptr);
    FPixelReader::SavePixelsToDisk(*CaptureRenderTarget, FilePath);
  }

  UFUNCTION(BlueprintCallable)
  USceneCaptureComponent2D *GetCaptureComponent2D()
  {
    return CaptureComponent2D;
  }

  UFUNCTION(BlueprintCallable)
  UTextureRenderTarget2D *GetCaptureRenderTarget()
  {
    return CaptureRenderTarget;
  }

  /// Immediate enqueues render commands of the scene at the current time.
  void EnqueueRenderSceneImmediate();

  /// Blocks until the render thread has finished all it's tasks.
  void WaitForRenderThreadToFinish() {
    TRACE_CPUPROFILER_EVENT_SCOPE(ASceneCaptureSensor::WaitForRenderThreadToFinish);
    // NOTE: FlushRenderingCommands() blocks game thread → worsens RPC timeout with multi-sensor.
    // Keep disabled. GPU readback throttle in PixelReader.cpp handles backpressure instead.
    // FlushRenderingCommands();
  }

  void EnableGBuffers(bool Enable = true)
  {
    bEnableGBuffers = Enable;
  }

  bool AreGBuffersEnabled() const
  {
    return bEnableGBuffers;
  }
  struct
  {
    FCameraGBufferUint8 SceneColor;
    FCameraGBufferUint8 SceneDepth;
    FCameraGBufferUint8 SceneStencil;
    FCameraGBufferUint8 GBufferA;
    FCameraGBufferUint8 GBufferB;
    FCameraGBufferUint8 GBufferC;
    FCameraGBufferUint8 GBufferD;
    FCameraGBufferUint8 GBufferE;
    FCameraGBufferUint8 GBufferF;
    FCameraGBufferUint8 Velocity;
    FCameraGBufferUint8 SSAO;
    FCameraGBufferUint8 CustomDepth;
    FCameraGBufferUint8 CustomStencil;
  } CameraGBuffers;

protected:

  void CaptureSceneExtended();

  // UE5: GBuffer API removed — SendGBufferTextures disabled
  // virtual void SendGBufferTextures(FGBufferRequest& GBuffer);

  virtual void BeginPlay() override;

  virtual void PrePhysTick(float DeltaSeconds) override;
  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) override;

  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

  virtual void SetUpSceneCaptureComponent(USceneCaptureComponent2D &SceneCapture) {}

  /// Render target necessary for scene capture.
  UPROPERTY(EditAnywhere)
  UTextureRenderTarget2D *CaptureRenderTarget = nullptr;

  /// Scene capture component.
  // UE5: Changed from USceneCaptureComponent2D_CARLA* to USceneCaptureComponent2D*
  UPROPERTY(EditAnywhere)
  USceneCaptureComponent2D *CaptureComponent2D = nullptr;

  UPROPERTY(EditAnywhere)
  float TargetGamma = 2.4f;

  /// Image width in pixels.
  UPROPERTY(EditAnywhere)
  uint32 ImageWidth = 800u;

  /// Image height in pixels.
  UPROPERTY(EditAnywhere)
  uint32 ImageHeight = 600u;

  /// Whether to render the post-processing effects present in the scene.
  UPROPERTY(EditAnywhere)
  bool bEnablePostProcessingEffects = true;

  /// Whether to change render target format to PF_A16B16G16R16, offering 16bit / channel
  UPROPERTY(EditAnywhere)
  bool bEnable16BitFormat = false;

  /// Whether to change render target format to PF_A16B16G16R16, offering 16bit / channel
  UPROPERTY(EditAnywhere)
  bool bEnableGBuffers = false;

// UE5: GBuffer API removed — SendGBuffer and SendGBufferTexturesInternal disabled.
// The FGBufferRequest / EGBufferTextureID types no longer exist in UE5.
// GBuffer data must be retrieved via render targets and CaptureScene() + ReadPixels().
//
// private:
//   template <typename SensorT, typename CameraGBufferT>
//   static void SendGBuffer(SensorT& Self, CameraGBufferT& CameraGBuffer,
//       FGBufferRequest& GBufferData, EGBufferTextureID TextureID) { ... }
//
// protected:
//   template <typename T>
//   void SendGBufferTexturesInternal(T& Self, FGBufferRequest& GBufferData) { ... }

};
