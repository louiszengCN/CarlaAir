// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "ActorWithRandomEngine.h"

#include "Util/RandomEngine.h"

AActorWithRandomEngine::AActorWithRandomEngine(const FObjectInitializer& ObjectInitializer) :
  Super(ObjectInitializer)
{
  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
}

void AActorWithRandomEngine::OnConstruction(const FTransform &Transform)
{
  Super::OnConstruction(Transform);
  if (!IsValid(RandomEngine)) { return; }
  RandomEngine->Seed(Seed);
}

#if WITH_EDITOR
void AActorWithRandomEngine::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
  Super::PostEditChangeProperty(PropertyChangedEvent);
  if (PropertyChangedEvent.Property) {
    if (bGenerateRandomSeed) {
      Seed = URandomEngine::GenerateRandomSeed();
      bGenerateRandomSeed = false;
    }
    if (!IsValid(RandomEngine)) { return; }
    RandomEngine->Seed(Seed);
  }
}
#endif // WITH_EDITOR

void AActorWithRandomEngine::SetSeed(const int32 InSeed)
{
  if (!IsValid(RandomEngine)) { return; }
  Seed = InSeed;
  RandomEngine->Seed(InSeed);
}
