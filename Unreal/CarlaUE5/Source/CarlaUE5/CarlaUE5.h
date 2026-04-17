// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

// UE5.7 PCH workaround — two separate issues fixed here:
//
// 1. Algo::Sort circular-include (primary fix):
//    Include chain: Sorting.h:45 → IntroSort.h → Invoke.h → AutoRTFM.h → Memory.h
//    → CoreGlobals.h → UnrealString.h → Array.h, where TArray::Sort() references
//    Algo::Sort as a non-dependent name (looked up at parse time, before Algo/Sort.h
//    has been processed). Forward-declare both overloads to break the cycle.
//
// 2. UE_AUTORTFM_OPEN macro redefinition (secondary):
//    HAL/FMemory.inl defines UE_AUTORTFM_OPEN as a no-op before AutoRTFM.h can
//    define it, triggering -Wmacro-redefined on clang. Suppressed below.
namespace Algo {
  template <typename RangeType> void Sort(RangeType&&);
  template <typename RangeType, typename PredicateType> void Sort(RangeType&&, PredicateType);
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmacro-redefined"
#include "Engine.h"
#pragma clang diagnostic pop

#include "Util/NonCopyable.h"
