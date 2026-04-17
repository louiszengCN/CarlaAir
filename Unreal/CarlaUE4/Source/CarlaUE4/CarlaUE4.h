// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

// UE5.7 macOS workaround: HAL/FMemory.inl defines UE_AUTORTFM_OPEN as no-op before
// AutoRTFM.h can define it, causing a macro redefinition error. Suppress it.
//
// Algo::Sort circular-include fix:
// Sorting.h:45 → IntroSort.h → Invoke.h → AutoRTFM.h → Memory.h → CoreGlobals.h
// → UnrealString.h → Array.h, where Array.h's TArray::Sort() body references
// Algo::Sort as a non-dependent qualified name (looked up at parse time).
// Algo/Sort.h:49 hasn't been reached yet, so Algo::Sort is undefined → error.
// Forward-declare the two Sort overloads here so they are visible during the chain.
namespace Algo {
  template <typename RangeType> void Sort(RangeType&&);
  template <typename RangeType, typename PredicateType> void Sort(RangeType&&, PredicateType);
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmacro-redefined"
#include "Engine.h"
#pragma clang diagnostic pop

#include "Util/NonCopyable.h"
