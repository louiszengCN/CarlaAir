// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "CarWheelRear.h"

UCarWheelRear::UCarWheelRear()
{
    WheelRadius = 18.f;
    WheelWidth = 15.0f;
    WheelMass = 20.0f;
    CorneringStiffness = 1000.0f;
    FrictionForceMultiplier = 2.0f;
    bAffectedByBrake = true;
    bAffectedByHandbrake = true;
    bAffectedBySteering = false;
    MaxSteerAngle = 0.f;

    // Setup suspension forces
    SuspensionForceOffset = FVector::ZeroVector;
    SuspensionMaxRaise = 10.0f;
    SuspensionMaxDrop = 10.0f;
    SuspensionDampingRatio = 1.05f;
    SpringRate = 250.0f;
}
