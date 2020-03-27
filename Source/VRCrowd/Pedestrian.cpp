// Fill out your copyright notice in the Description page of Project Settings.
#include "Pedestrian.h"
#include "Net/UnrealNetwork.h"
#include "PedestrianBridgeManager.h"
#
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
// Sets default values
APedestrian::APedestrian() {
  // Set this actor to call Tick() every frame.  You can turn this off to
  // improve performance if you don't need it.
  ID = -1;
  not_ref_added = true;
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.bStartWithTickEnabled = true;
}

// Called when the game starts or when spawned
void APedestrian::BeginPlay() {
    Super::BeginPlay();
  if (Manager == nullptr) {
    return;
  }
  ID = Manager->AddNewPedestrian();
  Manager->InitializePedestrian(this);
  FootTargetR0 = FootTargetR + this->GetActorLocation();
  FootTargetL0 = FootTargetL + this->GetActorLocation();
  X0 = GetActorLocation().X;
  Y0 = GetActorLocation().Y;
  StanceFoot0 = WhichFoot ? FootTargetL0 : FootTargetR0;
  refy = (X0 - XOffset);
  Z0 = GetActorLocation().Z;
  //GetSkeletalMeshComponent()->SetComponentTickEnabled(true);
}

// Called every frame
void APedestrian::Tick(float DeltaTime0) {
    
    Super::Tick(DeltaTime0);
    
  if (Manager == nullptr || ID < 0) {
    return;
  }
  float DeltaTime = DeltaTime0 * Manager->SimulationSpeed;
  std::array<float, 5> y;
  Manager->GetLastState(this, DeltaTime, y);
  bool switch_foot = Manager->IsSwitchFoot(this);
  float Y = GetActorLocation().Y;
  FVector StanceFoot;
  if (switch_foot) {
    Y0 = Y;
    X00 = X0;
    X0 = GetActorLocation().X;
    Y0 = GetActorLocation().Y;
    Z0 = GetActorLocation().Z;
    StanceFoot0 = StanceFoot;
    if (WhichFoot) {
      FootTargetL0 = FootTargetL + GetActorLocation();
      FootTargetL0.Z = 0;
    } else {
      FootTargetR0 = FootTargetR + GetActorLocation();
      FootTargetR0.Z = 0;
    }
    u0 = y[4];
    WhichFoot = !WhichFoot;
  }
  if (time < 2) {
    return;
  } else if (not_ref_added) {
    if (switch_foot) {
      u0 = y[4] - y[2];
      not_ref_added = false;
    } else {
      return;
    }
  }
  if (WhichFoot) {
    StanceFoot = FootTargetR0;
  } else {
    StanceFoot = FootTargetL0;
  }
  float u = y[4];
  //  printf("%i %f %f\n", ID, y[0], y[2]);
  float tnext = TimeNextStep_tnext;
  

  float a = 1.0 - (tnext - time) / (tnext - tprev);
  if (tnext == tprev) {
      a = 1.0;
  }
  float b =
      a;
  // a=std::sqrt(std::sin(a*M_PI/2));
  float L = COMDistanceFromFoot_L;
  float p = VerticalStepWidth_p;
  //  COMVertical = a*Forward;
  COMLateral = (y[2] - u) * -100;
  float steprate = std::min(1.0f, tnext - tprev);
  COMSagittal = a*steprate * L * 100 * ForwardSpeed*VisualForwardSpeedScale;
  COMVertical= (std::sin(a*3.14159))*L*100*StepHeight;
 
  //  COMVertical = StepHeight*std::sin(a)
  //    float a=std::sin(y[1]
  float Omegap = std::sqrt(9.81 / COMDistanceFromFoot_L);
  float bmin = (1 - 2 * WhichFoot) * LateralControlWidth_bmin;
  float u2 = (y[3] / Omegap + bmin + y[2] - u0) * -100;
  float s0 = (2*b-1) * ForwardSpeed*steprate* VisualForwardSpeedScale * L * 100  - COMSagittal;

  float u02 = StanceFoot0.X - GetActorLocation().X;
  if (WhichFoot) {
    FootTargetL = FVector(a * u2 + (1 - a) * u02, s0, 0);
    FootTargetR = FootTargetR0 - GetActorLocation();
  } else {
    FootTargetR = FVector((a * u2) + (1 - a) * u02, s0, 0);
    FootTargetL = FootTargetL0 - GetActorLocation();
  }
  SetActorLocation(FVector(StanceFoot.X + COMLateral,
                           StanceFoot.Y + COMSagittal, COMVertical),
                   false, nullptr, ETeleportType::TeleportPhysics);
}
