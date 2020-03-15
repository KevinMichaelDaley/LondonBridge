// Fill out your copyright notice in the Description page of Project Settings.

#include "Pedestrian.h"
#include "PedestrianBridgeManager.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
// Sets default values
APedestrian::APedestrian() {
  // Set this actor to call Tick() every frame.  You can turn this off to
  // improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = true;
  ID = -1;
  not_ref_added=true;
}

// Called when the game starts or when spawned
void APedestrian::BeginPlay() {
  Super::BeginPlay();
  if (Manager == nullptr) {
    return;
  }
  ID = Manager->AddNewPedestrian();
  Manager->InitializePedestrian(this);
  FootTargetR0=FootTargetR;
  FootTargetL0=FootTargetL;
  X0=GetActorLocation().X;
  Y0=GetActorLocation().Y;
  refy=(X0-XOffset);
  Z0=GetActorLocation().Z;
}

// Called every frame
void APedestrian::Tick(float DeltaTime) {
  Super::Tick(DeltaTime);
  if (Manager == nullptr || ID < 0) {
    return;
  }
  std::array<float, 5> y;
  Manager->GetLastState(this, DeltaTime, y);
  bool switch_foot=Manager->IsSwitchFoot(this);
  float Y=GetActorLocation().Y;
  FVector StanceFoot;
  if (switch_foot) {
    Y0=Y;
    X00=X0;
    X0=GetActorLocation().X;
    Y0=GetActorLocation().Y;
    Z0=GetActorLocation().Z;
    if(WhichFoot){ 
        FootTargetL0 = FootTargetL+GetActorLocation();
    }
    else{
        FootTargetR0 = FootTargetR+GetActorLocation();
    }
    u0=y[4];
    WhichFoot = !WhichFoot;
  }
  if(time<20){ return;}
  else if(not_ref_added){
      if(switch_foot){
          u0=y[4]-y[2];
          not_ref_added=false;
      }
      else{
          return;
      }
  }
  if(WhichFoot){
      StanceFoot=FootTargetR0;
  } else {
       StanceFoot=FootTargetL0;
  }
 float u=y[4];
//  printf("%i %f %f\n", ID, y[0], y[2]);
 float a = std::fmod(y[0]/(VerticalStepWidth_p)+1,2)/2.0;
// a=std::sqrt(std::sin(a*M_PI/2));
 float L=COMDistanceFromFoot_L*100;
  COMVertical = std::cos(y[0])*L+L;
  float tnext=TimeNextStep_tnext;
  COMLateral = (y[2]-u);
  COMSagittal = std::sqrt(std::max(0.0f, L*10000-COMVertical*COMVertical-COMLateral*COMLateral));

//  COMVertical = StepHeight*std::sin(a)
//    float a=std::sin(y[1]
    float Omegap=std::sqrt(9.81/COMDistanceFromFoot_L);
    float bmin=(1-2*WhichFoot)*LateralControlWidth_bmin;
    float u2=(y[3]/Omegap+bmin+y[2]-u0);
  float s0=std::sin(y[0])*L;
  if (!WhichFoot) {
        float u02=FootTargetL0.X-GetActorLocation().X;
        FootTargetL = FVector(u2+u02, s0,
                         -COMVertical);
        FootTargetR=FootTargetR0-GetActorLocation();
  } else{
        float u02=FootTargetR0.X-GetActorLocation().X;
        FootTargetR = FVector((u2)+u02, s0,
                        -COMVertical);
        FootTargetL=FootTargetL0-GetActorLocation();
  }
  SetActorLocation(FVector(StanceFoot.X+COMLateral,Y0+COMSagittal,COMVertical),false,nullptr,ETeleportType::TeleportPhysics);
}
