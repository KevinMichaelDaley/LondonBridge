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
  FootTargetR0=FootTargetR+GetActorLocation();
  FootTargetL0=FootTargetL+GetActorLocation();
  X0=GetActorLocation().X;
  Y0=GetActorLocation().Y;
  StanceFoot0=WhichFoot?FootTargetL0:FootTargetR0;
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
    StanceFoot0=StanceFoot;
    if(WhichFoot){ 
        FootTargetL0 = FootTargetL+GetActorLocation();
        FootTargetL0.Z=0;
    }
    else{
        FootTargetR0 = FootTargetR+GetActorLocation();
        FootTargetR0.Z=0;
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
 float L=COMDistanceFromFoot_L;
 float p=VerticalStepWidth_p;
  COMVertical = (L*100-std::cos(std::fmod(y[0],2*p)-p)*L*100);
  float tnext=TimeNextStep_tnext;
  COMLateral = -(y[2]-u)*100;
  COMSagittal = std::sqrt(std::max(0.0f, L*L*10000-COMVertical*COMVertical-COMLateral*COMLateral));

//  COMVertical = StepHeight*std::sin(a)
//    float a=std::sin(y[1]
    float Omegap=std::sqrt(9.81/COMDistanceFromFoot_L);
    float bmin=(1-2*WhichFoot)*LateralControlWidth_bmin;
    float u2=(y[3]/Omegap+bmin);
  float s0 = StanceFoot.Y-(2*a-1)*std::sin(p)*L*200-StanceFoot.Y-COMSagittal;

  float u02=StanceFoot0.X-GetActorLocation().X;
  if (WhichFoot) {
        FootTargetL = FVector(-a*u2*100+(1-a)*u02, s0,
                         0);
        FootTargetR=FootTargetR0-GetActorLocation();
  } else{
        FootTargetR = FVector((-u2)*100+(1-a)*u02, s0,
                        0);
        FootTargetL=FootTargetL0-GetActorLocation();
        

  }
  SetActorLocation(FVector(StanceFoot.X+COMLateral,StanceFoot.Y+COMSagittal,COMVertical),false,nullptr,ETeleportType::TeleportPhysics);
}
