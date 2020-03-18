// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <array>
#include <vector>
#include "Pedestrian.generated.h"

class APedestrianBridgeManager;
UCLASS()
class VRCROWD_API APedestrian : public AActor {
  GENERATED_BODY()

public:
  // Sets default values for this actor's properties
  APedestrian();
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float COMSagittal;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float COMLateral;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float COMVertical;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  bool WhichFoot;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  int ID;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float ForwardSpeed;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  FVector4 InitialSimulationState;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  APedestrianBridgeManager *Manager;
  UPROPERTY(EditAnywhere, BlueprintReadWrite, replicated)
  FVector FootTargetL;
  UPROPERTY(EditAnywhere, BlueprintReadWrite, replicated)
  FVector FootTargetR;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float VerticalAmplitudeOffset_a;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float LateralAmplitudeOffset_a;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float VerticalStepWidth_p;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float LateralStepWidth_p;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float VerticalBaseFreq_omega;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float LateralBaseFreq_omega;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float VerticalDamping_lambda;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float LateralDiscontinuity_nu;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float LateralDamping_lambda;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float Mass;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float COMDistanceFromFoot_L;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float LateralControlWidth_bmin;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float StepHeight;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  bool RandomizeFrequency;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  bool RandomizePhase;

  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float FrequencyMismatch;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float XOffset;
  UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
  float time;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
  float tprev;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float TimeNextStep_tnext;

protected:
  FVector FootTargetL0;
  FVector FootTargetR0;
  FVector StanceFoot0;
  float refy;
  bool not_ref_added;
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;

public:
  float X0, X00, u0, Y0, Z0;
  // Called every frame
  virtual void Tick(float DeltaTime) override;
};
