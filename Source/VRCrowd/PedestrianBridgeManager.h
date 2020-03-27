// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "CoreMinimal.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "WalkerParams.h"
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include "PedestrianBridgeManager.generated.h"
template <typename T>
int update_maximum(std::atomic<T> &maximum_value, T const &value) noexcept {
  T prev_value = maximum_value;
  while (prev_value < value &&
         !maximum_value.compare_exchange_weak(prev_value, value))
    ;
  return prev_value;
}

class System;
class APedestrian;
UCLASS()
class VRCROWD_API APedestrianBridgeManager : public AActor {
  GENERATED_BODY()

public:
  // Sets default values for this actor's properties
  APedestrianBridgeManager();
  virtual ~APedestrianBridgeManager();
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float TimeStart;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  FVector4 BridgeInitialState;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float BridgeDampingVertical;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float BridgeFrequencyVertical;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float BridgeMass;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float BridgeDampingLateral;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float BridgeFrequencyLateral;
  UPROPERTY(VisibleAnywhere)
  TArray<APedestrian*> crowd;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  bool UseFullVerticalModel;

  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  float CrowdInterval;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  AActor *BridgeEntity;

  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  AActor *Template;

  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  bool IsGenerateCrowd;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  int Nx;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  int Ny;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  FVector BackLeft;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  FVector FrontRight;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  int MaxNSteps;
  UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
  float OrderParameter;
  UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
  float BridgeLateral;
  UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
  float BridgeVertical;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
      float time;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
      FVector4 SimulationState;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
  int N;
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  TSubclassOf<APedestrian> PedestrianTemplateClass;

  UPROPERTY(EditAnywhere, BlueprintReadWrite)
      float SimulationSpeed;
  
protected:
  
  float LastBridgePeriodEnd, FirstBridgePeriodEnd;
  int NBridgePeriods;
  FVector SpawnLocation;
  float MaxBridgeAmplitudeLastPeriodVertical,
      MaxBridgeAmplitudeLastPeriodLateral;
  std::vector<float> StepPhases;
  int NSteps;
  std::mutex mtx;
  float next_frame_time;
  std::vector<float> state, bmin, u, tnext;
  FVector BridgeLocation0;
  std::vector<WalkerParams> Params;
  System *sys;
  std::atomic<int> MaxID;
  bool running;
  std::thread *integrating_thread;
  static void integrate_static(float DeltaTime, APedestrianBridgeManager *inst);
  float timeLastAdd;
  float state0prev, state0prevX;
public:
  int AddNewPedestrian();
  void InitializePedestrian(APedestrian *walker);
  bool IsSwitchFoot(APedestrian *walker);
  void GetLastState(APedestrian *walker, float DeltaTime,
                    std::array<float, 5> &last_state);
  // Called every frame
  virtual void Tick(float DeltaTime) override;
  virtual void BeginPlay() override;
};
