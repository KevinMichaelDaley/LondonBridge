// Fill out your copyright notice in the Description page of Project Settings.

#include "PedestrianBridgeManager.h"
#include "Pedestrian.h"
#include "full_animated_model.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <vector>
#define USE_STDERR_LOG 0
#if USE_STDERR_LOG
#define DEBUGLOG(...) fprintf(stderr, __VA_ARGS__)
#else
#define DEBUGLOG(...)
#endif
// Sets default values
APedestrianBridgeManager::APedestrianBridgeManager() {
  // Set this actor to call Tick() every frame.  You can turn this off to
  // improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.bStartWithTickEnabled = true;
  SetTickGroup(TG_LastDemotable);
  sys = new System();
  SetActorTickInterval(0.04);
  time = TimeStart;
  for (int i = 0; i < 4; ++i) {
    state.push_back(0);
  }
  timeLastAdd = 0;
  MaxID = 0;
  integrating_thread = nullptr;
  N = 0;
  running = false;
  state0prev = state0prevX = 0.0;
  MaxBridgeAmplitudeLastPeriodVertical = MaxBridgeAmplitudeLastPeriodLateral =
      NBridgePeriods = NSteps = 0;
}
void APedestrianBridgeManager::BeginPlay() {
  Super::BeginPlay();
  BridgeLocation0 = BridgeEntity->GetActorLocation();
  OrderParameter = 0;
  NSteps = 0;
  StepPhases.resize(MaxNSteps, 0.0);
  LastBridgePeriodEnd = 0.0;
  SpawnLocation = Template->GetActorLocation();

  Template->SetActorLocation(FVector(-10000, -10000, 1000000));
  Template->SetActorTickEnabled(false);

  
}

APedestrianBridgeManager::~APedestrianBridgeManager() {
  delete sys;
  if (integrating_thread != nullptr) {
    running = false;
    integrating_thread->join();
    delete integrating_thread;
    integrating_thread = nullptr;
  }
}
// Called when the game starts or when spawned
int APedestrianBridgeManager::AddNewPedestrian() {
  int max_old = MaxID++;
  for (int i = 0; i < 4; ++i) {
    state.push_back(0);
  }
  u.push_back(0);
  bmin.push_back(0);
  tnext.push_back(0);
  WalkerParams wp{0};
  Params.push_back(wp);
  DEBUGLOG("Adding walker %i", max_old);
  ++N;
  return max_old;
}
bool APedestrianBridgeManager::IsSwitchFoot(APedestrian *walker) {
  int ID = walker->ID;
  if (ID >= N || ID < 0 || bmin.size()<=ID)
    return false;
  bool b = (bmin[ID] < 0) != walker->WhichFoot;
  if (b) {
    walker->tprev = walker->TimeNextStep_tnext;
    walker->TimeNextStep_tnext = tnext[ID];
    if (time > 30.0 && LastBridgePeriodEnd > 0) {
      float BridgePeriod =
          (LastBridgePeriodEnd - FirstBridgePeriodEnd) / NBridgePeriods;
      StepPhases[(NSteps++) % MaxNSteps] =
          (time - LastBridgePeriodEnd) * 2 * 3.14159 / BridgePeriod;
    }
  }
  return b;
}
void APedestrianBridgeManager::InitializePedestrian(APedestrian *walker) {
    int ID = walker->ID;
    if (ID >= N || ID < 0)
        return;
  if (walker->RandomizeFrequency) {
    walker->VerticalBaseFreq_omega += walker->FrequencyMismatch *
                                      (rand() % 100 - 50) * 0.01 *
                                      walker->VerticalBaseFreq_omega;

    walker->LateralBaseFreq_omega += walker->FrequencyMismatch *
        (rand() % 100 - 50) * 0.01 *
        walker->LateralBaseFreq_omega;
  }
  if (walker->RandomizePhase) {
    walker->WhichFoot = rand() % 2;
    walker->TimeNextStep_tnext = (rand() % 100) * 0.005 / walker->LateralBaseFreq_omega;
  }
  walker->TimeNextStep_tnext += time;
  
  Params[ID] = WalkerParams{
      walker->VerticalAmplitudeOffset_a, walker->LateralAmplitudeOffset_a,
      walker->VerticalStepWidth_p,       walker->LateralStepWidth_p,
      walker->VerticalBaseFreq_omega,    walker->LateralBaseFreq_omega,
      walker->LateralDiscontinuity_nu,   walker->VerticalDamping_lambda,
      walker->LateralDamping_lambda,     walker->Mass,
      walker->COMDistanceFromFoot_L,     walker->ForwardSpeed};
  FVector4 st = walker->InitialSimulationState;
  float ui = walker->WhichFoot ? -walker->FootTargetR.X / 100.0 + st.X
                               : -walker->FootTargetL.X / 100.0 + st.X;
  float L = walker->COMDistanceFromFoot_L;
  float f = walker->VerticalBaseFreq_omega;
  state[sys->X_INDEX(ID)] = st.X;
  state[sys->V_INDEX(ID)] = st.Y;
  state[sys->Y_INDEX(ID)] = st.Z;
  state[sys->U_INDEX(ID)] = st.W;
  u[ID] = ui;
  bmin[ID] =
      walker->LateralControlWidth_bmin * (1 - 2 * walker->WhichFoot);
  tnext[ID] = tnext[ID] = walker->TimeNextStep_tnext;
  walker->time = time;
  walker->tprev = std::max(0.0f, tnext[ID] - 0.5f / Params[ID].omegal);
  DEBUGLOG("Initialized walker %i\n", ID);
}
void APedestrianBridgeManager::GetLastState(APedestrian *walker,
                                            float DeltaTime,
                                            std::array<float, 5> &y) {
  int ID = walker->ID;
  if (ID >= N || ID < 0 || u.size()<=ID)
    return;

  y[0] = state[sys->X_INDEX(ID)];
  y[1] = state[sys->V_INDEX(ID)];
  y[2] = state[sys->Y_INDEX(ID)];
  y[3] = state[sys->U_INDEX(ID)];
  y[4] = u[ID];
  DEBUGLOG("Updated state of %i: %f %f %f %f\n", ID, y[0], y[1], y[2], y[3]);
  walker->time = time;
}
// Called every frame
void APedestrianBridgeManager::Tick(float DeltaTime0) {
    float DeltaTime = DeltaTime0 * SimulationSpeed;
    if (N > 0)
    {
        sys->integrate(DeltaTime, Params, BridgeDampingVertical,
            BridgeFrequencyVertical, BridgeMass, BridgeDampingLateral,
            BridgeFrequencyLateral, (state.data()), (u.data()), (bmin.data()), (tnext.data()), time,
            true, UseFullVerticalModel, N);

        if (BridgeEntity != nullptr) {
            FVector bridgeloc = BridgeEntity->GetActorLocation();
            bridgeloc.X = state[2] * 100 + BridgeLocation0.X;
            bridgeloc.Z = state[0] * 100 + BridgeLocation0.Z;
            BridgeEntity->SetActorLocation(bridgeloc);
        }
    }

    SimulationState = FVector4(1000.0*state[0], 1000.0 * state[1], 1000.0 * state[2], 1000.0 * state[3]);
    Super::Tick(DeltaTime);
    if (time - TimeStart < 1e-4) {
        state[0] = BridgeInitialState.X;
        state[1] = BridgeInitialState.Y;
        state[2] = BridgeInitialState.Z;
        state[3] = BridgeInitialState.W;
    }
    else if (state[2] < 0.0 && state0prev>0.0)
    {
        if (LastBridgePeriodEnd > 0) {
            NBridgePeriods++;
        }
        else {
            FirstBridgePeriodEnd = time;
        }
        BridgeLateral = MaxBridgeAmplitudeLastPeriodLateral;
        MaxBridgeAmplitudeLastPeriodLateral = 0;
        LastBridgePeriodEnd = time;
    }
    if (state[0] < 0.0 && state0prevX>0.0) {
        BridgeVertical = MaxBridgeAmplitudeLastPeriodVertical;
        MaxBridgeAmplitudeLastPeriodVertical = 0;
    }
    state0prev = state[2];
    state0prevX = state[0];
    DEBUGLOG("Ticking with N=%lu\n", Params.size());
    DEBUGLOG("...Integrated for T=%f, dt=%f\n", time, DeltaTime);
    time = time + DeltaTime;
    float phase_s = 0, phase_c = 0;
    if (NSteps > MaxNSteps) {
        for (int i = 0; i < MaxNSteps; ++i) {
            float phi = StepPhases[i];
            phase_s += std::sin(phi) / MaxNSteps;
            phase_c += std::cos(phi) / MaxNSteps;
        }
    }
    MaxBridgeAmplitudeLastPeriodLateral =
        std::max(MaxBridgeAmplitudeLastPeriodLateral, std::abs(state[2]));
    MaxBridgeAmplitudeLastPeriodVertical =
        std::max(MaxBridgeAmplitudeLastPeriodVertical, std::abs(state[0]));
    OrderParameter = std::sqrt(phase_s * phase_s + phase_c * phase_c);
    if (IsGenerateCrowd && (time >= CrowdInterval + timeLastAdd || crowd.Num() == 0)) {

        float X0 = std::min(BackLeft.X, FrontRight.X);
        float Y0 = std::min(BackLeft.Y, FrontRight.Y);
        float Y1 = std::max(FrontRight.Y, BackLeft.Y);
        float X1 = std::max(FrontRight.X, BackLeft.X);

        float a = (N % Nx) / (float)Nx;
        float b = 0.001 * (N / Nx) / (float)Ny;
        FVector loc =
            FVector(a * (X1)+(1 - a) * X0, b * Y1 + (1 - b) * Y0, 0);
        std::stringstream ss;
        ss << "Pedestrian_" << (crowd.Num() + 1);
        FActorSpawnParameters parms;
        parms.Template = Template;
        parms.Name = FName(ss.str().c_str());
        parms.SpawnCollisionHandlingOverride =
            ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
        APedestrian* spawned = Cast<APedestrian>(GetWorld()->SpawnActor<AActor>(
            PedestrianTemplateClass, loc, FRotator::ZeroRotator, parms));
        if (spawned) {
            spawned->SetActorTickEnabled(true);
            Template->SetActorTickEnabled(false);
            crowd.Add(spawned);
            spawned->SetActorLocation(loc + SpawnLocation);
            timeLastAdd = time;
        }
    }

}
