// Fill out your copyright notice in the Description page of Project Settings.

#include "PedestrianBridgeManager.h"
#include "Pedestrian.h"
#include "full_animated_model.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
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
  SetTickGroup(TG_LastDemotable);
  sys = new System();
  time = TimeStart;
  for (int i = 0; i < 4; ++i) {
    state_v1.push_back(0);
    state_v0.push_back(0);
  }
  state1=state=bmin1=bmin=tnext1=tnext=u1=u=nullptr;
  MaxID = 0;
  integrating_thread=nullptr;
  N=0;
  running=false;
}

APedestrianBridgeManager::~APedestrianBridgeManager() { 
    delete sys; 
    if(integrating_thread!=nullptr){
        running=false;
        integrating_thread->join();
        delete integrating_thread;
        integrating_thread=nullptr;
    }
}
// Called when the game starts or when spawned
int APedestrianBridgeManager::AddNewPedestrian() {
  int max_old = MaxID++;
  for (int i = 0; i < 4; ++i) {
    state_v0.push_back(0);
    state_v1.push_back(0);
  }
  ++N;
  u_v1.push_back(0);
  u_v0.push_back(0);
  bmin_v1.push_back(0);
  bmin_v0.push_back(0);
  tnext_v1.push_back(0);
  tnext_v0.push_back(0);
  WalkerParams wp{0};
  Params.push_back(wp);
  DEBUGLOG("Adding walker %i", max_old);
  return max_old;
}
bool APedestrianBridgeManager::IsSwitchFoot(APedestrian *walker) {
  int ID = walker->ID;
  if (ID >= N || ID < 0 || bmin==nullptr)
    return false;
  bool b=(bmin[ID]<0)!=walker->WhichFoot;
  if(b){
      walker->tprev=walker->TimeNextStep_tnext;
      walker->TimeNextStep_tnext=tnext[ID];
  }
  return b;

}
void APedestrianBridgeManager::InitializePedestrian(APedestrian *walker) {
  if(walker->RandomizeFrequency){
      walker->VerticalBaseFreq_omega+=walker->FrequencyMismatch*(rand()%100-50)*0.005*walker->VerticalBaseFreq_omega;
  }
  if(walker->RandomizePhase){
      walker->WhichFoot=rand()%2;
      float p=walker->VerticalStepWidth_p;
      walker->InitialSimulationState.X=std::fmod((rand()%100)*0.01*2*p,2*p)-p;
  }
  int ID = walker->ID;
  if (ID >= N || ID < 0)
    return;
  Params[ID] = WalkerParams{walker->VerticalAmplitudeOffset_a,
                            walker->LateralAmplitudeOffset_a,
                            walker->VerticalStepWidth_p,
                            walker->LateralStepWidth_p,
                            walker->VerticalBaseFreq_omega,
                            walker->LateralBaseFreq_omega,
                            walker->LateralDiscontinuity_nu,
                            walker->LateralDamping_lambda,
                            walker->Mass,
                            walker->COMDistanceFromFoot_L, walker->ForwardSpeed};
  FVector4 st = walker->InitialSimulationState;
  float ui=walker->WhichFoot?-walker->FootTargetR.X/100.0+st.X:-walker->FootTargetL.X/100.0+st.X;
  float L=walker->COMDistanceFromFoot_L;
  float f=walker->VerticalBaseFreq_omega;
  state_v0[sys->X_INDEX(ID)] = st.X;
  state_v0[sys->V_INDEX(ID)] = st.Y;
  state_v0[sys->Y_INDEX(ID)] = st.Z;
  state_v0[sys->U_INDEX(ID)] = st.W;
  state_v1[sys->X_INDEX(ID)] = st.X;
  state_v1[sys->V_INDEX(ID)] = st.Y;
  state_v1[sys->Y_INDEX(ID)] = st.Z;
  state_v1[sys->U_INDEX(ID)] = st.W;
  u_v0[ID]=u_v1[ID]=ui;
  bmin_v0[ID]=bmin_v1[ID]=walker->LateralControlWidth_bmin*(1-2*walker->WhichFoot);
  tnext_v0[ID]=tnext_v1[ID]=walker->TimeNextStep_tnext;
  walker->time = time;
  walker->tprev=std::max(0.0f,tnext_v0[ID]-0.5f/Params[ID].omegal);
  DEBUGLOG("Initialized walker %i\n", ID);
}
void APedestrianBridgeManager::GetLastState(APedestrian *walker,
                                            float DeltaTime,
                                            std::array<float, 5>& y) {
  int ID = walker->ID;
  if (ID >= N || ID < 0 || state==nullptr)
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
void APedestrianBridgeManager::Tick(float DeltaTime) {
  if(state==nullptr){
      state=state_v0.data();
      bmin=bmin_v0.data();
      state1=state_v1.data();
      bmin1=bmin_v1.data();
      u=u_v0.data();
      u1=u_v1.data();
      tnext=tnext_v0.data();
      tnext1=tnext_v1.data();
      running=true;
  }
  if (N == 0 or MaxID.load() == 0)
    return;
  sys->integrate(DeltaTime, Params, BridgeDampingVertical,
                   BridgeFrequencyVertical, BridgeMass, BridgeDampingLateral,
                   BridgeFrequencyLateral, (state), (u), (bmin), (tnext),time,true, UseFullVerticalModel,N);
  Super::Tick(DeltaTime);
  if (time-TimeStart<1e-4) {
    state[0] = BridgeInitialState.X;
    state[1] = BridgeInitialState.Y;
    state[2] = BridgeInitialState.Z;
    state[3] = BridgeInitialState.W;
  }
  DEBUGLOG("Ticking with N=%lu\n", Params.size());
  DEBUGLOG("...Integrated for T=%f, dt=%f\n", time, DeltaTime);
  time = time + DeltaTime;
  if(BridgeEntity!=nullptr){
      FVector bridgeloc=BridgeEntity->GetActorLocation();
      bridgeloc.X+=state[3]*DeltaTime;
      bridgeloc.Z+=state[1]*DeltaTime;
      BridgeEntity->SetActorLocation(bridgeloc);
  }
}
