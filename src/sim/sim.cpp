#include "sim.h"

#include <chrono>
#include <cstdint>
#include <stdio.h>

namespace SimITL{

  // 20kHz scheduler, is enough to run PID at 8khz
  const int64_t FREQUENCY = 8e3;//20e3;
  const int64_t DELTA = 1e6 / FREQUENCY;

  Sim& Sim::getInstance() {
    static Sim simulator;
    return simulator;
  }

  Sim::Sim()
  {
    mPhysics.setSimState(&mSimState);
  }

  Sim::~Sim() {
  }

  std::chrono::system_clock::time_point start;

  void wsUpdateThread(Sim * sim){
    sim->wsThreadRunning = true;
    while (sim->running) {
      BF::updateSerial();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    sim->wsThreadRunning = false;
    fmt::print("wsThread end!!\n");
  }

  void Sim::init(const StateInit& stateInit) {
    running = true;

    if(!wsThreadRunning){
      fmt::print("Starting ws update thread\n");
      wsThread = std::thread(wsUpdateThread, this);
    }

    mPhysics.initState(stateInit);
    
    //reset rc data to valid data...
    BF::resetRcData();

    fmt::print("Initializing betaflight\n");
    BF::setEepromFileName((const char *)stateInit.eepromName);
    BF::init();

    start = hr_clock::now();
  }

  void Sim::reinitPhysics(const StateInit& stateInit){
    mPhysics.initState(stateInit);
  }

  void Sim::update(const StateInput& stateInput){
    if(!mPhysics.checkSimState()){
      return; // no SimState, no sim!
    }

    int64_t stateUpdateDelta = static_cast<int64_t>(stateInput.delta * 1000000.0);

    if(stateUpdateDelta > static_cast<int64_t>(100000)){
      stateUpdateDelta = static_cast<int64_t>(100000);
    }
    if(stateUpdateDelta < static_cast<int64_t>(0)){
      stateUpdateDelta = static_cast<int64_t>(1);
    }
    
    total_delta += stateUpdateDelta;

    //update rc data
    BF::setRcData(stateInput.rcData);

    mPhysics.updateState(stateInput);
    
    //rc data is updated independently
    simStep();
  }

  const StateOutput& Sim::getStateUpdate() const{
    return mSimState.stateOutput;
  }

  void Sim::command(const CommandType cmd){
    mPhysics.updateCommands(cmd);
  };

  void Sim::simStep() {
    for (auto k = 0u; (total_delta - DELTA) >= 0; k++) {
      total_delta -= DELTA;
      const double dt = static_cast<double>(DELTA) / 1e6f;

      mPhysics.updateGyro(dt);
  
      // updates betaflight data and schedules bf update
      BF::update(DELTA, mSimState);

      mPhysics.updatePhysics(dt);
    }
  }

  void Sim::stop(){
    // stopping the ws coms kind of corrupts managed memory of the engine under linux...
    /*
    running = false;

    BF::stopSerial();

    if(wsThread.joinable()){
      wsThread.join();
    }
    */
  }

}