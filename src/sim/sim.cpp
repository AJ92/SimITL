#include "sim.h"

#include <chrono>
#include <cstdint>
#include <stdio.h>

extern "C" {
  #include "dyad.h"
}

namespace SimITL{

  // 20kHz scheduler, is enough to run PID at 8khz
  const int64_t FREQUENCY = 8e3;//20e3;
  const int64_t DELTA = 1e6 / FREQUENCY;


  Sim::Sim()
  {
    mPhysics.setSimState(&mSimState);
  }

  Sim& Sim::getInstance() {
    static Sim simulator;
    return simulator;
  }

  Sim::~Sim() {
    //dyad_shutdown();
  }

  std::chrono::system_clock::time_point start;

  void wsUpdateThread(Sim * sim){
    //dyad_init();
    //dyad_setTickInterval(0.2f);
    //dyad_setUpdateTimeout(0.0f);

    while (sim->running) {
        //dyad_update();
        BF::updateSerial();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    //dyad_shutdown();
    fmt::print("wsThread end!!\n");
  }

  void updStateUpdateThread(Sim * sim){
    auto lastUpdate = hr_clock::now();
    while(sim->udpStateUpdate() && sim->running){
      
      const auto updateDone = hr_clock::now();
      const auto diff = updateDone - lastUpdate;
      int diffus = (int) to_us(diff);
    }
  }

  bool Sim::connect() {
    running = true;

    fmt::print("Starting ws update thread\n");
    tcpThread = std::thread(wsUpdateThread, this);

    fmt::print("Waiting for shared memory...\n");
    
    bool receptionInit = false;
    bool transmissionInit = false;
    while(!(receptionInit && transmissionInit)){
      if(stopped){
        return false; // handle interrupt
      }

      // init share memory buffers
      if(!receptionInit){
        sharedMemoryReceptionBuffer = SimITLMemOpen(8096, "SimITLGameState");
        receptionInit = sharedMemoryReceptionBuffer != nullptr;
      }
      if(!transmissionInit){
        sharedMemoryTransmissionBuffer = SimITLMemOpen(8096, "SimITLSimState");
        transmissionInit = sharedMemoryTransmissionBuffer != nullptr;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } 

    fmt::print("Waiting for init packet\n");

    PacketType type = PacketType::Error;
    InitPacket initPacket = {};
    bool receivedInitPackage = false;

    while(!receivedInitPackage){
      if(stopped){
        return false; // handle interrupt
      }

      const size_t len = static_cast<size_t>(
        SimITLMemRead(
          sharedMemoryReceptionBuffer, 
          static_cast<void *>(stateReceptionBuffer.data()),
          static_cast<int>(stateReceptionBuffer.size())
        )
      );

      fmt::print("Read init packet: {}\n", len);

      //probe for packet type
      if (!convert(type, stateReceptionBuffer, len) || type != PacketType::Init) {
        fmt::print("Error receiving init packet\n");
        continue;
      }

      if(!convert(initPacket, stateReceptionBuffer, len)){
        fmt::print("Error converting init packet\n");
        continue;
      }

      mPhysics.initState(initPacket);
      receivedInitPackage = true;
    }

    //non blocking
    //dyad_setUpdateTimeout(0.0);

    //reset rc data to valid data...
    BF::resetRcData();

    fmt::print("Initializing betaflight\n");
    BF::setEepromFileName((char *) initPacket.eepromName);
    BF::init();

    //bf::rescheduleTask(bf::TASK_RX, 1);

    start = hr_clock::now();

    //sending the same package back for verification...
    const size_t len = static_cast<size_t>(
      SimITLMemWrite(
        sharedMemoryTransmissionBuffer,
        reinterpret_cast<void *>(&initPacket),
        static_cast<int>(sizeof(InitPacket))
      )
    );

    fmt::print("Done, sending response\n\n");

    // receive first state in sync to init the state
    bool receivedInitialState = false;
    StatePacket state = {};
    while(!receivedInitialState){
      const size_t len = static_cast<size_t>(
        SimITLMemRead(
          sharedMemoryReceptionBuffer, 
          static_cast<void *>(stateReceptionBuffer.data()),
          static_cast<int>(stateReceptionBuffer.size())
        )
      );

      //probe for packet type
      if (!convert(type, stateReceptionBuffer, len) || type != PacketType::State) {
        fmt::print("Error receiving initial state\n");
        continue;
      }

      if (!convert(state, stateReceptionBuffer, len)){
        fmt::print("Error converting initial state packet\n");
        continue;
      }
      
      mPhysics.updateState(state);
      receivedInitialState = true;
    }

    fmt::print("Starting udp update threads\n");
    stateUdpThread = std::thread(updStateUpdateThread, this);

    return true;
  }

  bool Sim::udpStateUpdate(){
    while(sendStateUpdatePacketQueue.size() > 0 || sendStateOsdUpdatePacketQueue.size() > 0){
      std::lock_guard<std::mutex> guard(statePacketMutex);

      if (sendStateOsdUpdatePacketQueue.size() > 0) {
        auto& update = sendStateOsdUpdatePacketQueue.front();
        const size_t len = static_cast<size_t>(
          SimITLMemWrite(
            sharedMemoryTransmissionBuffer,
            reinterpret_cast<void *>(&update),
            static_cast<int>(sizeof(StateOsdUpdatePacket))
          )
        );

        if(len == 0){
          fmt::print("Couldn't write StateOsdUpdatePacket\n");
        }
        sendStateOsdUpdatePacketQueue.pop();
      }

      if(sendStateUpdatePacketQueue.size() > 0) {
        auto& update = sendStateUpdatePacketQueue.front();
        const size_t len = static_cast<size_t>(
          SimITLMemWrite(
            sharedMemoryTransmissionBuffer,
            reinterpret_cast<void *>(&update),
            static_cast<int>(sizeof(StateUpdatePacket))
          )
        );

        if(len == 0){
          fmt::print("Couldn't write StateUpdatePacket\n");
        }
        sendStateUpdatePacketQueue.pop();
      }
    }


    const size_t len = static_cast<size_t>(
      SimITLMemRead(
        sharedMemoryReceptionBuffer, 
        static_cast<void *>(stateReceptionBuffer.data()),
        static_cast<int>(stateReceptionBuffer.size())
      )
    );

    PacketType type = PacketType::Error;

    //probe for packet type
    if (!convert(type, stateReceptionBuffer, len) || type == PacketType::Error) {
      return true;
    }

    if(type == PacketType::Init){
      InitPacket initPacket = {};
      if(convert(initPacket, stateReceptionBuffer, len)){
        mPhysics.initState(initPacket);
      }
      return true;
    }

    if(type == PacketType::State){
      StatePacket statePacket = {};
      if(convert(statePacket, stateReceptionBuffer, len)){
        std::lock_guard<std::mutex> guard(statePacketMutex);
        if((receivedStatePacketQueue.size() < maxQueueSize)){
          receivedStatePacketQueue.push(statePacket);
        }

        if((statePacket.commands & CommandType::Stop) == CommandType::Stop){
          fmt::print("Stop command received\n");
          running = false;
          false;
        }
      }
      return true;
    }
    
    return true;
  }

  int64_t stepCount = 0;
  int64_t stepTimeSum = 0;
  std::chrono::system_clock::time_point lastStepTime;

  bool Sim::step() {
    if(!mPhysics.checkSimState()){
      return false; // no SimState, no sim!
    }

    std::lock_guard<std::mutex> guard(statePacketMutex);
    while(receivedStatePacketQueue.size() > 0){

      statePacketsReceived++;

      //calculate time diff
      const auto now = hr_clock::now();
      const auto stepTime = now - lastStepTime;

      int64_t stepTimeUS = to_us(stepTime);

      lastStepTime = now;
      stepTimeUS = std::min(stepTimeUS, static_cast<int64_t>(100000));

      stepCount++;
      stepTimeSum += stepTimeUS;

      if((stepCount % 1000) == 0){
        avgStepTime = stepTimeSum / std::max(stepCount, static_cast<int64_t>(1));
        stepCount = 1;
        stepTimeSum = stepTimeUS;
      }

      StatePacket& statePacketUpdate = receivedStatePacketQueue.front();

      int64_t stateUpdateDelta = static_cast<int64_t>(statePacketUpdate.delta * 1000000.0);

      if(stateUpdateDelta > static_cast<int64_t>(100000)){
        stateUpdateDelta = static_cast<int64_t>(100000);
      }
      if(stateUpdateDelta < static_cast<int64_t>(0)){
        stateUpdateDelta = static_cast<int64_t>(1);
      }
      
      total_delta += stateUpdateDelta;

      //update rc data
      BF::setRcData(statePacketUpdate.rcData);

      mPhysics.updateState(statePacketUpdate);
      
      receivedStatePacketQueue.pop();

      //rc data is updated independently
      simStep();

      if(sendStateUpdatePacketQueue.size() < maxQueueSize){
        sendStateUpdatePacketQueue.push(mSimState.stateUpdatePacket);
      }

      //prepare update
      if (mSimState.osdChanged && (sendStateOsdUpdatePacketQueue.size() < maxQueueSize)) {
        sendStateOsdUpdatePacketQueue.push(mSimState.osdUpdatePacket);
        mSimState.osdChanged = false;
      } 
    }

    return running;
  }

  bool Sim::simStep() {
    for (auto k = 0u; (total_delta - DELTA) >= 0; k++) {
      total_delta -= DELTA;
      const double dt = static_cast<double>(DELTA) / 1e6f;

      mPhysics.updateGyro(dt);
  
      // updates betaflight data and schedules bf update
      if(BF::update(DELTA, mSimState)){
        bfSchedules++;
      }
      
      simSteps++;

      mPhysics.updatePhysics(dt);
    }

    return true;
  }

  void Sim::stop(){
    running = false;

    if(sharedMemoryReceptionBuffer != nullptr){
      SimITLMemDestroy(sharedMemoryReceptionBuffer);
    }
    if(sharedMemoryTransmissionBuffer != nullptr){
      SimITLMemDestroy(sharedMemoryTransmissionBuffer);
    }

    if(tcpThread.joinable()){
      tcpThread.join();
    }
    if(stateUdpThread.joinable()){
      stateUdpThread.join();
    }
    
    stopped = true;
  }

}