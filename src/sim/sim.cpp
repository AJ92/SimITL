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
    : recv_state_socket(kissnet::endpoint("localhost", 30713))
    , send_state_socket(kissnet::endpoint("localhost", 30715)) 
  {
    mPhysics.setSimState(&mSimState);

  #ifdef _WIN32
    u_long opt = 1;
    int32_t SioUdpConnreset = IOC_IN | IOC_VENDOR | 12;
    ioctlsocket(recv_state_socket, SioUdpConnreset, &opt);
  #endif
    recv_state_socket.bind();
  }

  Sim& Sim::getInstance() {
    static Sim simulator;
    return simulator;
  }

  Sim::~Sim() {
    //dyad_shutdown();
  }

  std::chrono::system_clock::time_point start;

  void tcpUpdateThread(Sim * sim){
    dyad_init();
    //dyad_setTickInterval(0.2f);
    dyad_setUpdateTimeout(0.0f);

    while (sim->running) {
        dyad_update();
         std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    dyad_shutdown();
    fmt::print("tcpThread end!!\n");
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

    fmt::print("Starting tcp update threads\n");
    tcpThread = std::thread(tcpUpdateThread, this);


    fmt::print("Waiting for init packet\n");

    PacketType type = PacketType::Error;
    InitPacket initPacket = {};
    bool receivedInitPackage = false;

    while(!receivedInitPackage){
      const size_t len = receive(recv_state_socket, stateReceptionBuffer);
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
    send_state_socket.send(reinterpret_cast<const std::byte*>(&initPacket), sizeof(InitPacket));
    fmt::print("Done, sending response\n\n");

    // receive first state in sync to init the state
    bool receivedInitialState = false;
    StatePacket state = {};
    while(!receivedInitialState){
      const size_t len = receive(recv_state_socket, stateReceptionBuffer);

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
        send_state_socket.send(reinterpret_cast<const std::byte*>(&update), sizeof(StateOsdUpdatePacket));
        sendStateOsdUpdatePacketQueue.pop();
      }

      if(sendStateUpdatePacketQueue.size() > 0) {
        auto& update = sendStateUpdatePacketQueue.front();
        send_state_socket.send(reinterpret_cast<const std::byte*>(&update), sizeof(StateUpdatePacket));
        sendStateUpdatePacketQueue.pop();
      }
    }


    const size_t len = receive(recv_state_socket, stateReceptionBuffer);

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
    //dyad_update();

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

    recv_state_socket.close();
    send_state_socket.close();

    if(tcpThread.joinable()){
      tcpThread.join();
    }
    if(stateUdpThread.joinable()){
      stateUdpThread.join();
    }
    stopped = true;
  }

}