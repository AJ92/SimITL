#ifndef SIMITL_H
#define SIMITL_H

#ifdef _WIN32
#include "winsock2.h"
#endif

#include "util/vector_math.h"
#include "network/packets.h"

#include "util/sharedmem.h"

#include "util/LowPassFilter.h"
#include "util/SimplexNoise.h"
#include "util/SampleCurve.h"

#include <array>
#include <cstdint>
#include <optional>
#include <tuple>
#include <thread>
#include <mutex>
#include <queue>

#include <fmt/format.h>

#include <chrono>

#include "sim/bf.h"
#include "sim/state.h"
#include "sim/physics.h"

using hr_clock = std::chrono::high_resolution_clock;

namespace SimITL{

  template <typename R, typename P>
  auto to_us(std::chrono::duration<R, P> t) {
      return std::chrono::duration_cast<std::chrono::microseconds>(t).count();
  }

  template <typename R, typename P>
  auto to_ms(std::chrono::duration<R, P> t) {
      return std::chrono::duration_cast<std::chrono::milliseconds>(t).count();
  }

  class Sim {
  public:
    static Sim& getInstance();

    ~Sim();

    // initialize
    bool connect();
    // udp update thread
    bool udpStateUpdate();
    // sim update step
    bool step();
    //stop threads
    void stop();

    float getRcData(uint8_t channel);
    uint32_t getRcDataTimeUs();

    uint64_t getMicrosPassed();

    int64_t statePacketsReceived = 0;
    int64_t simSteps = 0;
    int64_t bfSchedules = 0;
    int64_t avgStepTime = 100;

    bool running = false;
    bool stopped = false;

    //NEW restructed sim:
    SimState mSimState{};
    Physics mPhysics{};

  private:
    // state update mutex (reception is in seperate thread)
    std::mutex statePacketMutex;

    std::array<std::byte, 2048> stateReceptionBuffer{};
    
    // state update from rendering side
    std::queue<StatePacket> receivedStatePacketQueue {};

    // update queues for rendering side
    uint32_t maxQueueSize = 10U;
    std::queue<StateUpdatePacket> sendStateUpdatePacketQueue {};
    std::queue<StateOsdUpdatePacket> sendStateOsdUpdatePacketQueue {};

    uint16_t rc_data[16] {};
    uint32_t rcDataReceptionTimeUs;

    int64_t total_delta = 0;

    vec3 acceleration = {0, 0, 0};

    LowPassFilter gyroLowPassFilter[3]{};
    LowPassFilter motorPwmLowPassFilter[4] = {};

    //current battery voltage
    float batVoltage    = 0.0f; // in V
    float batVoltageSag = 0.0f; // in V but saged
    double batCapacity   = 0.0f; // in mAh

    // shared memory buffer
    // buffer for data that is received from the gameclient
    void * sharedMemoryReceptionBuffer = nullptr;
    // buffer for data that is sent to the gameclient
    void * sharedMemoryTransmissionBuffer = nullptr;

    std::thread tcpThread{};
    std::thread stateUdpThread{};

    static void update_rotation(double dt, StatePacket& state);

    float motor_current(float volts, float kV);
    float motor_torque(float volts, float rpm, float kV, float R, float I0);
    float prop_thrust(float rpm, float vel);
    float prop_torque(float rpm, float vel);

    void updateBat(double dt);
    float shiftedPhase(const double dt, float hz, float phase);
    mat3 motorNoise(const double dt, MotorState& motorState);
    void updateGyroNoise(const StatePacket& state, vec3& angularNoise);
    void updateMotorNoise(const double dt, const StatePacket& state, vec3& angularNoise);

    bool simStep();

    // dyad init called?
    bool networkingInitialized = false;

    // osd string to check for updates
    uint8_t osd[16*30] {};

    // copies data to the type struct...
    template <typename T, size_t buff_size>
    bool convert(T& out, const std::array<std::byte, buff_size>& buf, const size_t length){
      bool success = false;
      size_t packetSize = sizeof(T);
      if(packetSize <= length){
        memcpy(&out, &buf[0], sizeof(T));
        success = true;
      }
      return success;
    }

    // protected for testing
  protected:
    Sim();
    
  };

}

#endif // SIMITL_H