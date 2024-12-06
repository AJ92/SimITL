#ifndef SIMITL_H
#define SIMITL_H

#ifdef _WIN32
#include "winsock2.h"
#endif

#include "util/vector_math.h"
#include "network/packets.h"

#include "sharedmem.h"

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
    void init(const StateInit& stateInit);
    // update physics params 
    void reinitPhysics(const StateInit& stateInit);
    // update the simulation according to new inputs
    void update(const StateInput& stateInput);

    //TODO: remove
    // sim update step
    void step();

    // retrieves the current state output data
    const StateOutput& getStateUpdate() const;

    // executes a command
    void command(const CommandType cmd);

    //stop threads
    void stop();

    float getRcData(uint8_t channel);
    uint32_t getRcDataTimeUs();

    uint64_t getMicrosPassed();

    bool running = false;
    bool wsThreadRunning = false;

    SimState mSimState{};
    Physics mPhysics{};

  private:
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

    std::thread wsThread{};
    std::thread stateUdpThread{};

    static void update_rotation(double dt, StateInput& state);


    void simStep();

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