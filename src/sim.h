#ifndef SIMFLIGHT
#define SIMFLIGHT

#ifdef _WIN32
#include "winsock2.h"
#endif

#include "vector_math.h"
#include "packets.h"

#include <array>
#include <cstdint>
#include <optional>
#include <tuple>
#include <thread>
#include <mutex>
#include <queue>

#include <fmt/format.h>
#include <kissnet.hpp>

#include <chrono>

using hr_clock = std::chrono::high_resolution_clock;

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
  struct MotorState {
    vmath::vec3 position = {0, 0, 0};
    float rpm = 0;
    float thrust = 0;
  };

  static Sim& getInstance();

  ~Sim();

  // initialize
  bool connect();
  // udp update thread
  bool udpStateUpdate();
  // udp rc thread
  bool udpRcUpdate();
  // sim update step
  bool step();
  //stop threads
  void stop();

  float getRcData(uint8_t channel);
  uint32_t getRcDataTimeUs();

  uint64_t micros_passed = 0;
  int64_t sleep_timer = 0;
  int64_t simSteps = 0;
  int64_t bfSchedules = 0;
  int64_t avgStepTime = 100;

  bool running = false;
  bool stopped = false;

  std::array<MotorState, 4> motorsState {};

  int armingDisabledFlags = 0;

private:

  // initial quad/physics params
  InitPacket initPacket = {};

  // current internal state
  StatePacket statePacket = {};
  std::mutex statePacketMutex;
  // state update from rendering side
  std::queue<StatePacket> receivedStatePacketQueue {};

  // updates for rendering side
  std::queue<StateUpdatePacket> sendStateUpdatePacketQueue {};
  std::queue<StateOsdUpdatePacket> sendStateOsdUpdatePacketQueue {};

  std::mutex rcMutex;
  uint16_t rc_data[16] {};
  uint32_t rcDataReceptionTimeUs;

  uint64_t total_delta = 0;

  uint64_t last_osd_time = 0;
  vmath::vec3 acceleration = {0, 0, 0};

  //current battery voltage
  float batVoltage    = 0.0f; // in V
  float batVoltageSag = 0.0f; // in V but saged
  float batCapacity   = 0.0f; // in mAh

  kissnet::udp_socket recv_state_socket;
  kissnet::udp_socket recv_rcdat_socket;
  kissnet::udp_socket send_state_socket;

  std::thread stateUdpThread{};
  std::thread rcUdpThread{};

  static void update_rotation(double dt, StatePacket& state);

  float motor_torque(float volts, float rpm);
  float prop_thrust(float rpm, float vel);
  float prop_torque(float rpm, float vel);

  void updateBat(double dt);
  void updateNoise(double dt, StatePacket& state);

  // dyad init called?
  bool networkingInitialized = false;

  // osd string to check for updates
  uint8_t osd[16*30] {};

  // protected for testing
protected:

  void set_gyro(const StatePacket& state, const vmath::vec3& acceleration);

  float calculate_motors(double dt,
                         StatePacket& state,
                         std::array<MotorState, 4>& motors);

  vmath::vec3 calculate_physics(double dt,
                                StatePacket& state,
                                const std::array<MotorState, 4>& motors,
                                float motorsTorque);

  void set_rc_data(float data[8], uint32_t timeUs);

  Sim();

};

#endif