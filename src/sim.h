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

private:

  // initial quad/physics params
  InitPacket initPacket = {};

  // state update from rendering side
  StatePacket statePacket = {};
  std::mutex statePacketMutex;
  StatePacket statePacketUpdate = {};
  std::mutex statePacketUpdateMutex;

  uint64_t total_delta = 0;

  uint64_t last_osd_time = 0;
  vmath::vec3 acceleration = {0, 0, 0};

  kissnet::udp_socket recv_socket;
  kissnet::udp_socket send_socket;

  std::thread stateUdpThread;

  static void update_rotation(float dt, StatePacket& state);

  float motor_torque(float volts, float rpm);
  float prop_thrust(float rpm, float vel);
  float prop_torque(float rpm, float vel);

  // protected for testing
protected:

  void set_gyro(const StatePacket& state, const vmath::vec3& acceleration);

  float calculate_motors(float dt,
                         StatePacket& state,
                         std::array<MotorState, 4>& motors);

  vmath::vec3 calculate_physics(float dt,
                                StatePacket& state,
                                const std::array<MotorState, 4>& motors,
                                float motorsTorque);

  void set_rc_data(float data[8]);

  Sim();

public:
  uint64_t micros_passed = 0;
  int64_t sleep_timer = 0;
  int64_t simSteps = 0;
  int64_t bfSchedules = 0;

  bool running = false;

  uint16_t rc_data[16] {};
  std::array<MotorState, 4> motorsState {};

  int armingDisabledFlags = 0;

  static Sim& getInstance();

  ~Sim();

  // initialize
  void connect();
  // udp update thread
  bool udpUpdate();
  // sim update step
  bool step();
};

#endif