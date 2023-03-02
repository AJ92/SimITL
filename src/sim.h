#ifndef SIMFLIGHT
#define SIMFLIGHT

#ifdef _WIN32
#include "winsock2.h"
#endif

#include "vector_math.h"

#include <array>
#include <cstdint>
#include <optional>
#include <tuple>

#include <fmt/format.h>
#include <kissnet.hpp>


class Sim {
   public:
    struct MotorState {
        vmath::vec3 position = {0, 0, 0};
        float rpm = 0;
        float thrust = 0;
    };

   private:

    //InitPacket init_packet;

    uint64_t total_delta = 0;

    uint64_t last_osd_time = 0;
    vmath::vec3 acceleration = {0, 0, 0};

    std::array<MotorState, 4> motorsState;

    kissnet::udp_socket recv_socket;
    kissnet::udp_socket send_socket;

    //static void update_rotation(float dt, StatePacket& state);

    float motor_torque(float volts, float rpm);
    float prop_thrust(float rpm, float vel);
    float prop_torque(float rpm, float vel);

    // protected for testing
   protected:

  /*
    void set_gyro(const StatePacket& state, const vmath::vec3& acceleration);

    float calculate_motors(float dt,
                           const StatePacket& state,
                           std::array<MotorState, 4>& motors);

    vmath::vec3 calculate_physics(float dt,
                                  StatePacket& state,
                                  const std::array<MotorState, 4>& motors,
                                  float motorsTorque);

    void set_rc_data(std::array<FloatT, 8> data);
  */

    Sim();

   public:
    uint64_t micros_passed = 0;
    int64_t sleep_timer = 0;

    static Sim& getInstance();

    ~Sim();

    void connect();

    bool step();
};

#endif