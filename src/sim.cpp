#include "sim.h"

#include <chrono>
#include <cstdint>

extern "C" {
  #include "dyad.h"
}

namespace bf {
  extern "C" {
    #include "common/maths.h"

    #include "fc/init.h"
    #include "fc/runtime_config.h"
    #include "fc/tasks.h"

    #include "flight/imu.h"

    #include "scheduler/scheduler.h"
    #include "sensors/sensors.h"

    #include "drivers/accgyro/accgyro_fake.h"
    #include "drivers/pwm_output.h"
    //#include "drivers/pwm_output_fake.h"

    #include "rx/msp.h"

    //#include "io/displayport_fake.h"
    #include "io/gps.h"

    #include "src/target.h"

    #undef ENABLE_STATE

    void EnableState(stateFlags_t mask) {
        stateFlags |= mask;
    }
  }
}  // namespace bf


#define USE_QUAT_ORIENTATION

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

const static auto GYRO_SCALE = 16.4f;
const static auto RAD2DEG = (180.0f / float(M_PI));
const static auto ACC_SCALE = (256 / 9.80665f);

const static auto OSD_UPDATE_TIME = 1e6 / 60;

const auto AIR_RHO = 1.225f;

// 20kHz scheduler, is enough to run PID at 8khz
const auto FREQUENCY = 20e3;
const auto DELTA = 1e6 / FREQUENCY;

//TODO: fix ?
/*
void Simulator::set_gyro(const StatePacket& state,
                         const vmath::vec3& acceleration) {
    using namespace vmath;
    mat3 basis = state.rotation.value;

    vec3 pos = state.position.value;
    quat rotation = mat3_to_quat(basis);
    vec3 gyro = xform_inv(basis, state.angularVelocity.value);

    vec3 accelerometer =
      xform_inv(basis, acceleration) / init_packet.quad_mass.value;

    int16_t x, y, z;
    if (bf::sensors(bf::SENSOR_ACC)) {
#ifdef USE_QUAT_ORIENTATION
        bf::imuSetAttitudeQuat(
          rotation[3], -rotation[2], rotation[0], -rotation[1]);
#else
        x = int16_t(
          bf::constrain(int(-accelerometer[2] * ACC_SCALE), -32767, 32767));
        y = int16_t(
          bf::constrain(int(accelerometer[0] * ACC_SCALE), -32767, 32767));
        z = int16_t(
          bf::constrain(int(-accelerometer[1] * ACC_SCALE), -32767, 32767));
        bf::fakeAccSet(bf::fakeAccDev, x, y, z);
#endif
    }

    x = int16_t(
      bf::constrain(int(-gyro[2] * GYRO_SCALE * RAD2DEG), -32767, 32767));
    y = int16_t(
      bf::constrain(int(-gyro[0] * GYRO_SCALE * RAD2DEG), -32767, 32767));
    z = int16_t(
      bf::constrain(int(gyro[1] * GYRO_SCALE * RAD2DEG), -32767, 32767));
    bf::fakeGyroSet(bf::fakeGyroDev, x, y, z);

    const auto
      DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS =
        1.113195f;
    const auto cosLon0 = 0.63141842418f;

    // set gps:
    static int64_t last_millis = 0;
    int64_t millis = micros_passed / 1000;

    if (millis - last_millis > 100) {
        bf::EnableState(bf::GPS_FIX);
        bf::gpsSol.numSat = 10;
        bf::gpsSol.llh.lat =
          int32_t(
            -pos[2] * 100 /
            DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS) +
          508445910;
        bf::gpsSol.llh.lon =
          int32_t(
            pos[0] * 100 /
            (cosLon0 *
             DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS)) +
          43551050;
        bf::gpsSol.llh.altCm = int32_t(pos[1] * 100);
        bf::gpsSol.groundSpeed =
          uint16_t(length(state.linearVelocity.value) * 100);
        bf::GPS_update |= bf::GPS_MSP_UPDATE;

        last_millis = millis;
    }
}
*/

float Sim::motor_torque(float volts, float rpm) {
    auto current =
      (volts - rpm / init_packet.motor_kv.value) / init_packet.motor_R.value;

    if (current > 0)
        current = std::max(0.0f, current - init_packet.motor_I0.value);
    else if (current < 0)
        current = std::min(0.0f, current + init_packet.motor_I0.value);
    return current * 60 / (init_packet.motor_kv.value * 2.0f * float(M_PI));
}

float Sim::prop_thrust(float rpm, float vel) {
    using namespace vmath;
    // max thrust vs velocity:
    auto propF = init_packet.prop_thrust_factors.value[0].value * vel * vel +
                 init_packet.prop_thrust_factors.value[1].value * vel +
                 init_packet.prop_thrust_factors.value[2].value;
    const auto max_rpm = init_packet.prop_max_rpm.value;
    const auto prop_a = init_packet.prop_a_factor.value;
    propF = std::max(0.0f, propF);

    // thrust vs rpm (and max thrust)
    const auto b = (propF - prop_a * max_rpm * max_rpm) / max_rpm;
    const auto result = b * rpm + prop_a * rpm * rpm;

    return std::max(result, 0.0f);
}

float Sim::prop_torque(float rpm, float vel) {
    return prop_thrust(rpm, vel) * init_packet.prop_torque_factor.value;
}

//TODO: fix ?
/*
float Sim::calculate_motors(float dt,
                                  const StatePacket& state,
                                  std::array<MotorState, 4>& motors) {
    using namespace vmath;

    const float motor_dir[4] = {1.0, -1.0, -1.0, 1.0};

    float resPropTorque = 0;

    const auto up = get_axis(state.rotation.value, 1);

    for (int i = 0; i < 4; i++) {
        // const auto r = xform(state.rotation.value, motors[i].position);
        const auto linVel = state.linearVelocity.value;
        const auto vel = std::max(0.0f, dot(linVel, up));

        auto rpm = motors[i].rpm;

        const auto volts =
          bf::motorsPwm[i] / 1000.0f * init_packet.quad_vbat.value;
        const auto torque = motor_torque(volts, rpm);

        const auto ptorque = prop_torque(rpm, vel);
        const auto net_torque = torque - ptorque;
        const auto domega = net_torque / init_packet.prop_inertia.value;
        const auto drpm = (domega * dt) * 60.0f / (2.0f * float(M_PI));

        const auto kv = init_packet.motor_kv.value;
        const auto maxdrpm = fabsf(volts * init_packet.motor_kv.value - rpm);
        rpm += clamp(drpm, -maxdrpm, maxdrpm);

        motors[i].thrust = prop_thrust(rpm, vel);
        motors[i].rpm = rpm;
        resPropTorque += motor_dir[i] * torque;
    }

    return resPropTorque;
}
*/

/*
void Sim::update_rotation(float dt, StatePacket& state) {
    using namespace vmath;
    const auto w = state.angularVelocity.value * dt;
    const mat3 W = {
      vec3{1, -w[2], w[1]}, vec3{w[2], 1, -w[0]}, vec3{-w[1], w[0], 1}};
    state.rotation.value = W * state.rotation.value;
}
*/

//TODO: fix ?
/*
vmath::vec3 Sim::calculate_physics(
  float dt,
  StatePacket& state,
  const std::array<MotorState, 4>& motors,
  float motorsTorque) {
    using namespace vmath;
    vec3 acceleration;

    auto gravity_force = vec3{0, -9.81f * init_packet.quad_mass.value, 0};

    // force sum:
    vec3 total_force = gravity_force;

    // drag:
    float vel2 = length2(state.linearVelocity.value);
    auto dir = normalize(state.linearVelocity.value);
    auto local_dir = xform_inv(state.rotation.value, dir);
    float area = dot(init_packet.frame_drag_area.value, abs(local_dir));
    total_force = total_force - dir * 0.5 * AIR_RHO * vel2 *
                                  init_packet.frame_drag_constant.value * area;

    // motors:
    for (auto i = 0u; i < 4; i++) {
        total_force = total_force +
                      xform(state.rotation.value, vec3{0, motors[i].thrust, 0});
    }

    acceleration = total_force / init_packet.quad_mass.value;
    state.linearVelocity.value = state.linearVelocity.value + acceleration * dt;

    assert(std::isfinite(length(state.linearVelocity.value)));

    // moment sum around origin:
    vec3 total_moment = get_axis(state.rotation.value, 1) * motorsTorque;

    for (auto i = 0u; i < 4; i++) {
        auto force = xform(state.rotation.value, {0, motors[i].thrust, 0});
        auto rad = xform(state.rotation.value, motors[i].position);
        total_moment = total_moment + cross(rad, force);
    }

    vec3 inv_inertia = init_packet.quad_inv_inertia.value;
    mat3 inv_tensor = {vec3{inv_inertia[0], 0, 0},
                       vec3{0, inv_inertia[1], 0},
                       vec3{0, 0, inv_inertia[2]}};
    inv_tensor =
      state.rotation.value * inv_tensor * transpose(state.rotation.value);
    vec3 angularAcc = xform(inv_tensor, total_moment);
    assert(std::isfinite(angularAcc[0]) && std::isfinite(angularAcc[1]) &&
           std::isfinite(angularAcc[2]));
    state.angularVelocity.value = state.angularVelocity.value + angularAcc * dt;

    //TODO: fix ?
    //update_rotation(dt, state);

    return acceleration;
}
*/

//TODO: fix ?
/*
void Sim::set_rc_data(std::array<FloatT, 8> data) {
    std::array<uint16_t, 8> rcData;
    for (int i = 0; i < 8; i++) {
        rcData[i] = uint16_t(1500 + data[i].value * 500);
    }

    bf::rxMspFrameReceive(&rcData[0], 8);
}
*/

Sim::Sim()
    : recv_socket(kissnet::endpoint("localhost", 7777)),
      send_socket(kissnet::endpoint("localhost", 6666)) {
    recv_socket.bind();
}

Sim& Sim::getInstance() {
    static Simulator simulator;
    return simulator;
}

Sim::~Sim() {
    dyad_shutdown();
}

void Sim::connect() {
    fmt::print("Waiting for init packet\n");

  //TODO: fix ?
   // init_packet = receive<InitPacket>(recv_socket);

    for (auto i = 0u; i < 4; i++) {
      //TODO: fix ?
      //motorsState[i].position = init_packet.quad_motor_pos.value[i].value;
    }

    fmt::print("Initializing dyad\n");
    dyad_init();
    dyad_setUpdateTimeout(0.001);

    fmt::print("Initializing betaflight\n");
    bf::init();

    fmt::print("Done, sending true\n\n");
    BoolT t;
    t.value = true;
    send_socket.send(reinterpret_cast<const std::byte*>(&t), sizeof(BoolT));
}

bool Sim::step() {
    auto stateOrStop = receive<StatePacket, true>(recv_socket);
    if (!stateOrStop) {
        return false;
    }
    auto state = *stateOrStop;

    const auto deltaMicros = int(state.delta.value * 1e6);
    total_delta += deltaMicros;

    // const auto last = hr_clock::now();

    dyad_update();

    // auto dyad_time = hr_clock::now() - last;
    // long long dyad_time_i = to_us(dyad_time);

    // update rc at 100Hz, otherwise rx loss gets reported:
    set_rc_data(state.rcData.value);

    for (auto k = 0u; total_delta - DELTA >= 0; k++) {
        total_delta -= DELTA;
        micros_passed += DELTA;
        const float dt = DELTA / 1e6f;

        set_gyro(state, acceleration);

        if (sleep_timer > 0) {
            sleep_timer -= DELTA;
            sleep_timer = std::max(int64_t(0), sleep_timer);
        } else {
            bf::scheduler();
        }

        if (state.crashed.value) continue;

        float motorsTorque = calculate_motors(dt, state, motorsState);

        acceleration = calculate_physics(dt, state, motorsState, motorsTorque);
    }

    if (micros_passed - last_osd_time > OSD_UPDATE_TIME) {
        last_osd_time = micros_passed;
        StateOsdUpdatePacket update;
        update.angularVelocity.value = state.angularVelocity.value;
        update.linearVelocity.value = state.linearVelocity.value;
        for (int y = 0; y < VIDEO_LINES; y++) {
            for (int x = 0; x < CHARS_PER_LINE; x++) {
                update.osd.value[y * CHARS_PER_LINE + x] = bf::osdScreen[y][x];
            }
        }
        send(send_socket, update);
    } else {
        StateUpdatePacket update;
        update.angularVelocity.value = state.angularVelocity.value;
        update.linearVelocity.value = state.linearVelocity.value;
        send(send_socket, update);
    }

    return true;
}

/*********************
 * Betaflight Stuff: *
 *********************/
extern "C" {
  void systemInit(void) {
      int ret;

      printf("[system] Init...\n");

      bf::SystemCoreClock = 500 * 1000000;  // fake 500MHz
      bf::FLASH_Unlock();
  }

  void systemReset(void) {
      printf("[system] Reset!\n");

      exit(0);
  }

  void systemResetToBootloader(void) {
      printf("[system] ResetToBootloader!\n");

      exit(1);
  }

  uint32_t micros(void) {
      return Sim::getInstance().micros_passed & 0xFFFFFFFF;
  }

  uint32_t millis(void) {
      return (Sim::getInstance().micros_passed / 1000) & 0xFFFFFFFF;
  }

  void microsleep(uint32_t usec) {
      Sim::getInstance().sleep_timer = usec;
  }

  void delayMicroseconds(uint32_t usec) {
      microsleep(usec);
  }

  void delay(uint32_t ms) {
      microsleep(ms * 1000);
  }

/*
  uint64_t nanos64_real(void)
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec*1e9 + ts.tv_nsec) - (start_time.tv_sec*1e9 + start_time.tv_nsec);
  }

  uint64_t micros64_real(void)
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
  }

  uint64_t millis64_real(void)
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
  }
*/

}
