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
    #include "drivers/pwm_output_fake.h"

    //added, not sure if needed
    #include "rx/rx.h"
    #include "rx/msp.h"

    #include "io/displayport_fake.h"
    #include "io/gps.h"

    #include "src/target.h"

    #undef ENABLE_STATE

    void EnableState(stateFlags_t mask) {
        stateFlags |= mask;
    }

    static float readRCSim(const bf::rxRuntimeState_t *rxRuntimeState, uint8_t channel)
    {
        UNUSED(rxRuntimeState);
        return Sim::getInstance().rc_data[channel];
    }

    static uint8_t rxRCFrameStatus(bf::rxRuntimeState_t *rxRuntimeState)
    {
        UNUSED(rxRuntimeState);
        return bf::RX_FRAME_COMPLETE;
    }

    extern int16_t motorsPwm[MAX_SUPPORTED_MOTORS];
  }
}  // namespace bf

void copy(vmath::vec3& out, const Vec3F& in){
  out[0] = in.x;
  out[1] = in.y;
  out[2] = in.z;
}

void copy(vmath::mat3& out, const Vec3F* in){
  for(int i = 0; i < 3; i++){
    copy(out[i], in[i]);
  }
}

void copy(Vec3F& out, const vmath::vec3& in){
  out.x = in[0];
  out.y = in[1];
  out.z = in[2];
}

void copy(Vec3F* out, const vmath::mat3& in){
  for(int i = 0; i < 3; i++){
    copy(out[i], in[i]);
  }
}

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

void Sim::set_gyro(const StatePacket& state,
                         const vmath::vec3& acceleration) {
    using namespace vmath;
    mat3 basis;
    copy(basis, state.rotation);

    quat rotation = mat3_to_quat(basis);
    vec3 angularVelocity;
    copy(angularVelocity, state.angularVelocity);
    vec3 gyro = xform_inv(basis, angularVelocity);

    vec3 accelerometer =
      xform_inv(basis, acceleration) / initPacket.quadMass;

    int16_t x, y, z;
    if (bf::sensors(bf::SENSOR_ACC)) {
#ifdef USE_QUAT_ORIENTATION
        bf::imuSetAttitudeQuat(
          rotation[3], -rotation[2], -rotation[0], rotation[1]);
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
      bf::constrain(int(gyro[0] * GYRO_SCALE * RAD2DEG), -32767, 32767));
    z = int16_t(
      bf::constrain(int(-gyro[1] * GYRO_SCALE * RAD2DEG), -32767, 32767));
    bf::fakeGyroSet(bf::fakeGyroDev, x, y, z);

    const auto
      DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS =
        1.113195f;
    const auto cosLon0 = 0.63141842418f;

    // set gps:
    static int64_t last_millis = 0;
    int64_t millis = micros_passed / 1000;

    if (millis - last_millis > 100) {

        vec3 pos;
        copy(pos, state.position);


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
        vec3 linearVelocity;
        copy(linearVelocity, state.linearVelocity);
        bf::gpsSol.groundSpeed =
          uint16_t(length(linearVelocity) * 100);
        bf::GPS_update |= bf::GPS_MSP_UPDATE;

        last_millis = millis;
    }
}

float Sim::motor_torque(float volts, float rpm) {
    auto current =
      (volts - rpm / initPacket.motorKV) / initPacket.motorR;

    if (current > 0)
        current = std::max(0.0f, current - initPacket.motorI0);
    else if (current < 0)
        current = std::min(0.0f, current + initPacket.motorI0);
    return current * 60 / (initPacket.motorKV * 2.0f * float(M_PI));
}

float Sim::prop_thrust(float rpm, float vel) {
    using namespace vmath;
    // max thrust vs velocity:
    auto propF = initPacket.propThrustFactor.x * vel * vel +
                 initPacket.propThrustFactor.y * vel +
                 initPacket.propThrustFactor.z;
    const auto max_rpm = initPacket.propMaxRpm;
    const auto prop_a = initPacket.propAFactor;
    propF = std::max(0.0f, propF);

    // thrust vs rpm (and max thrust)
    const auto b = (propF - prop_a * max_rpm * max_rpm) / max_rpm;
    const auto result = b * rpm + prop_a * rpm * rpm;

    return std::max(result, 0.0f);
}

float Sim::prop_torque(float rpm, float vel) {
    return prop_thrust(rpm, vel) * initPacket.propTorqueFactor;
}


float Sim::calculate_motors(float dt,
                                  const StatePacket& state,
                                  std::array<MotorState, 4>& motors) {
    using namespace vmath;

    const float motor_dir[4] = {1.0, -1.0, -1.0, 1.0};

    float resPropTorque = 0;

    mat3 rotation;
    copy(rotation, state.rotation);
    const auto up = get_axis(rotation, 1);

    for (int i = 0; i < 4; i++) {
        // const auto r = xform(state.rotation.value, motors[i].position);
        vec3 linVel;
        copy(linVel, state.linearVelocity);
        const auto vel = std::max(0.0f, dot(linVel, up));

        auto rpm = motors[i].rpm;

        //prevent division by 0
        float vbat = std::max(1.0f, initPacket.quadVbat);

        const auto volts =
          bf::motorsPwm[i] / 1000.0f * vbat;
        const auto torque = motor_torque(volts, rpm);

        const auto ptorque = prop_torque(rpm, vel);
        const auto net_torque = torque - ptorque;
        const auto domega = net_torque / initPacket.propInertia;
        const auto drpm = (domega * dt) * 60.0f / (2.0f * float(M_PI));

        const auto kv = initPacket.motorKV;
        const auto maxdrpm = fabsf(volts * initPacket.motorKV - rpm);
        rpm += clamp(drpm, -maxdrpm, maxdrpm);

        motors[i].thrust = prop_thrust(rpm, vel);
        motors[i].rpm = rpm;
        resPropTorque += motor_dir[i] * torque;
    }

    return resPropTorque;
}

void Sim::update_rotation(float dt, StatePacket& state) {
    using namespace vmath;
    vec3 angularVelocity;
    copy(angularVelocity, state.angularVelocity);
    const auto w = angularVelocity * dt;
    const mat3 W = {
      vec3{1, -w[2], w[1]}, vec3{w[2], 1, -w[0]}, vec3{-w[1], w[0], 1}};
    
    mat3 rotation;
    copy(rotation, state.rotation);
    rotation = W * rotation;
    copy(state.rotation, rotation);
}

vmath::vec3 Sim::calculate_physics(
  float dt,
  StatePacket& state,
  const std::array<MotorState, 4>& motors,
  float motorsTorque
) {
    using namespace vmath;
    vec3 acceleration;

    auto gravity_force = vec3{0, -9.81f * initPacket.quadMass, 0};

    // force sum:
    vec3 total_force = gravity_force;

    // drag:
    vec3 linearVelocity;
    copy(linearVelocity, state.linearVelocity);

    float vel2 = length2(linearVelocity);
    auto dir = normalize(linearVelocity);

    mat3 rotation;
    copy(rotation, state.rotation);
    auto local_dir = xform_inv(rotation, dir);

    vec3 frameDragArea;
    copy(frameDragArea, initPacket.frameDragArea);
    float area = dot(frameDragArea, abs(local_dir));

    total_force = total_force - dir * 0.5 * AIR_RHO * vel2 *
                                  initPacket.frameDragConstant * area;

    // motors:
    for (auto i = 0u; i < 4; i++) {
        total_force = total_force +
                      xform(rotation, vec3{0, motors[i].thrust, 0});
    }

    acceleration = total_force / initPacket.quadMass;

    linearVelocity = linearVelocity + acceleration * dt;
    copy(state.linearVelocity, linearVelocity);
    
    assert(std::isfinite(length(linearVelocity)));

    // moment sum around origin:
    vec3 total_moment = get_axis(rotation, 1) * motorsTorque;

    for (auto i = 0u; i < 4; i++) {
        auto force = xform(rotation, {0, motors[i].thrust, 0});
        auto rad = xform(rotation, motors[i].position);
        total_moment = total_moment + cross(rad, force);
    }

    vec3 inv_inertia;
    copy(inv_inertia, initPacket.quadInvInertia);
    mat3 inv_tensor = {vec3{inv_inertia[0], 0, 0},
                       vec3{0, inv_inertia[1], 0},
                       vec3{0, 0, inv_inertia[2]}};
    inv_tensor =
      rotation * inv_tensor * transpose(rotation);
    vec3 angularAcc = xform(inv_tensor, total_moment);
    assert(std::isfinite(angularAcc[0]) && std::isfinite(angularAcc[1]) &&
           std::isfinite(angularAcc[2]));

    vec3 angularVelocity;
    copy(angularVelocity, state.angularVelocity);
    angularVelocity = angularVelocity + angularAcc * dt;
    copy(state.angularVelocity, angularVelocity);

    update_rotation(dt, state);

    return acceleration;
}

void Sim::set_rc_data(float data[8]) {
  std::array<uint16_t, 8> rcData;
  for (int i = 0; i < 8; i++) {
    rcData[i] = uint16_t(1500 + data[i] * 500);
    rc_data[i] = rcData[i];
  }

  //bf::rxMspFrameReceive(&rcData[0], 8);

//hack to trick bf into using sim data...
  bf::rxRuntimeState.channelCount = SIMULATOR_MAX_RC_CHANNELS;
  bf::rxRuntimeState.rcReadRawFn = bf::readRCSim;
  bf::rxRuntimeState.rcFrameStatusFn = bf::rxRCFrameStatus;

  bf::rxRuntimeState.rxProvider = bf::RX_PROVIDER_UDP;
}

Sim::Sim()
  : recv_socket(kissnet::endpoint("localhost", 7777))
  , send_socket(kissnet::endpoint("localhost", 6666)) 
{
  recv_socket.bind();
}

Sim& Sim::getInstance() {
  static Sim simulator;
  return simulator;
}

Sim::~Sim() {
  dyad_shutdown();
}

void Sim::connect() {
  //reset rc data to valid data...
  for(int i = 0; i < SIMULATOR_MAX_RC_CHANNELS; i++){
    rc_data[i] = 1000U;
  }

  fmt::print("Waiting for init packet\n");

  initPacket = receive<InitPacket>(recv_socket);

  for (auto i = 0u; i < 4; i++) {
    motorsState[i].position[0] = initPacket.quadMotorPos[i].x;
    motorsState[i].position[1] = initPacket.quadMotorPos[i].y;
    motorsState[i].position[2] = initPacket.quadMotorPos[i].z;
  }

  fmt::print("Initializing dyad\n");
  dyad_init();
  dyad_setUpdateTimeout(0.001);

  fmt::print("Initializing betaflight\n");
  bf::init();

  bf::rescheduleTask(bf::TASK_RX, 1);

  fmt::print("Done, sending response\n\n");
  //sending the same package back for verification...
  send_socket.send(reinterpret_cast<const std::byte*>(&initPacket), sizeof(InitPacket));

//hack
  //bf::unsetArmingDisabled(bf::armingDisableFlags_e::ARMING_DISABLED_BOOT_GRACE_TIME);
}

bool Sim::step() {
  armingDisabledFlags = (int)bf::getArmingDisableFlags();

  auto state = receive<StatePacket>(recv_socket);
  if (state.type == PacketType::Error) {
    fmt::print("Error receiving packet\n");
    return true;
  }

  if((state.commands & CommandType::Stop) == CommandType::Stop){
    fmt::print("Stop command received\n");
    return false;
  }

  const auto deltaMicros = int(state.delta * 1e6);
  total_delta += deltaMicros;

  // const auto last = hr_clock::now();

  dyad_update();

  // auto dyad_time = hr_clock::now() - last;
  // long long dyad_time_i = to_us(dyad_time);

  // update rc at 100Hz, otherwise rx loss gets reported:
  set_rc_data(state.rcData);

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

    if (state.crashed > 0) continue;

    float motorsTorque = calculate_motors(dt, state, motorsState);

    acceleration = calculate_physics(dt, state, motorsState, motorsTorque);
  }

  if (micros_passed - last_osd_time > OSD_UPDATE_TIME) {
    last_osd_time = micros_passed;
    StateOsdUpdatePacket update;
    update.angularVelocity = state.angularVelocity;
    update.linearVelocity = state.linearVelocity;
    for (int y = 0; y < VIDEO_LINES; y++) {
      for (int x = 0; x < CHARS_PER_LINE; x++) {
        update.osd[y * CHARS_PER_LINE + x] = bf::osdScreen[y][x];
      }
    }
    send_socket.send(reinterpret_cast<const std::byte*>(&update), sizeof(StateOsdUpdatePacket));
  } else {
    StateUpdatePacket update;
    update.angularVelocity = state.angularVelocity;
    update.linearVelocity = state.linearVelocity;
    send_socket.send(reinterpret_cast<const std::byte*>(&update), sizeof(StateUpdatePacket));
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
      //bf::FLASH_Unlock();
  }

  void systemReset(void) {
      printf("[system] Reset!\n");

      exit(0);
  }

  void systemResetToBootloader(void) {
      printf("[system] ResetToBootloader!\n");

      exit(1);
  }

  uint64_t micros64(void)
  {
      return Sim::getInstance().micros_passed;
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
