#include "sim.h"

#include <chrono>
#include <cstdint>
#include <stdio.h>

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
    #include "sensors/battery_fake.h"

    #include "build/debug.h"

    #undef ENABLE_STATE

    //custom macro with bf namespaces
    #define BF_DEBUG_SET(mode, index, value) do { if (bf::debugMode == (mode)) { bf::debug[(index)] = (value); } } while (0)

    void EnableState(stateFlags_t mask) {
        stateFlags |= mask;
    }

    static float readRCSim(const bf::rxRuntimeState_t *rxRuntimeState, uint8_t channel)
    {
        UNUSED(rxRuntimeState);
        return Sim::getInstance().getRcData(channel);
    }

    static uint32_t rcFrameTimeUs(void)
    {
      return Sim::getInstance().getRcDataTimeUs();
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

const static auto OSD_UPDATE_TIME = 1e6 / 30;

const auto AIR_RHO = 1.225f;

// 20kHz scheduler, is enough to run PID at 8khz
const auto FREQUENCY = 20e3;//40e3;
const auto DELTA = 1e6 / FREQUENCY;

void Sim::set_gyro(const StatePacket& state,
                   const vmath::vec3& acceleration, 
                   const vmath::vec3& noise
) {
    using namespace vmath;
    mat3 basis;
    copy(basis, state.rotation);
    quat rotation = mat3_to_quat(basis);

    vec3 angularVelocity;
    copy(angularVelocity, state.angularVelocity);
    angularVelocity = angularVelocity + noise;
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


float Sim::calculate_motors(double dt,
                            StatePacket& state,
                            std::array<MotorState, 4>& motors) 
{
    using namespace vmath;

    const float motor_dir[4] = {1.0, -1.0, -1.0, 1.0};

    float resPropTorque = 0;

    mat3 rotation;
    copy(rotation, state.rotation);
    const auto up = get_axis(rotation, 1);

    for (int i = 0; i < 4; i++) {
        float propHealthFactor = (1.0f - state.propDamage[i]);

        // const auto r = xform(state.rotation.value, motors[i].position);
        vec3 linVel;
        copy(linVel, state.linearVelocity);
        const auto vel = std::max(0.0f, dot(linVel, up));

        auto rpm = motors[i].rpm;

        //prevent division by 0
        float vbat = std::max(1.0f, batVoltageSag);

        const auto volts =
          bf::motorsPwm[i] / 1000.0f * vbat;
        const auto torque = motor_torque(volts, rpm);

        const auto ptorque = prop_torque(rpm, vel) * propHealthFactor;
        const auto net_torque = torque - ptorque;
        const auto domega = net_torque / initPacket.propInertia;
        const auto drpm = (domega * dt) * 60.0f / (2.0f * float(M_PI));

        const auto kv = initPacket.motorKV;
        const auto maxdrpm = fabsf(volts * initPacket.motorKV - rpm);
        rpm += clamp(drpm, -maxdrpm, maxdrpm);

        //if disarmed reset rpm to 0
        if((bf::armingFlags & (bf::ARMED)) == 0){
          rpm = 0.0f;
        }

        motors[i].thrust = prop_thrust(rpm, vel) * propHealthFactor;
        motors[i].rpm = rpm;
        resPropTorque += motor_dir[i] * torque;
    }

    return resPropTorque;
}

void Sim::updateBat(double dt) {
  if(batVoltage > (3.5f * initPacket.quadBatCellCount)){
    batVoltage = initPacket.quadBatVoltage - 
      ((0.7 * initPacket.quadBatCellCount) * 
      (1.0 - (batCapacity / initPacket.quadBatCapacity)));
  }
  else{
    batVoltage -= 0.5f * dt;
  }

  batVoltage = std::max(batVoltage, 0.0f);

  bf::setCellCount(initPacket.quadBatCellCount);
  bf::voltageMeter_t* vMeter = bf::getVoltageMeter();

  float rpmSum = 0.0f;
  for(int i = 0; i < 4; i++){
    rpmSum += motorsState[i].rpm;
  }
  
  float vSag = 0.3 * (rpmSum / initPacket.propMaxRpm);

  batVoltageSag = batVoltage - vSag;

  vMeter->unfiltered      = batVoltageSag * 1e2;
  vMeter->displayFiltered = batVoltageSag * 1e2;
  vMeter->sagFiltered     = batVoltage    * 1e2;

  batCapacity -= (rpmSum / initPacket.propMaxRpm) * 800.0f * (1.0f / batCapacity) * dt;
  batCapacity = std::max(batCapacity, 0.0f);
}

// -1.0 , 1.0
static float randf(){
  return (static_cast<float>(rand()) / static_cast <float> (RAND_MAX)) * 2.0f - 1.0f;
}

static float rpmToHz(float rpm){
  return rpm / 60.0f;
}

float Sim::motorNoise(const double dt, MotorState& motor, float yAxis){
  float motorFreqHz = rpmToHz(motor.rpm);

  constexpr float pi2 = M_PI * 2.0f;
  float phaseShift = (pi2 * dt * motorFreqHz);// + (yAxis * M_PI / 2.0f);
  float phase = motor.phase + phaseShift;
  if(phase > pi2){
    phase = phase - pi2;
  }
  motor.phase = phase;
  return sinf(phase);
}

void Sim::updateGyroNoise(const StatePacket& state, vmath::vec3& angularNoise){
  // white noise
  float whiteNoiseX = randf() * state.gyroBaseNoiseAmp;
  float whiteNoiseY = randf() * state.gyroBaseNoiseAmp;
  float whiteNoiseZ = randf() * state.gyroBaseNoiseAmp;

  angularNoise[0] = whiteNoiseX;
  angularNoise[1] = whiteNoiseY;
  angularNoise[2] = whiteNoiseZ;
}

void Sim::updateMotorNoise(double dt, const StatePacket& state, vmath::vec3& angularNoise){
  //float timeSec = static_cast<double>(micros_passed) / 1e6;
  float maxRpm = initPacket.motorKV * initPacket.quadBatCellCount * 4.2;

  //float m1Hz = rpmToHz(motorsState[0].rpm);
  float rpmFactorM1 = std::max(0.0f, motorsState[0].rpm) / maxRpm;
  float rpmFactorM2 = std::max(0.0f, motorsState[1].rpm) / maxRpm;
  float rpmFactorM3 = std::max(0.0f, motorsState[2].rpm) / maxRpm;
  float rpmFactorM4 = std::max(0.0f, motorsState[3].rpm) / maxRpm;

  float dmgFactorM1 = (state.propDamage[0] + 0.05f);
  float dmgFactorM2 = (state.propDamage[1] + 0.05f);
  float dmgFactorM3 = (state.propDamage[2] + 0.05f);
  float dmgFactorM4 = (state.propDamage[3] + 0.05f);

  // only call once per dt, adapts motor phase!
  float m1Noise = motorNoise(dt, motorsState[0], 0.0f);
  float m2Noise = motorNoise(dt, motorsState[1], 0.0f);
  float m3Noise = motorNoise(dt, motorsState[2], 0.0f);
  float m4Noise = motorNoise(dt, motorsState[3], 0.0f);


  float noiseX = 
    m1Noise * rpmFactorM1 * dmgFactorM1 * state.motor1Imbalance.x +
    m2Noise * rpmFactorM2 * dmgFactorM2 * state.motor2Imbalance.x +
    m3Noise * rpmFactorM3 * dmgFactorM3 * state.motor3Imbalance.x +
    m4Noise * rpmFactorM4 * dmgFactorM4 * state.motor4Imbalance.x;

  float noiseY = 
    m1Noise * rpmFactorM1 * dmgFactorM1 * state.motor1Imbalance.y +
    m2Noise * rpmFactorM2 * dmgFactorM2 * state.motor2Imbalance.y +
    m3Noise * rpmFactorM3 * dmgFactorM3 * state.motor3Imbalance.y +
    m4Noise * rpmFactorM4 * dmgFactorM4 * state.motor4Imbalance.y;

  BF_DEBUG_SET(bf::DEBUG_SIM, 3, (1.0f + noiseX) * 1000.0f);
  //BF_DEBUG_SET(bf::DEBUG_SIM, 2, m1Hz);

  /*
    oscillation1f(state.motor1Imbalance.x * rpmFactorM1 * dmgFactorM1, m1Hz, timeSec) + 
    oscillation1f(state.motor2Imbalance.x * rpmFactorM2 * dmgFactorM2, m2Hz, timeSec) +
    oscillation1f(state.motor3Imbalance.x * rpmFactorM3 * dmgFactorM3, m3Hz, timeSec) +
    oscillation1f(state.motor4Imbalance.x * rpmFactorM4 * dmgFactorM4, m4Hz, timeSec);

  float noiseY = 
    oscillation1f(state.motor1Imbalance.y * rpmFactorM1 * dmgFactorM1, m1Hz + M_PIf / 2.0f, timeSec) +
    oscillation1f(state.motor2Imbalance.y * rpmFactorM2 * dmgFactorM2, m2Hz + M_PIf / 2.0f, timeSec) +
    oscillation1f(state.motor3Imbalance.y * rpmFactorM3 * dmgFactorM3, m3Hz + M_PIf / 2.0f, timeSec) +
    oscillation1f(state.motor4Imbalance.y * rpmFactorM4 * dmgFactorM4, m4Hz + M_PIf / 2.0f, timeSec);

  float noiseZ = 
    (noiseX + noiseY) * state.motor1Imbalance.z * state.motor2Imbalance.z * state.motor3Imbalance.z * state.motor4Imbalance.z;
  */

  double noiseZ = noiseX * noiseY * 0.1f; 

  angularNoise[0] = noiseX;
  angularNoise[1] = 0.0f; // noiseY;
  angularNoise[2] = 0.0f; // noiseZ;

}

void Sim::update_rotation(double dt, StatePacket& state) {
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
  double dt,
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

    // 0.85 was 0.5, but 85 feels more like a 3"
    total_force = total_force - dir * 1.0f * AIR_RHO * vel2 *
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

void Sim::set_rc_data(float data[8], uint32_t timeUs) {
  std::array<uint16_t, 8> rcData;
  for (int i = 0; i < 8; i++) {
    rcData[i] = uint16_t(1500 + data[i] * 500);
    rc_data[i] = rcData[i];
  }
  rcDataReceptionTimeUs = timeUs;
  //bf::rxMspFrameReceive(&rcData[0], 8);
  //hack to trick bf into using sim data...
  bf::rxRuntimeState.channelCount     = SIMULATOR_MAX_RC_CHANNELS;
  bf::rxRuntimeState.rcReadRawFn      = bf::readRCSim;
  bf::rxRuntimeState.rcFrameStatusFn  = bf::rxRCFrameStatus;
  bf::rxRuntimeState.rxProvider       = bf::RX_PROVIDER_UDP;
  bf::rxRuntimeState.rcFrameTimeUsFn  = bf::rcFrameTimeUs;
  bf::rxRuntimeState.lastRcFrameTimeUs = timeUs;
}

Sim::Sim()
  : recv_state_socket(kissnet::endpoint("localhost", 7777))
  , recv_rcdat_socket(kissnet::endpoint("localhost", 7778))
  , send_state_socket(kissnet::endpoint("localhost", 6666)) 
{
#ifdef _WIN32
  u_long opt = 1;
  int32_t SioUdpConnreset = IOC_IN | IOC_VENDOR | 12;
  ioctlsocket(recv_state_socket, SioUdpConnreset, &opt);
  ioctlsocket(recv_rcdat_socket, SioUdpConnreset, &opt);
#endif
  recv_state_socket.bind();
  recv_rcdat_socket.bind();
}

Sim& Sim::getInstance() {
  static Sim simulator;
  return simulator;
}

Sim::~Sim() {
  dyad_shutdown();
}

std::chrono::system_clock::time_point start;

void updStateUpdateThread(Sim * sim){
  auto lastUpdate = hr_clock::now();
  while(sim->udpStateUpdate() && sim->running){
    
    const auto updateDone = hr_clock::now();
    const auto diff = updateDone - lastUpdate;
    int diffus = (int) to_us(diff);
  }
}

void updRcUpdateThread(Sim * sim){
  auto lastUpdate = hr_clock::now();
  while(sim->udpRcUpdate() && sim->running){
    
    const auto updateDone = hr_clock::now();
    const auto diff = updateDone - lastUpdate;
    int diffus = (int) to_us(diff);
  }
}

bool Sim::connect() {

  if(!networkingInitialized){
    fmt::print("Initializing dyad\n");
    dyad_init();
    networkingInitialized = true;
  }

  //blocking
  dyad_setUpdateTimeout(1.0);

  running = true;
  fmt::print("Waiting for init packet\n");

  bool receivedInitPackage = false;
  while(!receivedInitPackage){
    initPacket = receive<InitPacket>(recv_state_socket);
    if (initPacket.type == PacketType::Error) {
      fmt::print("Error receiving init packet\n");
    }
    else{
      receivedInitPackage = true;
    }
  }

  //non blocking
  dyad_setUpdateTimeout(0.0);

  //reset rc data to valid data...
  for(int i = 0; i < SIMULATOR_MAX_RC_CHANNELS; i++){
    rc_data[i] = 1000U;
  }

  batVoltage = initPacket.quadBatVoltage;
  batVoltageSag = batVoltage;
  batCapacity = initPacket.quadBatCapacity;

  for (auto i = 0u; i < 4; i++) {
    motorsState[i].position[0] = initPacket.quadMotorPos[i].x;
    motorsState[i].position[1] = initPacket.quadMotorPos[i].y;
    motorsState[i].position[2] = initPacket.quadMotorPos[i].z;
  }

  fmt::print("Initializing betaflight\n");
  bf::init();

  //bf::rescheduleTask(bf::TASK_RX, 1);

  start = hr_clock::now();

  //sending the same package back for verification...
  send_state_socket.send(reinterpret_cast<const std::byte*>(&initPacket), sizeof(InitPacket));
  fmt::print("Done, sending response\n\n");

  // receive first state in sync to init the state
  bool receivedInitialState = false;
  while(!receivedInitialState){
    auto state = receive<StatePacket>(recv_state_socket);
    if (state.type == PacketType::Error) {
      fmt::print("Error receiving initial state\n");
      //return false;
    }
    else{
      statePacket = state;
      receivedInitialState = true;
    }
  }

  fmt::print("Starting udp update threads\n");
  //stateUdpThread = std::thread(&Sim::udpUpdate, this);
  stateUdpThread = std::thread(updStateUpdateThread, this);
  rcUdpThread = std::thread(updRcUpdateThread, this);

  return true;
}

bool Sim::udpStateUpdate(){

  if(sendStateUpdatePacketQueue.size() > 0 || sendStateOsdUpdatePacketQueue.size() > 0){
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

  auto state = receive<StatePacket>(recv_state_socket);
  if (state.type == PacketType::Error) {
    fmt::print("Error receiving StatePacket packet\n");
    return true;
  }

  if((state.commands & CommandType::Stop) == CommandType::Stop){
    fmt::print("Stop command received\n");
    running = false;
    false;
  }

  std::lock_guard<std::mutex> guard(statePacketMutex);
  receivedStatePacketQueue.push(state);

  return true;
}

int16_t rcUpdateFreqDbg = 0;
std::chrono::system_clock::time_point lastRcStepTime;

float lastRcValue = 0.0f;

bool Sim::udpRcUpdate(){ 
  auto state = receive<StateRcUpdatePacket>(recv_rcdat_socket);
  if (state.type == PacketType::Error) {
    fmt::print("Error receiving StateRcUpdatePacket packet\n");
    return true;
  }

  rcUpdateFreqDbg -= 5000;
  //BF_DEBUG_SET(bf::DEBUG_SIM, 3, rcUpdateFreqDbg);

  if(lastRcValue != state.rcData[2]){

    const auto now = hr_clock::now();
    const auto stepTime = now - lastRcStepTime;
    int64_t stepTimeMS = to_ms(stepTime);
    lastRcStepTime = now;

    //printf("rxSampleDiff @%d v:%f\n", stepTimeMS, (state.rcData[2] * 1000.0f) + 1000.0f);
    //BF_DEBUG_SET(bf::DEBUG_SIM, 2, (state.rcData[2] * 1000.0f) + 1000.0f);
    lastRcValue = state.rcData[2];
  }

  std::lock_guard<std::mutex> guard(rcMutex);
  set_rc_data(state.rcData, micros_passed & 0xFFFFFFFF /*static_cast<uint32_t>(state.delta * 1e6)*/);
  return true;
}

int64_t stepCount = 0;
int64_t stepTimeSum = 0;
std::chrono::system_clock::time_point lastStepTime;

bool Sim::step() {
  using namespace vmath;

  dyad_update();

  std::lock_guard<std::mutex> guard(statePacketMutex);
  if(receivedStatePacketQueue.size() > 0){

    //calculate time diff
    const auto now = hr_clock::now();
    const auto stepTime = now - lastStepTime;

    int64_t stepTimeUS = to_us(stepTime);

    lastStepTime = now;
    stepTimeUS = std::min(stepTimeUS, static_cast<int64_t>(32000));

    stepCount++;
    stepTimeSum += stepTimeUS;

    if((stepCount % 1000) == 0){
      avgStepTime = stepTimeSum / stepCount;
      stepCount = 1;
      stepTimeSum = stepTimeUS;
    }

    StatePacket& statePacketUpdate = receivedStatePacketQueue.front();

    int64_t stateUpdateDelta = static_cast<int64_t>(statePacketUpdate.delta * 1000000.0);

    if(stateUpdateDelta > static_cast<int64_t>(32000)){
      stateUpdateDelta = static_cast<int64_t>(32000);
    }
    if(stateUpdateDelta < static_cast<int64_t>(0)){
      stateUpdateDelta = static_cast<int64_t>(1000);
    }
    
    if((statePacketUpdate.commands & CommandType::Repair) == CommandType::Repair){
      //recharge
      batVoltage = initPacket.quadBatVoltage;
      batVoltageSag = batVoltage;
      batCapacity = initPacket.quadBatCapacity;
    }

    total_delta += static_cast<uint64_t>(stateUpdateDelta);

    //angular velocity update
    vec3 currentAngularVelocity;
    copy(currentAngularVelocity, statePacket.angularVelocity);

    vec3 newAngularVelocity;
    copy(newAngularVelocity, statePacketUpdate.angularVelocity);

    vec3 angularVelocityDiff = newAngularVelocity - currentAngularVelocity;

    //BF_DEBUG_SET(bf::DEBUG_SIM, 0, static_cast<int16_t>(angularVelocityDiff[0] * 30000.0f));
    //BF_DEBUG_SET(bf::DEBUG_SIM, 1, static_cast<int16_t>(angularVelocityDiff[1] * 30000.0f));
    //BF_DEBUG_SET(bf::DEBUG_SIM, 2, static_cast<int16_t>(angularVelocityDiff[2] * 30000.0f));
    //BF_DEBUG_SET(bf::DEBUG_SIM, 3, static_cast<int16_t>(stateUpdateDelta));

    float i_angular = 0.2f;
    //linear interpolation, strength defined by diff between states
    vec3 angularVelocitySmoothed = currentAngularVelocity + angularVelocityDiff * clamp(abs(angularVelocityDiff) * i_angular, 0.0f, 0.75f);

    // update state
    copy(statePacket.angularVelocity, angularVelocitySmoothed);

    //linear velocity update
    vec3 currentLinearVelocity;
    copy(currentLinearVelocity, statePacket.linearVelocity);

    vec3 newLinearVelocity;
    copy(newLinearVelocity, statePacketUpdate.linearVelocity);

    vec3 linearVelocityDiff = newLinearVelocity - currentLinearVelocity;

    float i_linear = 0.05f;
    //linear interpolation, strength defined by diff between states
    vec3 linearVelocitySmoothed = currentLinearVelocity + linearVelocityDiff * clamp(abs(linearVelocityDiff) * i_linear, 0.0f, 0.75f);

    copy(statePacket.linearVelocity, linearVelocitySmoothed);

    statePacket.vbat            = statePacketUpdate.vbat;

    statePacket.motor1Imbalance = statePacketUpdate.motor1Imbalance;
    statePacket.motor2Imbalance = statePacketUpdate.motor2Imbalance;
    statePacket.motor3Imbalance = statePacketUpdate.motor3Imbalance;
    statePacket.motor4Imbalance = statePacketUpdate.motor4Imbalance;

    statePacket.gyroBaseNoiseAmp = statePacketUpdate.gyroBaseNoiseAmp;
    statePacket.gyrobaseNoiseFreq = statePacketUpdate.gyrobaseNoiseFreq;

    statePacket.frameHarmonic1Amp = statePacketUpdate.frameHarmonic1Amp;
    statePacket.frameHarmonic1Freq = statePacketUpdate.frameHarmonic1Freq;

    statePacket.frameHarmonic2Amp = statePacketUpdate.frameHarmonic2Amp;
    statePacket.frameHarmonic2Freq = statePacketUpdate.frameHarmonic2Freq;

    statePacket.propDamage[0] = statePacketUpdate.propDamage[0];
    statePacket.propDamage[1] = statePacketUpdate.propDamage[1];
    statePacket.propDamage[2] = statePacketUpdate.propDamage[2];
    statePacket.propDamage[3] = statePacketUpdate.propDamage[3];


    memcpy( statePacket.rotation, statePacketUpdate.rotation, 3 * sizeof(Vec3F) );
  
    receivedStatePacketQueue.pop();

    //copy osd and check of change
    bool osdChanged = false;
    for (int y = 0; y < VIDEO_LINES; y++) {
      for (int x = 0; x < CHARS_PER_LINE; x++) {
        //TODO: access to osdScreen has to be guarded
        if(osd[y * CHARS_PER_LINE + x] != bf::osdScreen[y][x]){
          osdChanged = true;
        }
        osd[y * CHARS_PER_LINE + x] = bf::osdScreen[y][x];
      }
    }

    //prepare update
    //if (micros_passed - last_osd_time > OSD_UPDATE_TIME) {
    if (osdChanged || (micros_passed - last_osd_time > OSD_UPDATE_TIME)) {
      last_osd_time = micros_passed;
      StateOsdUpdatePacket update;

      update.angularVelocity = statePacket.angularVelocity;
      update.linearVelocity = statePacket.linearVelocity;
      update.motorRpm[0] = motorsState[0].rpm;
      update.motorRpm[1] = motorsState[1].rpm;
      update.motorRpm[2] = motorsState[2].rpm;
      update.motorRpm[3] = motorsState[3].rpm;
      for (int y = 0; y < VIDEO_LINES; y++) {
        for (int x = 0; x < CHARS_PER_LINE; x++) {
          //TODO: access to osdScreen has to be guarded
          update.osd[y * CHARS_PER_LINE + x] = bf::osdScreen[y][x];
        }
      }

      sendStateOsdUpdatePacketQueue.push(update);
      
    } else {
      StateUpdatePacket update;

      update.angularVelocity = statePacket.angularVelocity;
      update.linearVelocity = statePacket.linearVelocity;
      update.motorRpm[0] = motorsState[0].rpm;
      update.motorRpm[1] = motorsState[1].rpm;
      update.motorRpm[2] = motorsState[2].rpm;
      update.motorRpm[3] = motorsState[3].rpm;
      
      sendStateUpdatePacketQueue.push(update);
    }


  }
  else{
    // no new data received
    return true;
  }

  armingDisabledFlags = (int)bf::getArmingDisableFlags();

  //rc data is updated independently

  vec3 gyroNoise;
  vec3 motorNoise;
  vec3 combinedGyroNoise;

  for (auto k = 0u; total_delta - DELTA >= 0; k++) {
    total_delta -= DELTA;
    micros_passed += DELTA;
    const double dt = DELTA / 1e6f;

    updateGyroNoise(statePacket, gyroNoise);
    updateMotorNoise(dt, statePacket, motorNoise);
    combinedGyroNoise = gyroNoise + motorNoise;
    constexpr float cutoffFreq = 300.0f;
    combinedGyroNoise[0] = gyroLowPassFilterX.update(combinedGyroNoise[0], static_cast<float>(dt), cutoffFreq);
    combinedGyroNoise[1] = gyroLowPassFilterY.update(combinedGyroNoise[1], static_cast<float>(dt), cutoffFreq);
    combinedGyroNoise[2] = gyroLowPassFilterZ.update(combinedGyroNoise[2], static_cast<float>(dt), cutoffFreq);

    set_gyro(statePacket, acceleration, combinedGyroNoise);

    if (sleep_timer > 0) {
      sleep_timer -= DELTA;
      sleep_timer = std::max(int64_t(0), sleep_timer);
    } else {
      bf::scheduler();
      bfSchedules++;
    }

    simSteps++;

    if (statePacket.crashed > 0) continue;

    float motorsTorque = calculate_motors(dt, statePacket, motorsState);
    acceleration = calculate_physics(dt, statePacket, motorsState, motorsTorque);

    updateBat(dt);
  }

  return running;
}

void Sim::stop(){
  running = false;

  recv_state_socket.close();
  recv_rcdat_socket.close();
  send_state_socket.close();

  if(stateUdpThread.joinable()){
    stateUdpThread.join();
  }
  stopped = true;
}

float Sim::getRcData(uint8_t channel){
  std::lock_guard<std::mutex> guard(rcMutex);
  return rc_data[channel];
}

uint32_t Sim::getRcDataTimeUs(){
  std::lock_guard<std::mutex> guard(rcMutex);
  return rcDataReceptionTimeUs;
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
