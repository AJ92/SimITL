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

    #include "drivers/accgyro/accgyro_virtual.h"
    #include "drivers/pwm_output.h"
    #include "drivers/pwm_output_fake.h"
    #include "sensors/current.h"

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

    char * EEPROM_FILENAME = 0;

    static void setEepromFileName(const char* filename = "eeprom.bin"){
      const size_t maxFileSize = 32;
      EEPROM_FILENAME = new char[maxFileSize];
      std::fill(EEPROM_FILENAME, EEPROM_FILENAME + maxFileSize, 0);
      memcpy(EEPROM_FILENAME, filename, strnlen(filename, maxFileSize));
    }

    extern int16_t motorsPwm[MAX_SUPPORTED_MOTORS];
  }
}  // namespace bf

using namespace vmath;

inline void copy(vec3& out, const Vec3F& in){
  out[0] = in.x;
  out[1] = in.y;
  out[2] = in.z;
}

inline void copy(mat3& out, const Vec3F* in){
  for(int i = 0; i < 3; i++){
    copy(out[i], in[i]);
  }
}

inline void copy(Vec3F& out, const vec3& in){
  out.x = in[0];
  out.y = in[1];
  out.z = in[2];
}

inline void copy(Vec3F* out, const mat3& in){
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

const auto AIR_RHO = 1.225f;

// 20kHz scheduler, is enough to run PID at 8khz
const auto FREQUENCY = 8e3;//20e3;
const auto DELTA = 1e6 / FREQUENCY;

void Sim::set_gyro(const double dt,
                   const StatePacket& state,
                   const vmath::vec3& acceleration, 
                   const vmath::vec3& noise
) {
    mat3 basis;
    copy(basis, state.rotation);
    quat rotation = mat3_to_quat(basis);

    vec3 angularVelocity;
    copy(angularVelocity, state.angularVelocity);
    angularVelocity = angularVelocity + noise;

    constexpr float cutoffFreq = 300.0f;
    angularVelocity[0] = gyroLowPassFilter[0].update(angularVelocity[0], dt, cutoffFreq);
    angularVelocity[1] = gyroLowPassFilter[1].update(angularVelocity[1], dt, cutoffFreq);
    angularVelocity[2] = gyroLowPassFilter[2].update(angularVelocity[2], dt, cutoffFreq);

    vec3 gyro = xform_inv(basis, angularVelocity);

    //todo: fix acceleration for fake acc ( else part of ifdef enables acc in black box explorer )
    auto gravity_force = vec3{0, -9.81f * initPacket.quadMass, 0};
    vec3 accelerometer =
      xform_inv(basis, acceleration - gravity_force) / std::max(initPacket.quadMass, 0.01f);
  

    int16_t x, y, z;
    if (bf::sensors(bf::SENSOR_ACC)) {
//#ifdef USE_QUAT_ORIENTATION
        bf::imuSetAttitudeQuat(rotation[3], -rotation[2], -rotation[0], rotation[1]);

//#else
        x = int16_t(
          bf::constrain(int(-accelerometer[2] * ACC_SCALE), -32767, 32767));
        y = int16_t(
          bf::constrain(int(accelerometer[0] * ACC_SCALE), -32767, 32767));
        z = int16_t(
          bf::constrain(int(accelerometer[1] * ACC_SCALE), -32767, 32767));
        bf::virtualAccSet(bf::virtualAccDev, x, y, z);
//#endif
    }

    x = int16_t(
      bf::constrain(int(-gyro[2] * GYRO_SCALE * RAD2DEG), -32767, 32767));
    y = int16_t(
      bf::constrain(int(gyro[0] * GYRO_SCALE * RAD2DEG), -32767, 32767));
    z = int16_t(
      bf::constrain(int(-gyro[1] * GYRO_SCALE * RAD2DEG), -32767, 32767));
    bf::virtualGyroSet(bf::virtualGyroDev, x, y, z);

    const auto
      DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS =
        1.113195f;
    const auto cosLon0 = 0.63141842418f;

    // set gps:
    static int64_t last_millis = 0;
    int64_t millis = micros_passed / 1000;

    //if (millis - last_millis > 100) {
    {
        vec3 pos;
        copy(pos, state.position);

        bf::EnableState(bf::GPS_FIX);
        bf::gpsSol.numSat = 10;
        bf::gpsSol.llh.lat =
          int32_t(
            pos[2] * 100 /
            DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS) +
          initPacket.gps.lat;
        bf::gpsSol.llh.lon =
          int32_t(
            pos[0] * 100 /
            (cosLon0 *
             DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS)) +
          initPacket.gps.lon;
        bf::gpsSol.llh.altCm = int32_t(pos[1] * 100) + initPacket.gps.alt;
        vec3 linearVelocity;
        copy(linearVelocity, state.linearVelocity);
        bf::gpsSol.groundSpeed =
          uint16_t(length(linearVelocity) * 100);
        bf::GPS_update |= bf::GPS_MSP_UPDATE;

        last_millis = millis;
    }
}

float Sim::motor_torque(float volts, float rpm, float kV, float R, float I0) {
    auto current =
      (volts - rpm / std::max(kV, 0.01f)) / std::max(R, 0.0f);

    if (current > 0)
        current = std::max(0.0f, current - I0);
    else if (current < 0)
        current = std::min(0.0f, current + I0);
    return current * 60 / (std::max(kV, 0.01f) * 2.0f * float(M_PI));
}

float Sim::prop_thrust(float rpm, float vel) {
    // max thrust vs velocity:
    auto propF = initPacket.propThrustFactor.x * vel * vel +
                 initPacket.propThrustFactor.y * vel +
                 initPacket.propThrustFactor.z;

    const auto max_rpm = std::max(initPacket.propMaxRpm, 0.01f);
    const auto prop_a = initPacket.propAFactor;
    propF = std::max(0.0f, propF);

    // thrust vs rpm (and max thrust) 
    const auto b = (propF - prop_a * max_rpm * max_rpm ) / max_rpm;
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
    const float motor_dir[4] = {1.0, -1.0, -1.0, 1.0};

    float resPropTorque = 0;

    mat3 rotation;
    copy(rotation, state.rotation);
    const auto up = get_axis(rotation, 1);

    vec3 linVel;
    copy(linVel, state.linearVelocity);
    const auto vel = std::max(0.0f, dot(linVel, up));

    constexpr float maxEffectSpeed = 15.0f; // m/s
    const float speed = std::abs(length(linVel)); // m/s
    float speedFactor = std::min(speed / maxEffectSpeed, 1.0f);

    for (int i = 0; i < 4; i++) {
      // 1.0 - effect
      float propHealthFactor = (1.0f - state.propDamage[i]);
      // 1.0 + effect
      float groundEffect = 1.0f + ((state.groundEffect[i] * state.groundEffect[i]) * 0.7f);
      
      // positive value depending on how much thrust is given against actual movement direction of quad
      float reverseThrust = std::max(0.0f, dot(normalize(linVel), normalize(motors[i].thrust * up) * -1.0f));
      // keep between 0.0 and 1.0, takes 25% that point the most against movement direction
      reverseThrust = std::max(0.0f, reverseThrust - 0.75f) * 4.0f;
      reverseThrust = reverseThrust * reverseThrust;
      float propWashNoise = std::max(0.0f, 0.5f * (SimplexNoise::noise(reverseThrust * speed * motors[i].thrust) + 1.0f));



      // 1.0 - effect
      float propwashEffect = 1.0f - (propWashNoise * reverseThrust * 0.75f) * speedFactor;

      auto rpm = motors[i].rpm;
      const auto kV = initPacket.motorKV[i];
      const auto R  = initPacket.motorR[i];
      const auto I0 = initPacket.motorI0[i];

      //prevent division by 0
      float vbat = std::max(1.0f, batVoltageSag);

      float armed = ((bf::armingFlags & (bf::ARMED)) == 0) ? 0.0f : 1.0f;
      //prevents remaining low rpm during disarm
      const auto initalCurrentThres = I0 * armed;

      const auto volts = motorPwmLowPassFilter[i].update(bf::motorsPwm[i], dt, 120.0f) / 1000.0f * vbat;
      const auto torque = motor_torque(volts, rpm, kV, R, initalCurrentThres);
      const auto ptorque = prop_torque(rpm, vel) * propHealthFactor;
      const auto net_torque = torque - ptorque;

      const auto domega = net_torque / std::max(initPacket.propInertia, 0.00000001f);
      const auto drpm = (domega * dt) * 60.0f / (2.0f * float(M_PI));

      const auto maxdrpm = fabsf(volts * kV - rpm);
      rpm += clamp(drpm, -maxdrpm, maxdrpm);

      motors[i].torque = net_torque;
      motors[i].thrust = prop_thrust(rpm, vel) * propHealthFactor * groundEffect * propwashEffect;
      motors[i].rpm = rpm;
      resPropTorque += motor_dir[i] * torque;

      if(i == 0){
        BF_DEBUG_SET(bf::DEBUG_SIM, 0, reverseThrust    * 1000);
        BF_DEBUG_SET(bf::DEBUG_SIM, 1, speedFactor      * 1000);
        BF_DEBUG_SET(bf::DEBUG_SIM, 2, propwashEffect   * 1000);
        BF_DEBUG_SET(bf::DEBUG_SIM, 3, motors[i].thrust * 1000);
      }
    }

    return resPropTorque;
}

void Sim::updateBat(double dt) {
 
  const float batCapacityFull = std::max(initPacket.quadBatCapacity, 1.0f);
  batVoltage = batVoltageCurve.sample(1.0f - (batCapacity / batCapacityFull)) * initPacket.quadBatCellCount;

  batVoltage = std::max(batVoltage, 0.1f);

  bf::setCellCount(initPacket.quadBatCellCount);
  bf::voltageMeter_t* vMeter = bf::getVoltageMeter();
  bf::currentMeter_t* cMeter = bf::getCurrentMeter();

  float rpmSum = 0.0f;
  for(int i = 0; i < 4; i++){
    rpmSum += motorsState[i].rpm;
  }
  
  const float powerFactor = std::max(0.0f, std::min(1.0f, ((rpmSum / std::max(initPacket.propMaxRpm, 0.01f)) / 4.0f)));
  const float powerFactor2 = powerFactor * powerFactor;
  const float chargeFactorInv = 1.0f - (batCapacity / std::max(initPacket.quadBatCapacityCharged, 1.0f));
 
  float vSag = initPacket.maxVoltageSag * powerFactor2 + // power dependency
  (initPacket.maxVoltageSag * chargeFactorInv * chargeFactorInv * powerFactor2); // charge state dependency

  batVoltageSag = batVoltage - vSag;

  vMeter->unfiltered      = batVoltageSag * 1e2;
  vMeter->displayFiltered = batVoltageSag * 1e2;
  vMeter->sagFiltered     = batVoltage    * 1e2;

  float currentmAs = powerFactor2 * initPacket.maxAmpDraw;
  // minimum 2W consumption clamped to max 2mA/s to account for running electronics
  const float mAMin = std::min(2.0f, 2.0f / std::max(batVoltageSag, 0.01f));
  currentmAs = std::max(currentmAs, mAMin );

  // 1W = 1V * 1A
  //P = I * V

  // centi ampere 1/100
  // milliAmpSeconds * 3600 / 1000 * 100
  cMeter->amperage = currentmAs * 3.6 * 1e2;
  cMeter->amperageLatest = cMeter->amperage;

  batCapacity -= currentmAs * dt;

  cMeter->mAhDrawn = initPacket.quadBatCapacityCharged - batCapacity;

  // negative cappa allows to drop voltage below 3.5V
  //batCapacity = std::max(batCapacity, 0.1f);
}

// -1.0 , 1.0
inline float randf(){
  return (static_cast<float>(rand()) / static_cast <float> (RAND_MAX)) * 2.0f - 1.0f;
}

inline float rpmToHz(float rpm){
  return rpm / 60.0f;
}

// calculates next phase 
float Sim::shiftedPhase(const double dt, float hz, float phaseStart){
  constexpr float pi2 = M_PI * 2.0f;
  float phaseShift = (pi2 * dt * hz);
  float phaseUpdated = phaseStart + phaseShift;
  if(std::abs(phaseUpdated) > pi2){ // keep number low. is used for sin/cos anyways
    phaseUpdated = phaseUpdated - (pi2 * static_cast<int>(phaseUpdated / pi2));
  }
  return phaseUpdated;
}

vmath::mat3 Sim::motorNoise(const double dt, MotorState& motor){
  // update phase and harmonics
  motor.phase          = shiftedPhase(dt, rpmToHz(motor.rpm)       , motor.phase);
  motor.phaseHarmonic1 = shiftedPhase(dt, rpmToHz(motor.rpm) * 2.0f, motor.phaseHarmonic1);
  motor.phaseHarmonic2 = shiftedPhase(dt, rpmToHz(motor.rpm) * 3.0f, motor.phaseHarmonic2);

  float sinPhase = sinf(motor.phase);
  float sinPhaseH1 = sinf(motor.phaseHarmonic1);
  float sinPhaseH2 = sinf(motor.phaseHarmonic2);

  float cosPhase = cosf(motor.phase);
  float cosPhaseH1 = cosf(motor.phaseHarmonic1);
  float cosPhaseH2 = cosf(motor.phaseHarmonic2);

  return {vec3{sinPhase, sinPhaseH1, sinPhaseH2}, 
          vec3{cosPhase, cosPhaseH1, cosPhaseH2},
          vec3{sinPhase + cosPhase, sinPhaseH1 + cosPhaseH1, sinPhaseH2 + cosPhaseH2}};
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

void Sim::updateMotorNoise(const double dt, const StatePacket& state, vmath::vec3& angularNoise){
  float maxV = initPacket.quadBatCellCount * 4.2;

  // per motor 0 - 3
  vec4 motorKV = toVec4(initPacket.motorKV);
  vec4 maxRpm = maximum((motorKV * maxV), 0.1f);

  vec4 motorRpm = {
    motorsState[0].rpm,
    motorsState[1].rpm,
    motorsState[2].rpm,
    motorsState[3].rpm
  };

  vec4 rpmFactor    = maximum(motorRpm, 0.0f) / maxRpm;
  vec4 rpmFactor2   = rpmFactor * rpmFactor;
  vec4 dmgFactor    = toVec4(state.propDamage) + 0.05f;
  vec4 rpmDmgFactor = dmgFactor * rpmFactor2;

  // only call once per dt, adapts motor phase!
  std::array<mat3, 4> mNoise = {
    motorNoise(dt, motorsState[0]),
    motorNoise(dt, motorsState[1]),
    motorNoise(dt, motorsState[2]),
    motorNoise(dt, motorsState[3])
  };

  vec3 noise{0.0f, 0.0f, 0.0f};
  for(int i = 0; i < 4; i++){
    noise[0] += 
      // noise
      mNoise[i][0][0] * state.motorImbalance[i].x * rpmDmgFactor[i] +
      // harmonic 1
      mNoise[i][0][1] * state.motorImbalance[i].x * rpmDmgFactor[i] * initPacket.propHarmonic1Amp +
      // harmonic 2
      mNoise[i][0][2] * state.motorImbalance[i].x * rpmDmgFactor[i] * initPacket.propHarmonic2Amp;

    noise[1] +=
       // motor noise
      mNoise[i][1][0] * state.motorImbalance[i].y * rpmDmgFactor[i] +
      // harmonic 1
      mNoise[i][1][1] * state.motorImbalance[i].y * rpmDmgFactor[i] * initPacket.propHarmonic1Amp +
      // harmonic 2
      mNoise[i][1][2] * state.motorImbalance[i].y * rpmDmgFactor[i] * initPacket.propHarmonic2Amp;

    noise[2] += ( 
      // motor noise
      mNoise[i][2][0] * state.motorImbalance[i].z * rpmDmgFactor[i] +
      //harmonic 1
      mNoise[i][2][1] * state.motorImbalance[i].z * rpmDmgFactor[i] * initPacket.propHarmonic1Amp +
      //harmonic 2
      mNoise[i][2][2] * state.motorImbalance[i].z * rpmDmgFactor[i] * initPacket.propHarmonic2Amp) * 0.5f;
  }

  // frame noise 
  frameHarmonicPhase1 = shiftedPhase(dt, state.frameHarmonic1Freq + randf() * 70.0f, frameHarmonicPhase1);
  frameHarmonicPhase2 = shiftedPhase(dt, state.frameHarmonic2Freq + randf() * 60.0f, frameHarmonicPhase2);

  vec4 rpmFactorHDec  = minimum(maximum(motorRpm, 0.0f) / (maxRpm * 0.15f), 1.0f);
  float rpmFactorH = sum(rpmFactorHDec) * 0.25f;

  vec4 rpmFactorH1Inc  = minimum(maximum(motorRpm, 0.0f) / (maxRpm * 0.43f), 1.0f);
  float rpmFactorH1Inv = sum(1.0f - rpmFactorH1Inc) * 0.25f;

  vec4 rpmFactorH2Inc  = minimum(maximum(motorRpm, 0.0f) / (maxRpm * 0.3f), 1.0f);
  float rpmFactorH2Inv =  sum(1.0f - rpmFactorH2Inc) * 0.25f;

  noise[0] += //frame harmonic 1
    ( state.motorImbalance[0].x * dmgFactor[0] +
      state.motorImbalance[1].x * dmgFactor[1] +
      state.motorImbalance[2].x * dmgFactor[2] +
      state.motorImbalance[3].x * dmgFactor[3]
    ) * 0.25f * state.frameHarmonic1Amp * sinf(frameHarmonicPhase1) * rpmFactorH1Inv * rpmFactorH;
  noise[0] +=//frame harmonic 2
    ( state.motorImbalance[0].x * dmgFactor[0] +
      state.motorImbalance[1].x * dmgFactor[1] +
      state.motorImbalance[2].x * dmgFactor[2] +
      state.motorImbalance[3].x * dmgFactor[3]
    ) * 0.25f * state.frameHarmonic2Amp * sinf(frameHarmonicPhase2)  * rpmFactorH2Inv * rpmFactorH;

  noise[1] += //frame harmonic 1
    ( state.motorImbalance[0].y * dmgFactor[0] +
      state.motorImbalance[1].y * dmgFactor[1] +
      state.motorImbalance[2].y * dmgFactor[2] +
      state.motorImbalance[3].y * dmgFactor[3]
    ) * 0.25f * state.frameHarmonic1Amp * cosf(frameHarmonicPhase1) * rpmFactorH1Inv * rpmFactorH;
  noise[1] += //frame harmonic 2
    ( state.motorImbalance[0].y * dmgFactor[0] +
      state.motorImbalance[1].y * dmgFactor[1] +
      state.motorImbalance[2].y * dmgFactor[2] +
      state.motorImbalance[3].y * dmgFactor[3]
    ) * 0.25f * state.frameHarmonic2Amp * cosf(frameHarmonicPhase2)  * rpmFactorH2Inv * rpmFactorH;

  noise[2] += //frame harmonic 1
    ( state.motorImbalance[0].z * dmgFactor[0] +
      state.motorImbalance[1].z * dmgFactor[1] +
      state.motorImbalance[2].z * dmgFactor[2] +
      state.motorImbalance[3].z * dmgFactor[3]
    ) * 0.25f * state.frameHarmonic1Amp * sinf(frameHarmonicPhase1) * cosf(frameHarmonicPhase1) * rpmFactorH1Inv * rpmFactorH;
  noise[2] += //frame harmonic 2
    ( state.motorImbalance[0].z * dmgFactor[0] +
      state.motorImbalance[1].z * dmgFactor[1] +
      state.motorImbalance[2].z * dmgFactor[2] +
      state.motorImbalance[3].z * dmgFactor[3]
    ) * 0.25f * state.frameHarmonic2Amp * sinf(frameHarmonicPhase2) * cosf(frameHarmonicPhase2) * rpmFactorH2Inv * rpmFactorH;

  //BF_DEBUG_SET(bf::DEBUG_SIM, 3, (1.0f + noiseX) * 1000.0f);
  //BF_DEBUG_SET(bf::DEBUG_SIM, 2, m1Hz);

  angularNoise = noise;
}

void Sim::update_rotation(double dt, StatePacket& state) {
    vec3 angularVelocity;
    copy(angularVelocity, state.angularVelocity);
    const auto w = angularVelocity * dt;
    const mat3 W = {
      vec3{    1, -w[2],  w[1]}, 
      vec3{ w[2],     1, -w[0]}, 
      vec3{-w[1],  w[0],     1}
    };
    
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
    float areaLinear = dot(frameDragArea, abs(local_dir));
    float areaAngular = dot(frameDragArea, local_dir);

    vec3 dragDir = dir * 0.5f * AIR_RHO * vel2 * initPacket.frameDragConstant;
    vec3 dragLinear = dragDir * areaLinear;
    vec3 dragAngular = dragDir * areaAngular;
    total_force = total_force - dragLinear;

    // motors:
    for (auto i = 0u; i < 4; i++) {
      total_force = total_force + xform(rotation, vec3{0, motors[i].thrust, 0});
    }

    acceleration = total_force / std::max(initPacket.quadMass, 0.001f);

    linearVelocity = linearVelocity + acceleration * dt;
    copy(state.linearVelocity, linearVelocity);
    
    assert(std::isfinite(length(linearVelocity)));

    // moment sum around origin:
    vec3 total_moment = get_axis(rotation, 1) * motorsTorque;
    
    // drag induced momentum
    angularDrag = xform_inv(rotation, dragAngular) * 0.001f;
    total_moment = total_moment + get_axis(rotation, 0) * angularDrag[1];
    total_moment = total_moment + get_axis(rotation, 1) * angularDrag[0];
    total_moment = total_moment + get_axis(rotation, 2) * angularDrag[2];

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
    inv_tensor = rotation * inv_tensor * transpose(rotation);
    vec3 angularAcc = xform(inv_tensor, total_moment);
    assert(std::isfinite(angularAcc[0]) && std::isfinite(angularAcc[1]) &&
           std::isfinite(angularAcc[2]));

    vec3 angularVelocity;
    copy(angularVelocity, state.angularVelocity);       // test drag rotation
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
  : recv_state_socket(kissnet::endpoint("localhost", 30713))
  , recv_rcdat_socket(kissnet::endpoint("localhost", 30714))
  , send_state_socket(kissnet::endpoint("localhost", 30715)) 
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

  // is calculated now
  batVoltage = 0.1f;
  batVoltageSag = batVoltage;
  batCapacity = initPacket.quadBatCapacityCharged;

  for (auto i = 0u; i < 4; i++) {
    motorsState[i].position[0] = initPacket.quadMotorPos[i].x;
    motorsState[i].position[1] = initPacket.quadMotorPos[i].y;
    motorsState[i].position[2] = initPacket.quadMotorPos[i].z;
  }

  fmt::print("Initializing betaflight\n");
  initPacket.eepromName[31] = '\0';
  bf::setEepromFileName((char *) initPacket.eepromName);
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
  if((receivedStatePacketQueue.size() < maxQueueSize)){
    receivedStatePacketQueue.push(state);
  }

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
  dyad_update();

  std::lock_guard<std::mutex> guard(statePacketMutex);
  while(receivedStatePacketQueue.size() > 0){

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
    
    if((statePacketUpdate.commands & CommandType::Repair) == CommandType::Repair){
      //recharge
      batCapacity = initPacket.quadBatCapacityCharged;
    }

    if((statePacketUpdate.commands & CommandType::Reset) == CommandType::Reset){
      //reset physics
      statePacket.linearVelocity = {0.0f, 0.0f, 0.0f};
      statePacket.angularVelocity = {0.0f, 0.0f, 0.0f};
    }

    total_delta += static_cast<uint64_t>(stateUpdateDelta);

    //angular velocity update
    vec3 currentAngularVelocity;
    copy(currentAngularVelocity, statePacket.angularVelocity);

    vec3 newAngularVelocity;
    copy(newAngularVelocity, statePacketUpdate.angularVelocity);

    vec3 angularVelocityDiff = newAngularVelocity - currentAngularVelocity;

    //BF_DEBUG_SET(bf::DEBUG_SIM, 0, static_cast<int16_t>(angularVelocityDiff[0] * 1000.0f));
    //BF_DEBUG_SET(bf::DEBUG_SIM, 1, static_cast<int16_t>(angularVelocityDiff[1] * 1000.0f));
    //BF_DEBUG_SET(bf::DEBUG_SIM, 2, static_cast<int16_t>(angularVelocityDiff[2] * 1000.0f));
    //BF_DEBUG_SET(bf::DEBUG_SIM, 3, static_cast<int16_t>(statePacketUpdate.delta * 1000.0f));

    float i_angular = (statePacketUpdate.contact == 1) ? 0.5f : 0.0f;
    //linear interpolation, strength defined by diff between states
    vec3 angularVelocityMixed = currentAngularVelocity + angularVelocityDiff * i_angular;

    // update state
    copy(statePacket.angularVelocity, angularVelocityMixed);


    //linear velocity update
    vec3 currentLinearVelocity;
    copy(currentLinearVelocity, statePacket.linearVelocity);

    vec3 newLinearVelocity;
    copy(newLinearVelocity, statePacketUpdate.linearVelocity);

    vec3 linearVelocityDiff = newLinearVelocity - currentLinearVelocity;

    float i_linear = (statePacketUpdate.contact == 1) ? 0.5f : 0.0f;

    //linear interpolation, strength defined by diff between states
    vec3 linearVelocityMixed = currentLinearVelocity + linearVelocityDiff * i_linear;

    copy(statePacket.linearVelocity, linearVelocityMixed);

    statePacket.vbat            = statePacketUpdate.vbat;

    statePacket.motorImbalance[0] = statePacketUpdate.motorImbalance[0];
    statePacket.motorImbalance[1] = statePacketUpdate.motorImbalance[1];
    statePacket.motorImbalance[2] = statePacketUpdate.motorImbalance[2];
    statePacket.motorImbalance[3] = statePacketUpdate.motorImbalance[3];

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

    statePacket.groundEffect[0] = statePacketUpdate.groundEffect[0];
    statePacket.groundEffect[1] = statePacketUpdate.groundEffect[1];
    statePacket.groundEffect[2] = statePacketUpdate.groundEffect[2];
    statePacket.groundEffect[3] = statePacketUpdate.groundEffect[3];

    statePacket.position = statePacketUpdate.position;

    memcpy( statePacket.rotation, statePacketUpdate.rotation, 3 * sizeof(Vec3F) );
  
    receivedStatePacketQueue.pop();

    armingDisabledFlags = (int)bf::getArmingDisableFlags();

    //rc data is updated independently

    simStep();

    mat3 basis;
    copy(basis, statePacket.rotation);
    quat orientation = mat3_to_quat(basis);
    stateUpdate.orientation.w = orientation[3];
    stateUpdate.orientation.x = orientation[0];
    stateUpdate.orientation.y = orientation[1];
    stateUpdate.orientation.z = orientation[2];
    stateUpdate.angularVelocity = statePacket.angularVelocity;
    stateUpdate.linearVelocity = statePacket.linearVelocity;
    stateUpdate.motorRpm[0] = motorsState[0].rpm;
    stateUpdate.motorRpm[1] = motorsState[1].rpm;
    stateUpdate.motorRpm[2] = motorsState[2].rpm;
    stateUpdate.motorRpm[3] = motorsState[3].rpm;

    if(sendStateUpdatePacketQueue.size() < maxQueueSize){
      sendStateUpdatePacketQueue.push(stateUpdate);
    }


    //copy osd and check of change
    bool osdChanged = false;
    for (int y = 0; y < VIDEO_LINES; y++) {
      for (int x = 0; x < CHARS_PER_LINE; x++) {
        //TODO: access to osdScreen has to be guarded
        if(osdUpdate.osd[y * CHARS_PER_LINE + x] != bf::osdScreen[y][x]){
          osdChanged = true;
        }
        osdUpdate.osd[y * CHARS_PER_LINE + x] = bf::osdScreen[y][x];
      }
    }

    //prepare update
    if (osdChanged && (sendStateOsdUpdatePacketQueue.size() < maxQueueSize)) {
      sendStateOsdUpdatePacketQueue.push(osdUpdate);
    } 
  }

  return running;
}

bool Sim::simStep() {
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

    set_gyro(dt, statePacket, acceleration, combinedGyroNoise);

    if (sleep_timer > 0) {
      sleep_timer -= DELTA;
      sleep_timer = std::max(int64_t(0), sleep_timer);
    } else {
      bf::scheduler();
      bfSchedules++;
    }

    simSteps++;

    float motorsTorque = calculate_motors(dt, statePacket, motorsState);
    acceleration = calculate_physics(dt, statePacket, motorsState, motorsTorque);

    updateBat(dt);
  }

  return true;
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

}
