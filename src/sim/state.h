#ifndef SIMITL_STATE_H
#define SIMITL_STATE_H
#include "util/vector_math.h"
#include "util/LowPassFilter.h"
#include "network/packets.h"

#include <array>
#include <cstdint>


namespace SimITL{

  // copy helper
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

  // per motor realtime state
  struct MotorState {
    vec3 position = {0.0f, 0.0f, 0.0f};
    // pwm signal in percent [0,1]
    float pwm = 0.0f;
    //low pass filtered pwm value
    LowPassFilter pwmLowPassFilter{};
    // motor core temp in deg C
    float temp = 0.0f;
    // current running through motor in Amps
    float current = 0.0f;
    // motor revolutions per minute
    float rpm = 0.0f;
    // thrust output of motor / propeller combo
    float thrust = 0.0f;
    // motor torque
    float mTorque = 0.0f;
    // propeller torque, counter acting motor torque
    float pTorque = 0.0f;

    // low pass filtered prop wash
    LowPassFilter propWashLowPassFilter{};

    // sinusoidal phase of the motor rotation used for noise simulation
    float phase = 0.0f;
    // phase freq * 2
    float phaseHarmonic1 = 0.0f;
    // phase freq * 3
    float phaseHarmonic2 = 0.0f;
    // phase freq * 0.01f
    float phaseSlow = 0.0f;

    // is the motor destroyed by over temp
    bool burnedOut = false;
  };

  struct BatteryState {
    // current battery voltage
    float batVoltage    = 0.0f; // in V
    // current sagged battery voltage
    float batVoltageSag = 0.0f; // in V
    // current battery capacity
    double batCapacity  = 0.0f; // in mAh

    // current amp draw in amps
    double amperage = 0.0f;
    // current mAh drawn from battery
    double mAhDrawn = 0.0f;
  };

  /**
   * \brief Stores the state of the simulation including incoming and 
   * outgoing network packets.
   */
  struct SimState {
    // initial quad/physics params
    StateInit stateInit {};

    // current internal state
    StateInput stateInput {};

    // outgoing packets
    StateOutput stateOutput {};

    bool armed = false;
    int armingDisabledFlags = 0;

    bool beep = false;

    // rc data
    uint16_t rcData[16] {};
    uint32_t rcDataReceptionTimeUs = 0U;

    std::array<MotorState, 4> motorsState {};
    BatteryState batteryState {};

    vec3 acceleration{0, 0, 0};

    vec3 gyroNoise{0, 0, 0};
    vec3 motorNoise{0, 0, 0};
    vec3 combinedNoise{0, 0, 0};

    LowPassFilter gyroLowPassFilter[3]{};

    // gyro / acc
    quat rotation{1, 0, 0, 1};
    vec3 gyro{0, 0, 0};
    vec3 acc{0, 0, 0};

    float frameHarmonicPhase1 = 0.0f;
    float frameHarmonicPhase2 = 0.0f;

    // time passed in micro seconds
    uint64_t microsPassed = 0;
  };

} // SIMITL_STATE_H

#endif