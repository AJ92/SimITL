#pragma once

/**
 * \file config.h
 * \brief Header-only quadcopter configuration loader.
 *
 * Loads a quad JSON config (e.g. config/quad/vtx-slayer-one-4.json) and
 * resolves references to motor, propeller, battery config files by
 * matching their \c name field.  Returns a \c QuadConfig struct whose
 * nested sub-structs mirror the JSON file hierarchy.
 *
 * \code
 *   auto cfg = loadQuadConfig("config/quad/vtx-slayer-one-4.json");
 *
 *   // Populate simulation structs
 *   StateInit  si;   cfg.applyTo(si);
 *   StateInput si2;  cfg.applyInputDefaultsTo(si2);
 * \endcode
 */

#include <string>
#include <filesystem>
#include <stdexcept>
#include <fmt/format.h>

#include "network/packets.h"
#include "json.h"

// ============================================================================
// QuadConfig – structured representation of all config parameters
// ============================================================================

/// Parameters loaded from a motor JSON config file.
struct MotorConfig {
  float kv        = 3300.0f;
  float r         = 0.135f;
  float i0        = 0.8f;
  float rth       = 12.0f;
  float cth       = 9.5f;
  float maxT      = 128.0f;
  Vec3F imbalance = { 13.0f, 7.0f, 5.0f };
  float mass      = 0.019f;  ///< mass of a single motor (kg)
};

/// Parameters loaded from a propeller JSON config file.
struct PropellerConfig {
  int    bladeCount       = 3;
  float  maxRpm           = 36000.0f;
  float  aFactor          = 6.25e-9f;
  float  torqueFactor     = 0.009f;
  float  inertia          = 0.00000245f;
  Vec3F  thrustFactor     = { -0.0000111f, -0.14f, 9.55f };
  float  harmonic1Amp     = 0.1f;
  float  harmonic2Amp     = 0.3f;
  float  mass             = 0.0028f;  ///< mass of a single propeller (kg)
};

/// Parameters loaded from a battery JSON config file.
struct BatteryConfig {
  float maxVoltageSag        = 1.3f;
  int   cellCount            = 4;
  float capacityCharged      = 850.0f;
  float capacity             = 850.0f;
  float mass                 = 0.11f;  ///< mass of the battery (kg)
};

/// Parameters loaded from the quad JSON config file itself (frame + sensors).
struct FrameConfig {
  // -- Frame aerodynamics / mass -------------------------------------------
  Vec3F dragArea     = { 0.008f, 0.0077f, 0.008f };
  float dragConstant = 1.2f;
  float mass         = 0.10f;  ///< frame-only mass (kg); total = frame + 4*motor + 4*prop + battery
  // y axis inertia has to be flipped to prevent freak outs (runaway)
  Vec3F invInertia   = { 900.0f, -480.0f, 900.0f };

  // -- Motor positions (4 motors) ------------------------------------------
  Vec3F motorPos[4] = {
    {  0.067f, 0.00498f, -0.0555f },
    {  0.067f, 0.00498f,  0.0555f },
    { -0.067f, 0.00498f, -0.0555f },
    { -0.067f, 0.00498f,  0.0555f },
  };

  float motorDir[4] = { -1.0f, 1.0f, 1.0f, -1.0f};

  /// Per-motor KV variance multiplier (from quad config, optional).
  /// All ones means no variance.
  Vec4F motorVariance = { 1.0f, 1.0f, 1.0f, 1.0f };

  // -- Prop wash -----------------------------------------------------------
  float minPropWashSpeed      = 1.0f;
  float maxPropWashSpeed      = 9.0f;
  float propWashAngleOfAttack = 0.14f;
  float propWashFactor        = 1.0f;

  // -- Gyro noise (applied at runtime) -------------------------------------
  float gyroBaseNoiseAmp  = 0.015f;
  float gyrobaseNoiseFreq = 333.0f;

  // -- Frame harmonics (applied at runtime) --------------------------------
  float frameHarmonic1Amp  = 0.0f;
  float frameHarmonic1Freq = 0.0f;
  float frameHarmonic2Amp  = 0.0f;
  float frameHarmonic2Freq = 0.0f;

  // -- Misc ----------------------------------------------------------------
  float ambientTemp = 25.0f;
};

/// Complete quadcopter configuration assembled from all JSON files.
struct QuadConfig {
  FrameConfig     frame;
  MotorConfig     motor;
  PropellerConfig propeller;
  BatteryConfig   battery;

  // -- Convenience: apply to simulation structs ----------------------------

  /// Populate a StateInit from this configuration.
  void applyTo(StateInit& s) const {
    // Frame
    s.frameDragArea     = frame.dragArea;
    s.frameDragConstant = frame.dragConstant;
    // Total mass = frame + 4 motors + 4 propellers + battery
    s.quadMass          = frame.mass
                        + 4.0f * motor.mass
                        + 4.0f * propeller.mass
                        + battery.mass;
    s.quadInvInertia    = frame.invInertia;
    // flip sign to acount of left/right hand coordinate system
    // quad will otherwise freak out (runaway)
    s.quadInvInertia.y = -s.quadInvInertia.y;

    // Prop wash
    s.minPropWashSpeed      = frame.minPropWashSpeed;
    s.maxPropWashSpeed      = frame.maxPropWashSpeed;
    s.propWashAngleOfAttack = frame.propWashAngleOfAttack;
    s.propWashFactor        = frame.propWashFactor;

    // Motor positions
    for (int i = 0; i < 4; i++)
      s.quadMotorPos[i] = frame.motorPos[i];

    for (int i = 0; i < 4; i++)
      s.quadMotorDir[i] = frame.motorDir[i];

    s.ambientTemp = frame.ambientTemp;

    // Motor – apply variance to differentiate individual motors
    {
      float baseKV = motor.kv;
      float baseR  = motor.r;
      float baseI0 = motor.i0;
      const float* var = &frame.motorVariance.x;
      for (int i = 0; i < 4; i++) {
        s.motorKV[i] = baseKV * var[i];
        s.motorR[i]  = baseR;
        s.motorI0[i] = baseI0;
      }
      s.motorRth  = motor.rth;
      s.motorCth  = motor.cth;
      s.motorMaxT = motor.maxT;
    }

    // Propeller
    s.propBladeCount   = static_cast<uint8_t>(propeller.bladeCount);
    s.propMaxRpm       = propeller.maxRpm;
    s.propAFactor      = propeller.aFactor;
    s.propTorqueFactor = propeller.torqueFactor;
    s.propInertia      = propeller.inertia;
    s.propThrustFactor = propeller.thrustFactor;
    s.propHarmonic1Amp = propeller.harmonic1Amp;
    s.propHarmonic2Amp = propeller.harmonic2Amp;

    // Battery
    s.maxVoltageSag       = battery.maxVoltageSag;
    s.quadBatCellCount    = static_cast<uint8_t>(battery.cellCount);
    s.quadBatCapacityCharged = battery.capacityCharged;
    s.quadBatCapacity     = battery.capacity;
  }

  /// Populate config-derived StateInput fields (gyro noise, frame
  /// harmonics, motor imbalance, battery voltage).
  /// Call this after initInputRuntimeDefaults() to overlay the values
  /// that come from configuration.
  void applyInputDefaultsTo(StateInput& s) const {
    // Gyro noise
    s.gyroBaseNoiseAmp  = frame.gyroBaseNoiseAmp;
    s.gyrobaseNoiseFreq = frame.gyrobaseNoiseFreq;

    // Frame harmonics
    s.frameHarmonic1Amp  = frame.frameHarmonic1Amp;
    s.frameHarmonic1Freq = frame.frameHarmonic1Freq;
    s.frameHarmonic2Amp  = frame.frameHarmonic2Amp;
    s.frameHarmonic2Freq = frame.frameHarmonic2Freq;

    // Motor imbalance (same shape applied to all four motors)
    for (int i = 0; i < 4; i++)
      s.motorImbalance[i] = motor.imbalance;

    // Battery fully-charged voltage
    s.vbat = static_cast<float>(battery.cellCount) * 4.2f;
  }

  /// Print all configuration parameters to stdout.
  void print() const {
    float totalMass = frame.mass
                    + 4.0f * motor.mass
                    + 4.0f * propeller.mass
                    + battery.mass;

    fmt::print("-- Quad Config --\n");
    fmt::print("Frame:\n");
    fmt::print("  mass (dry):       {} kg\n", frame.mass);
    fmt::print("  invInertia:       ({}, {}, {})\n",
               frame.invInertia.x, frame.invInertia.y,
               frame.invInertia.z);
    fmt::print("  dragArea:         ({}, {}, {})\n",
               frame.dragArea.x, frame.dragArea.y,
               frame.dragArea.z);
    fmt::print("  dragConstant:     {}\n", frame.dragConstant);
    fmt::print("  gyroNoiseAmp:     {}\n", frame.gyroBaseNoiseAmp);
    fmt::print("  gyroNoiseFreq:    {} Hz\n", frame.gyrobaseNoiseFreq);
    fmt::print("  propWash:         min={}, max={}, aoa={}, factor={}\n",
               frame.minPropWashSpeed, frame.maxPropWashSpeed,
               frame.propWashAngleOfAttack,
               frame.propWashFactor);

    fmt::print("Motor:\n");
    fmt::print("  kv:               {}\n", motor.kv);
    fmt::print("  r:                {} ohm\n", motor.r);
    fmt::print("  i0:               {} A\n", motor.i0);
    fmt::print("  mass:             {} kg\n", motor.mass);
    fmt::print("  imbalance:        ({}, {}, {})\n",
               motor.imbalance.x, motor.imbalance.y,
               motor.imbalance.z);

    fmt::print("Propeller:\n");
    fmt::print("  blades:           {}\n", propeller.bladeCount);
    fmt::print("  maxRpm:           {}\n", propeller.maxRpm);
    fmt::print("  aFactor:          {}\n", propeller.aFactor);
    fmt::print("  torqueFactor:     {}\n", propeller.torqueFactor);
    fmt::print("  inertia:          {} kg*m^2\n", propeller.inertia);
    fmt::print("  thrustFactor:     ({}, {}, {})\n",
               propeller.thrustFactor.x,
               propeller.thrustFactor.y,
               propeller.thrustFactor.z);
    fmt::print("  mass:             {} kg\n", propeller.mass);

    fmt::print("Battery:\n");
    fmt::print("  cells:            {}S\n", battery.cellCount);
    fmt::print("  maxVoltageSag:    {} V\n", battery.maxVoltageSag);
    fmt::print("  capacity:         {} mAh\n",
               static_cast<int>(battery.capacityCharged));
    fmt::print("  mass:             {} kg\n", battery.mass);

    fmt::print("Total AUW:         {} kg\n", totalMass);
  }
};

// ============================================================================
// Internal helpers
// ============================================================================

namespace cfg_detail {

/// Find a config file inside \p directory whose \c name JSON field matches
/// \p targetName.  Returns the full path or throws if not found.
inline std::string findConfigByName(const std::string& directory,
                                    const std::string& targetName) {
  namespace fs = std::filesystem;

  if (!fs::is_directory(directory))
    throw std::runtime_error("Config directory not found: " + directory);

  for (auto& entry : fs::directory_iterator(directory)) {
    if (entry.path().extension() != ".json") continue;
    try {
      auto cfg = json::parseFile(entry.path().string());
      if (cfg.get("name").asString() == targetName)
        return entry.path().string();
    } catch (...) {
      // skip unparseable files
    }
  }
  throw std::runtime_error("Config not found matching \"" + targetName +
                           "\" in " + directory);
}

} // namespace cfg_detail

// ============================================================================
// Public API – loader function
// ============================================================================

/// Load a complete quadcopter configuration from a quad JSON file,
/// resolving all referenced component configs (motor, propeller, battery).
///
/// \param quadConfigPath  Path to the quad JSON file, e.g.
///                        "config/quad/vtx-slayer-one-4.json".
/// \return                Fully populated QuadConfig.
inline QuadConfig loadQuadConfig(const std::string& quadConfigPath) {
  namespace fs = std::filesystem;
  namespace cfg = cfg_detail;

  // -- Determine the config root directory --------------------------------
  // The quad config lives in <root>/quad/<name>.json, so the root is
  // the parent of the "quad" folder.
  fs::path quadPath(quadConfigPath);
  fs::path cfgRoot = quadPath.parent_path().parent_path();

  auto quad = json::parseFile(quadConfigPath);

  QuadConfig result;

  // -- Frame parameters ---------------------------------------------------
  result.frame.dragArea     = quad.get("frameDragArea").asVec3();
  result.frame.dragConstant = static_cast<float>(
      quad.get("frameDragConstant").asNumber(1.2));
  result.frame.mass         = static_cast<float>(
      quad.get("mass").asNumber(0.29));
  result.frame.invInertia   = quad.get("invInertia").asVec3();

  result.frame.minPropWashSpeed      = static_cast<float>(
      quad.get("minPropWashSpeed").asNumber(1.0));
  result.frame.maxPropWashSpeed      = static_cast<float>(
      quad.get("maxPropWashSpeed").asNumber(18.0));
  result.frame.propWashAngleOfAttack = static_cast<float>(
      quad.get("propWashAngleOfAttack").asNumber(0.5));
  result.frame.propWashFactor        = static_cast<float>(
      quad.get("propWashFactor").asNumber(1.0));

  // Motor positions
  result.frame.motorPos[0] = quad.get("motor1Pos").asVec3();
  result.frame.motorPos[1] = quad.get("motor2Pos").asVec3();
  result.frame.motorPos[2] = quad.get("motor3Pos").asVec3();
  result.frame.motorPos[3] = quad.get("motor4Pos").asVec3();

  result.frame.motorDir[0] = static_cast<float>(
      quad.get("motor1Dir").asNumber(-1.0));
  result.frame.motorDir[1] = static_cast<float>(
      quad.get("motor2Dir").asNumber(1.0));
  result.frame.motorDir[2] = static_cast<float>(
      quad.get("motor3Dir").asNumber(1.0));
  result.frame.motorDir[3] = static_cast<float>(
      quad.get("motor4Dir").asNumber(-1.0));

  // Motor variance (optional)
  if (quad.has("motorVariance"))
    result.frame.motorVariance = quad.get("motorVariance").asVec4();

  // Gyro noise
  result.frame.gyroBaseNoiseAmp  = static_cast<float>(
      quad.get("gyroBaseNoiseAmp").asNumber(0.015));
  result.frame.gyrobaseNoiseFreq = static_cast<float>(
      quad.get("gyrobaseNoiseFreq").asNumber(333.0));

  // Frame harmonics
  result.frame.frameHarmonic1Amp  = static_cast<float>(
      quad.get("frameHarmonic1Amp").asNumber(0.0));
  result.frame.frameHarmonic1Freq = static_cast<float>(
      quad.get("frameHarmonic1Freq").asNumber(0.0));
  result.frame.frameHarmonic2Amp  = static_cast<float>(
      quad.get("frameHarmonic2Amp").asNumber(0.0));
  result.frame.frameHarmonic2Freq = static_cast<float>(
      quad.get("frameHarmonic2Freq").asNumber(0.0));

  // Ambient temperature
  result.frame.ambientTemp = static_cast<float>(
      quad.get("ambientTemp").asNumber(25.0));

  // -- Motor config -------------------------------------------------------
  std::string motorName = quad.get("motor").asString();
  if (!motorName.empty()) {
    auto motorPath = cfg::findConfigByName(
        (cfgRoot / "motor").string(), motorName);
    auto motor = json::parseFile(motorPath);

    result.motor.kv  = static_cast<float>(
        motor.get("motorKV").asNumber(3300.0));
    result.motor.r   = static_cast<float>(
        motor.get("motorR").asNumber(0.135));
    result.motor.i0  = static_cast<float>(
        motor.get("motorI0").asNumber(0.8));
    result.motor.rth = static_cast<float>(
        motor.get("motorRth").asNumber(12.0));
    result.motor.cth = static_cast<float>(
        motor.get("motorCth").asNumber(9.5));
    result.motor.maxT = static_cast<float>(
        motor.get("motorMaxT").asNumber(128.0));

    result.motor.imbalance = motor.get("motorImbalance").asVec3();
    result.motor.mass      = static_cast<float>(
        motor.get("mass").asNumber(0.019));
  }

  // -- Propeller config ---------------------------------------------------
  std::string propName = quad.get("propeller").asString();
  if (!propName.empty()) {
    auto propPath = cfg::findConfigByName(
        (cfgRoot / "propeller").string(), propName);
    auto prop = json::parseFile(propPath);

    result.propeller.bladeCount   = prop.get("bladeCount").asInt(3);
    result.propeller.maxRpm       = static_cast<float>(
        prop.get("propMaxRpm").asNumber(36000.0));
    result.propeller.aFactor      = static_cast<float>(
        prop.get("propAFactor").asNumber(6.25e-9));
    result.propeller.torqueFactor = static_cast<float>(
        prop.get("propTorqueFactor").asNumber(0.009));
    result.propeller.inertia      = static_cast<float>(
        prop.get("propInertia").asNumber(0.00000245));
    result.propeller.thrustFactor = prop.get("propThrustFactor").asVec3();
    result.propeller.harmonic1Amp = static_cast<float>(
        prop.get("propHarmonic1Amp").asNumber(0.1));
    result.propeller.harmonic2Amp = static_cast<float>(
        prop.get("propHarmonic2Amp").asNumber(0.3));
    result.propeller.mass        = static_cast<float>(
        prop.get("mass").asNumber(0.0028));
  }

  // -- Battery config -----------------------------------------------------
  std::string batName = quad.get("battery").asString();
  if (!batName.empty()) {
    auto batPath = cfg::findConfigByName(
        (cfgRoot / "battery").string(), batName);
    auto bat = json::parseFile(batPath);

    result.battery.maxVoltageSag   = static_cast<float>(
        bat.get("maxVoltageSag").asNumber(1.3));
    result.battery.cellCount       = bat.get("batCellCount").asInt(4);
    result.battery.capacityCharged = static_cast<float>(
        bat.get("batCapacityCharged").asNumber(850.0));
    result.battery.capacity        = static_cast<float>(
        bat.get("batCapacity").asNumber(850.0));
    result.battery.mass           = static_cast<float>(
        bat.get("mass").asNumber(0.11));
  }

  return result;
}
