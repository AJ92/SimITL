#include <fmt/format.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <stdexcept>
#include <cstring>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

#include "network/packets.h"
#include "simitl.h"
#include "ghost.h"
#include "blackbox.h"
#include "config.h"

// ============================================================================
// Global simulation state
// ============================================================================

std::thread t{};
bool running = true;

StateInit stateInit = {};
StateInput stateInput = {};
StateOutput stateOutput = {};

float simulationDeltaSec = 0.001f;

uint64_t currentFrame = 1U;
uint64_t framePrintOsd = 100U;

// Ghost recording playback state
GhostData ghost;
uint64_t ghostSampleIndex = 0;
float waitSec = 8.0f;               // boot grace time wait (in seconds)
constexpr float armingSec = 0.1f;    // arm with throttle low before playback

// Blackbox playback state (for "bb" mode)
BlackboxData bbPlaybackData;
uint64_t bbPlaybackFrameIndex = 0;

// Path to the quad configuration JSON (set via CLI or default)
std::string quadConfigPath = "config/quad/vtx-slayer-one-4.json";

/// Fast-forward mode: if true, playback runs without sleeping (full speed).
bool fastForward = false;

/// Skip the 3-second startup summary pause.
bool noStartupSleep = false;

bool noOsd = false;



// Loaded quad configuration (populated once in main(), used by playback)
QuadConfig quadCfg;

/// Tracks the quadcopter's output state from the simulation.
struct QuadState {
  Vec3F position;         ///< World-space position (integrated from linear velocity)
  Vec4F orientation;      ///< Orientation as quaternion (from simulation output)
  Vec3F linearVelocity;   ///< Linear velocity (from simulation output)
  Vec3F angularVelocity;  ///< Angular velocity (from simulation output)
};

QuadState quadState = {};

/// Blackbox RC commands scaled to [-1, 1] for CLI display in bb/olbb modes.
float bbRcData[8] = {0.0f, 0.0f, -1.0f, 0.0f, -1.0f, -1.0f, -1.0f, -1.0f};

// ============================================================================
// Shared helper functions
// ============================================================================

void printOsdToCli()
{
  if(currentFrame % framePrintOsd != 0){
    return;
  }

  fmt::print("\n");
  for (int l = 0; l < 16; l++)
  {
    for (int c = 0; c < 30; c++)
    {
      uint8_t v = stateOutput.osd[(l * 30) + c];
      if (std::isprint(v))
      {
        std::cout << v;
      }
      else
      {
        std::cout << " ";
      }
    }
    fmt::print("\n");
  }
}

void printQuadStateToCli()
{
  if (currentFrame % framePrintOsd != 0) {
    return;
  }

  // Use blackbox RC commands for display in bb/olbb modes
  float* rcPrint = (!bbPlaybackData.frames.empty()) ? bbRcData : stateInput.rcData;

  // Convert quaternion (x, y, z, w) to YXZ Euler angles (roll, pitch, yaw)
  float qx = quadState.orientation.x;
  float qy = quadState.orientation.y;
  float qz = quadState.orientation.z;
  float qw = quadState.orientation.w;

  double roll  = std::atan2(2.0 * (qw * qx - qy * qz),
                            1.0 - 2.0 * (qx * qx + qz * qz));
  double pitch = std::asin(std::clamp(2.0 * (qx * qy + qz * qw), -1.0, 1.0));
  double yaw   = std::atan2(2.0 * (qy * qw - qx * qz),
                            1.0 - 2.0 * (qy * qy + qz * qz));

  double rollDeg  = roll  * 180.0 / M_PI;
  double pitchDeg = pitch * 180.0 / M_PI;
  double yawDeg   = yaw   * 180.0 / M_PI;

  fmt::print("rc=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}]\n"
             "pos=({:.2f}, {:.2f}, {:.2f})\n"
             "vel=({:.1f}, {:.1f}, {:.1f})\n"
             "euler=(r:{:.1f} p:{:.1f} y:{:.1f})\n",
             rcPrint[0], rcPrint[1], rcPrint[2], rcPrint[3],
             rcPrint[4], rcPrint[5], rcPrint[6], rcPrint[7],
             quadState.position.x, quadState.position.y, quadState.position.z,
             quadState.linearVelocity.x, quadState.linearVelocity.y, quadState.linearVelocity.z,
             rollDeg, pitchDeg, yawDeg
          );
}

void initInputRuntimeDefaults(StateInput& s)
{
  s.delta = simulationDeltaSec;

  // RC channels: -1.0 to 1.0 (mapped internally to Betaflight 1000-2000 range)
  s.rcData[0] =  0.0f; // roll    (centered)
  s.rcData[1] =  0.0f; // pitch   (centered)
  s.rcData[2] = -1.0f; // throttle (low)
  s.rcData[3] =  0.0f; // yaw     (centered)
  s.rcData[4] = -1.0f; // arm switch (disarmed)
  s.rcData[5] = -1.0f;
  s.rcData[6] = -1.0f;
  s.rcData[7] = -1.0f;

  s.motorPwm[0] = -1.0f;
  s.motorPwm[1] = -1.0f;
  s.motorPwm[2] = -1.0f;
  s.motorPwm[3] = -1.0f;

  // Identity rotation matrix
  s.rotation[0] = { 1.0f, 0.0f, 0.0f };
  s.rotation[1] = { 0.0f, 1.0f, 0.0f };
  s.rotation[2] = { 0.0f, 0.0f, 1.0f };

  s.position.x = 0.0f;
  s.position.y = 0.0f;
  s.position.z = 0.0f;

  s.angularVelocity.x = 0.0f;
  s.angularVelocity.y = 0.0f;
  s.angularVelocity.z = 0.0f;

  s.linearVelocity.x = 0.0f;
  s.linearVelocity.y = 0.0f;
  s.linearVelocity.z = 0.0f;

  for (int i = 0; i < 4; i++) {
    s.propDamage[i]   = 0.0f;
    s.groundEffect[i] = 0.0f;
  }

  s.contact = 0;

  s.openLoop = 0;
}

void applyGhostPose(const GhostFrame& frame, StateInput& input)
{
  input.position.x = static_cast<float>(frame.position.x);
  input.position.y = static_cast<float>(frame.position.y);
  input.position.z = static_cast<float>(frame.position.z);

  float qx = static_cast<float>(frame.orientation.x);
  float qy = static_cast<float>(frame.orientation.y);
  float qz = static_cast<float>(frame.orientation.z);
  float qw = static_cast<float>(frame.orientation.w);

  float xx = qx * qx, xy = qx * qy, xz = qx * qz, xw = qx * qw;
  float yy = qy * qy, yz = qy * qz, yw = qy * qw;
  float zz = qz * qz, zw = qz * qw;

  input.rotation[0] = { 1.0f - 2.0f * (yy + zz),        2.0f * (xy - zw),        2.0f * (xz + yw) };
  input.rotation[1] = {       2.0f * (xy + zw), 1.0f - 2.0f * (xx + zz),        2.0f * (yz - xw) };
  input.rotation[2] = {       2.0f * (xz - yw),        2.0f * (yz + xw), 1.0f - 2.0f * (xx + yy) };

  input.contact = 1;
}

void applyOrientationToInput(const Vec4F& quat, StateInput& input)
{
  float qx = quat.x;
  float qy = quat.y;
  float qz = quat.z;
  float qw = quat.w;

  float xx = qx * qx, xy = qx * qy, xz = qx * qz, xw = qx * qw;
  float yy = qy * qy, yz = qy * qz, yw = qy * qw;
  float zz = qz * qz, zw = qz * qw;

  input.rotation[0] = { 1.0f - 2.0f * (yy + zz),        2.0f * (xy - zw),        2.0f * (xz + yw) };
  input.rotation[1] = {       2.0f * (xy + zw), 1.0f - 2.0f * (xx + zz),        2.0f * (yz - xw) };
  input.rotation[2] = {       2.0f * (xz - yw),        2.0f * (yz + xw), 1.0f - 2.0f * (xx + yy) };
}

void initQuadStateFromGhost(QuadState& qs, const GhostData& ghost)
{
  if (ghost.samples.empty()) return;
  auto& first = ghost.samples.front();
  qs.position = {
    static_cast<float>(first.position.x),
    static_cast<float>(first.position.y),
    static_cast<float>(first.position.z)
  };
  qs.orientation = {
    static_cast<float>(first.orientation.x),
    static_cast<float>(first.orientation.y),
    static_cast<float>(first.orientation.z),
    static_cast<float>(first.orientation.w)
  };
}

void feedbackOutputToInput()
{
  applyOrientationToInput(stateOutput.orientation, stateInput);
  stateInput.angularVelocity = stateOutput.angularVelocity;
  stateInput.linearVelocity  = stateOutput.linearVelocity;
}

// ============================================================================
// Ghost playback helpers
// ============================================================================

static bool isPlaybackActive()
{
  return (currentFrame * simulationDeltaSec) >= waitSec && !ghost.samples.empty();
}

static bool isPoseLocked()
{
  return (currentFrame * simulationDeltaSec) < (waitSec + armingSec) && !ghost.samples.empty();
}

static void advanceGhostAndApplyRc()
{
  if (!isPlaybackActive())
    return;

  double playbackTime = (currentFrame * simulationDeltaSec) - waitSec;

  while (ghostSampleIndex + 1 < ghost.samples.size() &&
         ghost.samples[ghostSampleIndex + 1].time <= playbackTime)
  {
    ghostSampleIndex++;
  }

  auto& frame = ghost.samples[ghostSampleIndex];
  for (int i = 0; i < 8; i++) {
    stateInput.rcData[i] = frame.rcData[i];
  }
  stateInput.contact = 0;
}

static bool checkGhostEnd()
{
  if (!isPlaybackActive())
    return false;

  double playbackTime = (currentFrame * simulationDeltaSec) - waitSec;

  if (ghostSampleIndex >= ghost.samples.size() - 1 &&
      playbackTime >= ghost.samples.back().time)
  {
    fmt::print("reached end of ghost\n");
    running = false;
    return true;
  }
  return false;
}

static void overrideRcForPlaybackPhase()
{
  if (!isPlaybackActive())
    return;

  if ((currentFrame * simulationDeltaSec) < (waitSec + armingSec))
  {
    stateInput.rcData[4] = 1.0f;  // arm switch on
    stateInput.rcData[2] = -1.0f; // throttle low
  }
}

static void lockInitialPose()
{
  if (!isPoseLocked())
    return;

  initQuadStateFromGhost(quadState, ghost);
  applyGhostPose(ghost.samples.front(), stateInput);
}

static void updateQuadState()
{
  quadState.linearVelocity  = stateOutput.linearVelocity;
  quadState.angularVelocity = stateOutput.angularVelocity;
  quadState.orientation     = stateOutput.orientation;

  const float dt = simulationDeltaSec;
  quadState.position.x += quadState.linearVelocity.x * dt;
  quadState.position.y += quadState.linearVelocity.y * dt;
  quadState.position.z += quadState.linearVelocity.z * dt;
}

// ============================================================================
// Playback reset
// ============================================================================

void resetPlayback()
{
  currentFrame = 1U;
  ghostSampleIndex = 0;
  bbPlaybackFrameIndex = 0;
  running = true;
  quadState = {};
  initInputRuntimeDefaults(stateInput);
  quadCfg.applyInputDefaultsTo(stateInput);
}

// ============================================================================
// Ghost playback thread (existing mode)
// ============================================================================

void ghostPlaybackThread()
{
  resetPlayback();

  std::ofstream csvLog("quadstate.csv");
  csvLog << "frame,time,"
            "rc0,rc1,rc2,rc4,"
            "pos_x,pos_y,pos_z,"
            "ori_x,ori_y,ori_z,ori_w,"
            "vel_x,vel_y,vel_z,"
            "angvel_x,angvel_y,angvel_z,"
            "motorOut_1,motorOut_2,motorOut_3,motorOut_4\n";

  while (running)
  {
    advanceGhostAndApplyRc();

    if (checkGhostEnd())
      return;

    overrideRcForPlaybackPhase();
    lockInitialPose();

    simitl_update(stateInput);
    stateOutput = simitl_get_state();

    if (!isPoseLocked())
    {
      updateQuadState();
      feedbackOutputToInput();

      double csvTime = (currentFrame * simulationDeltaSec) - waitSec;
      if (isPlaybackActive())
        csvTime = ghost.samples[ghostSampleIndex].time;

      csvLog << currentFrame << ','
            << csvTime << ','
            << stateInput.rcData[0] << ',' << stateInput.rcData[1] << ','
            << stateInput.rcData[2] << ',' << stateInput.rcData[3] << ','
            << quadState.position.x << ',' << quadState.position.y << ',' << quadState.position.z << ','
            << quadState.orientation.x << ',' << quadState.orientation.y << ','
            << quadState.orientation.z << ',' << quadState.orientation.w << ','
            << quadState.linearVelocity.x << ',' << quadState.linearVelocity.y << ',' << quadState.linearVelocity.z << ','
            << quadState.angularVelocity.x << ',' << quadState.angularVelocity.y << ',' << quadState.angularVelocity.z << ','
            << stateOutput.motorOutput[0] * 2.0f - 1.0f << ',' << stateOutput.motorOutput[1] * 2.0f - 1.0f << ','
            << stateOutput.motorOutput[2] * 2.0f - 1.0f << ',' << stateOutput.motorOutput[3] * 2.0f - 1.0f << '\n';
    }

    if (!noOsd){
      printOsdToCli();
      printQuadStateToCli();
      fmt::print(".");
    }

    if (!fastForward) {
      int deltaMicroseconds = static_cast<int>(simulationDeltaSec * 1000 * 1000);
      std::this_thread::sleep_for(std::chrono::microseconds(deltaMicroseconds));
    }
    
    currentFrame++;
  }
}

// ============================================================================
// Idle simulation thread
// ============================================================================

/// Run the simulation loop with no external input. Just idles with default
/// (disarmed, throttle low, centered sticks) inputs.
void idleThread()
{
  resetPlayback();

  while (running)
  {
    stateInput.delta = 0.016f;
    simitl_update(stateInput);
    stateOutput = simitl_get_state();
    feedbackOutputToInput();
    updateQuadState();

    if (!noOsd){
      printOsdToCli();
      printQuadStateToCli();
      fmt::print(".");
    }

    int deltaMicroseconds = static_cast<int>(0.016f * 1000 * 1000);
    std::this_thread::sleep_for(std::chrono::microseconds(deltaMicroseconds));
    currentFrame++;
  }
}

// ============================================================================
// Blackbox RC conversion and playback helpers
// ============================================================================

/// Convert blackbox rcCommand to StateInput rcData.
/// rcCommand[0-3]: roll, pitch, yaw, throttle
///   roll/pitch/yaw: +/-500 -> rcData +/-1.0
///   throttle: 1000-2000 -> rcData -1.0 to 1.0
inline void applyBbRc(const BbFrame& frame, StateInput& input)
{
  // Roll: +/-500 -> +/-1.0
  input.rcData[0] = static_cast<float>(frame.rcCommand[0]) / 500.0f;
  // Pitch: +/-500 -> +/-1.0
  input.rcData[1] = static_cast<float>(frame.rcCommand[1]) / 500.0f;
  // Throttle: 1000-2000 -> -1.0 to 1.0
  input.rcData[2] = (static_cast<float>(frame.rcCommand[3]) - 1500.0f) / 500.0f;
  // Yaw: +/-500 -> +/-1.0
  input.rcData[3] = static_cast<float>(frame.rcCommand[2]) / 500.0f;

  // AUX channels: derive arm switch from blackbox flightModeFlags (bit 0 = ARM flag)
  input.rcData[4] = frame.isArmed() ? 1.0f : -1.0f;
  // rcData[5-7] stay at default (-1.0)
}

/// Extract blackbox rcCommand values scaled to [-1, 1] into a flat array (for display).
inline void extractBbRc(const BbFrame& frame, float* rcOut)
{
  rcOut[0] = static_cast<float>(frame.rcCommand[0]) / 500.0f;
  rcOut[1] = static_cast<float>(frame.rcCommand[1]) / 500.0f;
  rcOut[2] = (static_cast<float>(frame.rcCommand[3]) - 1500.0f) / 500.0f;
  rcOut[3] = static_cast<float>(frame.rcCommand[2]) / 500.0f;
  rcOut[4] = frame.isArmed() ? 1.0f : -1.0f;
  rcOut[5] = -1.0f;
  rcOut[6] = -1.0f;
  rcOut[7] = -1.0f;
}

// ============================================================================
// Blackbox playback helpers and thread
// ============================================================================

static bool isBbPlaybackActive()
{
  return (currentFrame * simulationDeltaSec) >= waitSec && !bbPlaybackData.frames.empty();
}

static void advanceBlackboxAndApplyRc()
{
  if (!isBbPlaybackActive())
    return;

  double playbackTime = (currentFrame * simulationDeltaSec) - waitSec;

  // Normalize blackbox times relative to first frame's time
  double t0 = static_cast<double>(bbPlaybackData.frames[0].time) / 1e6;

  while (bbPlaybackFrameIndex + 1 < bbPlaybackData.frames.size() &&
         (static_cast<double>(bbPlaybackData.frames[bbPlaybackFrameIndex + 1].time) / 1e6 - t0) <= playbackTime)
  {
    bbPlaybackFrameIndex++;
  }

  auto& frame = bbPlaybackData.frames[bbPlaybackFrameIndex];
  applyBbRc(frame, stateInput);
  extractBbRc(frame, bbRcData);
  stateInput.contact = 0;
}

static bool checkBlackboxEnd()
{
  if (!isBbPlaybackActive())
    return false;

  double playbackTime = (currentFrame * simulationDeltaSec) - waitSec;

  // Normalize blackbox time relative to first frame
  double t0 = static_cast<double>(bbPlaybackData.frames[0].time) / 1e6;
  double bbEndTime = static_cast<double>(bbPlaybackData.frames.back().time) / 1e6 - t0;

  if (bbPlaybackFrameIndex >= bbPlaybackData.frames.size() - 1 &&
      playbackTime >= bbEndTime)
  {
    fmt::print("reached end of blackbox playback\n");
    running = false;
    return true;
  }
  return false;
}

static void overrideRcForBbPlaybackPhase()
{
  if (!isBbPlaybackActive())
    return;

  // Always keep armed during playback — the blackbox was recorded while flying.
  // Without this, the FC disarms when pre-arm bench frames feed isArmed()=false.
  stateInput.rcData[4] = 1.0f;

  if ((currentFrame * simulationDeltaSec) < (waitSec + armingSec))
  {
    stateInput.rcData[2] = -1.0f; // throttle low during arming
  }
}

static bool isBbPoseLocked()
{
  return (currentFrame * simulationDeltaSec) < (waitSec + armingSec) && !bbPlaybackData.frames.empty();
}

static void lockBbInitialPose()
{
  if (!isBbPoseLocked())
    return;

  // Lock to identity orientation and origin position.
  // Blackbox has no pose data, so we keep the quad stationary at origin.
  stateInput.position = { 0.0f, 0.0f, 0.0f };
  stateInput.rotation[0] = { 1.0f, 0.0f, 0.0f };
  stateInput.rotation[1] = { 0.0f, 1.0f, 0.0f };
  stateInput.rotation[2] = { 0.0f, 0.0f, 1.0f };
  stateInput.contact = 1;
}

/// Blackbox playback thread: feeds blackbox RC commands into the simulation
/// (no tuning, no error metrics).
void blackboxPlaybackThread()
{
  resetPlayback();

  std::ofstream csvLog("quadstate_bb.csv");
  csvLog << "frame,time,"
            "rc0,rc1,rc2,rc3,"
            "pos_x,pos_y,pos_z,"
            "ori_x,ori_y,ori_z,ori_w,"
            "vel_x,vel_y,vel_z,"
            "angvel_x,angvel_y,angvel_z,"
            "motorOut_1,motorOut_2,motorOut_3,motorOut_4,"
            "bb_motorOut_1,bb_motorOut_2,bb_motorOut_3,bb_motorOut_4,"
            "bb_raw_gyro_x,bb_raw_gyro_y,bb_raw_gyro_z,"
            "bb_gyro_x,bb_gyro_y,bb_gyro_z,"
            "bb_acc_x,bb_acc_y,bb_acc_z\n";

  while (running)
  {
    advanceBlackboxAndApplyRc();

    if (checkBlackboxEnd()){
      return;
    }

    // Follow same pattern as ghost playback:
    // 1) overrideRc handles arm switch during arming period
    // 2) lockBbInitialPose holds the quad at origin with identity orientation
    //    and contact=1 during the first (waitSec + armingSec) seconds
    overrideRcForBbPlaybackPhase();
    lockBbInitialPose();

    simitl_update(stateInput);
    stateOutput = simitl_get_state();

    // During pose-locked phase, skip feedback so the sim doesn't overwrite
    // our locked rotation/position with potentially divergent dynamics.
    // After the lock period ends, feedback restores normal continuity.
    if (!isBbPoseLocked())
    {
      updateQuadState();
      feedbackOutputToInput();

      double csvTime = (currentFrame * simulationDeltaSec) - waitSec;
      if (isBbPlaybackActive())
        csvTime = static_cast<double>(bbPlaybackData.frames[bbPlaybackFrameIndex].time) / 1e6;

      auto& bbFrame = bbPlaybackData.frames[bbPlaybackFrameIndex];

      // Blackbox gyro values are already in deg/s (no gyroRawToRadps conversion needed)
      float bbRawGyroX = static_cast<float>(bbFrame.debug[0]);
      float bbRawGyroY = static_cast<float>(bbFrame.debug[1]);
      float bbRawGyroZ = static_cast<float>(bbFrame.debug[2]);

      float bbGyroX = static_cast<float>(bbFrame.gyroADC[0]);
      float bbGyroY = static_cast<float>(bbFrame.gyroADC[1]);
      float bbGyroZ = static_cast<float>(bbFrame.gyroADC[2]);

      float accScale = 1.0f / static_cast<float>(bbPlaybackData.header.acc1G) * 9.81f;
      float bbAccX = static_cast<float>(bbFrame.accSmooth[0]) * accScale;
      float bbAccY = static_cast<float>(bbFrame.accSmooth[1]) * accScale;
      float bbAccZ = static_cast<float>(bbFrame.accSmooth[2]) * accScale;

      int bbMotorMin = bbPlaybackData.header.motorOutputMin;
      int bbMotorMax = bbPlaybackData.header.motorOutputMax;
      float bbMotorScale = 2.0f / static_cast<float>(bbMotorMax - bbMotorMin);

      csvLog << currentFrame << ','
            << csvTime << ','
            << stateInput.rcData[0] << ',' << stateInput.rcData[1] << ','
            << stateInput.rcData[2] << ',' << stateInput.rcData[3] << ','
            << quadState.position.x << ',' << quadState.position.y << ',' << quadState.position.z << ','
            << quadState.orientation.x << ',' << quadState.orientation.y << ','
            << quadState.orientation.z << ',' << quadState.orientation.w << ','
            << quadState.linearVelocity.x << ',' << quadState.linearVelocity.y << ',' << quadState.linearVelocity.z << ','
            << quadState.angularVelocity.x * (180.0f / static_cast<float>(M_PI)) << ','
            << quadState.angularVelocity.y * (180.0f / static_cast<float>(M_PI)) << ','
            << quadState.angularVelocity.z * (180.0f / static_cast<float>(M_PI)) << ','
            << stateOutput.motorOutput[0] * 2.0f - 1.0f << ',' << stateOutput.motorOutput[1] * 2.0f - 1.0f << ','
            << stateOutput.motorOutput[2] * 2.0f - 1.0f << ',' << stateOutput.motorOutput[3] * 2.0f - 1.0f << ','
            << (static_cast<float>(bbFrame.motor[0]) - static_cast<float>(bbMotorMin)) * bbMotorScale - 1.0f << ','
            << (static_cast<float>(bbFrame.motor[1]) - static_cast<float>(bbMotorMin)) * bbMotorScale - 1.0f << ','
            << (static_cast<float>(bbFrame.motor[2]) - static_cast<float>(bbMotorMin)) * bbMotorScale - 1.0f << ','
            << (static_cast<float>(bbFrame.motor[3]) - static_cast<float>(bbMotorMin)) * bbMotorScale - 1.0f << ','
            << bbRawGyroX << ',' << bbRawGyroY << ',' << bbRawGyroZ << ','
            << bbGyroX << ',' << bbGyroY << ',' << bbGyroZ << ','
            << bbAccX << ',' << bbAccY << ',' << bbAccZ << '\n';
    }

    if (!noOsd) {
      printOsdToCli();
      printQuadStateToCli();
      fmt::print(".");
    }

    if (!fastForward) {
      int deltaMicroseconds = static_cast<int>(simulationDeltaSec * 1000 * 1000);
      std::this_thread::sleep_for(std::chrono::microseconds(deltaMicroseconds));
    }
    
    currentFrame++;
  }
}

// ============================================================================
// Open-loop blackbox playback helpers and thread
// ============================================================================

/// Apply blackbox motor values directly as open-loop PWM.
/// Converts from blackbox [motorOutputMin, motorOutputMax] range to physics [0, 1] range.
inline void applyBbMotorPwm(const BbFrame& frame, StateInput& input)
{
  int bbMotorMin = bbPlaybackData.header.motorOutputMin;
  int bbMotorMax = bbPlaybackData.header.motorOutputMax;
  float bbMotorRange = static_cast<float>(bbMotorMax - bbMotorMin);

  // Normalize motor values from [min, max] to [0.0, 1.0]
  // The physics engine expects pwm as a fraction of battery voltage (0 = off, 1 = full voltage).
  float rangeInv = (bbMotorRange > 0.0f) ? 1.0f / bbMotorRange : 1.0f;
  input.motorPwm[0] = (static_cast<float>(frame.motor[0]) - static_cast<float>(bbMotorMin)) * rangeInv;
  input.motorPwm[1] = (static_cast<float>(frame.motor[1]) - static_cast<float>(bbMotorMin)) * rangeInv;
  input.motorPwm[2] = (static_cast<float>(frame.motor[2]) - static_cast<float>(bbMotorMin)) * rangeInv;
  input.motorPwm[3] = (static_cast<float>(frame.motor[3]) - static_cast<float>(bbMotorMin)) * rangeInv;

  input.openLoop = 1;
  input.contact = 0;
}

static void advanceOlbbAndApplyMotorPwm()
{
  if (!isBbPlaybackActive())
    return;

  double playbackTime = (currentFrame * simulationDeltaSec) - waitSec;

  double t0 = static_cast<double>(bbPlaybackData.frames[0].time) / 1e6;

  while (bbPlaybackFrameIndex + 1 < bbPlaybackData.frames.size() &&
         (static_cast<double>(bbPlaybackData.frames[bbPlaybackFrameIndex + 1].time) / 1e6 - t0) <= playbackTime)
  {
    bbPlaybackFrameIndex++;
  }

  auto& frame = bbPlaybackData.frames[bbPlaybackFrameIndex];
  applyBbMotorPwm(frame, stateInput);
  extractBbRc(frame, bbRcData);
}

/// Open-loop blackbox playback thread: feeds blackbox motor values directly into
/// the physics engine, bypassing the Betaflight PID controller.
void blackboxOpenLoopThread()
{
  resetPlayback();

  std::ofstream csvLog("quadstate_olbb.csv");
  csvLog << "frame,time,"
            "rc0,rc1,rc2,rc3,"
            "pos_x,pos_y,pos_z,"
            "ori_x,ori_y,ori_z,ori_w,"
            "vel_x,vel_y,vel_z,"
            "angvel_x,angvel_y,angvel_z,"
            "motorOut_1,motorOut_2,motorOut_3,motorOut_4,"
            "bb_motorOut_1,bb_motorOut_2,bb_motorOut_3,bb_motorOut_4,"
            "bb_raw_gyro_x,bb_raw_gyro_y,bb_raw_gyro_z,"
            "bb_gyro_x,bb_gyro_y,bb_gyro_z,"
            "bb_acc_x,bb_acc_y,bb_acc_z\n";

  while (running)
  {
    advanceOlbbAndApplyMotorPwm();

    if (checkBlackboxEnd()){
      return;
    }

    lockBbInitialPose();

    simitl_update(stateInput);
    stateOutput = simitl_get_state();

    if (!isBbPoseLocked())
    {
      updateQuadState();

      //prevent speed from interfering with tuning
      stateOutput.linearVelocity.x = 0.0f;
      stateOutput.linearVelocity.y = 0.0f;
      stateOutput.linearVelocity.z = 0.0f;
      feedbackOutputToInput();

      double csvTime = (currentFrame * simulationDeltaSec) - waitSec;
      //if (isBbPlaybackActive())
      //  csvTime = static_cast<double>(bbPlaybackData.frames[bbPlaybackFrameIndex].time) / 1e6;

      auto& bbFrame = bbPlaybackData.frames[bbPlaybackFrameIndex];

      // Blackbox gyro values are already in deg/s (no gyroRawToRadps conversion needed)
      float bbRawGyroX = static_cast<float>(bbFrame.debug[0]);
      float bbRawGyroY = static_cast<float>(bbFrame.debug[1]);
      float bbRawGyroZ = static_cast<float>(bbFrame.debug[2]);

      float bbGyroX = static_cast<float>(bbFrame.gyroADC[0]);
      float bbGyroY = static_cast<float>(bbFrame.gyroADC[1]);
      float bbGyroZ = static_cast<float>(bbFrame.gyroADC[2]);

      float accScale = 1.0f / static_cast<float>(bbPlaybackData.header.acc1G) * 9.81f;
      float bbAccX = static_cast<float>(bbFrame.accSmooth[0]) * accScale;
      float bbAccY = static_cast<float>(bbFrame.accSmooth[1]) * accScale;
      float bbAccZ = static_cast<float>(bbFrame.accSmooth[2]) * accScale;

      int bbMotorMin = bbPlaybackData.header.motorOutputMin;
      int bbMotorMax = bbPlaybackData.header.motorOutputMax;
      float bbMotorScale = 2.0f / static_cast<float>(bbMotorMax - bbMotorMin);

      float olRc0 = static_cast<float>(bbFrame.rcCommand[0]) / 500.0f;
      float olRc1 = static_cast<float>(bbFrame.rcCommand[1]) / 500.0f;
      float olRc2 = (static_cast<float>(bbFrame.rcCommand[3]) - 1500.0f) / 500.0f;
      float olRc3 = static_cast<float>(bbFrame.rcCommand[2]) / 500.0f;

      csvLog << currentFrame << ','
            << csvTime << ','
            << olRc0 << ',' << olRc1 << ',' << olRc2 << ',' << olRc3 << ','
            << quadState.position.x << ',' << quadState.position.y << ',' << quadState.position.z << ','
            << quadState.orientation.x << ',' << quadState.orientation.y << ','
            << quadState.orientation.z << ',' << quadState.orientation.w << ','
            << quadState.linearVelocity.x << ',' << quadState.linearVelocity.y << ',' << quadState.linearVelocity.z << ','
            << quadState.angularVelocity.x * (180.0f / static_cast<float>(M_PI)) << ','
            << quadState.angularVelocity.y * (180.0f / static_cast<float>(M_PI)) << ','
            << quadState.angularVelocity.z * (180.0f / static_cast<float>(M_PI)) << ','
            << stateOutput.motorOutput[0] * 2.0f - 1.0f << ',' << stateOutput.motorOutput[1] * 2.0f - 1.0f << ','
            << stateOutput.motorOutput[2] * 2.0f - 1.0f << ',' << stateOutput.motorOutput[3] * 2.0f - 1.0f << ','
            << (static_cast<float>(bbFrame.motor[0]) - static_cast<float>(bbMotorMin)) * bbMotorScale - 1.0f << ','
            << (static_cast<float>(bbFrame.motor[1]) - static_cast<float>(bbMotorMin)) * bbMotorScale - 1.0f << ','
            << (static_cast<float>(bbFrame.motor[2]) - static_cast<float>(bbMotorMin)) * bbMotorScale - 1.0f << ','
            << (static_cast<float>(bbFrame.motor[3]) - static_cast<float>(bbMotorMin)) * bbMotorScale - 1.0f << ','
            << bbRawGyroX << ',' << bbRawGyroY << ',' << bbRawGyroZ << ','
            << bbGyroX << ',' << bbGyroY << ',' << bbGyroZ << ','
            << bbAccX << ',' << bbAccY << ',' << bbAccZ << '\n';
    }

    if (!noOsd) {
      printOsdToCli();
      printQuadStateToCli();
      fmt::print(".");
    }

    if (!fastForward) {
      int deltaMicroseconds = static_cast<int>(simulationDeltaSec * 1000 * 1000);
      std::this_thread::sleep_for(std::chrono::microseconds(deltaMicroseconds));
    }
   
    currentFrame++;
  }
}

// ============================================================================
// Blackbox trimming helpers
// ============================================================================

/// Trim blackbox frames based on start (skip first N seconds) and len
/// (only use N seconds total, from the effective start).
/// If both are 0 (default), no trimming is performed.
void trimBlackboxFrames(BlackboxData& bb, float startSec, float lenSec)
{
  if (bb.frames.empty())
    return;
  if (startSec <= 0.0f && lenSec <= 0.0f)
    return;

  double t0 = static_cast<double>(bb.frames[0].time) / 1e6;
  double effectiveStart = t0 + startSec;
  double effectiveEnd = (lenSec > 0.0f)
    ? (effectiveStart + lenSec)
    : std::numeric_limits<double>::max();

  std::vector<BbFrame> trimmed;
  trimmed.reserve(bb.frames.size());

  for (const auto& frame : bb.frames) {
    double t = static_cast<double>(frame.time) / 1e6;
    if (t >= effectiveStart && t < effectiveEnd) {
      trimmed.push_back(frame);
    }
  }

  fmt::print("Trimmed blackbox: skipped {} frames (range {:.3f}s to {:.3f}s, {} frames kept)\n",
             bb.frames.size() - trimmed.size(),
             effectiveStart, effectiveEnd,
             trimmed.size());

  bb.frames = std::move(trimmed);
}

/// Trim ghost samples based on start (skip first N seconds) and len
/// (only use N seconds total, from the effective start).
/// If both are 0 (default), no trimming is performed.
void trimGhostFrames(GhostData& ghost, float startSec, float lenSec)
{
  if (ghost.samples.empty())
    return;
  if (startSec <= 0.0f && lenSec <= 0.0f)
    return;

  double t0 = ghost.samples[0].time;
  double effectiveStart = t0 + startSec;
  double effectiveEnd = (lenSec > 0.0f)
    ? (effectiveStart + lenSec)
    : std::numeric_limits<double>::max();

  std::vector<GhostFrame> trimmed;
  trimmed.reserve(ghost.samples.size());

  for (const auto& frame : ghost.samples) {
    if (frame.time >= effectiveStart && frame.time < effectiveEnd) {
      trimmed.push_back(frame);
    }
  }

  fmt::print("Trimmed ghost: skipped {} frames (range {:.3f}s to {:.3f}s, {} frames kept)\n",
             ghost.samples.size() - trimmed.size(),
             effectiveStart, effectiveEnd,
             trimmed.size());

  ghost.samples = std::move(trimmed);
}

// ============================================================================
// Main entry point
// ============================================================================

int main(int argc, char* argv[])
{
  // Mode: "i" (idle/default), "g" (ghost), "bb" (blackbox), "olbb" (open-loop blackbox)
  std::string mode = "i";
  std::string ghostPath = "ghost.json";
  std::string bbPlaybackPath;
  float startSec    = 0.0f;
  float lenSec      = 0.0f;

  // Check for help flag before any mode processing
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      fmt::print("simitl-playback - quadcopter simulation playback tool\n");
      fmt::print("\n");
      fmt::print("Usage:\n");
      fmt::print("  simitl-playback [mode] [options] [file]\n");
      fmt::print("\n");
      fmt::print("Modes:\n");
      fmt::print("  i, idle               Run simulation idle (no input playback, default)\n");
      fmt::print("  g, ghost <path>       Playback a ghost.json recording\n");
      fmt::print("  bb, blackbox <path>   Playback a blackbox CSV recording\n");
      fmt::print("  olbb, openloop-blackbox <path>   Playback blackbox CSV in open-loop (bypass PID)\n");
      fmt::print("\n");
      fmt::print("Options:\n");
      fmt::print("  -h, --help            Show this help message and exit\n");
      fmt::print("  --config <path>       Quad config JSON path (default: config/quad/vtx-slayer-one-4.json)\n");
      fmt::print("  --start <sec>         Skip first N seconds of recording\n");
      fmt::print("  --len <sec>           Only play N seconds total from the effective start\n");
      fmt::print("  -ff                   Fast-forward (no sleep between sim steps)\n");
      fmt::print("  -ns, --no-startup-sleep  Skip 3-second startup pause\n");
      fmt::print("  -no, --no-osd            Skip OSD output to CLI\n");
      fmt::print("\n");
      fmt::print("Examples:\n");
      fmt::print("  simitl-playback g ghost.json --start 3 --len 3\n");
      fmt::print("  simitl-playback bb flight.bbl.csv -ff\n");
      fmt::print("  simitl-playback olbb flight.bbl.csv -ff\n");
      return 0;
    }
  }

  if (argc > 1) {
    std::string arg1 = argv[1];

    // Helper: parse common flags (--config, --start, --len, -ff) and return first positional arg
    auto parseArgs = [&](int startIdx) -> std::string {
      std::string positionalArg;
      for (int i = startIdx; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) {
          quadConfigPath = argv[++i];
        } else if (a == "--start" && i + 1 < argc) {
          startSec = std::stof(argv[++i]);
        } else if (a == "--len" && i + 1 < argc) {
          lenSec = std::stof(argv[++i]);
        } else if (a == "-ff") {
          fastForward = true;
        } else if (a == "-ns" || a == "--no-startup-sleep") {
          noStartupSleep = true;
        } else if (a == "-no" || a == "--no-osd") {
          noOsd = true;
        } else if (a.rfind("-", 0) != 0 && positionalArg.empty()) {
          positionalArg = a;
        }
      }
      return positionalArg;
    };

    if (arg1 == "g" || arg1 == "ghost") {
      mode = "g";
      std::string pos = parseArgs(2);
      ghostPath = pos.empty() ? "ghost.json" : pos;
    }
    else if (arg1 == "bb" || arg1 == "blackbox") {
      mode = "bb";
      std::string pos = parseArgs(2);
      bbPlaybackPath = pos;
      if (bbPlaybackPath.empty()) {
        fmt::print("Error: blackbox playback mode requires a CSV file path\n");
        return 1;
      }
    }
    else if (arg1 == "olbb" || arg1 == "openloop-blackbox") {
      mode = "olbb";
      std::string pos = parseArgs(2);
      bbPlaybackPath = pos;
      if (bbPlaybackPath.empty()) {
        fmt::print("Error: open-loop blackbox mode requires a CSV file path\n");
        return 1;
      }
    }
    else if (arg1 == "i" || arg1 == "idle") {
      mode = "i";
      parseArgs(2);
    }
    else {
      // Unknown argument -- treat as idle (default), but parse flags from all args
      mode = "i";
      parseArgs(1);
      fmt::print("Warning: unknown argument '{}', defaulting to idle mode\n", arg1);
    }
  }

  // ---- Playback modes (idle / ghost / blackbox / open-loop blackbox) ----
  fmt::print("simitl-playback: mode={}\n", mode);

  // Load data source for the selected mode
  if (mode == "g") {
    try {
      ghost = readGhostFile(ghostPath);
      fmt::print("Loaded ghost recording: trackId={}, quadId={}, {} frames\n",
                 ghost.trackId, ghost.quadId, ghost.samples.size());

      if (startSec > 0.0f || lenSec > 0.0f) {
        trimGhostFrames(ghost, startSec, lenSec);
      }
    } catch (const std::exception& e) {
      fmt::print("Warning: failed to load ghost file '{}': {}\n",
                 ghostPath, e.what());
      return 0;
    }
  }
  else if (mode == "bb" || mode == "olbb") {
    try {
      bbPlaybackData = readBlackboxFile(bbPlaybackPath);
      fmt::print("Loaded blackbox: {} frames, {} Hz log rate\n",
                 bbPlaybackData.frames.size(), bbPlaybackData.header.logRateHz);

      if (startSec > 0.0f || lenSec > 0.0f) {
        trimBlackboxFrames(bbPlaybackData, startSec, lenSec);
      }
    } catch (const std::exception& e) {
      fmt::print("Warning: failed to load blackbox file '{}': {}\n",
                 bbPlaybackPath, e.what());
      return 1;
    }
  }
  // Idle mode ("i"): no data source needed

  // Load quad config (shared across all playback modes)
  try {
    quadCfg = loadQuadConfig(quadConfigPath);
    quadCfg.applyTo(stateInit);
    fmt::print("Loaded quad config: {}\n", quadConfigPath);
  } catch (const std::exception& e) {
    fmt::print("Warning: failed to load quad config '{}': {}\n",
               quadConfigPath, e.what());
    return 1;
  }

  auto name = "test.bin";
  std::fill(stateInit.eepromName, stateInit.eepromName + 512, 0);
  memcpy(stateInit.eepromName, name, strnlen(name, 512));
  stateInit.eepromName[511] = '\0';

  initInputRuntimeDefaults(stateInput);
  quadCfg.applyInputDefaultsTo(stateInput);

  simitl_init(stateInit);

  // ==================================================================
  // Startup summary
  // ==================================================================
  fmt::print("\n===== STARTUP SUMMARY =====\n");
  fmt::print("Mode:              {}\n", mode);
  fmt::print("Config path:       {}\n", quadConfigPath);

  if (mode == "g") {
    fmt::print("Ghost path:        {}\n", ghostPath);
    fmt::print("Ghost frames:      {} (trackId={}, quadId={})\n",
               ghost.samples.size(), ghost.trackId, ghost.quadId);
  } else if (mode == "bb" || mode == "olbb") {
    fmt::print("Blackbox path:     {}\n", bbPlaybackPath);
    fmt::print("Blackbox frames:   {} ({} Hz log rate)\n",
               bbPlaybackData.frames.size(), bbPlaybackData.header.logRateHz);
    if (mode == "olbb") {
      fmt::print("Open-loop:         yes (PID bypassed)\n");
    }
  }

  if (startSec > 0.0f || lenSec > 0.0f) {
    fmt::print("Trim:              start={}s, len={}s\n", startSec, lenSec);
  }
  fmt::print("Fast-forward:      {}\n", fastForward ? "yes" : "no");
  fmt::print("Startup pause:     {} (override with -ns)\n",
             noStartupSleep ? "skipped" : "3 seconds");

  quadCfg.print();

  fmt::print("=============================\n\n");

  if (!noStartupSleep) {
    fmt::print("Starting in 3 seconds...\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }

  if (mode == "g") {
    t = std::thread(ghostPlaybackThread);
  } else if (mode == "bb") {
    t = std::thread(blackboxPlaybackThread);
  } else if (mode == "olbb") {
    t = std::thread(blackboxOpenLoopThread);
  } else {
    t = std::thread(idleThread);
  }
  t.join();

  return 0;
}
