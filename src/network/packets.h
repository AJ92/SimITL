#ifndef SIM_PACKETS_H
#define SIM_PACKETS_H

#ifdef _WIN32
  #include "winsock2.h"
#endif

#include "util/vector_math.h"

#include <cstdint>
#include <fmt/format.h>
#include <kissnet.hpp>

enum PacketType : int32_t {
  Error           = 0U,
  Init            = 1U,
  State           = 2U,
  StateUpdate     = 3U,
  StateOsdUpdate  = 4U,

  //count of types
  Count           = 5U
};

enum CommandType : int32_t {
  None      = 0U,
  Stop      = 1U,
  Repair    = 2U,
  Reset     = 4U,
  Reserved3 = 8U,
};

struct Vec3F{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Vec4F{
  float x = 1.0f;
  float y = 0.0f;
  float z = 0.0f;
  float w = 1.0f;
};

struct GpsData{
  // 48.420876993590745, 10.711263681338096
  int32_t lat = 484208769;
  int32_t lon = 107112636;
  int32_t alt = 0;
};

// initial quad settings.
struct InitPacket{
  PacketType type = PacketType::Init;

  float motorKV[4]    = {}; // KV
  float motorR[4]     = {}; // resistance
  float motorI0[4]    = {}; // idle current
  float motorRth   = 0.0f; // thermal resistance (deg C per Watt)
  float motorCth   = 0.0f; // thermal heat capacity (joules per deg C)
  float motorMaxT  = 0.0f; // max temp in C when motor windings short out

  uint8_t propBladeCount = 0U;
  float propMaxRpm = 0.0f;
  float propAFactor = 0.0f;
  float propTorqueFactor = 0.0f;
  float propInertia = 0.0f;
  Vec3F propThrustFactor {};
  float propHarmonic1Amp = 0.0f;
  float propHarmonic2Amp = 0.0f;

  Vec3F frameDragArea {};
  float frameDragConstant = 0.0f;

  float quadMass = 0.0f;
  Vec3F quadInvInertia {};
  float maxVoltageSag = 0.0f;
  uint8_t quadBatCellCount = 1;
  //charged up capacity, can be lower or higher than quadBatCapacity
  float quadBatCapacityCharged  = 0.0f;
  //battery capacacity rating
  float quadBatCapacity = 0.0f;
  Vec3F quadMotorPos[4] {};

  float ambientTemp = 0.0f;

  GpsData gps {};

  //eeprom file name
  uint8_t eepromName[32] = {};
};

//runtime quad parameters - simulation input
struct StatePacket{
  PacketType type = PacketType::State;

  float delta = 0.0f;

  float rcData[8] {};

  Vec3F position {};
  Vec3F rotation[3] {};

  Vec3F angularVelocity {};
  Vec3F linearVelocity {};

  // length is amplitude per axis
  Vec3F motorImbalance[4] {};

  float gyroBaseNoiseAmp = 0.0f;
  float gyrobaseNoiseFreq = 0.0f;

  float frameHarmonic1Amp = 0.0f;
  float frameHarmonic1Freq = 0.0f;

  float frameHarmonic2Amp = 0.0f;
  float frameHarmonic2Freq = 0.0f;

  float propDamage[4] {};
  float groundEffect[4] {};

  float vbat = 0.0f;

  // 1 true 0 false
  uint8_t contact = 0;

  // combination of CommandType (bitmask)
  int32_t commands = 0;
};

//simulation output for game
struct StateUpdatePacket{
  PacketType type = PacketType::StateUpdate;

  Vec4F orientation{};
  Vec3F angularVelocity {};
  Vec3F linearVelocity {};

  float motorRpm[4] {};
  float motorT[4] {};

  bool beep = false;
};

struct StateOsdUpdatePacket{
  PacketType type = PacketType::StateOsdUpdate;

  uint8_t osd[16*30] {};
};

// copies data to the type struct...
template <typename T, size_t buff_size>
bool convert(T& out, const std::array<std::byte, buff_size>& buf, const size_t length){
  bool success = false;
  size_t packetSize = sizeof(T);
  if(packetSize <= length){
    memcpy(&out, &buf[0], sizeof(T));
    success = true;
  }
  return success;
}

template <size_t buff_size>
size_t receive(kissnet::udp_socket& recv_socket, std::array<std::byte, buff_size>& buf){
  auto [len, socketStatus] = recv_socket.recv(buf);
  return (socketStatus != 0)? len : 0;
}

#endif