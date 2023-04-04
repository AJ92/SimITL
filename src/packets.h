#ifndef SIM_PACKETS_H
#define SIM_PACKETS_H

#ifdef _WIN32
  #include "winsock2.h"
#endif

#include "vector_math.h"

#include <cstdint>
#include <fmt/format.h>
#include <kissnet.hpp>

enum PacketType : int32_t {
  Error           = 0U,
  Init            = 1U,
  State           = 2U,
  StateUpdate     = 3U,
  StateOsdUpdate  = 4U,
  Rc              = 5U,

  //count of types
  Count           = 6U
};

enum CommandType : int32_t {
  None      = 0U,
  Stop      = 1U,
  Reserved1 = 2U,
  Reserved2 = 4U,
  Reserved3 = 8U,
};

struct Vec3F{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct InitPacket{
  PacketType type = PacketType::Init;

  float motorKV = 0.0f;
  float motorR = 0.0f;
  float motorI0 = 0.0f;

  float propMaxRpm = 0.0f;
  float propAFactor = 0.0f;
  float propTorqueFactor = 0.0f;
  float propInertia = 0.0f;
  Vec3F propThrustFactor {};

  Vec3F frameDragArea {};
  float frameDragConstant = 0.0f;

  float quadMass = 0.0f;
  Vec3F quadInvInertia {};
  float quadBatVoltage = 0.0f;
  uint8_t quadBatCellCount = 1;
  float quadBatCapacity = 0.0f;
  Vec3F quadMotorPos[4] {};
};

struct StatePacket{
  PacketType type = PacketType::State;

  float delta = 0.0f;
  Vec3F position {};
  Vec3F rotation[3] {};

  Vec3F angularVelocity {};
  Vec3F linearVelocity {};

  // length is amplitude per axis
  Vec3F motor1Imbalance {};
  Vec3F motor2Imbalance {};
  Vec3F motor3Imbalance {};
  Vec3F motor4Imbalance {};

  float gyroBaseNoiseAmp = 0.0f;
  float gyrobaseNoiseFreq = 0.0f;

  float frameHarmonic1Amp = 0.0f;
  float frameHarmonic1Freq = 0.0f;

  float frameHarmonic2Amp = 0.0f;
  float frameHarmonic2Freq = 0.0f;

  float vbat = 0.0f;

  // 1 true 0 false
  uint8_t crashed = 0;

  // combination of CommandType (bitmask)
  int32_t commands = 0;
};

struct StateUpdatePacket{
  PacketType type = PacketType::StateUpdate;

  Vec3F angularVelocity {};
  Vec3F linearVelocity {};

  float motorRpm[4] {};
};

struct StateOsdUpdatePacket{
  PacketType type = PacketType::StateOsdUpdate;

  Vec3F angularVelocity {};
  Vec3F linearVelocity {};

  float motorRpm[4] {};

  uint8_t osd[16*30] {};
};

// rc data needs own refresh rate and is seperate
struct StateRcUpdatePacket{
  PacketType type = PacketType::Rc;

  float delta = 0.0f;
  float rcData[8] {};
};

// copies data to the type struct...
template <typename T>
bool convert(T& out, const std::byte * data, const size_t length){
  bool success = false;
  size_t packetSize = sizeof(T);
  if(length <= packetSize){
    memcpy(&out, data, sizeof(T));
    success = true;
  }
  return success;
}

template <typename T>
auto receive(kissnet::udp_socket& recv_socket){
  std::array<std::byte, 2 * sizeof(T)> buf;
  auto [len, socketStatus] = recv_socket.recv(buf);

  T packet;

  //error
  if(socketStatus != 0){
    bool success = convert(packet, &buf[0], len);
    if(!success){
      packet.type = PacketType::Error;
    }
  }

  return packet;
}


#endif