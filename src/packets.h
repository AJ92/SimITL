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

  //count of types
  Count           = 5U
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

  // initial time of visualization side (unity)
  double timeVis = 0.0;
  // initial time of simulation side (this)
  double timeSim = 0.0;

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
  float quadVbat = 0.0f;
  Vec3F quadMotorPos[4] {};
};



struct StatePacket{
  PacketType type = PacketType::State;

  double time = 0.0;

  float delta = 0.0f;
  Vec3F position {};
  Vec3F rotation[3] {};

  Vec3F angularVelocity {};
  Vec3F linearVelocity {};

  float motorRpm[4] {};

  float rcData[8] {};

  // 1 true 0 false
  byte crashed = 0;

  // combination of CommandType (bitmask)
  int32_t commands = 0;
};

struct StateUpdatePacket{
  PacketType type = PacketType::StateUpdate;

  double time = 0.0;

  Vec3F angularVelocity {};
  Vec3F linearVelocity {};

  float motorRpm[4] {};
};

struct StateOsdUpdatePacket{
  PacketType type = PacketType::StateOsdUpdate;

  double time = 0.0;

  Vec3F angularVelocity {};
  Vec3F linearVelocity {};

  float motorRpm[4] {};

  byte osd[16*30] {};
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