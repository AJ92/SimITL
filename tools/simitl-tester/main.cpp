#include <fmt/format.h>
#include <thread>
#include <chrono>

#ifdef _WIN32
#include "winsock2.h"
#endif

#include <kissnet.hpp>
#include "network/packets.h"

kissnet::udp_socket sendStateSocket(kissnet::endpoint("localhost", 30713));
std::thread t{};
bool running = true;

InitPacket init = {};
StatePacket state = {};

void updateThread(){
    sendStateSocket.send(reinterpret_cast<const std::byte*>(&init), sizeof(InitPacket));

    while(running){
      sendStateSocket.send(reinterpret_cast<const std::byte*>(&state), sizeof(InitPacket));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

int main() {
  fmt::print("simitl-tester starting...\n");

  auto name = "test.bin";
  memcpy(init.eepromName, name, strnlen(name, 30));
  init.eepromName[31] = '\0';

  state.delta = 0.001;

  t = std::thread(updateThread);
  t.join();

  return 0;
}