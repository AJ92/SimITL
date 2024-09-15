#include <fmt/format.h>
#include <thread>
#include <chrono>
#include <array>

#include "network/packets.h"
#include "util/sharedmem.h"


// buffer for data that is received from the gameclient
void * sharedMemoryReceptionBuffer = nullptr;
// buffer for data that is sent to the gameclient
void * sharedMemoryTransmissionBuffer = nullptr;

std::thread t{};
bool running = true;

InitPacket init = {};
StatePacket state = {};

std::array<std::byte, 2048> stateReceptionBuffer{};

void updateThread(){
  const size_t initLen = static_cast<size_t>(
    SimITLMemWrite(
      sharedMemoryTransmissionBuffer,
      reinterpret_cast<void *>(&init),
      static_cast<int>(sizeof(InitPacket))
    )
  );

  fmt::print("initLen {}\n", initLen);

  while(running){

    size_t readLen = 1;
    while(readLen != 0){
      readLen = static_cast<size_t>(SimITLMemRead(
        sharedMemoryReceptionBuffer, 
        static_cast<void *>(stateReceptionBuffer.data()),
        static_cast<int>(stateReceptionBuffer.size())
      ));
      fmt::print("readLen {}\n", readLen);
    }

    const size_t sentLen = static_cast<size_t>(
      SimITLMemWrite(
        sharedMemoryTransmissionBuffer,
        reinterpret_cast<void *>(&state),
        static_cast<int>(sizeof(StatePacket))
      )
    );

    fmt::print("sentLen {}\n", sentLen);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int main() {
  fmt::print("simitl-tester starting...\n");

  sharedMemoryTransmissionBuffer  = SimITLMemCreate(8096, "SimITLGameState");
  sharedMemoryReceptionBuffer     = SimITLMemCreate(8096, "SimITLSimState");

  auto name = "test.bin";
  memcpy(init.eepromName, name, strnlen(name, 30));
  init.eepromName[31] = '\0';

  state.delta = 0.001;

  t = std::thread(updateThread);
  t.join();

  return 0;
}