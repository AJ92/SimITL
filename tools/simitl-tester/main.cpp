#include <fmt/format.h>
#include <thread>
#include <chrono>
#include <array>

#include "network/packets.h"
#include "simitl.h"

std::thread t{};
bool running = true;

StateInit stateInit = {};
StateInput stateInput = {};


uint64_t currentFrame = 0U;
uint64_t frameRestart = 500U;

void updateThread(){
  while(running){

    if(currentFrame>frameRestart){
      currentFrame = 0U;
      fmt::print("\n");
      fmt::print("simitl-tester restarting...\n");
      simitl_stop();
      simitl_init(stateInit);
    }

    simitl_update(stateInput);
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
    fmt::print(".");
    currentFrame++;
  }
}

int main() {
  fmt::print("simitl-tester starting...\n");

  auto name = "test.bin";
  std::fill(stateInit.eepromName, stateInit.eepromName + 512, 0);
  memcpy(stateInit.eepromName, name, strnlen(name, 512));
  stateInit.eepromName[511] = '\0';

  stateInput.delta = 0.016;

  simitl_init(stateInit);

  t = std::thread(updateThread);
  t.join();

  return 0;
}