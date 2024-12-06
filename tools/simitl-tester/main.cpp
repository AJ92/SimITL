#include <fmt/format.h>
#include <thread>
#include <chrono>
#include <array>

#include "network/packets.h"
#include "simitl.h"

std::thread t{};
bool running = true;

StateInit init = {};
StateInput state = {};


uint64_t currentFrame = 0U;
uint64_t frameRestart = 500U;

void updateThread(){
  while(running){

    if(currentFrame>frameRestart){
      currentFrame = 0U;
      fmt::print("\n");
      fmt::print("simitl-tester restarting...\n");
      simitl_stop();
      simitl_init(init);
    }

    simitl_update(state);
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
    fmt::print(".");
    currentFrame++;
  }
}

int main() {
  fmt::print("simitl-tester starting...\n");

  auto name = "test.bin";
  std::fill(init.eepromName, init.eepromName + 512, 0);
  memcpy(init.eepromName, name, strnlen(name, 512));
  init.eepromName[511] = '\0';

  state.delta = 0.016;

  simitl_init(init);

  t = std::thread(updateThread);
  t.join();

  return 0;
}