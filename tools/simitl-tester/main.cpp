#include <fmt/format.h>
#include <thread>
#include <chrono>
#include <array>

#include "network/packets.h"
#include "simitl.h"

#include <cctype>
#include <iostream>

std::thread t{};
bool running = true;

StateInit stateInit = {};
StateInput stateInput = {};
StateOutput stateOutput = {};

uint64_t currentFrame = 0U;
uint64_t frameRestart = 5U;

void printOsdToCli()
{
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

void updateThread()
{
  while (running)
  {

    if (currentFrame > frameRestart)
    {
      currentFrame = 0U;
      fmt::print("\n");
      fmt::print("simitl-tester restarting...\n");
      simitl_stop();
      simitl_init(stateInit);
    }

    simitl_update(stateInput);
    stateOutput = simitl_get_state();
    printOsdToCli();

    std::this_thread::sleep_for(std::chrono::milliseconds(1116));
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

  stateInput.delta = 1.116;

  simitl_init(stateInit);

  t = std::thread(updateThread);
  t.join();

  return 0;
}