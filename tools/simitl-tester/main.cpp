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

uint64_t currentFrame = 1U;
uint64_t framePrintOsd = 120U;
uint64_t frameRestart = 2000U;

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

void initStateDefaults(StateInit& s)
{
  // Motor
  for (int i = 0; i < 4; i++) {
    s.motorKV[i] = 2800.0f;
    s.motorR[i]  = 0.07f;
    s.motorI0[i] = 0.0075f;
  }
  s.motorRth  = 0.25f;
  s.motorCth  = 30.0f;
  s.motorMaxT = 128.0f;

  // Propeller
  s.propBladeCount   = 3;
  s.propMaxRpm       = 36000.0f;
  s.propAFactor      = 3.5e-9f;
  s.propTorqueFactor = 0.0087f;
  s.propInertia      = 3.75e-7f;
  s.propThrustFactor = { -0.000006f, -0.1f, 11.2f };
  s.propHarmonic1Amp = 0.1f;
  s.propHarmonic2Amp = 0.3f;

  // Frame
  s.frameDragArea     = { 0.0097f, 0.0081f, 0.0098f };
  s.frameDragConstant = 1.28f;
  s.quadMass          = 0.349f;
  s.quadInvInertia    = { 570.0f, 730.0f, 570.0f };

  // Motor positions ~5 inch
  s.quadMotorPos[0] = {  0.067175f, 0.005f, -0.067175f };
  s.quadMotorPos[1] = {  0.067175f, 0.005f,  0.067175f };
  s.quadMotorPos[2] = { -0.067175f, 0.005f, -0.067175f };
  s.quadMotorPos[3] = { -0.067175f, 0.005f,  0.067175f };

  // Battery
  s.maxVoltageSag          = 1.4f;
  s.quadBatCellCount       = 4;
  s.quadBatCapacityCharged = 1305.0f;
  s.quadBatCapacity        = 1300.0f;

  // Prop wash 
  s.minPropWashSpeed      = 1.0f;
  s.maxPropWashSpeed      = 12.0f;
  s.propWashAngleOfAttack = 0.1f;
  s.propWashFactor        = 1.0f;

  s.ambientTemp = 25.0f;
}

void initInputDefaults(StateInput& s)
{
  // 16.6 ms 60 fps
  s.delta = 0.016f;

  // RC channels: throttle (ch2) low, others centered (betaflight 1000-2000 range)
  s.rcData[0] = 1500.0f; // roll
  s.rcData[1] = 1500.0f; // pitch
  s.rcData[2] = 1000.0f; // throttle (low)
  s.rcData[3] = 1500.0f; // yaw
  s.rcData[4] = 1000.0f; // arm switch (disarmed)
  s.rcData[5] = 1000.0f;
  s.rcData[6] = 1000.0f;
  s.rcData[7] = 1000.0f;

  // Identity rotation matrix
  s.rotation[0] = { 1.0f, 0.0f, 0.0f };
  s.rotation[1] = { 0.0f, 1.0f, 0.0f };
  s.rotation[2] = { 0.0f, 0.0f, 1.0f };

  // Motor imbalance
  for (int i = 0; i < 4; i++) {
    s.motorImbalance[i] = { 13.0f, 7.0f, 5.0f };
  }

  // Gyro noise
  s.gyroBaseNoiseAmp  = 0.000287f;
  s.gyrobaseNoiseFreq = 228.0f;

  // Frame harmonics
  s.frameHarmonic1Amp  = 0.02242f;
  s.frameHarmonic1Freq = 275.0f;
  s.frameHarmonic2Amp  = 0.01f;
  s.frameHarmonic2Freq = 326.66f;

  // Battery fully charged 4s
  s.vbat = 16.8f;
}

void updateThread()
{
  while (running)
  {
    simitl_update(stateInput);
    stateOutput = simitl_get_state();
    printOsdToCli();

    // 16.6 ms 60 fps
    std::this_thread::sleep_for(std::chrono::microseconds(16600));
    fmt::print(".");

    currentFrame++;
  }
}



int main() {
  fmt::print("simitl-tester starting...\n");

  initStateDefaults(stateInit);

  auto name = "test.bin";
  std::fill(stateInit.eepromName, stateInit.eepromName + 512, 0);
  memcpy(stateInit.eepromName, name, strnlen(name, 512));
  stateInit.eepromName[511] = '\0';

  initInputDefaults(stateInput);

  simitl_init(stateInit);

  t = std::thread(updateThread);
  t.join();

  return 0;
}