//#include "packets.h"

#include "sim.h"
#include <fmt/format.h>
#include <chrono>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
//#include <unistd.h>

Sim* sim = nullptr;

void sigHandler(int s){
  printf("Caught signal %d\nShutting down...\n",s);
  if(sim != nullptr){
    sim->stop();
  }
}

void clearline() {
    fmt::print(
      "\r                                                                                              \r");
}

int main() {
    signal (SIGINT,sigHandler);

    sim = &Sim::getInstance();

    if(!sim->connect()){
      return 1;
    }

    auto start = hr_clock::now();
    auto lastLog = start;
    auto i = 0u;
    while (sim->step()) {
      const auto now = hr_clock::now();
      const auto dtLastLog = now - lastLog;
      long long logDelta = to_us(dtLastLog);

      // 1s
      if (logDelta > 1e6) {
        const auto elapsed_ms = now - start;
        long long elapsedUs = to_us(elapsed_ms);
        long long delta = sim->micros_passed - elapsedUs;

        clearline();
        fmt::print(
          "dt:     {:5.5f}, Sl/s: {:8.1f}, sch/s: {:8.1f}, avgST: {}, dis: {},\n" 
          "angAcc: {:5.2f} {:5.2f} {:5.2f},\n"
          "curr:   {:5.2f} {:5.2f} {:5.2f} {:5.2f},\n"
          "temp:   {:5.2f} {:5.2f} {:5.2f} {:5.2f},\n"
          "krpm:   {:5.2f} {:5.2f} {:5.2f} {:5.2f},\n"
          "thrust: {:5.3f} {:5.3f} {:5.3f} {:5.3f},\n"
          "ptorq:  {:5.3f} {:5.3f} {:5.3f} {:5.3f},\n"
          "mtorq:  {:5.3f} {:5.3f} {:5.3f} {:5.3f},\n"
          "aDrag:  {:5.3f} {:5.3f} {:5.3f}\n",
          delta / 1e6,
          (float)sim->simSteps - (float)sim->bfSchedules,
          (float)sim->bfSchedules,
          sim->avgStepTime,
          sim->armingDisabledFlags,
          sim->angularAcceleration[0],
          sim->angularAcceleration[1],
          sim->angularAcceleration[2],
          sim->motorsState[0].current,
          sim->motorsState[1].current,
          sim->motorsState[2].current,
          sim->motorsState[3].current,
          sim->motorsState[0].temp,
          sim->motorsState[1].temp,
          sim->motorsState[2].temp,
          sim->motorsState[3].temp,
          sim->motorsState[0].rpm / 1e3,
          sim->motorsState[1].rpm / 1e3,
          sim->motorsState[2].rpm / 1e3,
          sim->motorsState[3].rpm / 1e3,
          sim->motorsState[0].thrust / 9.81f,
          sim->motorsState[1].thrust / 9.81f,
          sim->motorsState[2].thrust / 9.81f,
          sim->motorsState[3].thrust / 9.81f,
          sim->motorsState[0].pTorque,
          sim->motorsState[1].pTorque,
          sim->motorsState[2].pTorque,
          sim->motorsState[3].pTorque,
          sim->motorsState[0].mTorque,
          sim->motorsState[1].mTorque,
          sim->motorsState[2].mTorque,
          sim->motorsState[3].mTorque,
          sim->angularDrag[0],
          sim->angularDrag[1],
          sim->angularDrag[2]
        );

        sim->simSteps = 0;
        sim->bfSchedules = 0;
        lastLog = now;
      }

      i++;
    }

    //incase internal error stopped the while loop...
    sim->stop();

    fmt::print("Stopped betaflight host process\n");
    // wait a bit till threads are dead...
    while(!sim->stopped){
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}
