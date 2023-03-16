//#include "packets.h"

#include "sim.h"

#include <fmt/format.h>

#include <chrono>

void clearline() {
    fmt::print(
      "\r                                                                    "
      "                          \r");
}

int main() {
    auto& simulator = Sim::getInstance();

    simulator.connect();

    auto start = hr_clock::now();
    auto lastLog = start;
    auto i = 0u;
    while (simulator.step()) {
        if (i % 200 == 0) {
            const auto now = hr_clock::now();
            const auto elapsed_ms = now - start;
            long long ms_i = to_us(elapsed_ms);
            long long delta = simulator.micros_passed - ms_i;

            const auto dtLastLog = now - lastLog;
            float factor = (float) to_ms(dtLastLog) / 1000.0f;

            clearline();
            fmt::print("e.ms: {}, f.ms: {}, dt: {}, s/s: {}, sch/s: {}, rpm: {} {} {} {}, dis: {}          ",
                       ms_i,
                       simulator.micros_passed,
                       delta,
                       factor * (float)simulator.simSteps -
                       factor * (float)simulator.bfSchedules,
                       factor * (float)simulator.bfSchedules,
                       simulator.motorsState[0].rpm,
                       simulator.motorsState[1].rpm,
                       simulator.motorsState[2].rpm,
                       simulator.motorsState[3].rpm,
                       simulator.armingDisabledFlags);

            simulator.simSteps = 0;
            simulator.bfSchedules = 0;
            lastLog = now;
        }

        i++;
    }

    fmt::print("Stopped betaflight host process\n");

    return 0;
}
