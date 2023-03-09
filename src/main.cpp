//#include "packets.h"

#include "sim.h"

#include <fmt/format.h>

#include <chrono>

using hr_clock = std::chrono::high_resolution_clock;

template <typename R, typename P>
auto to_us(std::chrono::duration<R, P> t) {
    return std::chrono::duration_cast<std::chrono::microseconds>(t).count();
}

template <typename R, typename P>
auto to_ms(std::chrono::duration<R, P> t) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(t).count();
}

void clearline() {
    fmt::print(
      "\r                                                                    "
      "                          \r");
}

int main() {
    auto& simulator = Sim::getInstance();

    simulator.connect();

    auto start = hr_clock::now();
    auto i = 0u;
    while (simulator.step()) {
        if (i % 10 == 0) {
            const auto elapsed_ms = hr_clock::now() - start;
            long long ms_i = to_us(elapsed_ms);
            long long delta = simulator.micros_passed - ms_i;

            clearline();
            fmt::print("elapsed ms: {}, fake ms: {}, dt: {}, rc: {} {} {} {}, arm_dis: {}",
                       ms_i,
                       simulator.micros_passed,
                       delta,
                       simulator.motorsState[0].rpm,
                       simulator.motorsState[1].rpm,
                       simulator.motorsState[2].rpm,
                       simulator.motorsState[3].rpm,
                       simulator.armingDisabledFlags);
        }

        i++;
    }

    fmt::print("Stopped betaflight host process\n");

    return 0;
}
