# simitl-tester

A standalone test tool that drives the SimITL simulation loop without requiring a real game engine client. It lets you run, tune, and verify the SimITL library in isolation and connect to the **Betaflight Configurator** for live config changes.

---

## Overview

`simitl-tester` initialises SimITL with a hardcoded quadcopter model, then spins a 60 Hz update loop that:

1. Calls `simitl_update()` each frame with the current `StateInput` (RC channels, delta time, pose, …).
2. Reads back `StateOutput` (orientation, angular/linear velocity, motor RPMs/temperatures, OSD).
3. Prints the OSD buffer to the terminal every 120 frames (~2 s) so you can watch the Betaflight OSD without a screen.

Because the SimITL shared library listens for a Betaflight Configurator connection in the background, you can open the configurator, connect to the virtual FC and interact with it normally while `simitl-tester` keeps the sim alive.

---

## Dependencies

| Dependency | Role |
|---|---|
| `SimITL` | The core sim-in-the-loop shared library (built as part of the main project) |
| `sitl` | Betaflight SITL target static library |
| `fmt-header-only` | Formatted console output ([fmtlib](https://github.com/fmtlib/fmt)) |
| pthreads / Win32 | Threading for the update loop |

---

## Building

`simitl-tester` is built as part of the top-level CMake project. From the repository root use the provided scripts:

```
./setup.sh
./build.sh
```

Note: use wsl under windows.

---

## Running

```bash
./simitl-tester
```

The tester will:
- Load (or create) an EEPROM file named `test.bin` in the working directory. This is where Betaflight stores its settings between runs.
- Start printing a `.` to stdout every frame and dump the OSD grid every ~2 seconds.

Press **Ctrl-C** to stop.

---

## Connecting to Betaflight Configurator

1. Start `simitl-tester`.
2. Open [Betaflight Configurator](https://app.betaflight.com/).
3. Under options enable `manual connection mode`.
4. Use `Manual Selection` and enter `ws://localhost:5761` as port, press `Connect`.
5. Supported motor protocol is PWM only. Set arm to aux 1.

---

## Default Quad Model

The tester initialises a light 5-inch race quad (`initStateDefaults`). Key parameters:

| Parameter | Value |
|---|---|
| Motor KV | 2800 KV |
| Motor resistance | 0.07 Ω |
| Max RPM | 36 000 RPM |
| Quad mass | 349 g |
| Battery | 4S, 1300 mAh |
| Motor arm length | ~67 mm (≈ 5 inch) |
| Ambient temperature | 25 °C |

RC channels are initialised with throttle low and all other channels centred (Betaflight 1000–2000 µs range). The arm switch (channel 5) is set to disarmed (`1000`).

| Channel | Function | Default |
|---|---|---|
| 1 | Roll | 1500 |
| 2 | Pitch | 1500 |
| 3 | Throttle | 1000 (low) |
| 4 | Yaw | 1500 |
| 5 | Arm switch | 1000 (disarmed) |

---

## Simulation Output

Each call to `simitl_get_state()` returns a `StateOutput` containing:

- **Orientation** – quaternion (`Vec4F`)
- **Angular velocity** – rad/s (`Vec3F`)
- **Linear velocity** – m/s (`Vec3F`)
- **Motor RPMs** – one float per motor
- **Motor temperatures** – °C per motor
- **Motor status** – `MotorNone`, `MotorDamaged`, or `MotorBurnedOut`
- **OSD buffer** – 16 × 30 byte character grid (printed to terminal every 120 frames)
- **Beeper** – on/off flag
