# simitl-playback

A standalone C++ test tool that drives the **SimITL** simulation loop without requiring a real game engine client. Supports three playback modes: idle, ghost recording replay, and blackbox RC feed.

## Modes

### 1. Idle Mode (`i`) — Default

Runs a bare simulation loop with neutral inputs (disarmed, throttle low, centered sticks). No external data source required.

```
simitl-playback [i] [--config <quad-config.json>]
```

- **Inputs**: None — simulation runs with default (idle) inputs
- **Outputs**: OSD and quad state printed to console every 100 frames
- **Purpose**: Quick smoke-test, FC boot verification, baseline sanity check

### 2. Ghost Mode (`g`)

Replays a pre-recorded ghost flight trajectory through the simulation, feeding the ghost's RC inputs and locking the initial pose.

```
simitl-playback g [ghost.json] [--config <quad-config.json>] [-ff]
```

- **Inputs**: A ghost recording (`ghost.json`, default if omitted) containing RC inputs, position, and orientation over time
- **Process**: Feeds ghost RC data into the sim at real-time speed (or full speed with `-ff`), tracks position/attitude output
- **Outputs**: `quadstate.csv` with per-frame position, orientation, velocities
- **Purpose**: Visual validation, trajectory comparison, regression testing

### 3. Blackbox Playback Mode (`bb`)

Feeds RC commands from a real-world **Betaflight Blackbox** recording into the simulation, driving the quad with recorded stick inputs.

```
simitl-playback bb <blackbox.csv> [--config <quad-config.json>] [-ff]
```

- **Inputs**: Betaflight blackbox CSV (`.bbl.csv`), quad config path (optional)
- **Process**: Boot wait (8s), arm sequence (0.1s), then feeds blackbox RC commands into the sim time-synchronized (or full speed with `-ff`)
- **Outputs**: `quadstate_bb.csv` with per-frame position, orientation, velocities
- **Purpose**: Validate sim response to real pilot inputs, compare flight behavior

## CLI Reference

```
simitl-playback [mode] [options] [file]

Modes:
  i, idle               Run simulation idle (no input playback, default)
  g, ghost <path>       Playback a ghost.json recording
  bb, blackbox <path>   Playback a blackbox CSV recording

Options:
  -h, --help            Show this help message and exit
  --config <path>       Quad config JSON path
  --start <sec>         Skip first N seconds of recording
  --len <sec>           Only play N seconds total from the effective start
  -ff                   Fast-forward mode (no sleep between sim steps)
  -no, --no-osd         Skip OSD output to CLI
```

| Flag | Description |
|------|-------------|
| `--config <file>` | Path to quad configuration JSON (default: `config/quad/vtx-slayer-one-4.json`) |
| `--start <sec>` | Skip first N seconds of recording |
| `--len <sec>` | Only play N seconds from the effective start |
| `-ff` | **Fast-forward mode**: skip all `sleep_for` calls, running the simulation as fast as possible. Useful for quick data export or batch processing. |
| `-no`, `--no-osd` | Skip the OSD frame dump to the terminal. Reduces output noise when only CSV or quad-state data is needed. |

## Examples

```bash
# Idle mode (default)
simitl-playback

# Ghost playback
simitl-playback g ghost.json

# Ghost playback with trimming
simitl-playback g ghost.json --start 3 --len 3

# Blackbox playback (feed real RC into sim)
simitl-playback bb btfl_079.bbl.csv

# Blackbox playback at full speed
simitl-playback bb btfl_079.bbl.csv -ff

# With custom config
simitl-playback g ghost.json --config my-quad.json
```

## Output Files

| File | Description |
|---|---|
| `quadstate.csv` | Ghost playback — per-frame position, orientation, velocities |
| `quadstate_bb.csv` | Blackbox playback — per-frame position, orientation, velocities |

## Build

```bash
./../../build.sh
```

The project cross-builds for both Linux and Windows (using MinGW). The Linux binary is at `../../build/linux/install/bin/simitl-playback`.

## Quick Start

```bash
# Idle mode (default)
simitl-playback

# Ghost playback
simitl-playback g ghost.json

# Blackbox playback (feed real RC into sim)
simitl-playback bb btfl_079.bbl.csv
```

## Convert Ghost Recordings

```bash
base64 -d ghost.bin | zcat > ghost.json
```


## effective tuning

tuning differential evolution (de)
```
python tuner.py -c config/quad/vtx-slayer-one-4.json -s 20 -l 10 --maxiter 25 -ol -de btfl_079.bbl.csv
```

playback
```
../../build/win/install/bin/simitl-playback.exe olbb btfl_079.bbl.csv --config result/quad/vtx-slayer-one-4.json --start 20 -len 10 -ff -ns -no
```

plot playback
```
python plot_quadstate.py quadstate_olbb.csv --start 216 --length 8
```