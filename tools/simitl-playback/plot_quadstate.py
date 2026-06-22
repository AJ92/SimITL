#!/usr/bin/env python3
"""
Plot quadstate.csv data from simitl-playback.

Reads the CSV produced by the playback tool and plots all columns,
including optional reference columns from blackbox playback modes.

Supports all three CSV formats:

  ghost.csv (ghost playback):
    motorOut_1..4 (simulated motor outputs [-1,1])

  bb.csv / olbb.csv (blackbox / open-loop blackbox):
    motorOut_1..4      (simulated motor outputs [-1,1])
    bb_motorOut_1..4   (reference motor outputs [-1,1])
    bb_raw_gyro_x..z   (reference raw gyro in deg/s)
    bb_gyro_x..z       (reference filtered gyro in deg/s)
    bb_acc_x..z        (reference accelerometer in m/s^2)

If reference columns exist, they are overlaid (dashed) on the
corresponding sim plots. Otherwise only sim data is shown.

Orientation is converted from raw quaternion to YXZ Euler angles
(roll/pitch/yaw in degrees) matching the conversion in main.cpp.

Graphs:
  1. Top-down 2D trajectory (pos_x vs pos_z)  (Y-up: X-Z plane)
  2. Position components over time (pos_x, pos_y, pos_z)
  3. Orientation (Euler angles: roll, pitch, yaw) over time
  4. Linear velocity over time (vel_x, vel_y, vel_z)
  5. Angular velocity over time (sim + ref raw gyro if available)
  6. Motor outputs over time (sim + ref if available)

Usage:
  ./plot_quadstate.py                               # reads quadstate.csv
  ./plot_quadstate.py path/to/file.csv               # custom path
  ./plot_quadstate.py --start 2 --length 5           # time range [2s, 7s)
  ./plot_quadstate.py path.csv --start 1.5 --len 3   # custom file + range
"""

import argparse
import csv
import math
import sys
import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("Error: matplotlib is required. Install with: pip install matplotlib")
    sys.exit(1)
from scipy.signal import butter, sosfiltfilt


def read_quadstate(path: str) -> list[dict[str, float]]:
    """Read quadstate.csv and return list of row-dicts with all values as float."""
    rows: list[dict[str, float]] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k in row:
                row[k] = float(row[k])
            rows.append(row)
    return rows


def filter_time_range(
    rows: list[dict[str, float]], start: float, length: float
) -> list[dict[str, float]]:
    """Filter rows to [start, start+length) time window. If length <= 0, return all from start."""
    if start <= 0.0 and length <= 0.0:
        return rows
    end = start + length if length > 0.0 else float("inf")
    return [r for r in rows if r["time"] >= start and r["time"] < end]


def quat_to_euler_degrees(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """
    Convert quaternion (x, y, z, w) to YXZ Euler angles in degrees.

    Matches the conversion in main.cpp:
      roll  = rotation around body X (forward axis)
      pitch = rotation around body Z (lateral/right axis)
      yaw   = rotation around body Y (vertical axis)
    """
    roll = math.atan2(2.0 * (qw * qx - qy * qz),
                      1.0 - 2.0 * (qx * qx + qz * qz))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qx * qy + qz * qw))))
    yaw = math.atan2(2.0 * (qy * qw - qx * qz),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    return (roll * 180.0 / math.pi,
            pitch * 180.0 / math.pi,
            yaw * 180.0 / math.pi)


def has_column(rows: list[dict], col: str) -> bool:
    """Check if a column exists in the CSV data (by looking at the first row)."""
    return bool(rows) and col in rows[0]

def _lowpass_filter_gyro(data, time, cutoff_hz=100, order=4):
    """Apply a zero-phase Butterworth low-pass filter to gyro data.

    Filters each column of `data` (N x 3) using the sampling rate
    derived from the `time` vector.  Returns filtered copy."""
    if data.shape[0] < 2 * order + 1:
        return data
    diffs = np.diff(time)
    diffs = diffs[diffs > 1e-9]
    if len(diffs) == 0:
        return data
    dt = np.median(diffs)
    if dt <= 0:
        return data
    fs = 1.0 / dt
    nyq = 0.5 * fs
    cutoff_norm = cutoff_hz / nyq
    if cutoff_norm >= 1.0:
        return data
    sos = butter(order, cutoff_norm, btype="low", output="sos")
    return sosfiltfilt(sos, data, axis=0)

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot quadstate.csv data from simitl-playback"
    )
    parser.add_argument("path", nargs="?", default="quadstate.csv",
                        help="Path to the CSV file (default: quadstate.csv)")
    parser.add_argument("--start", type=float, default=0.0,
                        help="Start time in seconds (default: 0.0)")
    parser.add_argument("--length", "--len", type=float, default=0.0, dest="length",
                        help="Length of time window in seconds (default: all data)")
    args = parser.parse_args()

    rows_all = read_quadstate(args.path)
    if not rows_all:
        print(f"No data found in '{args.path}'")
        sys.exit(1)

    rows = filter_time_range(rows_all, args.start, args.length)
    if not rows:
        print(f"No data in specified time range ({args.start}s - {args.start + args.length}s)")
        sys.exit(1)

    print(f"Loaded {len(rows_all)} frames from '{args.path}'")
    print(f"Time range: {rows_all[0]['time']:.3f}s - {rows_all[-1]['time']:.3f}s")
    if args.start > 0.0 or args.length > 0.0:
        print(f"Plot window:  {rows[0]['time']:.3f}s - {rows[-1]['time']:.3f}s "
              f"({len(rows)} frames)")

    # Detect available columns
    has_ref_motor       = has_column(rows, "bb_motorOut_1")
    has_ref_raw_gyro    = has_column(rows, "bb_raw_gyro_x")
    has_ref_gyro        = has_column(rows, "bb_gyro_x")
    has_acc             = has_column(rows, "bb_acc_x")

    # Extract all raw columns
    time     = [r["time"]      for r in rows]
    pos_x    = [r["pos_x"]     for r in rows]
    pos_y    = [r["pos_y"]     for r in rows]
    pos_z    = [r["pos_z"]     for r in rows]
    vel_x    = [r["vel_x"]     for r in rows]
    vel_y    = [r["vel_y"]     for r in rows]
    vel_z    = [r["vel_z"]     for r in rows]
    angvel_x = [r["angvel_x"]  for r in rows]
    angvel_y = [r["angvel_y"]  for r in rows]
    angvel_z = [r["angvel_z"]  for r in rows]

    # Convert quaternion to Euler angles (matches main.cpp)
    euler = [quat_to_euler_degrees(r["ori_x"], r["ori_y"], r["ori_z"], r["ori_w"])
             for r in rows]
    roll   = [e[0] for e in euler]
    pitch  = [e[1] for e in euler]
    yaw    = [e[2] for e in euler]

    # Motor outputs (simulated)
    motor1 = [r["motorOut_1"] for r in rows]
    motor2 = [r["motorOut_2"] for r in rows]
    motor3 = [r["motorOut_3"] for r in rows]
    motor4 = [r["motorOut_4"] for r in rows]

    # Reference motor outputs (only in bb/olbb mode CSVs)
    if has_ref_motor:
        bb_motor1 = [r["bb_motorOut_1"] for r in rows]
        bb_motor2 = [r["bb_motorOut_2"] for r in rows]
        bb_motor3 = [r["bb_motorOut_3"] for r in rows]
        bb_motor4 = [r["bb_motorOut_4"] for r in rows]

    # Reference gyro (only in bb/olbb mode CSVs)
    if has_ref_gyro:
        bb_gyro_x = [r["bb_gyro_x"] for r in rows]
        bb_gyro_y = [r["bb_gyro_y"] for r in rows]
        bb_gyro_z = [r["bb_gyro_z"] for r in rows]

    if has_ref_raw_gyro:
        bb_raw_gyro_x = [r["bb_raw_gyro_x"] for r in rows]
        bb_raw_gyro_y = [r["bb_raw_gyro_y"] for r in rows]
        bb_raw_gyro_z = [r["bb_raw_gyro_z"] for r in rows]

    # --- Build graphs -------------------------------------------------------
    fig, ((ax_traj, ax_pos), (ax_ori, ax_vel), (ax_angvel, ax_motor)) = plt.subplots(
        3, 2, figsize=(13, 10)
    )

    # -- Graph 1: Top-down 2D trajectory (pos_x vs pos_z) -------------------
    ax_traj.plot(pos_x, pos_z, "b-", linewidth=0.8, label="Flight path")
    ax_traj.plot(pos_x[0], pos_z[0], "go", markersize=6, label="Start")
    ax_traj.plot(pos_x[-1], pos_z[-1], "ro", markersize=6, label="End")
    ax_traj.set_xlabel("pos_x (m)")
    ax_traj.set_ylabel("pos_z (m)")
    ax_traj.set_title("Top-down 2D Trajectory (Y-up: X-Z plane)")
    ax_traj.grid(True, alpha=0.3)
    ax_traj.axis("equal")
    ax_traj.legend()

    # -- Graph 2: Position components over time -----------------------------
    ax_pos.plot(time, pos_x, label="pos_x", linewidth=0.8)
    ax_pos.plot(time, pos_y, label="pos_y", linewidth=0.8)
    ax_pos.plot(time, pos_z, label="pos_z", linewidth=0.8)
    ax_pos.set_xlabel("Time (s)")
    ax_pos.set_ylabel("Position (m)")
    ax_pos.set_title("Position vs Time")
    ax_pos.grid(True, alpha=0.3)
    ax_pos.legend()

    # -- Graph 3: Orientation (Euler angles) over time ----------------------
    ax_ori.plot(time, roll, label="roll", linewidth=0.8)
    ax_ori.plot(time, pitch, label="pitch", linewidth=0.8)
    ax_ori.plot(time, yaw, label="yaw", linewidth=0.8)
    ax_ori.set_xlabel("Time (s)")
    ax_ori.set_ylabel("Angle (deg)")
    ax_ori.set_title("Orientation (Euler Angles) vs Time")
    ax_ori.grid(True, alpha=0.3)
    ax_ori.legend()

    # -- Graph 4: Linear velocity over time ---------------------------------
    ax_vel.plot(time, vel_x, label="vel_x", linewidth=0.8)
    ax_vel.plot(time, vel_y, label="vel_y", linewidth=0.8)
    ax_vel.plot(time, vel_z, label="vel_z", linewidth=0.8)
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel("Velocity (m/s)")
    ax_vel.set_title("Linear Velocity vs Time")
    ax_vel.grid(True, alpha=0.3)
    ax_vel.legend()

    # -- Graph 5: Angular velocity + gyro over time (deg/s) -----------------
    ax_angvel.plot(time, angvel_x, "C0-",  linewidth=0.8, label="sim gyro_x")
    ax_angvel.plot(time, angvel_y, "C1-",  linewidth=0.8, label="sim gyro_y")
    ax_angvel.plot(time, angvel_z, "C2-",  linewidth=0.8, label="sim gyro_z")

    if has_ref_gyro and has_ref_raw_gyro:
        ax_angvel.plot(time, bb_gyro_x, "C0--", linewidth=0.4, label="ref gyro_x")
        ax_angvel.plot(time, bb_gyro_y, "C1--", linewidth=0.4, label="ref gyro_y")
        ax_angvel.plot(time, bb_gyro_z, "C2--", linewidth=0.4, label="ref gyro_z")

        res = _lowpass_filter_gyro(
            np.column_stack([bb_raw_gyro_x, bb_raw_gyro_y, bb_raw_gyro_z]),
            time, 66, 4)

        bb_raw_gyro_x = res[:, 0]
        bb_raw_gyro_y = res[:, 1]
        bb_raw_gyro_z = res[:, 2]

        ax_angvel.plot(time, bb_raw_gyro_x, "C0--", linewidth=1, label="ref raw_gyro_x")
        ax_angvel.plot(time, bb_raw_gyro_y, "C1--", linewidth=1, label="ref raw_gyro_y")
        ax_angvel.plot(time, bb_raw_gyro_z, "C2--", linewidth=1, label="ref raw_gyro_z")
        ax_angvel.set_title("Angular Velocity / Gyro (sim vs ref)")
    else:
        ax_angvel.set_title("Angular Velocity / Gyro (sim)")

    ax_angvel.set_xlabel("Time (s)")
    ax_angvel.set_ylabel("Angular velocity (deg/s)")
    ax_angvel.grid(True, alpha=0.3)
    ax_angvel.legend(fontsize="small", ncol=2)

    # -- Graph 6: Motor outputs over time -----------------------------------
    motor_label = "sim" if has_ref_motor else ""
    ax_motor.plot(time, motor1, "C0-",  linewidth=0.8, label=f"{motor_label} M1".strip())
    ax_motor.plot(time, motor2, "C1-",  linewidth=0.8, label=f"{motor_label} M2".strip())
    ax_motor.plot(time, motor3, "C2-",  linewidth=0.8, label=f"{motor_label} M3".strip())
    ax_motor.plot(time, motor4, "C3-",  linewidth=0.8, label=f"{motor_label} M4".strip())

    if has_ref_motor:
        ax_motor.plot(time, bb_motor1, "C0--", linewidth=0.8, label="ref M1")
        ax_motor.plot(time, bb_motor2, "C1--", linewidth=0.8, label="ref M2")
        ax_motor.plot(time, bb_motor3, "C2--", linewidth=0.8, label="ref M3")
        ax_motor.plot(time, bb_motor4, "C3--", linewidth=0.8, label="ref M4")
        ax_motor.set_title("Motor Outputs (sim vs ref)")
    else:
        ax_motor.set_title("Motor Outputs (sim)")

    ax_motor.set_xlabel("Time (s)")
    ax_motor.set_ylabel("Motor output")
    ax_motor.grid(True, alpha=0.3)
    ax_motor.legend(fontsize="small", ncol=2)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
