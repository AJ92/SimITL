import numpy as np
import argparse
import csv

# Raw gyro ADC to rad/s conversion (standard Betaflight constant)
GYRO_RAW_TO_RADPS = 0.0010653


def load_blackbox_csv(path):
    """Load a Betaflight blackbox CSV and return column names + data.

    Utilizes Python's built-in compiled CSV module for faster execution and
    efficient memory utilization compared to hand-rolled line parsing.

    Returns:
        (col_names, data_array)
    """
    metadata = {}
    cols = []
    data_rows = []
    header_found = False

    # Read the file line-by-line using the csv reader to properly handle quoting
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            
            # The first value determines if we are in metadata or the header
            first_val = row[0].strip()
            
            if not header_found:
                # Check for standard column headers. Stripping double quotes 
                # handles variations in different CSV exporter versions.
                if first_val in ("loopIteration", "time") or first_val.strip('"') in ("loopIteration", "time"):
                    cols = [c.strip().strip('"') for c in row]
                    header_found = True
                else:
                    # Save key-value pairs of metadata present before the headers
                    if len(row) == 2:
                        metadata[row[0].strip().strip('"')] = row[1].strip().strip('"')
            else:
                # Parse data rows into floats
                parsed_row = []
                for val in row:
                    val = val.strip()
                    if not val or val == '""':
                        parsed_row.append(0.0)  # Handle empty cells safely
                    else:
                        try:
                            parsed_row.append(float(val))
                        except ValueError:
                            # Skip row if it contains corrupt or non-numeric characters
                            break
                
                # Verify that row length matches header length before keeping it
                if len(parsed_row) == len(cols):
                    data_rows.append(parsed_row)

    if not cols:
        raise ValueError("Could not find column header line in blackbox file")
    if not data_rows:
        raise ValueError("No data rows could be parsed")

    return cols, np.array(data_rows, dtype=float)


def find_active_windows(signal, threshold=10, min_len=20):
    """Find windows where the absolute derivative of signal exceeds threshold.
    Returns list of (start_idx, end_idx) tuples. This is useful to isolate
    regions where the pilot made deliberate stick movements.
    """
    deriv = np.abs(np.diff(signal))
    active = np.where(deriv > threshold)[0]
    if len(active) == 0:
        return []

    # Cluster indices that are close together into logical motion windows
    clusters = [[active[0]]]
    for i in active[1:]:
        if i - clusters[-1][-1] <= min_len:
            clusters[-1].append(i)
        else:
            clusters.append([i])

    windows = []
    for c in clusters:
        if len(c) >= 5:
            # Pad the window boundaries slightly to capture start and settle transients
            start = max(0, c[0] - 10)
            end = min(len(signal), c[-1] + 30)
            windows.append((start, end))
    return windows


def measure_delay_cross_correlation(ref_signal, response_signal, max_lag_samples=80):
    """Measure delay using cross-correlation on differentiated signals.
    Using derivatives emphasizes timing changes (steps/edges) and mitigates
    biases from low-frequency drift or steady-state offsets.
    """
    # Differentiate to turn step inputs into spikes
    ref_deriv = np.diff(ref_signal)
    resp_deriv = np.diff(response_signal)

    if len(ref_deriv) < 2 or len(resp_deriv) < 2:
        return 0

    # Subtract mean to isolate AC components
    ref_deriv = ref_deriv - np.mean(ref_deriv)
    resp_deriv = resp_deriv - np.mean(resp_deriv)

    # Normalize vectors to unit length to ensure normalized cross-correlation
    ref_norm = ref_deriv / (np.linalg.norm(ref_deriv) + 1e-10)
    resp_norm = resp_deriv / (np.linalg.norm(resp_deriv) + 1e-10)

    # Compute correlation across signals
    corr = np.correlate(resp_norm, ref_norm, mode="same")
    mid = len(corr) // 2
    n_look = min(max_lag_samples, len(corr) - mid)
    if n_look <= 0:
        return 0
    
    # Locate maximum correlation peak in the positive lag search window.
    # Using np.abs ensures we find the true physical delay even if an axis's 
    # sign convention is inverted between the sensors and the motor mixer.
    lag = int(np.argmax(np.abs(corr[mid:mid + n_look])))
    return lag


def measure_delay_phase(ref_signal, response_signal, max_lag_samples=80):
    """Measure delay by finding the lag that minimizes mean squared error
    between ref and a shifted version of response.
    
    Optimized to use a fixed overlap length to prevent variable-length 
    evaluation bias.
    """
    best_lag = 0
    best_mse = float("inf")

    # Demean signals to ignore constant offset differences
    ref = ref_signal - np.mean(ref_signal)
    resp = response_signal - np.mean(response_signal)

    max_lag = min(max_lag_samples, len(ref) // 4)
    if max_lag <= 0 or len(resp) <= max_lag:
        return 0

    # Ensure evaluation window length remains constant for all lag steps
    eval_len = len(ref) - max_lag

    for lag in range(0, max_lag):
        shifted = resp[lag : lag + eval_len]
        aligned = ref[:eval_len]
        
        mse = np.mean((aligned - shifted) ** 2)
        if mse < best_mse:
            best_mse = mse
            best_lag = lag

    return best_lag


def main():
    parser = argparse.ArgumentParser(
        description="Measure RC-to-gyro response delay from a Blackbox CSV"
    )
    parser.add_argument("csv_file", help="Path to blackbox CSV file")
    parser.add_argument("--start", type=float, default=None,
                        help="Start time in seconds (default: auto-detect active segment)")
    parser.add_argument("--length", type=float, default=None,
                        help="Duration in seconds (default: auto-detect)")
    args = parser.parse_args()

    # Load data
    cols, data = load_blackbox_csv(args.csv_file)

    # Build column index map
    idx = {name: i for i, name in enumerate(cols)}

    # Verify standard required columns
    required = ["rcCommand[0]", "rcCommand[1]", "rcCommand[2]",
                "motor[0]", "motor[1]", "motor[2]", "motor[3]",
                "setpoint[0]", "setpoint[1]", "setpoint[2]",
                "time"]
    for r in required:
        if r not in idx:
            print(f"Error: Required column '{r}' not found.")
            return

    # Attempt to locate raw gyro columns with automatic fallbacks
    # Prioritize 'debug' over 'gyroADC' for older logs with GYRO_SCALED active
    gyro_prefixes = ["debug", "gyroRaw", "gyroADC", "gyro"]
    gyro_cols = []
    for prefix in gyro_prefixes:
        test_cols = [f"{prefix}[0]", f"{prefix}[1]", f"{prefix}[2]"]
        if all(c in idx for c in test_cols):
            gyro_cols = test_cols
            break

    if not gyro_cols:
        print("Error: Could not locate standard raw gyro columns.")
        return
    else:
        print(f"Using gyro columns: {gyro_cols}")

    # Calculate sampling period using the median time differences 
    # to safeguard against dropped/skipped frames in the log
    time_us = data[:, idx["time"]]
    t_sec = (time_us - time_us[0]) / 1e6
    dt = np.median(np.diff(t_sec)) if len(t_sec) > 1 else 0.002
    fs = 1.0 / dt

    # Extract RC signals
    rc_roll = data[:, idx["rcCommand[0]"]]
    rc_pitch = data[:, idx["rcCommand[1]"]]
    rc_yaw = data[:, idx["rcCommand[2]"]]

    # Extract raw gyro and convert to rad/s
    gyro_raw_roll = data[:, idx[gyro_cols[0]]] * GYRO_RAW_TO_RADPS
    gyro_raw_pitch = data[:, idx[gyro_cols[1]]] * GYRO_RAW_TO_RADPS
    gyro_raw_yaw = data[:, idx[gyro_cols[2]]] * GYRO_RAW_TO_RADPS

    # Extract raw motor outputs
    motor = data[:, [idx[f"motor[{i}]"] for i in range(4)]]
    
    # Isolate individual axis commands using reversed mixer formulas (Betaflight Quad X layout)
    # Motor 1 is index 0 (Rear Right), Motor 2 is index 1 (Front Right)
    # Motor 3 is index 2 (Rear Left), Motor 4 is index 3 (Front Left)
    # Note: motor_pitch corrected to Front - Rear to align with positive pitch commands (nose up)
    motor_roll =  -motor[:, 0] - motor[:, 1] + motor[:, 2] + motor[:, 3]
    motor_pitch = -motor[:, 0] + motor[:, 1] - motor[:, 2] + motor[:, 3]
    motor_yaw =   -motor[:, 0] + motor[:, 1] + motor[:, 2] - motor[:, 3]

    # Extract setpoints
    sp_roll = data[:, idx["setpoint[0]"]]
    sp_pitch = data[:, idx["setpoint[1]"]]
    sp_yaw = data[:, idx["setpoint[2]"]]

    # ---- Establish analysis window ----
    windows = find_active_windows(rc_roll, threshold=8)

    print(f"Loaded {len(data)} frames, sample rate: {fs:.1f} Hz")
    print(f"Total duration: {t_sec[-1]:.1f}s")
    print(f"Detected {len(windows)} active window(s) in roll RC command")
    print()

    if args.start is not None:
        # Use user-specified window bounds
        start_us = time_us[0] + args.start * 1e6
        if args.length:
            end_us = start_us + args.length * 1e6
        else:
            end_us = time_us[-1]
        mask = (time_us >= start_us) & (time_us < end_us)
        indices = np.where(mask)[0]
        analysis_start = int(indices[0]) if len(indices) > 0 else 0
        analysis_end = int(indices[-1] + 1) if len(indices) > 0 else len(data)
    elif windows:
        # Select the active window containing the maximum overall RC stick rate change
        best_win = max(windows, key=lambda w: np.sum(np.abs(np.diff(rc_roll[w[0]:w[1]]))))
        analysis_start = best_win[0]
        analysis_end = best_win[1]
        args.start = t_sec[analysis_start]
        args.length = t_sec[analysis_end] - t_sec[analysis_start]
        print(f"Auto-selected window: {args.start:.3f}s - {args.start + args.length:.3f}s ({args.length:.3f}s)")
    else:
        # Fallback to the full log scope if no active sections are found
        analysis_start = 0
        analysis_end = len(data)
        print("No active windows found, using full range")

    n = analysis_end - analysis_start
    print(f"Analysis window: {n} samples ({n / fs:.2f}s)")
    print()

    # Slice the arrays to the active window
    def slc(sig):
        return sig[analysis_start:analysis_end]

    rc_r = slc(rc_roll)
    rc_p = slc(rc_pitch)
    rc_y = slc(rc_yaw)
    gr_r = slc(gyro_raw_roll)
    gr_p = slc(gyro_raw_pitch)
    gr_y = slc(gyro_raw_yaw)
    sp_r = slc(sp_roll)
    sp_p = slc(sp_pitch)
    sp_y = slc(sp_yaw)
    
    # Sliced axis-specific motor mix arrays
    mot_r = slc(motor_roll)
    mot_p = slc(motor_pitch)
    mot_y = slc(motor_yaw)

    # ---- Delay calculations ----
    print("=== Delay measurements (cross-correlation of derivative signals) ===\n")

    # 1. RC command -> raw gyro (Total loop latency)
    print("1) RC command -> raw gyro")
    print("   Total latency path: RX -> Filtering -> PID -> Motor -> Physics -> Gyro")
    for name, rc, gyro in [("Roll", rc_r, gr_r), ("Pitch", rc_p, gr_p), ("Yaw", rc_y, gr_y)]:
        lag = measure_delay_cross_correlation(rc, gyro, max_lag_samples=80)
        print(f"   {name}: {lag * dt * 1000:.2f} ms ({lag} samples)")

    print()

    # 2. Setpoint -> raw gyro (System physical response)
    print("2) Setpoint -> raw gyro")
    print("   Excludes RX/stick-filtering delay. Measures PID + hardware + physical response.")
    for name, sp, gyro in [("Roll", sp_r, gr_r), ("Pitch", sp_p, gr_p), ("Yaw", sp_y, gr_y)]:
        lag = measure_delay_cross_correlation(sp, gyro, max_lag_samples=80)
        print(f"   {name}: {lag * dt * 1000:.2f} ms ({lag} samples)")

    print()

    # 3. Setpoint -> motor (PID loop computation speed)
    print("3) Setpoint -> axis motor mix")
    print("   Pure PID computation + output protocol delay (should be minimal)")
    for name, sp, mot_axis in [("Roll", sp_r, mot_r), ("Pitch", sp_p, mot_p), ("Yaw", sp_y, mot_y)]:
        lag = measure_delay_cross_correlation(sp, mot_axis, max_lag_samples=20)
        # Warn if the axis is silent to prevent reading noise peaks
        warn_str = ""
        if np.std(sp) < 0.1:
            warn_str = " (Warning: inactive axis in this window)"
        print(f"   {name}: {lag * dt * 1000:.2f} ms ({lag} samples){warn_str}")

    print()

    # 4. RC -> motor (Input-to-actuation speed)
    print("4) RC command -> axis motor mix")
    print("   RX delay + stick filtering + PID processing delay")
    for name, rc, mot_axis in [("Roll", rc_r, mot_r), ("Pitch", rc_p, mot_p), ("Yaw", rc_y, mot_y)]:
        lag = measure_delay_cross_correlation(rc, mot_axis, max_lag_samples=80)
        warn_str = ""
        if np.std(rc) < 0.1:
            warn_str = " (Warning: inactive axis in this window)"
        print(f"   {name}: {lag * dt * 1000:.2f} ms ({lag} samples){warn_str}")

    print()

    # 5. Setpoint -> gyro (Using the alternative MSE alignment method)
    print("5) Setpoint -> raw gyro (MSE alignment method)")
    for name, sp, gyro in [("Roll", sp_r, gr_r), ("Pitch", sp_p, gr_p), ("Yaw", sp_y, gr_y)]:
        lag = measure_delay_phase(sp, gyro, max_lag_samples=80)
        print(f"   {name}: {lag * dt * 1000:.2f} ms ({lag} samples)")

    print()
    print("=== Interpretation ===")
    print(f"  Sample period: {dt * 1000:.3f} ms")
    print(f"  Window bounds: {t_sec[analysis_start]:.3f}s to {t_sec[analysis_end - 1]:.3f}s")
    print()
    print("  Note: Measurement resolution is limited to +/- 1 sample period.")
    print("  For tuning simulations, Setpoint -> Gyro (#2 or #5) represents the delay")
    print("  inherent to the physical system (ESC, motors, propellers, and frame inertia).")


if __name__ == "__main__":
    main()