import numpy as np
import argparse
import json
import os
import sys
import subprocess
import shutil
import re
from scipy.optimize import minimize, differential_evolution
from scipy.signal import butter, sosfiltfilt

M_PI = 3.14159265358979

# ---------------------------------------------------------------------------
# Config loader helpers (keep original logic for reading base config)
# ---------------------------------------------------------------------------

def _find_config_by_name(directory, target_name):
    """Scan JSON files in `directory`, return the path of the first whose
    "name" field matches `target_name`.  Raises ValueError if not found."""
    if not os.path.isdir(directory):
        raise ValueError(f"Config directory not found: {directory}")
    for entry in os.listdir(directory):
        if not entry.endswith(".json"):
            continue
        path = os.path.join(directory, entry)
        try:
            with open(path) as f:
                cfg = json.load(f)
            if cfg.get("name") == target_name:
                return path
        except Exception:
            continue
    raise ValueError(f"Config not found matching \"{target_name}\" in {directory}")


def load_quad_config(config_path):
    """Load a complete quadcopter configuration from a quad JSON file,
    resolving all referenced component configs (motor, propeller, battery).

    Returns a dict with keys: frame, motor, propeller, battery.
    Each value is a dict mirroring the JSON structure.
    """
    config_path = os.path.normpath(config_path)

    with open(config_path) as f:
        quad = json.load(f)

    # Config root is the parent of the "quad" directory
    cfg_root = os.path.dirname(os.path.dirname(config_path))

    result = {}

    # --- Frame parameters (from quad JSON directly) ---
    result["frame"] = {
        "dragArea":          vec3_from_obj(quad.get("frameDragArea", {"x": 0.008, "y": 0.0077, "z": 0.008})),
        "dragConstant":      float(quad.get("frameDragConstant", 1.2)),
        "mass":              float(quad.get("mass", 0.10)),
        "invInertia":        vec3_from_obj(quad.get("invInertia", {"x": 900.0, "y": 480.0, "z": 900.0})),
        "motorPos": [
            vec3_from_obj(quad.get("motor1Pos", {"x": 0.067, "y": 0.005, "z": -0.056})),
            vec3_from_obj(quad.get("motor2Pos", {"x": 0.067, "y": 0.005, "z":  0.056})),
            vec3_from_obj(quad.get("motor3Pos", {"x": -0.067, "y": 0.005, "z": -0.056})),
            vec3_from_obj(quad.get("motor4Pos", {"x": -0.067, "y": 0.005, "z":  0.056})),
        ],
        "motorDir": [
            float(quad.get("motor1Dir", -1.0)),
            float(quad.get("motor2Dir", 1.0)),
            float(quad.get("motor3Dir", 1.0)),
            float(quad.get("motor4Dir", -1.0)),
        ],
        "motorVariance":     vec4_from_obj(quad.get("motorVariance", {"x": 1.0, "y": 1.0, "z": 1.0, "w": 1.0})),
        "gyroBaseNoiseAmp":  float(quad.get("gyroBaseNoiseAmp", 0.015)),
        "gyrobaseNoiseFreq": float(quad.get("gyrobaseNoiseFreq", 333.0)),
        "minPropWashSpeed":      float(quad.get("minPropWashSpeed", 1.0)),
        "maxPropWashSpeed":      float(quad.get("maxPropWashSpeed", 18.0)),
        "propWashAngleOfAttack": float(quad.get("propWashAngleOfAttack", 0.5)),
        "propWashFactor":        float(quad.get("propWashFactor", 1.0)),
        "ambientTemp":       float(quad.get("ambientTemp", 25.0)),
    }

    # --- Motor config ---
    motor_name = quad.get("motor", "")
    if motor_name:
        motor_path = _find_config_by_name(os.path.join(cfg_root, "motor"), motor_name)
        with open(motor_path) as f:
            mc = json.load(f)
    else:
        mc = {}
    result["motor"] = {
        "kv":        float(mc.get("motorKV", 3300.0)),
        "r":         float(mc.get("motorR", 0.135)),
        "i0":        float(mc.get("motorI0", 0.8)),
        "rth":       float(mc.get("motorRth", 12.0)),
        "cth":       float(mc.get("motorCth", 9.5)),
        "maxT":      float(mc.get("motorMaxT", 128.0)),
        "imbalance": vec3_from_obj(mc.get("motorImbalance", {"x": 13.0, "y": 7.0, "z": 5.0})),
        "mass":      float(mc.get("mass", 0.019)),
    }
    result["_motor_path"] = motor_path
    result["_motor_name"] = motor_name

    # --- Propeller config ---
    prop_name = quad.get("propeller", "")
    if prop_name:
        prop_path = _find_config_by_name(os.path.join(cfg_root, "propeller"), prop_name)
        with open(prop_path) as f:
            pc = json.load(f)
    else:
        pc = {}
    result["propeller"] = {
        "bladeCount":   int(pc.get("bladeCount", 3)),
        "maxRpm":       float(pc.get("propMaxRpm", 36000.0)),
        "aFactor":      float(pc.get("propAFactor", 6.25e-9)),
        "torqueFactor": float(pc.get("propTorqueFactor", 0.009)),
        "inertia":      float(pc.get("propInertia", 2.45e-6)),
        "thrustFactor": vec3_from_obj(pc.get("propThrustFactor", {"x": -1.11e-5, "y": -0.14, "z": 9.55})),
        "harmonic1Amp": float(pc.get("propHarmonic1Amp", 0.1)),
        "harmonic2Amp": float(pc.get("propHarmonic2Amp", 0.3)),
        "mass":         float(pc.get("mass", 0.0028)),
    }
    result["_prop_path"] = prop_path
    result["_prop_name"] = prop_name

    # --- Battery config ---
    bat_name = quad.get("battery", "")
    if bat_name:
        bat_path = _find_config_by_name(os.path.join(cfg_root, "battery"), bat_name)
        with open(bat_path) as f:
            bc = json.load(f)
    else:
        bc = {}
    result["battery"] = {
        "maxVoltageSag":   float(bc.get("maxVoltageSag", 1.3)),
        "cellCount":       int(bc.get("batCellCount", 4)),
        "capacityCharged": float(bc.get("batCapacityCharged", 850.0)),
        "capacity":        float(bc.get("batCapacity", 850.0)),
        "mass":            float(bc.get("mass", 0.11)),
    }
    result["_bat_path"] = _find_config_by_name(os.path.join(cfg_root, "battery"), bat_name) if bat_name else ""
    result["_bat_name"] = bat_name

    return result


def vec3_from_obj(obj):
    """Extract (x, y, z) from a JSON object like {"x": ..., "y": ..., "z": ...}."""
    return np.array([float(obj.get("x", 0)),
                     float(obj.get("y", 0)),
                     float(obj.get("z", 0))], dtype=float)


def vec4_from_obj(obj):
    """Extract (x, y, z, w) from a JSON object."""
    return np.array([float(obj.get("x", 0)),
                     float(obj.get("y", 0)),
                     float(obj.get("z", 0)),
                     float(obj.get("w", 0))], dtype=float)


# ---------------------------------------------------------------------------
# Tuneable parameter definitions
# ---------------------------------------------------------------------------
TUNEABLE_PARAMS = [
    {'name': 'invInertia.x',  'section': 'frame',     'json_obj': 'invInertia', 'json_key': 'x',
     'extract': lambda c: float(c['frame']['invInertia'][0]), 'fmt': '.1f',
     'bounds_min': 300.0, 'bounds_max': 3000.0},
    {'name': 'invInertia.y',  'section': 'frame',     'json_obj': 'invInertia', 'json_key': 'y',
     'extract': lambda c: float(c['frame']['invInertia'][1]), 'fmt': '.1f',
     'bounds_min': 200, 'bounds_max': 3000.0},
    {'name': 'invInertia.z',  'section': 'frame',     'json_obj': 'invInertia', 'json_key': 'z',
     'extract': lambda c: float(c['frame']['invInertia'][2]), 'fmt': '.1f',
     'bounds_min': 300.0, 'bounds_max': 3000.0},
    {'name': 'propThrustFactor.z', 'section': 'propeller', 'json_obj': 'propThrustFactor', 'json_key': 'z',
     'extract': lambda c: float(c['propeller']['thrustFactor'][2]), 'fmt': '.6f',
     'bounds_min': 7, 'bounds_max': 25},
    {'name': 'propInertia',   'section': 'propeller', 'json_obj': None,         'json_key': 'propInertia',
     'extract': lambda c: float(c['propeller']['inertia']), 'fmt': '.3e',
     'bounds_min': 0.00000001, 'bounds_max': 0.00001},
    {'name': 'propAFactor',   'section': 'propeller', 'json_obj': None,         'json_key': 'propAFactor',
     'extract': lambda c: float(c['propeller']['aFactor']), 'fmt': '.3e',
     'bounds_min': 1.0e-11, 'bounds_max': 1.0e-8},
    {'name': 'propTorqueFactor', 'section': 'propeller', 'json_obj': None,     'json_key': 'propTorqueFactor',
     'extract': lambda c: float(c['propeller']['torqueFactor']), 'fmt': '.6f',
     'bounds_min': 0.00001, 'bounds_max': 0.1},
    {'name': 'motorR',          'section': 'motor',      'json_obj': None,         'json_key': 'motorR',
     'extract': lambda c: float(c['motor']['r']), 'fmt': '.4f',
     'bounds_min': 0.001, 'bounds_max': 0.5},
    {'name': 'motorI0',          'section': 'motor',      'json_obj': None,         'json_key': 'motorI0',
     'extract': lambda c: float(c['motor']['i0']), 'fmt': '.4f',
     'bounds_min': 0.001, 'bounds_max': 1.0},
    {'name': 'frameDragConstant', 'section': 'frame',   'json_obj': None,         'json_key': 'frameDragConstant',
     'extract': lambda c: float(c['frame']['dragConstant']), 'fmt': '.3f',
     'bounds_min': 0.5, 'bounds_max': 2.0},
]


# ---------------------------------------------------------------------------
# Result directory setup
# ---------------------------------------------------------------------------

def setup_result_directory(cfg, config_path, result_dir):
    """Create the result directory structure and copy all referenced config
    files into it.  Returns the path to the result quad config."""
    # Create subdirectories
    for sub in ["quad", "motor", "propeller", "battery"]:
        os.makedirs(os.path.join(result_dir, sub), exist_ok=True)

    # Copy quad config
    quad_name = os.path.basename(config_path)
    result_quad_path = os.path.join(result_dir, "quad", quad_name)
    shutil.copy2(config_path, result_quad_path)

    # Copy motor config
    motor_name = cfg.get("_motor_name", "")
    if motor_name:
        src = cfg["_motor_path"]
        dst = os.path.join(result_dir, "motor", os.path.basename(src))
        shutil.copy2(src, dst)

    # Copy propeller config
    prop_name = cfg.get("_prop_name", "")
    if prop_name:
        src = cfg["_prop_path"]
        dst = os.path.join(result_dir, "propeller", os.path.basename(src))
        shutil.copy2(src, dst)

    # Copy battery config
    bat_name = cfg.get("_bat_name", "")
    if bat_name:
        src = cfg["_bat_path"]
        dst = os.path.join(result_dir, "battery", os.path.basename(src))
        shutil.copy2(src, dst)

    return result_quad_path


def _apply_params_to_json(file_path, params_for_section, params):
    """Load a JSON file, apply the given parameter definitions, save back."""
    with open(file_path) as f:
        data = json.load(f)
    for idx, pdef in params_for_section:
        val = float(params[idx])
        obj = data
        if pdef.get('json_obj'):
            obj = data.setdefault(pdef['json_obj'], {})
        obj[pdef['json_key']] = val
    with open(file_path, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def modify_tuned_config(config_path, result_dir, params):
    """Update the config files in the result directory with the current
    tuning parameters."""
    # Group params by section for efficient file I/O
    by_section = {}
    for i, pdef in enumerate(TUNEABLE_PARAMS):
        by_section.setdefault(pdef['section'], []).append((i, pdef))

    # Quad config (frame params)
    if 'frame' in by_section:
        _apply_params_to_json(config_path, by_section['frame'], params)

    # Motor config
    if 'motor' in by_section:
        motor_dir = os.path.join(result_dir, "motor")
        for fname in os.listdir(motor_dir):
            if fname.endswith(".json"):
                _apply_params_to_json(os.path.join(motor_dir, fname), by_section['motor'], params)

    # Propeller config
    if 'propeller' in by_section:
        prop_dir = os.path.join(result_dir, "propeller")
        for fname in os.listdir(prop_dir):
            if fname.endswith(".json"):
                _apply_params_to_json(os.path.join(prop_dir, fname), by_section['propeller'], params)

    # Battery config
    if 'battery' in by_section:
        bat_dir = os.path.join(result_dir, "battery")
        for fname in os.listdir(bat_dir):
            if fname.endswith(".json"):
                _apply_params_to_json(os.path.join(bat_dir, fname), by_section['battery'], params)


# ---------------------------------------------------------------------------
# C++ simulation via simitl-playback
# ---------------------------------------------------------------------------

def find_playback_binary():
    """Locate the simitl-playback binary relative to this script."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    is_win = sys.platform in ('win32', 'msys', 'cygwin')

    if is_win:
        candidates = [
            os.path.join(script_dir, "..", "..", "build", "win", "install", "bin", "simitl-playback.exe"),
            os.path.join(script_dir, "..", "..", "build", "linux", "install", "bin", "simitl-playback"),
            "simitl-playback",
        ]
    else:
        candidates = [
            os.path.join(script_dir, "..", "..", "build", "linux", "install", "bin", "simitl-playback"),
            os.path.join(script_dir, "..", "..", "build", "win", "install", "bin", "simitl-playback.exe"),
            "simitl-playback",
        ]
    for c in candidates:
        if os.path.isfile(c) or shutil.which(c):
            return c
    raise FileNotFoundError("Could not find simitl-playback binary")


def run_simitl_playback(bb_file, config_path, start, length, working_dir, open_loop=False):
    """Run simitl-playback and wait for completion."""
    binary = find_playback_binary()
    mode = "olbb" if open_loop else "bb"
    csv_name = f"quadstate_{mode}.csv"
    csv_path = os.path.join(working_dir, csv_name)

    # Remove stale CSV if present
    if os.path.isfile(csv_path):
        os.remove(csv_path)

    cmd = [
        binary,
        mode, bb_file,
        "--config", config_path,
        "--start", str(start),
        "--len", str(length),
        "-ff", "-ns", "-no"
    ]

    subprocess.run(cmd, cwd=working_dir, capture_output=True, text=True)

    if not os.path.isfile(csv_path):
        raise RuntimeError(f"simitl-playback did not produce {csv_path}")

    return csv_path


def load_playback_csv(csv_path):
    """Load the simitl-playback output CSV and extract variables."""
    with open(csv_path) as f:
        header = f.readline().strip()

    cols = header.split(",")
    idx = {name: i for i, name in enumerate(cols)}

    required = ["angvel_x", "angvel_y", "angvel_z",
                "bb_gyro_x", "bb_gyro_y", "bb_gyro_z",
                "bb_acc_x", "bb_acc_y", "bb_acc_z",
                "motorOut_1", "motorOut_2", "motorOut_3", "motorOut_4",
                "bb_motorOut_1", "bb_motorOut_2", "bb_motorOut_3", "bb_motorOut_4"]
    for r in required:
        if r not in idx:
            raise ValueError(f"Column '{r}' not found in CSV header: {header}")

    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)

    if data.ndim == 1:
        data = data.reshape(1, -1)

    result = {
        "sim_gyro": data[:, [idx["angvel_x"], idx["angvel_y"], idx["angvel_z"]]],
        "ref_gyro": data[:, [idx["bb_raw_gyro_x"], idx["bb_raw_gyro_y"], idx["bb_raw_gyro_z"]]],
        "ref_acc":  data[:, [idx["bb_acc_x"], idx["bb_acc_y"], idx["bb_acc_z"]]],
        "sim_motor": data[:, [idx["motorOut_1"], idx["motorOut_2"], idx["motorOut_3"], idx["motorOut_4"]]],
        "ref_motor": data[:, [idx["bb_motorOut_1"], idx["bb_motorOut_2"], idx["bb_motorOut_3"], idx["bb_motorOut_4"]]],
    }

    if "time" in idx:
        result["time"] = data[:, idx["time"]]

    if "rc0" in idx and "rc1" in idx and "rc2" in idx and "rc3" in idx:
        result["rc"] = data[:, [idx["rc0"], idx["rc1"], idx["rc2"], idx["rc3"]]]

    return result


# ---------------------------------------------------------------------------
# Configuration summary
# ---------------------------------------------------------------------------

def compute_total_mass(cfg):
    """Total AUW = frame + 4*motors + 4*props + battery."""
    return (cfg["frame"]["mass"]
            + 4.0 * cfg["motor"]["mass"]
            + 4.0 * cfg["propeller"]["mass"]
            + cfg["battery"]["mass"])


def print_config(cfg):
    """Print all configuration parameters to stdout."""
    total_mass = compute_total_mass(cfg)

    print()
    print("-- Quad Config --")
    f = cfg["frame"]
    print("Frame:")
    print(f"  mass (dry):       {f['mass']} kg")
    print(f"  invInertia:       ({f['invInertia'][0]:.1f}, {f['invInertia'][1]:.1f}, {f['invInertia'][2]:.1f})")
    print(f"  dragArea:         ({f['dragArea'][0]}, {f['dragArea'][1]}, {f['dragArea'][2]})")
    print(f"  dragConstant:     {f['dragConstant']}")
    print(f"  motorPositions:   [{', '.join(f'({p[0]}, {p[1]}, {p[2]})' for p in f['motorPos'])}]")
    print(f"  motorDirections:  [{', '.join(f'{d}' for d in f['motorDir'])}]")
    print(f"  motorVariance:    ({f['motorVariance'][0]}, {f['motorVariance'][1]}, {f['motorVariance'][2]}, {f['motorVariance'][3]})")
    print(f"  gyroNoiseAmp:     {f['gyroBaseNoiseAmp']}")
    print(f"  gyroNoiseFreq:    {f['gyrobaseNoiseFreq']} Hz")
    print(f"  propWash:         min={f['minPropWashSpeed']}, max={f['maxPropWashSpeed']}, "
          f"aoa={f['propWashAngleOfAttack']}, factor={f['propWashFactor']}")

    m = cfg["motor"]
    print("Motor:")
    print(f"  kv:               {m['kv']}")
    print(f"  r:                {m['r']} ohm")
    print(f"  i0:               {m['i0']} A")
    print(f"  mass:             {m['mass']} kg")
    print(f"  imbalance:        ({m['imbalance'][0]}, {m['imbalance'][1]}, {m['imbalance'][2]})")

    p = cfg["propeller"]
    print("Propeller:")
    print(f"  blades:           {p['bladeCount']}")
    print(f"  maxRpm:           {p['maxRpm']}")
    print(f"  aFactor:          {p['aFactor']}")
    print(f"  torqueFactor:     {p['torqueFactor']}")
    print(f"  inertia:          {p['inertia']} kg*m^2")
    print(f"  thrustFactor:     ({p['thrustFactor'][0]}, {p['thrustFactor'][1]}, {p['thrustFactor'][2]})")
    print(f"  mass:             {p['mass']} kg")

    b = cfg["battery"]
    print("Battery:")
    print(f"  cells:            {b['cellCount']}S")
    print(f"  maxVoltageSag:    {b['maxVoltageSag']} V")
    print(f"  capacity:         {int(b['capacityCharged'])} mAh")
    print(f"  mass:             {b['mass']} kg")

    print(f"Total AUW:         {total_mass} kg")
    print()


# ---------------------------------------------------------------------------
# Optimization objective
# ---------------------------------------------------------------------------

_ANSI_RE = re.compile(r'\033\[[0-9;]*m')


def _visible_len(s):
    """Return the visible length of a string, ignoring ANSI escape codes."""
    return len(_ANSI_RE.sub('', s))


def _print_parts(parts, sep="  "):
    """Print a list of parts, wrapping at terminal width so no line exceeds it."""
    width = shutil.get_terminal_size().columns
    lines = []
    cur = []
    for part in parts:
        test = sep.join(cur + [part])
        if _visible_len(test) > width and cur:
            lines.append(sep.join(cur))
            cur = [part]
        else:
            cur.append(part)
    if cur:
        lines.append(sep.join(cur))
    for line in lines:
        print(line)


# Track state for objective-function display
_prev_params = None   # params from the previous call (for green highlighting)
_first_mse = None     # MSE from the first call (for MSE diff display)
_orig_params = None   # params from the first call (for per-param diff display)


def _print_objective_line(mse, params, *, first_mse=None, prev_params=None, orig_params=None):
    """Print a color-coded objective evaluation line.

    Every parameter shows its physical value (denormalized) and its difference
    from the original initial guess.
    """
    GREEN = "\033[32m"
    RED = "\033[31m"
    RESET = "\033[0m"

    if first_mse is None:
        first_mse = mse
        orig_params = params.copy()
        parts = [f"mse {mse:.3f}"]
        for i, pdef in enumerate(TUNEABLE_PARAMS):
            parts.append(f"{pdef['name']} {params[i]:{pdef['fmt']}}")
        _print_parts(parts)
        return first_mse, params.copy(), orig_params

    mse_diff = mse - first_mse
    parts = [f"mse {mse:.3f} {RED}({mse_diff:+.3f}){RESET}"]
    for i, pdef in enumerate(TUNEABLE_PARAMS):
        val = params[i]
        orig = orig_params[i]
        prev = prev_params[i]
        orig_diff = val - orig
        iter_diff = val - prev
        changed_this_iter = abs(iter_diff) > 1e-16
        if changed_this_iter:
            parts.append(f"{GREEN}{pdef['name']} {val:{pdef['fmt']}}{RESET}"
                         f"{RED}({orig_diff:+{pdef['fmt']}}){RESET}")
        else:
            parts.append(f"{pdef['name']} {val:{pdef['fmt']}}"
                         f"{RED}({orig_diff:+{pdef['fmt']}}){RESET}")
    _print_parts(parts)
    return first_mse, params.copy(), orig_params

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


def objective_function(normalized_params, initial_guess_raw, bb_file, result_quad_path, result_dir, start, length, working_dir,
                       bounds_min, bounds_max, open_loop):
    """Run the C++ simulation with normalized parameters, scaling them to physical values."""
    global _prev_params, _first_mse, _orig_params

    # --- PENALTY WALL START ---
    penalty = 0.0
    for i in range(len(normalized_params)):
        val = normalized_params[i]
        lo = bounds_min[i]
        hi = bounds_max[i]
        if val < lo:
            diff = lo - val
            penalty += 1e4 * diff + 1e5 * diff ** 2
        elif val > hi:
            diff = val - hi
            penalty += 1e4 * diff + 1e5 * diff ** 2
    # --- PENALTY WALL END ---

    clamped_norm = np.clip(normalized_params, bounds_min, bounds_max)
    raw_params = clamped_norm * initial_guess_raw

    modify_tuned_config(result_quad_path, result_dir, raw_params)

    try:
        csv_path = run_simitl_playback(bb_file, result_quad_path, start, length, working_dir, open_loop)
    except Exception as e:
        print(f"  simitl-playback error: {e}")
        return 1e9 + penalty

    try:
        result = load_playback_csv(csv_path)
    except Exception as e:
        print(f"  CSV load error: {e}")
        return 1e9 + penalty

    sim_gyro = result["sim_gyro"]
    ref_gyro = result["ref_gyro"]

    # Protect against simulation divergence (NaNs or Infs)
    if np.any(np.isnan(sim_gyro)) or np.any(np.isinf(sim_gyro)):
        return 1e9 + penalty

    if "time" in result:
        sim_gyro = _lowpass_filter_gyro(sim_gyro, result["time"])
        ref_gyro = _lowpass_filter_gyro(ref_gyro, result["time"])

    # Calculate Standard Deviations (Amplitude) per axis
    std_ref = np.std(ref_gyro, axis=0)
    std_sim = np.std(sim_gyro, axis=0)
    
    # Base threshold to prevent division by near-zero on quiet axes
    min_std = np.max(std_ref) * 0.05 + 1e-6
    std_ref_safe = np.maximum(std_ref, min_std)
    
    # 1. NORMALIZED MSE
    var_ref_safe = std_ref_safe ** 2
    mse_gyro_axis = np.mean((sim_gyro - ref_gyro) ** 2, axis=0)
    norm_mse_gyro = np.mean(mse_gyro_axis / var_ref_safe)
    
    # 2. AMPLITUDE PENALTY (Kills the "Flat Line" Trap)
    # If the drone doesn't move (std_sim = 0), this generates a massive penalty.
    # If the drone oscillates wildly, it also generates a massive penalty.
    std_mismatch = (std_sim - std_ref) / std_ref_safe
    std_penalty = np.mean(std_mismatch ** 2)
    
    # 3. CORRELATION PENALTY (Phase / Shape alignment)
    # Helps the optimizer find the correct timing even if amplitude is currently wrong.
    corrs = []
    for i in range(3):
        if std_ref_safe[i] > 1e-5 and std_sim[i] > 1e-5:
            r = np.corrcoef(ref_gyro[:, i], sim_gyro[:, i])[0, 1]
            # 1 - r maps perfect correlation to 0.0, and flat/inverted to 1.0+
            corrs.append(1.0 - r if not np.isnan(r) else 1.0)
        else:
            corrs.append(1.0)
    corr_penalty = np.mean(corrs)
    
    # COMBINED COST FUNCTION
    # Weighting: 
    # - 3.0 on Amplitude ensures a flat line (cost ~ 5.0) is instantly rejected in favor 
    #   of a moving but out-of-phase signal (cost ~ 3.0).
    # - 1.0 on Correlation guides it into phase alignment.
    # - 1.0 on MSE locks in the exact numerical fit.
    mse = float(norm_mse_gyro + 3.0 * std_penalty + 1.0 * corr_penalty)

    total_cost = mse + penalty

    _first_mse, _prev_params, _orig_params = _print_objective_line(
        total_cost, raw_params, first_mse=_first_mse, prev_params=_prev_params, orig_params=_orig_params
    )
    return total_cost


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Tune quadcopter physics parameters from Blackbox log using simitl-playback"
    )
    parser.add_argument("csv_file", nargs="?", default="btfl_079.bbl.csv",
                        help="Path to Blackbox CSV file (default: btfl_079.bbl.csv)")
    parser.add_argument("--config", "-c", default="config/quad/vtx-slayer-one-4.json",
                        help="Path to quad config JSON (default: config/quad/vtx-slayer-one-4.json)")
    parser.add_argument("--start", "-s", type=float, default=3.0,
                        help="Start time in seconds from beginning of log (default: 3.0)")
    parser.add_argument("--length", "-l", type=float, default=2.0,
                        help="Duration in seconds to analyze (default: 2.0)")
    parser.add_argument("--maxiter", type=int, default=100,
                        help="Maximum optimization iterations (default: 100)")
    parser.add_argument("--result-dir", default="result",
                        help="Directory for modified configs and working files (default: result)")
    parser.add_argument("--list-configs", action="store_true",
                        help="List available quad config files and exit")
    parser.add_argument("--open-loop", "-ol", action="store_true",
                        help="Use open-loop mode (bypass PID, feed motor values directly)")
    parser.add_argument("--differential_evolution", "-de", action="store_true",
                    help="Use differential evolution")

    args = parser.parse_args()

    # --- List available configs ---
    if args.list_configs:
        quad_dir = os.path.join(os.path.dirname(os.path.normpath(args.config)), "..", "quad")
        if not os.path.isdir(quad_dir):
            quad_dir = "config/quad"
        if os.path.isdir(quad_dir):
            print("Available quad configs:")
            for entry in sorted(os.listdir(quad_dir)):
                if entry.endswith(".json"):
                    path = os.path.join(quad_dir, entry)
                    try:
                        with open(path) as f:
                            c = json.load(f)
                        name = c.get("name", entry)
                        print(f"  {path}  ({name})")
                    except Exception:
                        print(f"  {path}")
        else:
            print(f"Quad config directory not found: {quad_dir}")
        exit(0)

    # Normalize paths
    config_path = os.path.normpath(args.config)
    bb_file = os.path.normpath(args.csv_file)
    result_dir = os.path.normpath(args.result_dir)
    working_dir = os.path.dirname(os.path.abspath(__file__))

    # Make paths absolute where relative
    if not os.path.isabs(config_path):
        config_path = os.path.join(working_dir, config_path)
    if not os.path.isabs(bb_file):
        bb_file = os.path.join(working_dir, bb_file)
    if not os.path.isabs(result_dir):
        result_dir = os.path.join(working_dir, result_dir)

    # --- Load base configuration ---
    print(f"Loading quad config: {config_path}")
    cfg = load_quad_config(config_path)
    print_config(cfg)

    # --- Setup result directory ---
    print(f"Setting up result directory: {result_dir}")
    result_quad_path = setup_result_directory(cfg, config_path, result_dir)
    print(f"Result quad config: {result_quad_path}")

    # --- Get Initial Physical Values ---
    initial_guess = [pdef['extract'](cfg) for pdef in TUNEABLE_PARAMS]

    parts = []
    for i, pdef in enumerate(TUNEABLE_PARAMS):
        parts.append(f"{pdef['name']}={initial_guess[i]:{pdef['fmt']}}")
    print(f"Initial guess: {', '.join(parts)}")
    print(f"Fixed parameters: motor_kV={cfg['motor']['kv']}, motor_I0={cfg['motor']['i0']}, "
          f"prop_thrust_factors=({cfg['propeller']['thrustFactor'][0]}, "
          f"{cfg['propeller']['thrustFactor'][1]}, {cfg['propeller']['thrustFactor'][2]}), "
          f"prop_max_rpm={cfg['propeller']['maxRpm']}")

    mode_name = "olbb (open-loop)" if args.open_loop else "bb (closed-loop)"
    print()
    print("--- Using C++ simulation (simitl-playback) ---")
    binary_path = find_playback_binary()
    print(f"Binary: {binary_path}")
    print(f"Mode:   {mode_name}")
    mode_arg = "olbb" if args.open_loop else "bb"
    print(f"Playback args: {mode_arg} {bb_file} --config {result_quad_path} "
          f"--start {args.start} --len {args.length} -ff -ns")
    print()

    # --- Setup Parameter Normalization ---
    # Convert parameters into a normalized representation starting at 1.0.
    initial_guess_raw = np.array(initial_guess, dtype=float)
    normalized_guess = np.ones(len(TUNEABLE_PARAMS), dtype=float)

    # Convert absolute bounds from TUNEABLE_PARAMS to normalized bounds.
    # TUNEABLE_PARAMS defines bounds_min/bounds_max as absolute physical
    # values.  The optimization works on normalized parameters (starting at
    # 1.0 = 1x the initial guess), so we divide by initial_guess_raw to get
    # the corresponding normalized range.
    bounds_min = np.array([
        pdef['bounds_min'] / initial_guess_raw[i]
        for i, pdef in enumerate(TUNEABLE_PARAMS)
    ])
    bounds_max = np.array([
        pdef['bounds_max'] / initial_guess_raw[i]
        for i, pdef in enumerate(TUNEABLE_PARAMS)
    ])

    # Pack bounds for SciPy's methods
    scipy_bounds = list(zip(bounds_min, bounds_max))

    # --- Run a quick sanity check before optimization ---
    print("Running initial evaluation...")
    init_mse = objective_function(
        normalized_guess,
        initial_guess_raw,
        bb_file, result_quad_path, result_dir,
        args.start, args.length, working_dir,
        bounds_min, bounds_max, args.open_loop
    )
    print(f"Initial MSE: {init_mse:.3f}")
    print()

    # --- Optimization ---
    print(f"Running optimization (max {args.maxiter} iterations)...")
    print()

    res = {}

    if args.differential_evolution:
        res = differential_evolution(
            objective_function,
            scipy_bounds,
            args=(initial_guess_raw, bb_file, result_quad_path, result_dir, args.start, args.length, working_dir,
                bounds_min, bounds_max, args.open_loop),
            maxiter=args.maxiter,
            popsize=8,
            tol=0,
            disp=True,
            polish=False,
        )
    else:
        res = minimize(
            objective_function,
            normalized_guess,
            bounds=scipy_bounds,  # <-- Pass bounds explicitly here!
            args=(initial_guess_raw, bb_file, result_quad_path, result_dir, args.start, args.length, working_dir,
                bounds_min, bounds_max, args.open_loop),
            method="Powell",
            options={"disp": True, "maxiter": args.maxiter},
        )

    # Extract optimization results and map them back to physical coordinates
    normalized_optimal = res.x.copy()
    clamped_normalized = np.clip(normalized_optimal, bounds_min, bounds_max)

    raw_params = normalized_optimal * initial_guess_raw
    clamped_params = clamped_normalized * initial_guess_raw

    def _clamp_note(raw_norm, lo, hi):
        """Return a suffix note if the normalized parameter was clamped."""
        if raw_norm <= lo + 1e-9:
            return "  [clamped at lower bound]"
        if raw_norm >= hi - 1e-9:
            return "  [clamped at upper bound]"
        return ""

    print()
    print("=== TUNING RESULTS ===")
    for i, pdef in enumerate(TUNEABLE_PARAMS):
        raw_val = raw_params[i]
        clamped_val = clamped_params[i]
        raw_norm = res.x[i]
        
        # Check boundary encounters on the normalized parameters
        if raw_norm <= bounds_min[i] + 1e-9 or raw_norm >= bounds_max[i] - 1e-9:
            note = _clamp_note(raw_norm, bounds_min[i], bounds_max[i])
            print(f"Optimal {pdef['name']}: {clamped_val:{pdef['fmt']}} (raw: {raw_val:{pdef['fmt']}}){note}")
        else:
            print(f"Optimal {pdef['name']}: {clamped_val:{pdef['fmt']}}")
            
    print(f"Config source: {args.config}")
    print(f"Result dir: {result_dir}")

    # Write final tuned parameters into the result directory config files
    print()
    print(f"Writing final parameters to {result_dir}...")
    modify_tuned_config(result_quad_path, result_dir, clamped_params)
    print("Done.")

    # Offer to update the original config using the denormalized, clamped values
    section_paths = {
        'frame':     config_path,
        'motor':     cfg.get('_motor_path', ''),
        'propeller': cfg.get('_prop_path', ''),
        'battery':   cfg.get('_bat_path', ''),
    }
    print()
    print("To apply these values, edit your config files:")
    for section_name in ['frame', 'motor', 'propeller', 'battery']:
        path = section_paths[section_name]
        if not path:
            continue
        section_params = [(i, p) for i, p in enumerate(TUNEABLE_PARAMS) if p['section'] == section_name]
        if not section_params:
            continue
        print(f"  {path}")
        for idx, pdef in section_params:
            key = f"{pdef['json_obj']}.{pdef['json_key']}" if pdef['json_obj'] else pdef['json_key']
            print(f"    {key} = {clamped_params[idx]:{pdef['fmt']}}")