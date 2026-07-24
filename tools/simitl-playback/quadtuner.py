#!/usr/bin/env python3
"""
quadtuner.py - Interactive GUI for manual quadcopter parameter tuning.

Provides a tkinter interface to:
  1. Load a quad config JSON file via file dialog
  2. Display and adjust simulation parameters via number inputs and sliders
  3. Run the C++ simitl-playback simulation with current parameters
  4. Plot motor and gyro results for manual evaluation

Usage:
  python quadtuner.py
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import numpy as np
import os
import sys
import shutil
import tempfile
import threading
import json
import subprocess
import math
import csv

# Must come before other imports due to matplotlib backend selection
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfiltfilt

# Import shared logic from tuner.py (same directory)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from tuner import (
    load_quad_config,
    setup_result_directory,
    modify_tuned_config,
    run_simitl_playback,
    load_playback_csv,
    find_playback_binary,
    TUNEABLE_PARAMS,
    _lowpass_filter_gyro,
)

# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------

class QuadTunerApp:
    """Main GUI application for manual quadcopter parameter tuning."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Quadcopter Parameter Tuner")
        self.root.geometry("920x780")
        self.root.minsize(800, 650)

        # State
        self.cfg = None                 # fully-resolved config dict
        self.param_vars: list = []      # list of (tk.DoubleVar, entry, scale, pdef)
        self.orig_params: list = []     # original parameter values (for reset)

        # StringVars / DoubleVars for UI binding
        self.config_path = tk.StringVar()
        self.bb_path = tk.StringVar()
        self.start_time = tk.DoubleVar(value=3.0)
        self.length = tk.DoubleVar(value=2.0)
        self.mode = tk.StringVar(value="olbb")
        self.status = tk.StringVar(value="Ready")

        # Plot persistence for overlay comparison
        self._plot_fig = None
        self._ax_rc = None
        self._ax_m = None
        self._ax_g = None
        self._plot_metrics_text = None
        self._run_results: list[dict] = []
        self._run_metrics: list[dict] = []
        self._ref_motor = None
        self._ref_gyro = None
        self._rc_data = None

        self._build_ui()

        # Attempt to auto-fill config path from tuner's default
        default_cfg = os.path.join(SCRIPT_DIR, "config-tuned", "quad", "vtx-slayer-one-4.json")
        if os.path.isfile(default_cfg):
            self.config_path.set(os.path.normpath(default_cfg))

        # Attempt to auto-fill BB path with first .bbl.csv in script dir
        for fname in sorted(os.listdir(SCRIPT_DIR)):
            if fname.endswith(".bbl.csv"):
                self.bb_path.set(os.path.join(SCRIPT_DIR, fname))
                break

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        # -- File selection ---------------------------------------------------
        file_frame = ttk.LabelFrame(self.root, text="File Selection", padding=10)
        file_frame.pack(fill=tk.X, padx=10, pady=(10, 4))

        ttk.Label(file_frame, text="Quad Config:").grid(
            row=0, column=0, sticky=tk.W, padx=(0, 5)
        )
        cfg_entry = ttk.Entry(file_frame, textvariable=self.config_path, width=60)
        cfg_entry.grid(row=0, column=1, padx=5, sticky=tk.EW)
        cfg_entry.config(state=tk.READABLE)
        ttk.Button(file_frame, text="Browse...", command=self.browse_config).grid(
            row=0, column=2, padx=(5, 0)
        )

        ttk.Label(file_frame, text="Blackbox CSV:").grid(
            row=1, column=0, sticky=tk.W, padx=(0, 5), pady=(5, 0)
        )
        bb_entry = ttk.Entry(file_frame, textvariable=self.bb_path, width=60)
        bb_entry.grid(row=1, column=1, padx=5, pady=(5, 0), sticky=tk.EW)
        bb_entry.config(state=tk.READABLE)
        ttk.Button(file_frame, text="Browse...", command=self.browse_bb).grid(
            row=1, column=2, padx=(5, 0), pady=(5, 0)
        )

        # -- Simulation settings ----------------------------------------------
        sim_frame = ttk.Frame(file_frame)
        sim_frame.grid(row=2, column=0, columnspan=3, sticky=tk.W, pady=(8, 0))

        ttk.Label(sim_frame, text="Start (s):").pack(side=tk.LEFT)
        ttk.Entry(sim_frame, textvariable=self.start_time, width=8).pack(
            side=tk.LEFT, padx=(2, 12)
        )

        ttk.Label(sim_frame, text="Length (s):").pack(side=tk.LEFT)
        ttk.Entry(sim_frame, textvariable=self.length, width=8).pack(
            side=tk.LEFT, padx=(2, 12)
        )

        ttk.Label(sim_frame, text="Mode:").pack(side=tk.LEFT)
        ttk.Radiobutton(
            sim_frame, text="bb (closed-loop)", variable=self.mode, value="bb"
        ).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(
            sim_frame, text="olbb (open-loop)", variable=self.mode, value="olbb"
        ).pack(side=tk.LEFT, padx=2)

        # make the file frame entry column stretchable
        file_frame.columnconfigure(1, weight=1)

        # -- Parameters panel (scrollable) ------------------------------------
        params_outer = ttk.LabelFrame(self.root, text="Tuning Parameters", padding=8)
        params_outer.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)

        canvas = tk.Canvas(params_outer, borderwidth=0, highlightthickness=0)
        scrollbar = ttk.Scrollbar(
            params_outer, orient=tk.VERTICAL, command=canvas.yview
        )
        self.params_frame = ttk.Frame(canvas)

        # Keep scroll region updated when frame size changes
        self.params_frame.bind(
            "<Configure>",
            lambda _e: canvas.configure(scrollregion=canvas.bbox("all")),
        )

        # Inner window -- stretch to full canvas width on resize
        _inner = canvas.create_window(
            (0, 0), window=self.params_frame, anchor="nw"
        )
        canvas.bind(
            "<Configure>",
            lambda e: canvas.itemconfig(_inner, width=e.width),
            add="+",
        )

        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Header row
        hdr_font = ("", 9, "bold")
        ttk.Label(self.params_frame, text="Parameter", font=hdr_font).grid(
            row=0, column=0, sticky=tk.W, padx=(0, 16), pady=(0, 4)
        )
        ttk.Label(self.params_frame, text="Value", font=hdr_font).grid(
            row=0, column=1, sticky=tk.W, padx=4, pady=(0, 4)
        )
        ttk.Label(self.params_frame, text="Slider", font=hdr_font).grid(
            row=0, column=2, sticky=tk.W, padx=4, pady=(0, 4)
        )
        self.params_frame.columnconfigure(1, weight=1)
        self.params_frame.columnconfigure(2, weight=3)

        # -- Action buttons ---------------------------------------------------
        btn_frame = ttk.Frame(self.root, padding=6)
        btn_frame.pack(fill=tk.X, padx=10, pady=2)

        self.sim_btn = ttk.Button(
            btn_frame, text="Simulate", command=self.simulate, width=18
        )
        self.sim_btn.pack(side=tk.LEFT, padx=(0, 8))

        ttk.Button(
            btn_frame, text="Reset Parameters", command=self.reset_params
        ).pack(side=tk.LEFT, padx=(0, 8))

        ttk.Button(
            btn_frame, text="Load Config Now", command=self._reload_config
        ).pack(side=tk.LEFT)

        self.status_label = ttk.Label(
            btn_frame, textvariable=self.status, font=("", 9, "italic")
        )
        self.status_label.pack(side=tk.RIGHT, padx=8)

        # -- Config summary (read-only text) ----------------------------------
        summary_frame = ttk.LabelFrame(self.root, text="Config Summary", padding=5)
        summary_frame.pack(fill=tk.BOTH, padx=10, pady=(2, 10))

        self.summary_text = tk.Text(
            summary_frame, height=7, wrap=tk.WORD, state=tk.DISABLED,
            font=("Consolas", 8),
        )
        self.summary_text.pack(fill=tk.BOTH, expand=True)

    # ------------------------------------------------------------------
    # File dialogs
    # ------------------------------------------------------------------

    def browse_config(self) -> None:
        path = filedialog.askopenfilename(
            title="Select Quad Config JSON",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=os.path.join(SCRIPT_DIR, "config", "quad")
            if os.path.isdir(os.path.join(SCRIPT_DIR, "config", "quad"))
            else SCRIPT_DIR,
        )
        if path:
            self.config_path.set(path)
            self._load_config(path)

    def browse_bb(self) -> None:
        path = filedialog.askopenfilename(
            title="Select Blackbox CSV",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialdir=SCRIPT_DIR,
        )
        if path:
            self.bb_path.set(path)

    # ------------------------------------------------------------------
    # Config loading
    # ------------------------------------------------------------------

    def _load_config(self, path: str) -> None:
        """Load a quad config and populate the parameter controls."""
        try:
            self.cfg = load_quad_config(path)
        except Exception as e:
            messagebox.showerror("Config Error", f"Failed to load config:\n{e}")
            self.status.set("Error loading config")
            return

        self._populate_params()
        self._update_summary()
        self.status.set(f"Loaded: {os.path.basename(path)}")

    def _reload_config(self) -> None:
        """Reload the config from the currently selected path."""
        path = self.config_path.get()
        if not path:
            messagebox.showinfo("Info", "No config path selected.")
            return
        if not os.path.isfile(path):
            messagebox.showerror("Error", f"File not found:\n{path}")
            return
        self._load_config(path)

    # ------------------------------------------------------------------
    # Parameter controls
    # ------------------------------------------------------------------

    def _populate_params(self) -> None:
        """Create or rebuild the parameter entry + slider controls."""
        # Destroy existing parameter rows (keep header at row 0)
        for child in self.params_frame.winfo_children():
            info = child.grid_info()
            row = int(info.get("row", 0))
            if row > 0:
                child.destroy()

        self.param_vars = []
        self.orig_params = []

        for i, pdef in enumerate(TUNEABLE_PARAMS):
            val = pdef["extract"](self.cfg)
            self.orig_params.append(val)

            var = tk.DoubleVar(value=val)

            # -- Label --
            ttk.Label(self.params_frame, text=pdef["name"]).grid(
                row=i + 1, column=0, sticky=tk.W, padx=(0, 8), pady=2
            )

            # -- Entry --
            entry = ttk.Entry(self.params_frame, textvariable=var)
            entry.grid(row=i + 1, column=1, padx=4, pady=2, sticky=tk.EW)

            # -- Slider --
            lo = pdef["bounds_min"]
            hi = pdef["bounds_max"]
            scale = ttk.Scale(
                self.params_frame,
                from_=lo,
                to=hi,
                variable=var,
                orient=tk.HORIZONTAL,
                length=280,
            )
            scale.grid(row=i + 1, column=2, sticky=tk.EW, padx=4, pady=2)

            self.param_vars.append((var, entry, scale, pdef))

            # Sync slider -> entry (round to desired precision)
            scale.config(command=lambda v, idx=i: self._on_slider_change(idx, v))

            # Sync entry -> slider (clamp + update)
            var.trace_add("write", lambda *_, idx=i: self._on_entry_change(idx))

    def _on_slider_change(self, idx: int, value: str) -> None:
        """Slider moved: round the value to the parameter's display precision."""
        try:
            var, _entry, _scale, pdef = self.param_vars[idx]
            fmt = pdef["fmt"]
            rounded = float(f"{float(value):{fmt}}")
            var.set(rounded)
        except (ValueError, IndexError):
            pass

    def _on_entry_change(self, idx: int) -> None:
        """Entry typed: clamp to bounds."""
        try:
            var, _entry, _scale, pdef = self.param_vars[idx]
            val = var.get()
            lo = pdef["bounds_min"]
            hi = pdef["bounds_max"]
            clamped = max(lo, min(hi, val))
            var.set(clamped)
        except (tk.TclError, ValueError, IndexError):
            pass

    def reset_params(self) -> None:
        """Reset all parameters to their originally loaded values."""
        for i, val in enumerate(self.orig_params):
            try:
                self.param_vars[i][0].set(val)
            except IndexError:
                break
        self.status.set("Parameters reset to original values")

    def get_current_params(self) -> list[float]:
        """Return a list of current parameter values in TUNEABLE_PARAMS order."""
        return [var.get() for var, _, _, _ in self.param_vars]

    # ------------------------------------------------------------------
    # Config summary display
    # ------------------------------------------------------------------

    def _update_summary(self) -> None:
        """Update the read-only config summary text widget."""
        if self.cfg is None:
            return

        f = self.cfg["frame"]
        m = self.cfg["motor"]
        p = self.cfg["propeller"]
        b = self.cfg["battery"]

        total_mass = f["mass"] + 4.0 * m["mass"] + 4.0 * p["mass"] + b["mass"]

        lines = [
            "Frame:",
            f"  mass={f['mass']:.4f} kg,  dragConstant={f['dragConstant']:.3f}",
            f"  invInertia=({f['invInertia'][0]:.1f}, {f['invInertia'][1]:.1f}, {f['invInertia'][2]:.1f})",
            f"  dragArea=({f['dragArea'][0]:.4f}, {f['dragArea'][1]:.4f}, {f['dragArea'][2]:.4f})",
            f"  gyroNoiseAmp={f['gyroBaseNoiseAmp']:.4f}  freq={f['gyrobaseNoiseFreq']:.1f} Hz",
            f"  motorDir=({f['motorDir'][0]:.0f}, {f['motorDir'][1]:.0f}, {f['motorDir'][2]:.0f}, {f['motorDir'][3]:.0f})",
            f"  motorVariance=({f['motorVariance'][0]:.2f}, {f['motorVariance'][1]:.2f}, {f['motorVariance'][2]:.2f}, {f['motorVariance'][3]:.2f})",
            f"Motor:  kv={m['kv']:.0f}  R={m['r']:.4f} Ohm  I0={m['i0']:.4f} A  mass={m['mass']:.4f} kg",
            f"Prop:   blades={p['bladeCount']}  aFactor={p['aFactor']:.3e}  torqueFactor={p['torqueFactor']:.5f}",
            f"        inertia={p['inertia']:.3e}  thrustZ={p['thrustFactor'][2]:.5f}",
            f"Battery: {b['cellCount']}S  {int(b['capacityCharged'])} mAh  sag={b['maxVoltageSag']:.2f}V",
            f"AUW: {total_mass:.4f} kg",
        ]
        text = "\n".join(lines)

        self.summary_text.config(state=tk.NORMAL)
        self.summary_text.delete(1.0, tk.END)
        self.summary_text.insert(tk.END, text)
        self.summary_text.config(state=tk.DISABLED)

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------

    def simulate(self) -> None:
        """Start the simulation in a background thread."""
        # Validate inputs
        cfg_path = self.config_path.get()
        bb_path = self.bb_path.get()

        if not cfg_path or not os.path.isfile(cfg_path):
            messagebox.showwarning("Warning", "Please select a valid quad config file.")
            return
        if not bb_path or not os.path.isfile(bb_path):
            messagebox.showwarning(
                "Warning", "Please select a valid blackbox CSV file."
            )
            return
        if self.cfg is None:
            messagebox.showwarning("Warning", "Config not loaded. Click 'Load Config Now'.")
            return

        self.sim_btn.config(state=tk.DISABLED)
        self.status.set("Running simulation ...")

        thread = threading.Thread(target=self._run_simulation, daemon=True)
        thread.start()

    def _run_simulation(self) -> None:
        """Run the C++ simulation and plotting (background thread)."""
        try:
            cfg_path = self.config_path.get()
            bb_file = self.bb_path.get()
            start = self.start_time.get()
            length = self.length.get()
            open_loop = self.mode.get() == "olbb"

            # Temporary directories for config copies and simulation output
            result_dir = tempfile.mkdtemp(prefix="qtuner_cfg_")
            work_dir = tempfile.mkdtemp(prefix="qtuner_sim_")

            # 1. Copy configs into result directory
            result_quad_path = setup_result_directory(self.cfg, cfg_path, result_dir)

            # 2. Apply current parameter values to the copied configs
            params = self.get_current_params()
            modify_tuned_config(result_quad_path, result_dir, params)

            # 3. Run simitl-playback
            csv_path = run_simitl_playback(
                bb_file, result_quad_path, start, length, work_dir, open_loop
            )

            # 4. Load results
            result = load_playback_csv(csv_path)

            # 5. Compute error metrics
            metrics = self._compute_metrics(result)

            # 6. Plot (must be on main thread)
            self.root.after(0, lambda: self._plot_results(result, metrics))

            # Update status
            m = metrics
            status_text = (
                f"Done - Combined: {m['combined']:.4f}  "
                f"MSE: {m['norm_mse']:.4f}  "
                f"Amp: {m['std_penalty']:.4f}  "
                f"Corr: {m['corr_penalty']:.4f}"
            )
            self.root.after(0, lambda: self.status.set(status_text))

            # Cleanup temp directories
            shutil.rmtree(result_dir, ignore_errors=True)
            shutil.rmtree(work_dir, ignore_errors=True)

        except Exception as e:
            self.root.after(
                0, lambda: messagebox.showerror("Simulation Error", str(e))
            )
            self.root.after(0, lambda: self.status.set(f"Error: {e}"))
        finally:
            self.root.after(0, lambda: self.sim_btn.config(state=tk.NORMAL))

    # ------------------------------------------------------------------
    # Metrics computation
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_metrics(result: dict) -> dict:
        """Compute MSE, amplitude penalty, and correlation penalty from sim/ref gyro."""
        sim_gyro = result["sim_gyro"]
        ref_gyro = result["ref_gyro"]

        time = result.get("time")
        if time is not None:
            sim_gyro_f = _lowpass_filter_gyro(sim_gyro, time)
            ref_gyro_f = _lowpass_filter_gyro(ref_gyro, time)
        else:
            sim_gyro_f = sim_gyro
            ref_gyro_f = ref_gyro

        std_ref = np.std(ref_gyro_f, axis=0)
        std_sim = np.std(sim_gyro_f, axis=0)

        min_std = np.max(std_ref) * 0.05 + 1e-6
        std_ref_safe = np.maximum(std_ref, min_std)

        # Normalised MSE
        var_ref_safe = std_ref_safe ** 2
        mse_axis = np.mean((sim_gyro_f - ref_gyro_f) ** 2, axis=0)
        norm_mse = float(np.mean(mse_axis / var_ref_safe))

        # Amplitude penalty
        std_mismatch = (std_sim - std_ref) / std_ref_safe
        std_penalty = float(np.mean(std_mismatch ** 2))

        # Correlation penalty
        corrs = []
        for i in range(3):
            if std_ref_safe[i] > 1e-5 and std_sim[i] > 1e-5:
                r = np.corrcoef(ref_gyro_f[:, i], sim_gyro_f[:, i])[0, 1]
                corrs.append(1.0 - r if not np.isnan(r) else 1.0)
            else:
                corrs.append(1.0)
        corr_penalty = float(np.mean(corrs))

        combined = norm_mse + 3.0 * std_penalty + 1.0 * corr_penalty

        return {
            "combined": combined,
            "norm_mse": norm_mse,
            "std_penalty": std_penalty,
            "corr_penalty": corr_penalty,
        }

    # ------------------------------------------------------------------
    # Plotting
    # ------------------------------------------------------------------

    def _plot_results(self, result: dict, metrics: dict) -> None:
        """Plot overlay: store this run, replot all runs with alpha fading.

        Old runs are plotted at progressively lower alpha (oldest ~0.25),
        the newest run is fully opaque (alpha=1.0).  Reference data from
        the first run is plotted once with dashed lines.
        """
        # Time vector for this run
        t = result.get("time")
        if t is None:
            t = np.arange(len(result["sim_gyro"]))

        # Check if existing figure is still open
        fig_valid = (
            self._plot_fig is not None
            and plt.fignum_exists(self._plot_fig.number)
        )

        # If figure is gone, or if start/length changed (time length mismatch),
        # reset all accumulated plotting state and start fresh.
        reset = not fig_valid
        if not reset and self._ref_motor is not None:
            if len(t) != len(self._ref_motor):
                reset = True

        if reset:
            self._run_results.clear()
            self._run_metrics.clear()
            self._ref_motor = None
            self._ref_gyro = None
            self._rc_data = None

        # Store this run (sim data only; ref stays the same across runs)
        self._run_results.append(
            {
                "time": t,
                "sim_motor": result["sim_motor"],
                "sim_gyro": result["sim_gyro"],  # raw rad/s, no conversion
            }
        )
        self._run_metrics.append(metrics)

        # Keep only newest 2 runs to bound memory
        if len(self._run_results) > 2:
            self._run_results.pop(0)
            self._run_metrics.pop(0)

        # Store reference data from the first run of this plotting session
        if self._ref_motor is None:
            self._ref_motor = result.get("ref_motor")
            self._ref_gyro = result.get("ref_gyro")  # raw rad/s, no conversion

        # Store RC data from current run (replots fresh every time)
        if "rc" in result:
            self._rc_data = result["rc"]

        colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
        rc_colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
        rc_labels = ["rc0 (roll)", "rc1 (pitch)", "rc2 (throttle)", "rc3 (yaw)"]
        gyro_labels = ["roll", "pitch", "yaw"]
        has_ref = self._ref_motor is not None and self._ref_gyro is not None
        N = len(self._run_results)

        if not fig_valid:
            # --- Create fresh figure (3 subplots) ---
            self._plot_fig, (self._ax_rc, self._ax_m, self._ax_g) = plt.subplots(
                3, 1, figsize=(11, 9), sharex=True
            )
            self._plot_fig.suptitle(
                "Simulation Results - Manual Tuning (overlay)",
                fontsize=13,
                fontweight="bold",
            )

            # RC plot
            self._ax_rc.set_ylabel("RC value")
            self._ax_rc.set_title("RC Commands", fontsize=10)
            self._ax_rc.set_ylim(-1.5, 1.5)
            self._ax_rc.grid(True, alpha=0.3)

            # Motor plot
            self._ax_m.set_ylabel("Motor output")
            self._ax_m.set_title(
                "Motor Outputs  (solid = sim, dashed = ref)", fontsize=10
            )
            self._ax_m.grid(True, alpha=0.3)

            # Gyro plot
            self._ax_g.set_xlabel("Time (s)")
            self._ax_g.set_ylabel("Angular velocity (deg/s)")
            self._ax_g.set_title(
                "Angular Velocity / Gyro  (solid = sim, dashed = ref)",
                fontsize=10,
            )
            self._ax_g.grid(True, alpha=0.3)

            # Metrics text placeholder
            self._plot_metrics_text = self._plot_fig.text(
                0.015, 0.015, "",
                fontsize=8.5, family="monospace",
                bbox=dict(
                    boxstyle="round,pad=0.5",
                    facecolor="lightyellow",
                    alpha=0.85,
                ),
            )
        else:
            # --- Clear existing lines from all axes ---
            for ax in (self._ax_rc, self._ax_m, self._ax_g):
                for line in ax.lines:
                    line.remove()

        # --- RC commands (replotted fresh every simulate) ---
        if self._rc_data is not None:
            rc_t = self._run_results[-1]["time"]
            for i in range(4):
                self._ax_rc.plot(
                    rc_t, self._rc_data[:, i],
                    color=rc_colors[i], linewidth=0.9,
                    alpha=0.8, label=rc_labels[i],
                )
            self._ax_rc.legend(fontsize="x-small", loc="upper right")

        # --- Reference data (plotted once, always visible) ---
        if has_ref:
            ref_t = self._run_results[0]["time"]
            for i in range(4):
                self._ax_m.plot(
                    ref_t, self._ref_motor[:, i],
                    color=colors[i], linestyle="--", linewidth=0.55,
                    alpha=0.7, label=f"ref motor {i+1}",
                )
            for i in range(3):
                self._ax_g.plot(
                    ref_t, self._ref_gyro[:, i],
                    color=colors[i], linestyle="--", linewidth=0.55,
                    alpha=0.7, label=f"ref {gyro_labels[i]}",
                )

        # --- All sim runs (oldest faded, newest opaque) ---
        for idx, run in enumerate(self._run_results):
            rt = run["time"]

            # Alpha: 0.25 for oldest, 1.0 for newest
            if N == 1:
                alpha = 1.0
            else:
                alpha = 0.25 + 0.75 * (idx / (N - 1))

            # Motors (label only first run to keep legend clean)
            for i in range(4):
                label = f"sim motor {i+1}" if idx == 0 else None
                self._ax_m.plot(
                    rt, run["sim_motor"][:, i],
                    color=colors[i], linewidth=0.9,
                    alpha=alpha, label=label,
                )

            # Gyro (label only first run)
            for i in range(3):
                label = f"sim {gyro_labels[i]}" if idx == 0 else None
                self._ax_g.plot(
                    rt, run["sim_gyro"][:, i],
                    color=colors[i], linewidth=0.9,
                    alpha=alpha, label=label,
                )

        # --- Legends for motor and gyro ---
        self._ax_m.legend(fontsize="x-small", loc="upper right")
        self._ax_g.legend(fontsize="x-small", loc="upper right")

        # --- Metrics text (latest run only) ---
        latest = self._run_metrics[-1]
        txt = (
            f"Latest Run  #{N}\n"
            f"  Combined:     {latest['combined']:.4f}\n"
            f"  Norm MSE:     {latest['norm_mse']:.4f}\n"
            f"  Amplitude:    {latest['std_penalty']:.4f}\n"
            f"  Correlation:  {latest['corr_penalty']:.4f}\n"
        )
        self._plot_metrics_text.set_text(txt)

        # --- Show or refresh ---
        if not fig_valid:
            plt.tight_layout(rect=[0, 0.05, 1, 0.96])
            plt.show(block=False)
        else:
            self._plot_fig.canvas.draw_idle()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    root = tk.Tk()
    _app = QuadTunerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
