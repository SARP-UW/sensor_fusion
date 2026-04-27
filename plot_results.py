"""
plot_results.py — Post-flight analysis for the MEKF sensor fusion engine.

Reads trajectory.csv produced by ./src/sensor_fusion and generates
flight_analysis.png with seven subplots:

  1. Altitude        — EKF pos_z vs vertical ground truth
  2. Velocity        — EKF vel_x/y/z vs vertical ground truth
  3. Horizontal drift — pos_x and pos_y (ideal = 0)
  4. Yaw             — heading derived from quaternion vs truth (0°)
  5. Gyro biases     — bg_x / bg_y / bg_z over time
  6. Accel biases    — ba_x / ba_y / ba_z over time
  7. Flight phase    — phase transitions as a step plot

All subplots share the same time axis.  Flight phase transitions are also
drawn as translucent background bands on every subplot for context.

NOTE: The vertical ground truth is regenerated here from hardcoded physics
constants that mirror DataGenerator.cpp.  If you change the burn time or
thrust acceleration in the generator, update BURN_TIME and BURN_ACC below
to match.  This duplication will be eliminated when the YAML config system
(Infrastructure phase) is implemented — the config will be the single source
of truth for all simulation parameters.
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import to_rgba

# =============================================================================
# Physics constants — keep in sync with DataGenerator.cpp
# =============================================================================
BURN_TIME    = 10.0    # seconds
BURN_ACC     = 20.0    # m/s²  (kinematic, upward)
GRAVITY      = 9.81    # m/s²
DESCENT_VEL  = -5.0    # m/s   (parachute terminal velocity)

# =============================================================================
# Flight phase colour map (must match FlightPhase enum in FlightPhase.hpp)
# =============================================================================
PHASE_COLOURS = {
    "PRE_LAUNCH": "#aaaaaa",
    "BOOST":      "#e8593c",
    "COAST":      "#3b8bd4",
    "DESCENT":    "#1d9e75",
    "LANDED":     "#444441",
}

# =============================================================================
# Ground truth generator (mirrors DataGenerator.cpp physics loop)
# =============================================================================

def generate_vertical_ground_truth(duration: float, dt: float = 0.01):
    times  = np.arange(0.0, duration, dt)
    pos_z  = np.zeros(len(times))
    vel_z  = np.zeros(len(times))

    z, v = 0.0, 0.0
    for i, t in enumerate(times):
        if t < BURN_TIME:
            acc = BURN_ACC
        elif v > 0:
            acc = -GRAVITY
        elif z > 0:
            acc = 0.0
            v   = DESCENT_VEL
        else:
            acc = 0.0
            v   = 0.0
            z   = 0.0

        if z > 0 or t < BURN_TIME:
            v += acc * dt
            z += v   * dt

        pos_z[i] = z
        vel_z[i] = v

    return times, pos_z, vel_z


def quat_to_yaw_deg(qw, qx, qy, qz):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.degrees(np.arctan2(siny, cosy))


# =============================================================================
# Phase band helper — draws translucent background bands on an axes
# =============================================================================

def shade_phases(ax, df):
    if "phase" not in df.columns:
        return
    phase_col = df["phase"].ffill()
    times     = df["time"].values
    prev_phase, start_t = phase_col.iloc[0], times[0]

    def _draw(phase, t0, t1):
        colour = PHASE_COLOURS.get(phase, "#ffffff")
        ax.axvspan(t0, t1, alpha=0.10, color=colour, linewidth=0, zorder=0)

    for t, ph in zip(times[1:], phase_col.iloc[1:]):
        if ph != prev_phase:
            _draw(prev_phase, start_t, t)
            prev_phase, start_t = ph, t
    _draw(prev_phase, start_t, times[-1])


# =============================================================================
# Main
# =============================================================================

def plot_trajectory(csv_path: str = "build/trajectory.csv",
                    out_path:  str = "flight_analysis.png"):

    # ---- Load EKF output ----
    try:
        df = pd.read_csv(csv_path)
        df.columns = df.columns.str.strip()
    except FileNotFoundError:
        print(f"Error: '{csv_path}' not found.  Run ./build/src/sensor_fusion first.")
        sys.exit(1)

    required = {"time", "pos_z", "vel_z", "quat_w", "quat_x", "quat_y", "quat_z"}
    missing  = required - set(df.columns)
    if missing:
        print(f"Error: CSV is missing columns: {missing}")
        print(f"Columns present: {list(df.columns)}")
        sys.exit(1)

    # ---- Ground truth ----
    gt_t, gt_pz, gt_vz = generate_vertical_ground_truth(
        duration=df["time"].max() + 1.0)

    # ---- Derived channels ----
    df["yaw_deg"] = quat_to_yaw_deg(
        df["quat_w"], df["quat_x"], df["quat_y"], df["quat_z"])

    has_biases = {"bg_x","bg_y","bg_z","ba_x","ba_y","ba_z"}.issubset(df.columns)
    has_vel_xy = {"vel_x","vel_y"}.issubset(df.columns)
    has_phase  = "phase" in df.columns

    # ---- Build phase transition legend ----
    phase_patches = [
        mpatches.Patch(color=c, alpha=0.5, label=p)
        for p, c in PHASE_COLOURS.items()
        if has_phase and p in df["phase"].values
    ]

    # ---- Layout ----
    n_rows = 4 + (2 if has_biases else 0) + (1 if has_phase else 0)
    fig, axes = plt.subplots(n_rows, 1, figsize=(12, 3.2 * n_rows), sharex=True)
    fig.suptitle("MEKF Flight Analysis", fontsize=14, y=1.001)

    row = 0

    # ------------------------------------------------------------------
    # 1. Altitude
    # ------------------------------------------------------------------
    ax = axes[row]; row += 1
    shade_phases(ax, df)
    ax.plot(gt_t, gt_pz, "k--", linewidth=1.5, label="Ground truth (vertical)")
    ax.plot(df["time"], df["pos_z"], "b-",  linewidth=2,   label="EKF altitude")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Altitude")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.4)

    # ------------------------------------------------------------------
    # 2. Velocity
    # ------------------------------------------------------------------
    ax = axes[row]; row += 1
    shade_phases(ax, df)
    ax.plot(gt_t, gt_vz, "k--", linewidth=1.5, label="Ground truth (vertical)")
    ax.plot(df["time"], df["vel_z"], "r-", linewidth=2, label="EKF vel_z")
    if has_vel_xy:
        ax.plot(df["time"], df["vel_x"], "g-", linewidth=1,
                alpha=0.7, label="EKF vel_x")
        ax.plot(df["time"], df["vel_y"], color="orange", linewidth=1,
                alpha=0.7, label="EKF vel_y")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("Velocity")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.4)

    # ------------------------------------------------------------------
    # 3. Horizontal position (drift from vertical)
    # ------------------------------------------------------------------
    ax = axes[row]; row += 1
    shade_phases(ax, df)
    if "pos_x" in df.columns:
        ax.plot(df["time"], df["pos_x"], "g-",
                alpha=0.8, label="pos_x (EKF)")
    if "pos_y" in df.columns:
        ax.plot(df["time"], df["pos_y"], color="orange",
                alpha=0.8, label="pos_y (EKF)")
    ax.axhline(0, color="k", linestyle="--", linewidth=1,
               label="Ideal (0 m)")
    ax.set_ylabel("Horizontal position (m)")
    ax.set_title("Horizontal drift from vertical (gyro noise)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.4)

    # ------------------------------------------------------------------
    # 4. Yaw (heading)
    # ------------------------------------------------------------------
    ax = axes[row]; row += 1
    shade_phases(ax, df)
    ax.plot(df["time"], df["yaw_deg"], "m-", linewidth=1.5, label="EKF yaw")
    ax.axhline(0, color="k", linestyle="--", linewidth=1, label="Truth (0°)")
    ax.set_ylabel("Heading (deg)")
    ax.set_title("Yaw / heading")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.4)

    # ------------------------------------------------------------------
    # 5. Gyro biases
    # ------------------------------------------------------------------
    if has_biases:
        ax = axes[row]; row += 1
        shade_phases(ax, df)
        ax.plot(df["time"], df["bg_x"], label="bg_x", linewidth=1.2)
        ax.plot(df["time"], df["bg_y"], label="bg_y", linewidth=1.2)
        ax.plot(df["time"], df["bg_z"], label="bg_z", linewidth=1.2)
        ax.axhline(0, color="k", linestyle="--", linewidth=0.8, alpha=0.5)
        ax.set_ylabel("Bias (rad/s)")
        ax.set_title("Gyro bias estimates  [truth = 0 for synthetic data]")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.4)

        # ------------------------------------------------------------------
        # 6. Accel biases
        # ------------------------------------------------------------------
        ax = axes[row]; row += 1
        shade_phases(ax, df)
        ax.plot(df["time"], df["ba_x"], label="ba_x", linewidth=1.2)
        ax.plot(df["time"], df["ba_y"], label="ba_y", linewidth=1.2)
        ax.plot(df["time"], df["ba_z"], label="ba_z", linewidth=1.2)
        ax.axhline(0, color="k", linestyle="--", linewidth=0.8, alpha=0.5)
        ax.set_ylabel("Bias (m/s²)")
        ax.set_title("Accel bias estimates  [truth = 0 for synthetic data]")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.4)

    # ------------------------------------------------------------------
    # 7. Flight phase
    # ------------------------------------------------------------------
    if has_phase:
        phase_map = {
            "PRE_LAUNCH": 0,
            "BOOST":      1,
            "COAST":      2,
            "DESCENT":    3,
            "LANDED":     4,
        }
        ax = axes[row]; row += 1
        phase_numeric = df["phase"].map(phase_map).fillna(-1)
        ax.step(df["time"], phase_numeric, where="post",
                color="#444441", linewidth=1.5)
        ax.set_yticks(list(phase_map.values()))
        ax.set_yticklabels(list(phase_map.keys()), fontsize=8)
        ax.set_ylabel("Phase")
        ax.set_title("Flight phase state machine")
        ax.grid(True, alpha=0.4, axis="x")

        if phase_patches:
            ax.legend(handles=phase_patches, loc="upper right",
                      fontsize=8, title="Phase bands")

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Plot saved to '{out_path}'")
    plt.show()


if __name__ == "__main__":
    csv  = sys.argv[1] if len(sys.argv) > 1 else "build/trajectory.csv"
    out  = sys.argv[2] if len(sys.argv) > 2 else "flight_analysis.png"
    plot_trajectory(csv, out)