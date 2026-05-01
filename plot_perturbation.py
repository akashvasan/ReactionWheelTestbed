"""
Plot PID telemetry from a perturbation_rejection run.

Reads perturbation_log.csv produced by demo_perturbation.py and saves a
4-panel figure showing:
  1. Platform angular velocity (IMU RPM) — the "position" the controller sees
  2. Reaction-wheel RPM — proxy for angular momentum / applied torque
  3. Tracking error (= target − platform RPM; target is 0)
  4. Motor duty cycle — the PID output

Usage:
  python3 plot_perturbation.py [path_to_csv]
  (defaults to perturbation_log.csv in the same directory)
"""

import sys
import os
import csv
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

LOG_PATH = os.path.join(os.path.dirname(__file__), "perturbation_log.csv")
OUT_PATH = os.path.join(os.path.dirname(__file__), "perturbation_plot.png")

def load(path):
    t, imu_rpm, wheel_rpm, error, duty, state = [], [], [], [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t_s"]))
            imu_rpm.append(float(row["imu_rpm"]))
            wheel_rpm.append(float(row["wheel_rpm"]))
            error.append(float(row["error"]))
            duty.append(float(row["duty"]))
            state.append(row["state"].strip())
    return (np.array(t), np.array(imu_rpm), np.array(wheel_rpm),
            np.array(error), np.array(duty), state)

def shade_states(ax, t, state):
    """Shade ACTIVE regions in light red, HOLD in light blue."""
    if len(t) < 2:
        return
    i = 0
    while i < len(state):
        s = state[i]
        j = i + 1
        while j < len(state) and state[j] == s:
            j += 1
        color = "#ffdddd" if s == "ACTIVE" else "#ddeeff"
        ax.axvspan(t[i], t[j - 1], alpha=0.35, color=color, linewidth=0)
        i = j

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else LOG_PATH
    t, imu_rpm, wheel_rpm, error, duty, state = load(path)

    wheel_accel = np.gradient(wheel_rpm, t)

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Perturbation Rejection — PID Telemetry", fontsize=14, fontweight="bold")

    ax = axes[0]
    ax.plot(t, imu_rpm, color="#1f77b4", linewidth=1.5, label="Platform RPM (IMU)")
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
    shade_states(ax, t, state)
    ax.set_ylabel("Platform RPM", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax2 = ax.twinx()
    ax.plot(t, wheel_rpm, color="#ff7f0e", linewidth=1.5, label="Wheel RPM")
    ax2.plot(t, wheel_accel, color="#d62728", linewidth=1.0, linestyle=":", alpha=0.7, label="Wheel accel (RPM/s)")
    shade_states(ax, t, state)
    ax.set_ylabel("Wheel RPM", fontsize=10, color="#ff7f0e")
    ax2.set_ylabel("Accel (RPM/s)", fontsize=9, color="#d62728")
    ax.tick_params(axis="y", labelcolor="#ff7f0e")
    ax2.tick_params(axis="y", labelcolor="#d62728")
    lines = ax.get_lines() + ax2.get_lines()
    ax.legend(lines, [l.get_label() for l in lines], loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.plot(t, error, color="#2ca02c", linewidth=1.5, label="Error (= −Platform RPM)")
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
    shade_states(ax, t, state)
    ax.set_ylabel("Error (RPM)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[3]
    ax.plot(t, duty, color="#9467bd", linewidth=1.5, label="Duty cycle (%)")
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
    shade_states(ax, t, state)
    ax.set_ylabel("Duty (%)", fontsize=10)
    ax.set_xlabel("Time (s)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    active_patch = mpatches.Patch(color="#ffdddd", alpha=0.7, label="ACTIVE (correcting)")
    hold_patch   = mpatches.Patch(color="#ddeeff", alpha=0.7, label="HOLD (near zero)")
    fig.legend(handles=[active_patch, hold_patch], loc="lower center",
               ncol=2, fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, 0.01))

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    fig.savefig(OUT_PATH, dpi=150, bbox_inches="tight")
    print(f"Saved → {OUT_PATH}")

if __name__ == "__main__":
    main()
