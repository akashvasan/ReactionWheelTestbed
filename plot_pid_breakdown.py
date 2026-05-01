"""
PID term breakdown plot.

Reads perturbation_log.csv (requires p_term/d_term columns added in the
updated demo_perturbation.py) and shows how each controller term contributes
to the total duty cycle over time.

Usage:
  python3 plot_pid_breakdown.py [path_to_csv]
"""

import sys
import os
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

LOG_PATH = os.path.join(os.path.dirname(__file__), "perturbation_log.csv")
OUT_PATH = os.path.join(os.path.dirname(__file__), "pid_breakdown_plot.png")

def load(path):
    rows = {"t_s": [], "imu_rpm": [], "error": [], "duty": [],
            "state": [], "p_term": [], "d_term": []}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if not {"p_term", "d_term"}.issubset(reader.fieldnames):
            print("ERROR: p_term/d_term columns not found in CSV.")
            print("Re-run demo_perturbation.py with the updated logging, then try again.")
            sys.exit(1)
        for row in reader:
            for k in rows:
                rows[k].append(row[k])
    t       = np.array(rows["t_s"],    dtype=float)
    imu_rpm = np.array(rows["imu_rpm"], dtype=float)
    error   = np.array(rows["error"],   dtype=float)
    duty    = np.array(rows["duty"],    dtype=float)
    p_term  = np.array(rows["p_term"],  dtype=float)
    d_term  = np.array(rows["d_term"],  dtype=float)
    state   = [s.strip() for s in rows["state"]]
    return t, imu_rpm, error, duty, p_term, d_term, state

def shade_states(ax, t, state):
    color_map = {"ACTIVE": "#ffdddd", "HOLD": "#ddeeff"}
    i = 0
    while i < len(state):
        s = state[i]
        j = i + 1
        while j < len(state) and state[j] == s:
            j += 1
        ax.axvspan(t[i], t[j-1], alpha=0.3, color=color_map.get(s, "#eeeeee"), linewidth=0)
        i = j

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else LOG_PATH
    t, imu_rpm, error, duty, p_term, d_term, state = load(path)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle("PID Term Breakdown — Perturbation Rejection", fontsize=13, fontweight="bold")

    ax = axes[0]
    ax.plot(t, imu_rpm, color="#1f77b4", linewidth=1.5, label="Platform RPM (IMU)")
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.5)
    shade_states(ax, t, state)
    ax.set_ylabel("Platform RPM", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    shade_states(ax, t, state)
    ax.fill_between(t, 0, p_term,            alpha=0.55, color="#1f77b4", label="P term  (Kp·error)")
    ax.fill_between(t, p_term, p_term+d_term, alpha=0.55, color="#2ca02c", label="D term  (Kd·Δerror/Δt)")
    ax.plot(t, duty, color="black", linewidth=1.2, linestyle="--", alpha=0.7, label="Total duty (%)")
    ax.axhline(0, color="black", linewidth=0.8, alpha=0.4)
    ax.set_ylabel("Duty contribution (%)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.plot(t, p_term, color="#1f77b4", linewidth=1.5, label="P term")
    ax.plot(t, d_term, color="#2ca02c", linewidth=1.5, label="D term")
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.5)
    shade_states(ax, t, state)
    ax.set_ylabel("Term value (%)", fontsize=10)
    ax.set_xlabel("Time (s)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    active_p = mpatches.Patch(color="#ffdddd", alpha=0.7, label="ACTIVE")
    hold_p   = mpatches.Patch(color="#ddeeff", alpha=0.7, label="HOLD")
    fig.legend(handles=[active_p, hold_p], loc="lower center", ncol=2,
               fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, 0.01))

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    fig.savefig(OUT_PATH, dpi=150, bbox_inches="tight")
    print(f"Saved → {OUT_PATH}")

if __name__ == "__main__":
    main()
