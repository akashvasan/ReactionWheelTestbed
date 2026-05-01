"""
Plot PID telemetry from a camera-tracking run.

Reads tracking_log.csv produced by cameraTracking.py and saves a 3-panel
figure showing:
  1. Target centroid X — where the pink blob sits in the frame (pixels)
  2. Normalised error — (centroid_x − frame_center) / frame_center  [-1, +1]
  3. Motor duty cycle — the PID output

Usage:
  python3 plot_tracking.py [path_to_csv]
  (defaults to tracking_log.csv in the same directory)
"""

import sys
import os
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

LOG_PATH = os.path.join(os.path.dirname(__file__), "tracking_log.csv")
OUT_PATH = os.path.join(os.path.dirname(__file__), "tracking_plot.png")

FRAME_W = 640
CX      = FRAME_W // 2

def load(path):
    t, cx_vals, error, duty, state = [], [], [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t_s"]))
            cx_vals.append(float(row["centroid_x"]) if row["centroid_x"] else None)
            error.append(float(row["error"]))
            duty.append(float(row["duty"]))
            state.append(row["state"].strip())
    return np.array(t), cx_vals, np.array(error), np.array(duty), state

def shade_states(ax, t, state):
    color_map = {"SEARCH": "#fff3cd", "TRACKING": "#d4edda", "CENTERED": "#cce5ff"}
    i = 0
    while i < len(state):
        s = state[i]
        j = i + 1
        while j < len(state) and state[j] == s:
            j += 1
        ax.axvspan(t[i], t[j - 1], alpha=0.4, color=color_map.get(s, "#eeeeee"), linewidth=0)
        i = j

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else LOG_PATH
    t, cx_vals, error, duty, state = load(path)

    cx_arr = np.array([v if v is not None else np.nan for v in cx_vals], dtype=float)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle("Camera Tracking — PID Telemetry", fontsize=14, fontweight="bold")

    ax = axes[0]
    ax.plot(t, cx_arr, color="#1f77b4", linewidth=1.5, label="Centroid X (px)")
    ax.axhline(CX, color="black", linewidth=1.0, linestyle="--", alpha=0.7, label=f"Frame center ({CX} px)")
    ax.fill_between(t, cx_arr, CX, where=~np.isnan(cx_arr), alpha=0.15, color="#1f77b4")
    shade_states(ax, t, state)
    ax.set_ylabel("Centroid X (pixels)", fontsize=10)
    ax.set_ylim(0, FRAME_W)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(t, error, color="#d62728", linewidth=1.5, label="Normalised error")
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
    ax.fill_between(t, error, 0, where=(error > 0),  alpha=0.15, color="#d62728", label="Right of center")
    ax.fill_between(t, error, 0, where=(error <= 0), alpha=0.15, color="#1f77b4", label="Left of center")
    shade_states(ax, t, state)
    ax.set_ylabel("Error (normalised)", fontsize=10)
    ax.set_ylim(-1.1, 1.1)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.text(0.01, 0.04,
            "error = (centroid_x − frame_center) / frame_center    "
            f"[−1 = far left, 0 = centered, +1 = far right;  frame_center = {CX} px]",
            transform=ax.transAxes, fontsize=7.5, color="#555555",
            verticalalignment="bottom")

    ax = axes[2]
    ax.plot(t, duty, color="#9467bd", linewidth=1.5, label="Duty cycle (%)")
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
    shade_states(ax, t, state)
    ax.set_ylabel("Duty (%)", fontsize=10)
    ax.set_xlabel("Time (s)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    search_p   = mpatches.Patch(color="#fff3cd", alpha=0.8, label="SEARCH")
    tracking_p = mpatches.Patch(color="#d4edda", alpha=0.8, label="TRACKING")
    center_p   = mpatches.Patch(color="#cce5ff", alpha=0.8, label="CENTERED")
    fig.legend(handles=[search_p, tracking_p, center_p], loc="lower center",
               ncol=3, fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, 0.01))

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    fig.savefig(OUT_PATH, dpi=150, bbox_inches="tight")
    print(f"Saved → {OUT_PATH}")

if __name__ == "__main__":
    main()
