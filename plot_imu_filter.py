"""
Raw vs. filtered IMU comparison plot.

Reconstructs the raw platform RPM from the logged gx/gz columns using the
same formula as demo_perturbation.py, then overlays the logged (LPF-filtered)
imu_rpm to show how much the low-pass filter smooths the signal.

Usage:
  python3 plot_imu_filter.py [path_to_csv]
"""

import sys
import os
import csv
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG_PATH = os.path.join(os.path.dirname(__file__), "perturbation_log.csv")
OUT_PATH = os.path.join(os.path.dirname(__file__), "imu_filter_plot.png")

RAD_S_TO_RPM = 60.0 / (2.0 * math.pi)
AXIS_SCALE   = 1.0 / math.sqrt(2)
LPF_ALPHA    = 0.2

def load(path):
    t, gx, gz, imu_rpm = [], [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t_s"]))
            gx.append(float(row["gx"]))
            gz.append(float(row["gz"]))
            imu_rpm.append(float(row["imu_rpm"]))
    return np.array(t), np.array(gx), np.array(gz), np.array(imu_rpm)

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else LOG_PATH
    t, gx, gz, lpf_rpm = load(path)

    raw_rpm = (gx - gz) * AXIS_SCALE * RAD_S_TO_RPM

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle("IMU Signal: Raw vs. Low-Pass Filtered  (α = {:.2f})".format(LPF_ALPHA),
                 fontsize=13, fontweight="bold")

    ax = axes[0]
    ax.plot(t, raw_rpm, color="#aec7e8", linewidth=1.0, alpha=0.85, label="Raw RPM  (gx−gz)/√2")
    ax.plot(t, lpf_rpm, color="#1f77b4", linewidth=1.8,              label=f"Filtered RPM  (EMA α={LPF_ALPHA})")
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.4)
    ax.set_ylabel("Platform RPM", fontsize=10)
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.text(0.01, 0.97, "Raw signal from IMU gyro axes (gx, gz) before any filtering.",
            transform=ax.transAxes, fontsize=8, color="#555555", va="top")

    ax = axes[1]
    diff = raw_rpm - lpf_rpm
    ax.fill_between(t, diff, 0, alpha=0.4, color="#ff7f0e", label="Noise removed by LPF")
    ax.plot(t, diff, color="#ff7f0e", linewidth=0.8, alpha=0.7)
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.4)
    ax.set_ylabel("Raw − Filtered (RPM)", fontsize=10)
    ax.set_xlabel("Time (s)", fontsize=10)
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)

    noise_std = np.std(diff)
    ax.text(0.01, 0.97, f"σ of removed noise = {noise_std:.2f} RPM",
            transform=ax.transAxes, fontsize=8.5, color="#333333", va="top",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="#fff3cd", alpha=0.8))

    plt.tight_layout()
    fig.savefig(OUT_PATH, dpi=150, bbox_inches="tight")
    print(f"Saved → {OUT_PATH}")
    print(f"  Noise std dev (raw − filtered): {noise_std:.3f} RPM")

if __name__ == "__main__":
    main()
