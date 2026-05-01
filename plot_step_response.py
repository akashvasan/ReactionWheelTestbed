"""
Step-response zoom plot for a single perturbation event.

Reads perturbation_log.csv, auto-detects the first significant disturbance
(|error| > TRIGGER_RPM), then plots a zoomed window around that event with
annotations for peak error, rise time, and settling time.

Usage:
  python3 plot_step_response.py [path_to_csv]
"""

import sys
import os
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG_PATH = os.path.join(os.path.dirname(__file__), "perturbation_log.csv")
OUT_PATH = os.path.join(os.path.dirname(__file__), "step_response_plot.png")

TRIGGER_RPM  = 5.0
SETTLED_RPM  = 0.5
SETTLED_WIN  = 6
PRE_PAD_S    = 0.5
POST_PAD_S   = 1.5

def load(path):
    t, imu_rpm, wheel_rpm, error, duty, state = [], [], [], [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t_s"]))
            imu_rpm.append(float(row["imu_rpm"]))
            wheel_rpm.append(float(row["wheel_rpm"]))
            error.append(float(row["error"]))
            duty.append(float(row["duty"]))
            state.append(row["state"].strip())
    return (np.array(t), np.array(imu_rpm), np.array(wheel_rpm),
            np.array(error), np.array(duty), state)

def find_event(t, error):
    trigger_idx = None
    for i, e in enumerate(error):
        if abs(e) > TRIGGER_RPM:
            trigger_idx = i
            break
    if trigger_idx is None:
        return None, None, None

    peak_idx = trigger_idx + np.argmax(np.abs(error[trigger_idx:trigger_idx + 200]))

    settled_idx = None
    for i in range(peak_idx, len(error) - SETTLED_WIN):
        if all(abs(error[i:i + SETTLED_WIN]) < SETTLED_RPM):
            settled_idx = i
            break

    return trigger_idx, peak_idx, settled_idx

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else LOG_PATH
    t, imu_rpm, wheel_rpm, error, duty, state = load(path)

    trigger_idx, peak_idx, settled_idx = find_event(t, error)
    if trigger_idx is None:
        print("No perturbation event found (no sample exceeded TRIGGER_RPM).")
        return

    t_start = t[trigger_idx] - PRE_PAD_S
    t_end   = t[settled_idx] + POST_PAD_S if settled_idx else t[-1]
    mask    = (t >= t_start) & (t <= t_end)

    tw = t[mask]
    ew = error[mask]
    iw = imu_rpm[mask]
    dw = duty[mask]

    t_trigger = t[trigger_idx]
    t_peak    = t[peak_idx]
    t_settled = t[settled_idx] if settled_idx else None

    rise_time   = t_peak - t_trigger
    settle_time = (t_settled - t_trigger) if t_settled else None
    peak_error  = error[peak_idx]

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("Step Response — Single Perturbation Event", fontsize=13, fontweight="bold")

    ax = axes[0]
    ax.plot(tw, iw, color="#1f77b4", linewidth=2, label="Platform RPM (IMU)")
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.5)
    ax.axvline(t_trigger, color="orange", linewidth=1.2, linestyle="--", alpha=0.8, label="Disturbance")
    ax.axvline(t_peak,    color="red",    linewidth=1.2, linestyle="--", alpha=0.8, label="Peak error")
    if t_settled:
        ax.axvline(t_settled, color="green", linewidth=1.2, linestyle="--", alpha=0.8, label="Settled")
    ax.set_ylabel("Platform RPM", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(tw, ew, color="#2ca02c", linewidth=2, label="Error (RPM)")
    ax.axhline(0,            color="black", linewidth=0.8, linestyle="--", alpha=0.5)
    ax.axhline( SETTLED_RPM, color="grey",  linewidth=0.8, linestyle=":",  alpha=0.7)
    ax.axhline(-SETTLED_RPM, color="grey",  linewidth=0.8, linestyle=":",  alpha=0.7)
    ax.axvline(t_trigger,    color="orange", linewidth=1.2, linestyle="--", alpha=0.8)
    ax.axvline(t_peak,       color="red",    linewidth=1.2, linestyle="--", alpha=0.8)
    if t_settled:
        ax.axvline(t_settled, color="green", linewidth=1.2, linestyle="--", alpha=0.8)

    ax.annotate(f"Peak: {peak_error:+.1f} RPM",
                xy=(t_peak, peak_error),
                xytext=(t_peak + 0.15, peak_error * 0.75),
                fontsize=8, color="red",
                arrowprops=dict(arrowstyle="->", color="red", lw=1.2))

    mid_rise  = (t_trigger + t_peak) / 2
    y_bracket = peak_error * 1.05
    ax.annotate("", xy=(t_peak, y_bracket), xytext=(t_trigger, y_bracket),
                arrowprops=dict(arrowstyle="<->", color="darkorange", lw=1.2))
    ax.text(mid_rise, y_bracket * 1.05, f"Rise: {rise_time:.2f} s",
            ha="center", fontsize=8, color="darkorange")

    if t_settled:
        mid_settle = (t_trigger + t_settled) / 2
        y_settle   = min(ew) * 1.1 if min(ew) < 0 else -abs(peak_error) * 0.5
        ax.annotate("", xy=(t_settled, y_settle), xytext=(t_trigger, y_settle),
                    arrowprops=dict(arrowstyle="<->", color="darkgreen", lw=1.2))
        ax.text(mid_settle, y_settle * 1.05, f"Settle: {settle_time:.2f} s",
                ha="center", fontsize=8, color="darkgreen")

    ax.text(0.01, 0.97, f"±{SETTLED_RPM} RPM settling band",
            transform=ax.transAxes, fontsize=7, color="grey", va="top")
    ax.set_ylabel("Error (RPM)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.plot(tw, dw, color="#9467bd", linewidth=2, label="Duty cycle (%)")
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.5)
    ax.axvline(t_trigger, color="orange", linewidth=1.2, linestyle="--", alpha=0.8)
    ax.axvline(t_peak,    color="red",    linewidth=1.2, linestyle="--", alpha=0.8)
    if t_settled:
        ax.axvline(t_settled, color="green", linewidth=1.2, linestyle="--", alpha=0.8)
    ax.set_ylabel("Duty (%)", fontsize=10)
    ax.set_xlabel("Time (s)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    summary  = f"Peak error:    {peak_error:+.1f} RPM\n"
    summary += f"Rise time:     {rise_time:.2f} s\n"
    summary += f"Settling time: {settle_time:.2f} s" if settle_time else "Settling time: —"
    fig.text(0.13, 0.01, summary, fontsize=8.5, family="monospace",
             verticalalignment="bottom",
             bbox=dict(boxstyle="round,pad=0.4", facecolor="#f0f0f0", alpha=0.8))

    plt.tight_layout(rect=[0, 0.07, 1, 1])
    fig.savefig(OUT_PATH, dpi=150, bbox_inches="tight")
    print(f"Saved → {OUT_PATH}")
    print(f"  Peak error:    {peak_error:+.2f} RPM")
    print(f"  Rise time:     {rise_time:.3f} s")
    if settle_time:
        print(f"  Settling time: {settle_time:.3f} s")

if __name__ == "__main__":
    main()
