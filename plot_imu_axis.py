"""
IMU axis projection diagram.

Generates a static figure explaining how the BNO085 mounted at -45° in the
XZ plane is projected onto the platform's rotation axis using:

    platform_rpm = (gx - gz) / sqrt(2)  *  (60 / 2π)

Usage:
  python3 plot_imu_axis.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

OUT_PATH = os.path.join(os.path.dirname(__file__), "imu_axis_diagram.png")

def arrow(ax, x0, y0, dx, dy, color, label=None, lw=2.0, hw=0.04, hl=0.08):
    ax.annotate("", xy=(x0+dx, y0+dy), xytext=(x0, y0),
                arrowprops=dict(arrowstyle=f"->,head_width={hw},head_length={hl}",
                                color=color, lw=lw))
    if label:
        ax.text(x0+dx*1.08, y0+dy*1.08, label, color=color, fontsize=11,
                ha="center", va="center", fontweight="bold")

def main():
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    fig.suptitle("BNO085 IMU Axis Projection  (mounted at −45° in XZ plane)",
                 fontsize=13, fontweight="bold")

    ax = axes[0]
    ax.set_aspect("equal")
    ax.set_xlim(-1.6, 1.6)
    ax.set_ylim(-1.6, 1.6)
    ax.axis("off")
    ax.set_title("Side view (XZ plane) — platform axis = Y (into page)", fontsize=10)

    arrow(ax, 0, 0, 1.2, 0,   "#888888", label="World X", lw=1.5, hw=0.03, hl=0.06)
    arrow(ax, 0, 0, 0,   1.2, "#888888", label="World Z", lw=1.5, hw=0.03, hl=0.06)
    arrow(ax, 0, 0,-1.2, 0,   "#cccccc", lw=1.0, hw=0.02, hl=0.04)
    arrow(ax, 0, 0, 0,  -1.2, "#cccccc", lw=1.0, hw=0.02, hl=0.04)

    angle_rad = np.deg2rad(-45.0)
    imu_x = np.cos(angle_rad)
    imu_z = np.sin(angle_rad)

    w, h = 0.18, 0.10
    corners = np.array([[-w, -h], [w, -h], [w, h], [-w, h], [-w, -h]])
    R = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                  [np.sin(angle_rad),  np.cos(angle_rad)]])
    rotated = (R @ corners.T).T + np.array([imu_x*0.55, imu_z*0.55])
    ax.fill(rotated[:,0], rotated[:,1], color="#ffe0b2", zorder=3, alpha=0.9)
    ax.plot(rotated[:,0], rotated[:,1], color="#e65100", lw=1.5, zorder=4)
    ax.text(imu_x*0.55, imu_z*0.55 + 0.20, "BNO085", fontsize=8,
            ha="center", va="bottom", color="#e65100")

    arrow(ax, 0, 0, imu_x, imu_z, "#1f77b4", label="gx (IMU X)", lw=2.2)

    gz_x = np.cos(angle_rad + np.pi/2)
    gz_z = np.sin(angle_rad + np.pi/2)
    arrow(ax, 0, 0, gz_x, gz_z, "#d62728", label="gz (IMU Z)", lw=2.2)

    proj_angle = np.deg2rad(45)
    px, pz = np.cos(proj_angle), np.sin(proj_angle)
    arrow(ax, 0, 0, px*1.1, pz*1.1, "#2ca02c", lw=2.5,
          label="Platform\nrotation axis", hw=0.04, hl=0.08)

    theta = np.linspace(0, np.deg2rad(-45), 80)
    ax.plot(0.45*np.cos(theta), 0.45*np.sin(theta), color="#555555", lw=1.2, linestyle=":")
    ax.text(0.52, -0.12, "−45°", fontsize=9, color="#555555")

    ax.text(0, -1.45,
            r"$\omega_{platform} = \frac{g_x - g_z}{\sqrt{2}}$ × $\frac{60}{2\pi}$ RPM",
            ha="center", fontsize=12,
            bbox=dict(boxstyle="round,pad=0.4", facecolor="#e8f5e9", alpha=0.9))

    ax = axes[1]
    ax.set_aspect("equal")
    ax.set_xlim(-0.3, 2.0)
    ax.set_ylim(-0.5, 2.0)
    ax.axis("off")
    ax.set_title("Why  (gx − gz) / √2  gives the platform angular rate", fontsize=10)

    arrow(ax, 0.8, 0.8, 1.0, 0,   "#1f77b4", label="gx", lw=2.0)
    arrow(ax, 0.8, 0.8, 0,   1.0, "#d62728", label="gz", lw=2.0)
    arrow(ax, 0.8, 0.8, 0.707, 0.707, "#2ca02c",
          label="platform\naxis  (45°)", lw=2.5, hw=0.05, hl=0.09)

    ax.plot([1.8, 1.507], [0.8, 1.507], color="#1f77b4", lw=1.0, linestyle="--", alpha=0.6)
    ax.plot([0.8, 1.507], [1.8, 1.507], color="#d62728", lw=1.0, linestyle="--", alpha=0.6)

    ax.text(0.75, 0.50,
            "dot(gx_vec, axis) =  gx × cos45° = gx/√2\n"
            "dot(gz_vec, axis) = −gz × cos45° = −gz/√2\n\n"
            "Sum  →  (gx − gz) / √2",
            fontsize=9.5, va="top", color="#333333",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#f3f3f3", alpha=0.9))

    ax.text(0.75, -0.35,
            "gz contributes negatively because the IMU Z axis\n"
            "points away from the platform rotation axis.",
            fontsize=8.5, color="#666666", va="top", style="italic")

    plt.tight_layout()
    fig.savefig(OUT_PATH, dpi=150, bbox_inches="tight")
    print(f"Saved → {OUT_PATH}")

if __name__ == "__main__":
    main()
