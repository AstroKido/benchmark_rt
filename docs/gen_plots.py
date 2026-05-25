"""Generate all benchmark plots for benchmark_decisions.md.

Run from the repo root:
    python3 docs/gen_plots.py

Output: docs/res/plot_*.png
"""

import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path

RES  = Path(__file__).parent / "res"
DATA = Path(__file__).parent.parent / "results"
RES.mkdir(exist_ok=True)

STYLE = {
    "figure.dpi": 150,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "font.size": 10,
}
plt.rcParams.update(STYLE)


# ── 1 · DDS Round-Trip Latency ────────────────────────────────────────────────
def plot_latency():
    df   = pd.read_csv(DATA / "latency_v3_fixed_samples.csv")
    info = json.loads((DATA / "latency_v3_fixed_results.json").read_text())

    lat = df["latency_ms"].values
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))
    fig.suptitle("DDS Round-Trip Latency  (300 pulses, 0.5 Nm, FR_hip)", fontweight="bold")

    # histogram
    ax1.hist(lat, bins=40, color="#4C72B0", edgecolor="white", linewidth=0.4)
    for pct, val, col in [("P95", info["p95"], "#e07b39"),
                           ("P99", info["p99"], "#c0392b")]:
        ax1.axvline(val, color=col, linestyle="--", linewidth=1.2, label=f"{pct} = {val:.1f} ms")
    ax1.axvline(info["median"], color="#2ecc71", linestyle="-", linewidth=1.4,
                label=f"Median = {info['median']:.2f} ms")
    ax1.set_xlabel("Latency (ms)")
    ax1.set_ylabel("Count")
    ax1.set_title("Distribution")
    ax1.legend(fontsize=9)

    # CDF
    s = np.sort(lat)
    p = np.arange(1, len(s) + 1) / len(s) * 100
    ax2.plot(s, p, color="#4C72B0", linewidth=1.5)
    for pct, val, col in [(95, info["p95"], "#e07b39"), (99, info["p99"], "#c0392b")]:
        ax2.axhline(pct, color=col, linestyle="--", linewidth=0.8, alpha=0.7)
        ax2.axvline(val, color=col, linestyle="--", linewidth=0.8, alpha=0.7)
    ax2.set_xlabel("Latency (ms)")
    ax2.set_ylabel("Percentile (%)")
    ax2.set_title("CDF")
    ax2.set_ylim(0, 101)

    fig.tight_layout()
    fig.savefig(RES / "plot_latency.png", bbox_inches="tight")
    plt.close(fig)
    print("plot_latency.png")


# ── 2 · Packet Loss ───────────────────────────────────────────────────────────
def plot_packet_loss():
    df   = pd.read_csv(DATA / "packet_loss_intervals.csv")
    info = json.loads((DATA / "packet_loss_results.json").read_text())

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))
    fig.suptitle("Packet Loss  (60 s, 200 Hz LowCmd keep-alive)", fontweight="bold")

    # timeline
    dropouts = df[df["dropout"] == 1]
    ax1.scatter(df["t_s"], df["interval_ms"], s=1, color="#4C72B0", alpha=0.4, label="Normal")
    if not dropouts.empty:
        ax1.scatter(dropouts["t_s"], dropouts["interval_ms"], s=12, color="#c0392b",
                    zorder=5, label=f"Dropout ({len(dropouts)})")
    ax1.axhline(3.0, color="#e07b39", linestyle="--", linewidth=1, label="3 ms threshold")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Interval (ms)")
    ax1.set_title("Arrival interval timeline")
    ax1.legend(fontsize=9, markerscale=3)

    # interval histogram
    ax2.hist(df["interval_ms"], bins=60, color="#4C72B0", edgecolor="white", linewidth=0.3)
    ax2.axvline(info["interval_ms"]["median"], color="#2ecc71", linewidth=1.4,
                label=f"Median = {info['interval_ms']['median']:.3f} ms")
    ax2.axvline(info["interval_ms"]["p99"], color="#c0392b", linestyle="--", linewidth=1.2,
                label=f"P99 = {info['interval_ms']['p99']:.2f} ms")
    ax2.set_xlabel("Interval (ms)")
    ax2.set_ylabel("Count")
    ax2.set_title("Interval distribution")
    ax2.legend(fontsize=9)

    fig.tight_layout()
    fig.savefig(RES / "plot_packet_loss.png", bbox_inches="tight")
    plt.close(fig)
    print("plot_packet_loss.png")


# ── 3 · Encoder Jitter ────────────────────────────────────────────────────────
def plot_encoder_jitter():
    info = json.loads((DATA / "encoder_jitter_results.json").read_text())
    joints = list(info["joints"].keys())

    std_mrad = [info["joints"][j]["dq_jitter_std_rad"] * 1000 for j in joints]
    p99_mrad = [info["joints"][j]["dq_jitter_p99_rad"] * 1000 for j in joints]
    max_mrad = [info["joints"][j]["dq_jitter_max_rad"] * 1000 for j in joints]

    x = np.arange(len(joints))
    w = 0.26

    fig, ax = plt.subplots(figsize=(13, 5))
    fig.suptitle("Encoder Δq Jitter  (300 s standstill, 150 k frames)", fontweight="bold")

    ax.bar(x - w, std_mrad, w, label="Std",  color="#4C72B0")
    ax.bar(x,     p99_mrad, w, label="P99",  color="#e07b39")
    ax.bar(x + w, max_mrad, w, label="Max",  color="#c0392b")

    ax.set_xticks(x)
    ax.set_xticklabels([j.replace("_", "\n") for j in joints], fontsize=8)
    ax.set_ylabel("Δq (mrad)")
    ax.set_title("Per-joint jitter — Std / P99 / Max")
    ax.legend()

    fig.tight_layout()
    fig.savefig(RES / "plot_encoder_jitter.png", bbox_inches="tight")
    plt.close(fig)
    print("plot_encoder_jitter.png")


# ── 4 · Pulse Sweep ───────────────────────────────────────────────────────────
def plot_pulse_sweep():
    info  = json.loads((DATA / "pulse_summary.json").read_text())
    df    = pd.read_csv(DATA / "pulse_traces.csv")

    widths   = [int(k) for k in info]
    hit_rate = [info[k]["hit_rate"] * 100 for k in info]
    med_lat  = [info[k]["median_latency_ms"] for k in info]
    med_peak = [info[k]["median_peak"] for k in info]

    fig = plt.figure(figsize=(14, 4))
    fig.suptitle("Torque Pulse-Width Sweep  (0.5 Nm, FR_hip, 20 reps each)", fontweight="bold")
    gs = gridspec.GridSpec(1, 3, figure=fig)

    ax1, ax2, ax3 = fig.add_subplot(gs[0]), fig.add_subplot(gs[1]), fig.add_subplot(gs[2])

    colors = ["#4C72B0", "#55A868", "#C44E52", "#8172B2", "#CCB974"]

    ax1.bar([str(w) for w in widths], hit_rate, color=colors)
    ax1.set_xlabel("Pulse width (ms)")
    ax1.set_ylabel("Hit rate (%)")
    ax1.set_title("Detection hit rate")
    ax1.set_ylim(0, 105)
    ax1.axhline(100, color="k", linestyle=":", linewidth=0.8)

    ax2.bar([str(w) for w in widths], med_lat, color=colors)
    ax2.set_xlabel("Pulse width (ms)")
    ax2.set_ylabel("Latency (ms)")
    ax2.set_title("Median detection latency")

    ax3.bar([str(w) for w in widths], med_peak, color=colors)
    ax3.axhline(0.5, color="#c0392b", linestyle="--", linewidth=1, label="Commanded 0.5 Nm")
    ax3.set_xlabel("Pulse width (ms)")
    ax3.set_ylabel("τ_est (Nm)")
    ax3.set_title("Median peak τ_est")
    ax3.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(RES / "plot_pulse_sweep.png", bbox_inches="tight")
    plt.close(fig)
    print("plot_pulse_sweep.png")


# ── 5 · IMU Standstill Noise ─────────────────────────────────────────────────
def plot_imu_standstill():
    df = pd.read_csv(DATA / "standstill_300s_imu.csv")
    t  = df["t_sys"] - df["t_sys"].iloc[0]

    fig, axes = plt.subplots(2, 3, figsize=(14, 6), sharex=True)
    fig.suptitle("IMU Standstill Noise  (300 s, 160 k frames)", fontweight="bold")

    gyro_cols = [("gyro_x", "Gyro X"), ("gyro_y", "Gyro Y"), ("gyro_z", "Gyro Z")]
    acc_cols  = [("acc_x",  "Acc X"),  ("acc_y",  "Acc Y"),  ("acc_z",  "Acc Z")]
    gyro_unit = "rad/s"
    acc_unit  = "m/s²"

    for ax, (col, lbl) in zip(axes[0], gyro_cols):
        ax.plot(t, df[col], linewidth=0.3, color="#4C72B0", rasterized=True)
        ax.set_title(f"{lbl}  σ={df[col].std()*1000:.1f} mrad/s")
        ax.set_ylabel(gyro_unit)

    for ax, (col, lbl) in zip(axes[1], acc_cols):
        ax.plot(t, df[col], linewidth=0.3, color="#55A868", rasterized=True)
        ax.set_title(f"{lbl}  σ={df[col].std()*1000:.0f} mm/s²")
        ax.set_ylabel(acc_unit)
        ax.set_xlabel("Time (s)")

    fig.tight_layout()
    fig.savefig(RES / "plot_imu_standstill.png", bbox_inches="tight", dpi=120)
    plt.close(fig)
    print("plot_imu_standstill.png")


# ── 6 · IMU During Walking ───────────────────────────────────────────────────
def plot_imu_locomotion():
    df = pd.read_csv(DATA / "trot_imu.csv")
    t  = df["t_sys"] - df["t_sys"].iloc[0]

    fig, axes = plt.subplots(2, 3, figsize=(14, 6), sharex=True)
    fig.suptitle("IMU During Walking  (~102 s, 51 k frames)", fontweight="bold")

    gyro_cols = [("gyro_x", "Gyro X"), ("gyro_y", "Gyro Y"), ("gyro_z", "Gyro Z")]
    acc_cols  = [("acc_x",  "Acc X"),  ("acc_y",  "Acc Y"),  ("acc_z",  "Acc Z")]

    for ax, (col, lbl) in zip(axes[0], gyro_cols):
        ax.plot(t, df[col], linewidth=0.4, color="#4C72B0", rasterized=True)
        ax.set_title(lbl)
        ax.set_ylabel("rad/s")

    for ax, (col, lbl) in zip(axes[1], acc_cols):
        ax.plot(t, df[col], linewidth=0.4, color="#55A868", rasterized=True)
        ax.set_title(lbl)
        ax.set_ylabel("m/s²")
        ax.set_xlabel("Time (s)")

    fig.tight_layout()
    fig.savefig(RES / "plot_imu_locomotion.png", bbox_inches="tight", dpi=120)
    plt.close(fig)
    print("plot_imu_locomotion.png")


# ── 7 · Joint Positions During Walking ───────────────────────────────────────
def plot_joint_locomotion():
    df = pd.read_csv(DATA / "trot_joints.csv")
    t  = df["t_sys"] - df["t_sys"].iloc[0]

    leg_groups = {
        "FR": ["q_FR_hip", "q_FR_thigh", "q_FR_calf"],
        "FL": ["q_FL_hip", "q_FL_thigh", "q_FL_calf"],
        "RR": ["q_RR_hip", "q_RR_thigh", "q_RR_calf"],
        "RL": ["q_RL_hip", "q_RL_thigh", "q_RL_calf"],
    }
    colors = {"hip": "#4C72B0", "thigh": "#55A868", "calf": "#C44E52"}

    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    fig.suptitle("All 12 Joint Positions During Walking  (~102 s)", fontweight="bold")

    for ax, (leg, cols) in zip(axes, leg_groups.items()):
        for col in cols:
            joint = col.split("_")[-1]
            ax.plot(t, df[col], linewidth=0.5, color=colors[joint],
                    label=joint, rasterized=True)
        ax.set_ylabel("q (rad)")
        ax.set_title(f"{leg} leg")
        ax.legend(loc="upper right", fontsize=8)

    axes[-1].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(RES / "plot_joint_locomotion.png", bbox_inches="tight", dpi=120)
    plt.close(fig)
    print("plot_joint_locomotion.png")


if __name__ == "__main__":
    plot_latency()
    plot_packet_loss()
    plot_encoder_jitter()
    plot_pulse_sweep()
    plot_imu_standstill()
    plot_imu_locomotion()
    plot_joint_locomotion()
    print("All plots saved to docs/res/")
