#!/usr/bin/env python3
"""
Baseline analysis for VeloGuard
Yuveer Rajpal, The Shri Ram School

Processes the Bike&Safe dataset (Blauth da Silva & Tavares, 2022) —
handlebar-mounted smartphone IMU data from normal cycling — to extract
what "normal riding" looks like so I know what thresholds to set.

The main outputs are:
  - per-phase statistics (stationary / accelerating / cruising / braking / turning)
  - a threshold envelope: what values does normal riding produce, so a crash
    detector knows what NOT to trigger on
  - labelled time-series data for further analysis

Some data quirks I had to handle:
  - Routes 1/2 are semicolon-delimited, Route 3 uses commas
  - Accelerometer files have a duplicated Y column for some reason
  - One file is named *.csv.csv (yes, really)
  - Accel/gyro timestamps are nanoseconds; GPS is epoch milliseconds
    so I align them on relative time from each file's start

Usage:
  python baseline_analysis.py --data-dir /path/to/BikeAndSafe --out-dir output
"""

import argparse
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt

# ---- constants ---------------------------------------------------------------

FS = 50.0          # sensor rate (20ms period per dataset description)
GRAVITY_CUTOFF_HZ = 0.5
SMOOTH_WINDOW_S = 1.0

# phase classification — tuned by looking at the data
SPEED_STATIONARY = 0.7     # m/s, below this = stopped
ACCEL_THRESH = 0.20        # m/s^2 speed derivative
BRAKE_THRESH = -0.25
TURN_YAW_RATE = 0.20       # rad/s after smoothing (noise floor ~0.1, gentle turns 0.2-0.8)
MIN_PHASE_S = 1.0          # debounce: ignore phase segments shorter than this

PHASES = ["STATIONARY", "ACCELERATING", "CRUISING", "BRAKING", "TURNING"]
PHASE_COLORS = {
    "STATIONARY": "#9e9e9e",
    "ACCELERATING": "#2e7d32",
    "CRUISING": "#1565c0",
    "BRAKING": "#c62828",
    "TURNING": "#ef6c00",
}

# column name in the merged dataframe -> output label
BASELINE_METRICS = {
    "speed_ms": "speed",
    "lin_accel_mag_ms2": "lin_acc_mag",
    "total_accel_mag_ms2": "acc_mag",
    "jerk_ms3": "jerk",
    "gyro_magnitude_rads": "gyro_mag",
    "yaw_rate_rads": "gyro_z",
    "roll_deg": "roll",
    "pitch_deg": "pitch",
    "tilt_from_vertical_deg": "tilt",
}

# ---- loading -----------------------------------------------------------------

def sniff_delimiter(path):
    with open(path) as f:
        line = f.readline()
    return ";" if line.count(";") > line.count(",") else ","


def read_sensor_csv(path, n_values):
    # headerless: timestamp_ns, sensor_id, then n_values floats
    delim = sniff_delimiter(path)
    df = pd.read_csv(path, sep=delim, header=None, engine="python")
    df = df.dropna(axis=1, how="all")   # trailing separator creates empty col
    df = df.iloc[:, : 2 + n_values]
    return df


def repair_quantized_timestamps(df):
    # some gyro files have timestamps quantized to whole seconds (~98% duplicates)
    # samples are still evenly recorded at 50Hz so I rebuild a uniform timeline
    ts = df["ts"].to_numpy()
    if len(ts) > 1 and np.mean(np.diff(ts) == 0) > 0.5:
        df["ts"] = np.linspace(ts[0], ts[-1], len(ts))
    return df


def load_accelerometer(path):
    df = read_sensor_csv(path, 4)   # x, y, y(duplicate), z
    df.columns = ["ts", "id", "x", "y", "y_dup", "z"]
    df = df.drop(columns=["id", "y_dup"]).astype(float)
    return repair_quantized_timestamps(df)


def load_gyroscope(path):
    df = read_sensor_csv(path, 3)
    df.columns = ["ts", "id", "x", "y", "z"]
    df = df.drop(columns=["id"]).astype(float)
    return repair_quantized_timestamps(df)


def load_gps(path):
    delim = sniff_delimiter(path)
    df = pd.read_csv(path, sep=delim, header=None, engine="python")
    df = df.dropna(axis=1, how="all")
    df = df.iloc[:, :5]
    df.columns = ["tag", "lat", "lon", "speed", "ts"]
    df = df.drop(columns=["tag"]).astype(float)
    df = df.drop_duplicates(subset="ts").sort_values("ts").reset_index(drop=True)
    return df


def find_laps(data_dir):
    laps = []
    for root, _dirs, files in os.walk(data_dir):
        def match(kind):
            hits = [f for f in files if re.search(kind, f, re.IGNORECASE) and f.endswith(".csv")]
            return os.path.join(root, hits[0]) if hits else None
        acc, gyr, gps = match("accelerometer"), match("gyroscope"), match("GPS")
        if acc and gyr and gps:
            route = os.path.basename(os.path.dirname(root))
            lap = os.path.basename(root)
            laps.append({"name": f"{route} / {lap}", "accel": acc, "gyro": gyr, "gps": gps})
    return sorted(laps, key=lambda d: d["name"])


# ---- signal processing -------------------------------------------------------

def lowpass(signal, cutoff_hz, fs=FS, order=2):
    b, a = butter(order, cutoff_hz / (fs / 2), btype="low")
    return filtfilt(b, a, signal)


def smooth(signal, window_s=SMOOTH_WINDOW_S, fs=FS):
    n = max(3, int(window_s * fs) | 1)
    return pd.Series(signal).rolling(n, center=True, min_periods=1).mean().to_numpy()


def process_lap(lap):
    acc = load_accelerometer(lap["accel"])
    gyr = load_gyroscope(lap["gyro"])
    gps = load_gps(lap["gps"])

    # relative time in seconds — each stream from its own first sample
    acc["t"] = (acc["ts"] - acc["ts"].iloc[0]) / 1e9
    gyr["t"] = (gyr["ts"] - gyr["ts"].iloc[0]) / 1e9
    gps["t"] = (gps["ts"] - gps["ts"].iloc[0]) / 1e3

    # uniform 50Hz timeline over the overlap of all three streams
    t_end = min(acc["t"].iloc[-1], gyr["t"].iloc[-1], gps["t"].iloc[-1])
    t = np.arange(0.0, t_end, 1.0 / FS)
    df = pd.DataFrame({"t": t})

    for axis in "xyz":
        df[f"acc_{axis}"] = np.interp(t, acc["t"], acc[axis])
        df[f"gyro_{axis}"] = np.interp(t, gyr["t"], gyr[axis])
    df["speed"] = np.interp(t, gps["t"], gps["speed"])

    # gravity vector via low-pass, then subtract to get linear acceleration
    # phone is screen-up on handlebar so gravity mostly sits on +Z axis
    g = np.column_stack([lowpass(df[f"acc_{a}"].to_numpy(), GRAVITY_CUTOFF_HZ) for a in "xyz"])
    g_norm = np.linalg.norm(g, axis=1)
    g_norm[g_norm == 0] = 1e-9

    df["roll"]  = np.degrees(np.arctan2(g[:, 0], g[:, 2]))
    df["pitch"] = np.degrees(np.arctan2(g[:, 1], np.hypot(g[:, 0], g[:, 2])))
    df["tilt"]  = np.degrees(np.arccos(np.clip(g[:, 2] / g_norm, -1, 1)))

    lin = df[["acc_x", "acc_y", "acc_z"]].to_numpy() - g
    df["lin_acc_mag"] = np.linalg.norm(lin, axis=1)
    df["acc_mag"]     = np.linalg.norm(df[["acc_x", "acc_y", "acc_z"]].to_numpy(), axis=1)
    df["jerk"]        = np.abs(np.gradient(smooth(df["acc_mag"], 0.2), 1.0 / FS))
    df["gyro_mag"]    = np.linalg.norm(df[["gyro_x", "gyro_y", "gyro_z"]].to_numpy(), axis=1)

    # phase classification
    speed_s = smooth(df["speed"].to_numpy(), 2.0)
    dv = np.gradient(speed_s, 1.0 / FS)
    # smooth SIGNED yaw rate before taking abs — handlebar vibration is zero-mean
    # and cancels out; a real turn survives the average
    yaw_s = np.abs(smooth(df["gyro_z"].to_numpy(), 1.5))

    phase = np.full(len(df), "CRUISING", dtype=object)
    phase[dv > ACCEL_THRESH] = "ACCELERATING"
    phase[dv < BRAKE_THRESH] = "BRAKING"
    phase[(yaw_s > TURN_YAW_RATE) & (speed_s >= SPEED_STATIONARY)] = "TURNING"
    phase[speed_s < SPEED_STATIONARY] = "STATIONARY"
    df["phase"] = debounce_phases(phase, min_len=int(MIN_PHASE_S * FS))
    df["lap"] = lap["name"]
    return df


def debounce_phases(phase, min_len):
    # merge very short segments into the preceding phase to avoid noise spikes
    # creating spurious one-second "phases"
    phase = phase.copy()
    boundaries = np.flatnonzero(phase[1:] != phase[:-1]) + 1
    segments = np.split(np.arange(len(phase)), boundaries)
    for seg in segments[1:]:
        if len(seg) < min_len:
            phase[seg] = phase[seg[0] - 1]
    return phase


# ---- baseline stats ----------------------------------------------------------

def baseline_stats(df):
    rows = []
    for phase_name, grp in df.groupby("phase"):
        for label, col in BASELINE_METRICS.items():
            v = grp[col].to_numpy()
            rows.append({
                "phase": phase_name, "metric": label,
                "n_samples": len(v), "duration_s": round(len(v) / FS, 1),
                "mean": v.mean(), "std": v.std(), "min": v.min(),
                "p05": np.percentile(v, 5), "median": np.percentile(v, 50),
                "p95": np.percentile(v, 95), "p99": np.percentile(v, 99),
                "p99_9": np.percentile(v, 99.9), "max": v.max(),
            })
    out = pd.DataFrame(rows)
    out["phase"] = pd.Categorical(out["phase"], categories=PHASES, ordered=True)
    return out.sort_values(["phase", "metric"]).reset_index(drop=True)


def crash_thresholds(df):
    # upper envelope of normal riding — a VeloGuard threshold set above
    # max_observed would produce zero false positives on this dataset
    # p99_9 is a reasonable starting point if you want a bit of sensitivity
    moving = df[df["phase"] != "STATIONARY"]
    def env(col):
        v = moving[col].to_numpy()
        return {"p99": float(np.percentile(v, 99)),
                "p99_9": float(np.percentile(v, 99.9)),
                "max_observed": float(v.max())}
    return {
        "description": (
            "Envelope of NORMAL (non-crash) riding from 9 laps. "
            "A crash detector threshold set above 'max_observed' produced zero "
            "false positives on this dataset; 'p99_9' is a more sensitive starting point."
        ),
        "sampling_rate_hz": FS,
        "total_accel_magnitude_ms2": env("acc_mag"),
        "linear_accel_magnitude_ms2": env("lin_acc_mag"),
        "jerk_ms3": env("jerk"),
        "gyro_magnitude_rads": env("gyro_mag"),
        "tilt_from_vertical_deg": env("tilt"),
    }


# ---- plots -------------------------------------------------------------------

def plot_lap_timeline(df, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    t = df["t"]
    axes[0].plot(t, df["speed"],       color="k", lw=0.8)
    axes[0].set_ylabel("Speed (m/s)")
    axes[1].plot(t, df["lin_acc_mag"], color="k", lw=0.4)
    axes[1].set_ylabel("Lin. accel (m/s²)")
    axes[2].plot(t, df["gyro_z"],      color="k", lw=0.4)
    axes[2].set_ylabel("Yaw rate (rad/s)")
    axes[3].plot(t, df["tilt"],        color="k", lw=0.6)
    axes[3].set_ylabel("Tilt (deg)")
    axes[3].set_xlabel("Time (s)")

    phase = df["phase"].to_numpy()
    boundaries = np.flatnonzero(phase[1:] != phase[:-1]) + 1
    starts = np.concatenate([[0], boundaries])
    ends   = np.concatenate([boundaries, [len(phase)]])
    for s, e in zip(starts, ends):
        for ax in axes:
            ax.axvspan(t.iloc[s], t.iloc[e-1], color=PHASE_COLORS[phase[s]], alpha=0.18, lw=0)

    handles = [Patch(color=c, alpha=0.4, label=p) for p, c in PHASE_COLORS.items()]
    axes[0].legend(handles=handles, ncol=5, loc="upper right", fontsize=8)
    fig.suptitle(f"Cycling phases — {df['lap'].iloc[0]}")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_phase_distributions(df, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    metrics = [
        ("speed",       "Speed (m/s)"),
        ("lin_acc_mag", "Linear accel magnitude (m/s²)"),
        ("gyro_mag",    "Gyro magnitude (rad/s)"),
        ("tilt",        "Tilt from vertical (deg)"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    for ax, (col, label) in zip(axes.flat, metrics):
        data = [df.loc[df["phase"] == p, col] for p in PHASES]
        bp = ax.boxplot(data, tick_labels=PHASES, showfliers=False, patch_artist=True)
        for patch, p in zip(bp["boxes"], PHASES):
            patch.set_facecolor(PHASE_COLORS[p])
            patch.set_alpha(0.5)
        ax.set_ylabel(label)
        ax.tick_params(axis="x", labelsize=8)
    fig.suptitle("Baseline distributions per cycling phase (all laps, outliers hidden)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


# ---- main --------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="/Users/yuveer/Downloads/Kaggle")
    ap.add_argument("--out-dir",  default="output")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    laps = find_laps(args.data_dir)
    if not laps:
        sys.exit(f"No lap folders found under {args.data_dir}")
    print(f"Found {len(laps)} laps")

    frames = []
    for lap in laps:
        df = process_lap(lap)
        frames.append(df)
        mins   = df["t"].iloc[-1] / 60
        counts = df["phase"].value_counts(normalize=True).mul(100).round(1)
        summary = ", ".join(f"{p}: {counts.get(p, 0.0)}%" for p in PHASES)
        print(f"  {lap['name']}: {mins:.1f} min  —  {summary}")

    all_df = pd.concat(frames, ignore_index=True)

    stats = baseline_stats(all_df)
    stats.round(4).to_csv(os.path.join(args.out_dir, "baseline_summary.csv"), index=False)

    thresholds = crash_thresholds(all_df)
    with open(os.path.join(args.out_dir, "baseline_thresholds.json"), "w") as f:
        json.dump(thresholds, f, indent=2)

    # downsample to 10Hz to keep the labelled file manageable
    labelled = all_df.iloc[::5][["lap", "t", "speed", "acc_mag", "lin_acc_mag",
                                  "jerk", "gyro_mag", "gyro_z", "roll", "pitch",
                                  "tilt", "phase"]]
    labelled.round(4).to_csv(os.path.join(args.out_dir, "labelled_data_10hz.csv"), index=False)

    if not args.no_plots:
        plot_phase_distributions(all_df, os.path.join(args.out_dir, "phase_distributions.png"))
        for df in frames:
            safe = re.sub(r"\W+", "_", df["lap"].iloc[0]).strip("_")
            plot_lap_timeline(df, os.path.join(args.out_dir, f"timeline_{safe}.png"))

    total_min = all_df.groupby("lap")["t"].max().sum() / 60
    print(f"\nTotal riding time: {total_min:.1f} min ({len(all_df):,} samples @ {FS:.0f} Hz)")
    print("\nPhase split across all laps:")
    print(all_df["phase"].value_counts(normalize=True).mul(100).round(1).to_string())
    print(f"\nOutputs in {os.path.abspath(args.out_dir)}/")


if __name__ == "__main__":
    main()
