#!/usr/bin/env python3
"""
Parse controller telemetry CSV from results/real_tests/ and save figure PNGs.

Format matches reactive_circumnav telemetry (see read_and_plot_csv.py):
  x,y,theta, R_min, rho_0, min_dist,
  closest_lidar x,y, disk x,y, verA x,y,
  closest_lidar x,y (again), lidar_min x,y,
  dR, state, u
Optional extra columns (newer logs): dR_dot, kappa, dt, linear_velocity.

Usage:
  python3 plot_real_tests.py
  python3 plot_real_tests.py --csv /path/to/file.csv
  python3 plot_real_tests.py --show   # also open interactive windows
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Column indices — same as read_and_plot_csv.py (first 19 fields)
COL_X, COL_Y, COL_THETA = 0, 1, 2
COL_R_MIN, COL_RHO_0 = 3, 4
COL_MIN_DIST = 5
COL_CLOSEST_LIDAR_X, COL_CLOSEST_LIDAR_Y = 6, 7
COL_DISK_X, COL_DISK_Y = 8, 9
COL_VERA_X, COL_VERA_Y = 10, 11
COL_CLOSEST_LIDAR_POINT_X, COL_CLOSEST_LIDAR_POINT_Y = 12, 13
COL_LIDAR_POINT_X, COL_LIDAR_POINT_Y = 14, 15
COL_DR = 16
COL_STATE = 17
COL_U = 18
COL_DR_DOT = 19
COL_KAPPA = 20
COL_DT = 21
COL_LINEAR_VEL = 22


def load_csv(path: Path) -> np.ndarray:
  data = np.loadtxt(path, delimiter=",")
  if data.ndim == 1:
    data = data.reshape(1, -1)
  return data


def parse_telemetry(data: np.ndarray) -> dict:
  """Split raw matrix into named arrays; optional tail columns if present."""
  n, c = data.shape
  out = {
    "n_rows": n,
    "n_cols": c,
    "ticks": np.arange(n),
    "x": data[:, COL_X],
    "y": data[:, COL_Y],
    "theta": data[:, COL_THETA],
    "R_min": float(data[0, COL_R_MIN]),
    "rho_0": float(data[0, COL_RHO_0]),
    "min_dist": data[:, COL_MIN_DIST],
    "closest_lidar_x": data[:, COL_CLOSEST_LIDAR_X],
    "closest_lidar_y": data[:, COL_CLOSEST_LIDAR_Y],
    "disk_x": data[:, COL_DISK_X],
    "disk_y": data[:, COL_DISK_Y],
    "verA_x": data[:, COL_VERA_X],
    "verA_y": data[:, COL_VERA_Y],
    "closest_point_x": data[:, COL_CLOSEST_LIDAR_POINT_X],
    "closest_point_y": data[:, COL_CLOSEST_LIDAR_POINT_Y],
    "lidar_min_x": data[:, COL_LIDAR_POINT_X],
    "lidar_min_y": data[:, COL_LIDAR_POINT_Y],
    "dR": data[:, COL_DR],
    "state": data[:, COL_STATE],
    "u": data[:, COL_U],
  }
  if c > COL_DR_DOT:
    out["dR_dot"] = data[:, COL_DR_DOT]
  else:
    out["dR_dot"] = np.gradient(out["dR"], out["ticks"], edge_order=2)
  if c > COL_KAPPA:
    out["kappa"] = data[:, COL_KAPPA]
  else:
    out["kappa"] = None
  if c > COL_DT:
    out["dt"] = data[:, COL_DT]
    out["time_s"] = np.concatenate([[0.0], np.cumsum(out["dt"][:-1])])
  else:
    out["dt"] = None
    out["time_s"] = None
  if c > COL_LINEAR_VEL:
    out["linear_vel"] = data[:, COL_LINEAR_VEL]
  else:
    out["linear_vel"] = None
  return out


def plot_path(telem: dict, title: str, ax: plt.Axes) -> None:
  ax.plot(telem["x"], telem["y"], color="red", linewidth=1.2, label="robot")
  ax.plot(
    telem["closest_lidar_x"],
    telem["closest_lidar_y"],
    color="orange",
    alpha=0.7,
    linewidth=0.8,
    label="closest lidar (scan)",
  )
  ax.plot(telem["disk_x"], telem["disk_y"], color="blue", linewidth=1.0, label="disk center")
  ax.scatter(telem["x"][0], telem["y"][0], s=80, c="black", zorder=5, label="start")
  ax.scatter(telem["x"][-1], telem["y"][-1], s=120, marker="*", c="green", zorder=5, label="end")
  ax.scatter(
    telem["verA_x"][-1],
    telem["verA_y"][-1],
    s=100,
    c="green",
    edgecolors="k",
    zorder=5,
    label="VerA (last)",
  )
  ax.set_aspect("equal", adjustable="box")
  ax.set_xlabel("x, m")
  ax.set_ylabel("y, m")
  ax.set_title(title)
  ax.grid(True, alpha=0.3)
  ax.legend(loc="best", fontsize=8)


def plot_timeseries_main(telem: dict, title: str, fig: plt.Figure) -> None:
  if telem["time_s"] is not None:
    t = telem["time_s"]
    xlabel = "time, s"
  else:
    t = telem["ticks"].astype(float)
    xlabel = "tick #"

  nrows = 5
  if telem["kappa"] is not None:
    nrows += 1
  if telem["linear_vel"] is not None:
    nrows += 1
  if telem["dt"] is not None:
    nrows += 1

  axes = fig.subplots(nrows, 1, sharex=True)
  axes = np.atleast_1d(axes)
  row = 0

  axes[row].plot(t, telem["dR"], color="black", linewidth=0.8)
  axes[row].axhline(0.0, color="steelblue", linestyle="--", linewidth=0.8)
  axes[row].set_ylabel(r"$d_R$, m")
  axes[row].set_title(f"{title} — $d_R$")
  axes[row].grid(True, alpha=0.3)
  row += 1

  axes[row].plot(t, telem["dR_dot"], color="darkgreen", linewidth=0.8)
  axes[row].set_ylabel(r"$\dot{d}_R$")
  axes[row].grid(True, alpha=0.3)
  row += 1

  axes[row].plot(t, telem["u"], color="crimson", linewidth=0.8)
  axes[row].set_ylabel(r"$u$")
  axes[row].grid(True, alpha=0.3)
  row += 1

  axes[row].step(t, telem["state"], where="post", color="purple", linewidth=0.8)
  axes[row].set_ylabel("state")
  axes[row].grid(True, alpha=0.3)
  row += 1

  axes[row].plot(t, telem["min_dist"], color="teal", linewidth=0.8)
  axes[row].set_ylabel("min_dist")
  axes[row].grid(True, alpha=0.3)
  row += 1

  if telem["kappa"] is not None:
    axes[row].plot(t, telem["kappa"], color="darkviolet", linewidth=0.8)
    axes[row].set_ylabel(r"$\kappa$")
    axes[row].grid(True, alpha=0.3)
    row += 1

  if telem["linear_vel"] is not None:
    axes[row].plot(t, telem["linear_vel"], color="saddlebrown", linewidth=0.8)
    axes[row].set_ylabel(r"$v$")
    axes[row].grid(True, alpha=0.3)
    row += 1

  if telem["dt"] is not None:
    axes[row].plot(t, telem["dt"], color="gray", linewidth=0.8)
    axes[row].set_ylabel(r"$\Delta t$")
    axes[row].grid(True, alpha=0.3)
    row += 1

  axes[-1].set_xlabel(xlabel)
  fig.tight_layout()


def print_summary(name: str, telem: dict) -> None:
  dR = telem["dR"]
  print(f"\n=== {name} ===")
  print(f"  rows: {telem['n_rows']}, cols: {telem['n_cols']}")
  print(f"  R_min={telem['R_min']:.4f}, rho_0={telem['rho_0']:.4f}")
  print(f"  dR: min={dR.min():.4f} max={dR.max():.4f} mean={dR.mean():.4f}")
  print(f"  last state: {telem['state'][-1]:.0f}")


def process_file(csv_path: Path, out_dir: Path, show: bool) -> None:
  data = load_csv(csv_path)
  telem = parse_telemetry(data)
  stem = csv_path.stem
  print_summary(stem, telem)

  out_dir.mkdir(parents=True, exist_ok=True)

  fig1, ax1 = plt.subplots(figsize=(8, 8))
  plot_path(
    telem,
    f"{stem}\n$R_{{\\min}}={telem['R_min']:.2f}$ m, $\\rho_0={telem['rho_0']:.2f}$",
    ax1,
  )
  p1 = out_dir / f"{stem}_path.png"
  fig1.savefig(p1, dpi=150, bbox_inches="tight")
  if show:
    plt.show()
  plt.close(fig1)

  n_sig = 5 + sum(
    1
    for k in ("kappa", "linear_vel", "dt")
    if telem[k] is not None
  )
  fig2 = plt.figure(figsize=(10, 2.2 * n_sig))
  plot_timeseries_main(telem, stem, fig2)
  p2 = out_dir / f"{stem}_signals.png"
  fig2.savefig(p2, dpi=150, bbox_inches="tight")
  if show:
    plt.show()
  plt.close(fig2)

  print(f"  saved: {p1}")
  print(f"  saved: {p2}")


def default_real_tests_dir() -> Path:
  return Path(__file__).resolve().parent / "real_tests"


def parse_args() -> argparse.Namespace:
  p = argparse.ArgumentParser(description="Plot real_tests telemetry CSVs.")
  p.add_argument(
    "--csv",
    type=Path,
    default=None,
    help="Single CSV file (default: all *.csv in real_tests/)",
  )
  p.add_argument(
    "--out",
    type=Path,
    default=None,
    help="Output directory for PNGs (default: real_tests/figures/)",
  )
  p.add_argument(
    "--show",
    action="store_true",
    help="Open interactive matplotlib windows in addition to saving PNGs",
  )
  return p.parse_args()


def main() -> None:
  args = parse_args()
  base = default_real_tests_dir()
  out_dir = args.out if args.out else base / "figures"

  if args.csv:
    files = [args.csv.resolve()]
  else:
    files = sorted(base.glob("*.csv"))
    if not files:
      print(f"No CSV files in {base}")
      return

  for f in files:
    if not f.is_file():
      print(f"Skip missing: {f}")
      continue
    process_file(f, out_dir, args.show)


if __name__ == "__main__":
  main()
