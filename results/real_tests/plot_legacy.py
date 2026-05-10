#!/usr/bin/env python3
"""
Telemetry CSV + scan log (stamp_*, dt_s). Plots: dR (Y в [-rho,+rho]), dR_dot, u; опционально dt (--no-dt).
Подписи и стиль осей как в read_and_plot_csv.py (plot_distance + plot_dr_drdot_u + dt).

Usage:
  python3 plot_legacy.py TELEMETRY.csv SCAN_LOG.csv RHO
  python3 plot_legacy.py a.csv b.csv 0.15 --t1 5 --t2 40 --no-time
  python3 plot_legacy.py a.csv b.csv 0.2 --no-dt
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# Как read_and_plot_csv.py
FONT_SIZE_LABEL = 20
FONT_SIZE_TITLE = 28
FONT_SIZE_TICK = 20
FONT_SIZE_SUBPLOT_TITLE = 24

COL_R_MIN = 3
COL_DR = 16
COL_U = 18


def load_telemetry_matrix(path: Path) -> np.ndarray:
  data = np.loadtxt(path, delimiter=",")
  if data.ndim == 1:
    data = data.reshape(1, -1)
  return data


def load_scan_log_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
  if not path.is_file():
    raise FileNotFoundError(f"scan_log not found: {path}")

  raw = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
  n_rows = int(raw.shape[0]) if getattr(raw, "shape", ()) else 0
  if n_rows == 0:
    raise ValueError(
      f"scan_log has no data rows: {path}\n"
      f"  Expected: stamp_sec,stamp_nanosec,dt_s",
    )

  names = raw.dtype.names or ()
  if "dt_s" not in names or "stamp_sec" not in names or "stamp_nanosec" not in names:
    raise ValueError(f"scan_log columns: {names!r}: {path}")

  sec = np.atleast_1d(np.asarray(raw["stamp_sec"], dtype=float))
  nsec = np.atleast_1d(np.asarray(raw["stamp_nanosec"], dtype=float))
  dt = np.atleast_1d(np.asarray(raw["dt_s"], dtype=float))

  t_abs = sec + nsec * 1e-9
  t0 = t_abs[0]
  t_rel = t_abs - t0
  return t_rel, dt


def resolve_time_window(
    t_log: np.ndarray,
    t1: float | None,
    t2: float | None,
) -> tuple[float, float]:
  t_lo = float(t_log[0])
  t_hi = float(t_log[-1])
  win_a = t1 if t1 is not None else t_lo
  win_b = t2 if t2 is not None else t_hi
  if win_a > win_b:
    print("error: t1 must be <= t2", file=sys.stderr)
    sys.exit(1)
  win_a = max(win_a, t_lo)
  win_b = min(win_b, t_hi)
  if win_a >= win_b:
    print(
      f"error: empty window after clip to scan_log range [{t_lo:.6f}, {t_hi:.6f}] s",
      file=sys.stderr,
    )
    sys.exit(1)
  return win_a, win_b


def dR_dot_from_scan_dt(
    dR: np.ndarray,
    dt_on_plot: np.ndarray,
    t_plot: np.ndarray,
) -> np.ndarray:
  n = len(dR)
  if n == 0:
    return np.array([], dtype=float)
  if n == 1:
    return np.zeros(1, dtype=float)

  dt_fb = np.diff(t_plot)
  dt_step = np.asarray(dt_on_plot[:-1], dtype=float).copy()
  bad = ~np.isfinite(dt_step) | (dt_step <= 0.0)
  if dt_fb.size == dt_step.size:
    dt_step[bad] = dt_fb[bad]
  still = ~np.isfinite(dt_step) | (dt_step <= 0.0)
  if np.any(still) and dt_fb.size == dt_step.size:
    dt_step[still] = np.maximum(dt_fb[still], 1e-12)

  dR_dot = np.empty(n, dtype=float)
  dR_dot[:-1] = (dR[1:] - dR[:-1]) / dt_step
  dR_dot[-1] = (dR[-1] - dR[-2]) / dt_step[-1]
  return dR_dot


def make_time_axis(
    n_plot: int,
    dt_on_plot: np.ndarray,
    use_time: bool,
) -> tuple[np.ndarray, str, str]:
  """Как make_time_axis в read_and_plot_csv.py (cumsum(dt), сдвиг к нулю)."""
  ticks = np.arange(0, n_plot, dtype=float)
  if not use_time:
    return ticks, r"$ticks$", ""
  dt_slice = np.asarray(dt_on_plot, dtype=float).copy()
  if dt_slice.size != n_plot:
    print("Warning: --time requested, dt length mismatch. Using ticks.")
    return ticks, r"$ticks$", ""
  if not np.any(np.isfinite(dt_slice) & (dt_slice > 0)):
    print("Warning: --time requested, but CSV has no dt column. Using ticks.")
    return ticks, r"$ticks$", ""
  dt_slice = np.where(np.isfinite(dt_slice) & (dt_slice > 0), dt_slice, 0.0)
  time = np.cumsum(dt_slice)
  time -= time[0]
  return time, r"$time$, s", " vs time"


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("telemetry", type=Path)
  ap.add_argument("scan_log", type=Path)
  ap.add_argument(
    "rho",
    type=float,
    help="Границы оси Y для графика d_R: [-rho, +rho], м",
  )
  ap.add_argument("--t1", type=float, default=None)
  ap.add_argument("--t2", type=float, default=None)
  ap.add_argument(
    "--time",
    dest="use_time",
    action="store_true",
    help="X axis: cumulative time from scan dt (read_and_plot --time)",
  )
  ap.add_argument(
    "--no-time",
    dest="use_time",
    action="store_false",
    help="X axis: ticks (read_and_plot without --time)",
  )
  ap.set_defaults(use_time=True)
  ap.add_argument(
    "--no-dt",
    action="store_true",
    help="Не показывать панель $dt$ (лог скана по-прежнему нужен для $\\dot{d}_R$ и --time).",
  )
  args = ap.parse_args()

  if args.rho <= 0.0:
    print("error: rho must be positive", file=sys.stderr)
    sys.exit(1)

  tel = load_telemetry_matrix(args.telemetry)
  if tel.shape[1] <= COL_U or tel.shape[1] <= COL_R_MIN:
    print(f"error: telemetry needs columns through R_min and u", file=sys.stderr)
    sys.exit(1)

  r_min = float(tel[0, COL_R_MIN])
  dR_full = tel[:, COL_DR]
  u_full = tel[:, COL_U]
  n_full = len(dR_full)

  try:
    t_scan, dt_full = load_scan_log_csv(args.scan_log)
  except (FileNotFoundError, ValueError) as exc:
    print(f"error: {exc}", file=sys.stderr)
    sys.exit(1)

  win_a, win_b = resolve_time_window(t_scan, args.t1, args.t2)

  t_uniform = np.linspace(float(t_scan[0]), float(t_scan[-1]), n_full)
  mask_tel = (t_uniform >= win_a) & (t_uniform <= win_b)
  t_plot = t_uniform[mask_tel]
  dR = dR_full[mask_tel]
  u = u_full[mask_tel]

  if len(t_plot) == 0:
    print("error: no telemetry samples fall inside [--t1,--t2]", file=sys.stderr)
    sys.exit(1)

  mask_scan = (t_scan >= win_a) & (t_scan <= win_b)
  t_scan_w = t_scan[mask_scan]
  dt_w = dt_full[mask_scan]
  if len(t_scan_w) < 2:
    print("error: need at least 2 scan_log rows in time window", file=sys.stderr)
    sys.exit(1)

  def _dt_scalar(v: object) -> float:
    if v is None:
      return float("nan")
    s = str(v).strip().lower()
    if s == "nan":
      return float("nan")
    return float(v)

  dt_vals = np.array([_dt_scalar(v) for v in dt_w], dtype=float)
  valid = np.isfinite(dt_vals)
  if np.count_nonzero(valid) >= 2:
    dt_on_plot = np.interp(t_plot, t_scan_w[valid], dt_vals[valid])
  elif np.count_nonzero(valid) == 1:
    dt_on_plot = np.full_like(t_plot, dt_vals[valid][0], dtype=float)
  else:
    dt_on_plot = np.full_like(t_plot, np.nan, dtype=float)

  dR_dot = dR_dot_from_scan_dt(dR, dt_on_plot, t_plot)
  n_plot = len(dR)
  x_axis, x_label, title_suffix = make_time_axis(n_plot, dt_on_plot, args.use_time)

  n_axes = 3 if args.no_dt else 4
  fig, axes = plt.subplots(n_axes, 1, figsize=(24, 4 * n_axes), sharex=True)

  # Как plot_distance_to_equidistant: первая панель
  axes[0].plot(x_axis, dR, color="black")
  axes[0].axhline(0.0, color="blue", linestyle="--")
  axes[0].set_ylabel(r"$d_R(t)$, m", fontsize=FONT_SIZE_LABEL)
  axes[0].set_title(
    fr"Distance to equidistant{title_suffix}. $R = {r_min:.2f}$ m",
    fontsize=FONT_SIZE_TITLE,
  )
  axes[0].grid(True)
  axes[0].tick_params(axis="both", which="major", labelsize=FONT_SIZE_TICK)
  axes[0].set_ylim(-args.rho, args.rho)
  stats_text = (
    f'max:  {np.abs(np.max(dR)):.3f}\n'
    f'mean: {np.mean(dR):.3f}\n'
    f'std:  {np.std(dR):.3f}'
  )
  axes[0].text(
    0.98,
    0.98,
    stats_text,
    transform=axes[0].transAxes,
    fontsize=FONT_SIZE_LABEL,
    verticalalignment="top",
    horizontalalignment="right",
    bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
  )

  # Как plot_dr_drdot_u
  axes[1].plot(x_axis, dR_dot, color="green")
  axes[1].set_ylabel(r"$\dot{d}_R$", fontsize=FONT_SIZE_LABEL)
  axes[1].set_title(r"$\dot{d}_R$", fontsize=FONT_SIZE_SUBPLOT_TITLE)
  axes[1].grid(True)
  axes[1].tick_params(axis="both", which="major", labelsize=FONT_SIZE_TICK)

  axes[2].plot(x_axis, u, color="red")
  axes[2].set_ylabel(r"$u$", fontsize=FONT_SIZE_LABEL)
  axes[2].set_title(r"$u$", fontsize=FONT_SIZE_SUBPLOT_TITLE)
  axes[2].set_xlabel(x_label if args.no_dt else None, fontsize=FONT_SIZE_LABEL)
  axes[2].grid(True)
  axes[2].tick_params(axis="both", which="major", labelsize=FONT_SIZE_TICK)

  if not args.no_dt:
    axes[3].plot(x_axis, dt_on_plot, color="brown")
    axes[3].set_ylabel(r"$dt$, s", fontsize=FONT_SIZE_LABEL)
    axes[3].set_xlabel(x_label, fontsize=FONT_SIZE_LABEL)
    axes[3].set_title(r"$dt$", fontsize=FONT_SIZE_SUBPLOT_TITLE)
    axes[3].grid(True)
    axes[3].tick_params(axis="both", which="major", labelsize=FONT_SIZE_TICK)

  if len(x_axis) > 1:
    for ax in axes:
      ax.set_xlim(x_axis[0], x_axis[-1])

  plt.tight_layout()
  plt.show()


if __name__ == "__main__":
  main()
