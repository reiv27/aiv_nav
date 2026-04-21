"""
Plot controller telemetry from CSV: distance to equidistant (dR), dR_dot, control u,
and linear velocity v.

Linear velocity is read from the 23rd CSV column (after dt), as written by
reactive_circumnav.cpp (get_linear_velocity). Disk circles use R = |v|/|ω| with
ω = u from the log; without column 23, v is inferred via ω/κ when |κ| is large
enough, else R_min. The time plot may clip u/κ for display when v is missing.

Usage:
  python read_and_plot_csv_with_velocity.py <csv_path> <t1> <t2>
Example:
  python read_and_plot_csv_with_velocity.py controller_telemetry_curv.csv 0 1000
"""

import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


# CSV column indices (matches reactive_circumnav.cpp telemetry)
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
COL_DR_DOT = 19  # present if CSV has 20 columns (new controller log)
COL_KAPPA = 20   # curvature, present if CSV has 21 columns
COL_DT = 21      # dt, present if CSV has 22 columns
COL_LINEAR_VEL = 22  # linear velocity v (cmd_vel linear.x), column index if CSV has >= 23 cols
# Legacy 22-col logs: v ≈ u/κ only if |κ| is large enough (else NaN); clip for sane plots
V_FROM_KAPPA_ABS_MIN = 0.02
V_DISPLAY_CLIP_MAX = 3.0  # m/s, display-only when v is not in CSV
# R = |v|/|ω|; below this |ω| use R_min (straight motion)
TURN_RADIUS_OMEGA_EPS = 1e-9

FONT_SIZE_LABEL = 20
FONT_SIZE_TITLE = 28
FONT_SIZE_TICK = 20


def load_telemetry(csv_path, t1, t2):
  """Load CSV and return arrays for the full range and for slice [t1:t2]."""
  data = np.loadtxt(csv_path, delimiter=',')
  n = data.shape[0]
  return data, n, t1, t2


def extract_arrays(data, n, t1, t2):
  """Extract named arrays from raw data."""
  R_min = data[0, COL_R_MIN]
  rho_0 = data[0, COL_RHO_0]
  dR = data[:, COL_DR]
  u_full = data[:, COL_U]
  state = data[t1:t2, COL_STATE]
  control_signal = data[t1:t2, COL_U]
  if data.shape[1] > COL_DR_DOT:
    dR_dot = data[:, COL_DR_DOT]
  else:
    dR_dot = None
  if data.shape[1] > COL_KAPPA:
    kappa_full = data[:, COL_KAPPA]
  else:
    kappa_full = None
  if data.shape[1] > COL_DT:
    dt_full = data[:, COL_DT]
  else:
    dt_full = None
  if data.shape[1] >= COL_LINEAR_VEL + 1:
    linear_v_full = data[:, COL_LINEAR_VEL]
    linear_v_slice = linear_v_full[t1:t2]
  else:
    linear_v_full = None
    linear_v_slice = None
  if kappa_full is not None:
    kappa_slice = kappa_full[t1:t2]
  else:
    kappa_slice = None
  x = data[t1:t2, COL_X]
  y = data[t1:t2, COL_Y]
  disk_x = data[t1:t2, COL_DISK_X]
  disk_y = data[t1:t2, COL_DISK_Y]
  closest_lidar_x = data[t1:t2, COL_CLOSEST_LIDAR_X]
  closest_lidar_y = data[t1:t2, COL_CLOSEST_LIDAR_Y]
  verA_x = data[t1:t2, COL_VERA_X]
  verA_y = data[t1:t2, COL_VERA_Y]
  closest_lidar_point_x = data[t1:t2, COL_CLOSEST_LIDAR_POINT_X]
  closest_lidar_point_y = data[t1:t2, COL_CLOSEST_LIDAR_POINT_Y]
  return {
    'n': n, 't1': t1, 't2': t2,
    'R_min': R_min, 'rho_0': rho_0,
    'dR': dR, 'dR_dot': dR_dot, 'u_full': u_full, 'kappa_full': kappa_full, 'dt_full': dt_full,
    'linear_v_full': linear_v_full,
    'linear_v_slice': linear_v_slice,
    'kappa_slice': kappa_slice,
    'state': state, 'control_signal': control_signal,
    'x': x, 'y': y,
    'disk_x': disk_x, 'disk_y': disk_y,
    'closest_lidar_x': closest_lidar_x, 'closest_lidar_y': closest_lidar_y,
    'verA_x': verA_x, 'verA_y': verA_y,
    'closest_lidar_point_x': closest_lidar_point_x,
    'closest_lidar_point_y': closest_lidar_point_y,
  }


def print_dr_stats(dR, R_min, rho_0, state_last, skip=0, linear_v_full=None):
  """Print dR statistics and last state to stdout."""
  s = skip
  print(f"state: {state_last}")
  print(f"R_min: {R_min}")
  print(f"rho_0: {rho_0}")
  print(f"max dR: {np.max(dR[s:]):.6f}")
  print(f"min dR: {np.min(dR[s:]):.6f}")
  print(f"mean dR: {np.mean(dR[s:]):.6f}")
  print(f"std dR: {np.std(dR[s:]):.6f}")
  print(f"max mode dR: {np.max(dR[s:]) - np.min(dR[s:]):.6f}")
  if linear_v_full is not None:
    print(
      f"linear v: min {np.nanmin(linear_v_full[s:]):.6f}, "
      f"max {np.nanmax(linear_v_full[s:]):.6f}, "
      f"mean {np.nanmean(linear_v_full[s:]):.6f}",
    )
  else:
    print(
      'linear v: no column 23 in CSV — re-run sim with current reactive_circumnav '
      '(telemetry writes v after dt); plot uses u/κ estimate where |κ| is large enough',
    )


def disk_radii_from_v_omega(omega, linear_v, kappa, R_fallback):
  """
  Instantaneous path radius R = |v| / |ω| (unicycle). ω is angular command u.

  If v is missing per row, use v = ω/κ when |κ| is large enough (κ = ω/v).
  Otherwise R_fallback.
  """
  omega = np.asarray(omega, dtype=float)
  n = len(omega)
  eps_k = V_FROM_KAPPA_ABS_MIN
  eps_w = TURN_RADIUS_OMEGA_EPS
  if linear_v is not None:
    v = np.asarray(linear_v, dtype=float)
  elif kappa is not None:
    kap = np.asarray(kappa, dtype=float)
    v = np.divide(
      omega,
      kap,
      out=np.full(n, np.nan),
      where=np.abs(kap) >= eps_k,
    )
  else:
    v = np.full(n, np.nan)

  v_abs = np.abs(v)
  omega_abs = np.abs(omega)
  r = np.full(n, R_fallback, dtype=float)
  np.divide(v_abs, omega_abs, out=r, where=omega_abs >= eps_w)
  r = np.where(np.isfinite(r), r, R_fallback)
  return r


def plot_disk_and_path(arr, R_min):
  """First figure: disk, path, etc. — only for range [t1:t2] set at launch."""
  x = arr['x']
  y = arr['y']
  disk_x = arr['disk_x']
  disk_y = arr['disk_y']
  closest_lidar_x = arr['closest_lidar_x']
  closest_lidar_y = arr['closest_lidar_y']
  verA_x = arr['verA_x']
  verA_y = arr['verA_y']
  closest_lidar_point_x = arr['closest_lidar_point_x']
  closest_lidar_point_y = arr['closest_lidar_point_y']
  state = arr['state']
  disk_r = disk_radii_from_v_omega(
    arr['control_signal'],
    arr.get('linear_v_slice'),
    arr.get('kappa_slice'),
    R_min,
  )

  fig, ax = plt.subplots(figsize=(12, 12))
  ax.scatter(x[0], y[0], s=100, color='black', label='Start')
  ax.scatter(x[-1], y[-1], s=200, marker='*', color='green', label='End')
  ax.plot(x, y, color='red', label='Robot Path')
  ax.plot(closest_lidar_x, closest_lidar_y, color='orange', label='Closest Lidar Points')
  ax.plot(disk_x, disk_y, color='blue', label='Disk')
  ax.scatter(verA_x[-1], verA_y[-1], s=200, color='green', label='VerA')
  ax.scatter(
    closest_lidar_point_x[-1], closest_lidar_point_y[-1],
    s=200, color='orange', label='Closest Lidar Point',
  )

  for i in range(len(disk_x)):
    if state[i] == 2.0:
      continue
    circle = Circle(
      (disk_x[i], disk_y[i]), disk_r[i],
      fill=False, edgecolor='blue', alpha=0.3, linewidth=0.5,
    )
    ax.add_patch(circle)

  ax.set_xlabel(r'$x$', fontsize=FONT_SIZE_LABEL)
  ax.set_ylabel(r'$y$', fontsize=FONT_SIZE_LABEL)
  ax.set_title(
    r'Disk and path. $R = |v|/|\omega|$',
    fontsize=FONT_SIZE_TITLE,
  )
  ax.legend(fontsize=20)
  ax.tick_params(axis='both', which='major', labelsize=FONT_SIZE_TICK)
  ax.axis('equal')
  ax.grid(True)
  plt.show()
  plt.close()


def plot_distance_to_equidistant(dR, n, R_min, skip=0):
  """First figure: dR(t) with zero line and stats box."""
  ticks = np.arange(0, n)
  fig, ax = plt.subplots(figsize=(24, 8))
  ax.plot(ticks, dR, color='black')
  ax.axhline(0.0, color='blue', linestyle='--')
  ax.set_xlabel(r'$ticks$', fontsize=FONT_SIZE_LABEL)
  ax.set_ylabel(r'$d_R(t)$, m', fontsize=FONT_SIZE_LABEL)
  ax.set_title(fr'Distance to equidistant. $R = {R_min:.2f}$ m', fontsize=FONT_SIZE_TITLE)
  ax.grid(True)
  ax.tick_params(axis='both', which='major', labelsize=FONT_SIZE_TICK)
  s = skip
  stats_text = (
    f'max:  {np.abs(np.max(dR[s:])):.3f}\n'
    f'mean: {np.mean(dR[s:]):.3f}\n'
    f'std:  {np.std(dR[s:]):.3f}'
  )
  ax.text(
    0.98, 0.98, stats_text,
    transform=ax.transAxes,
    fontsize=FONT_SIZE_LABEL,
    verticalalignment='top',
    horizontalalignment='right',
    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
  )
  plt.show()
  plt.close()


def plot_dr_drdot_u(dR, dR_dot, u_full, n, kappa_full=None, dt_full=None, linear_v_full=None):
  """dR, dR_dot, u, v (always), kappa, dt. Uses all data [0:n]."""
  if dR_dot is None:
    dR_dot = np.gradient(dR, np.arange(n))
  ticks = np.arange(0, n)

  n_axes = 4  # dR, dR_dot, u, v — linear velocity always on same figure
  if kappa_full is not None:
    n_axes += 1
  if dt_full is not None:
    n_axes += 1
  fig, axes = plt.subplots(n_axes, 1, figsize=(24, 4 * n_axes), sharex=True)
  ax_idx = 0

  axes[ax_idx].plot(ticks, dR, color='black')
  axes[ax_idx].axhline(0.0, color='blue', linestyle='--')
  axes[ax_idx].set_ylabel(r'$d_R$, m', fontsize=FONT_SIZE_LABEL)
  axes[ax_idx].set_title(r'$d_R$', fontsize=24)
  axes[ax_idx].grid(True)
  axes[ax_idx].tick_params(axis='both', which='major', labelsize=FONT_SIZE_TICK)
  ax_idx += 1

  axes[ax_idx].plot(ticks, dR_dot, color='green')
  axes[ax_idx].set_ylabel(r'$\dot{d}_R$', fontsize=FONT_SIZE_LABEL)
  axes[ax_idx].set_title(r'$\dot{d}_R$', fontsize=24)
  axes[ax_idx].grid(True)
  axes[ax_idx].tick_params(axis='both', which='major', labelsize=FONT_SIZE_TICK)
  ax_idx += 1

  axes[ax_idx].plot(ticks, u_full, color='red')
  axes[ax_idx].set_ylabel(r'$u$', fontsize=FONT_SIZE_LABEL)
  axes[ax_idx].set_xlabel(None, fontsize=FONT_SIZE_LABEL)
  axes[ax_idx].set_title(r'$u$ (angular velocity command)', fontsize=24)
  axes[ax_idx].grid(True)
  axes[ax_idx].tick_params(axis='both', which='major', labelsize=FONT_SIZE_TICK)
  ax_idx += 1

  if linear_v_full is not None:
    v_series = linear_v_full
    v_title = r'$v$ (linear velocity)'
  elif kappa_full is not None:
    v_series = np.divide(
      u_full,
      kappa_full,
      out=np.full(n, np.nan),
      where=np.abs(kappa_full) >= V_FROM_KAPPA_ABS_MIN,
    )
    v_series = np.where(
      np.isfinite(v_series) & (np.abs(v_series) <= V_DISPLAY_CLIP_MAX),
      v_series,
      np.nan,
    )
    v_title = (
      r'$v \approx u/\kappa$ (no column 23; '
      fr'$|\kappa|\geq{V_FROM_KAPPA_ABS_MIN}$, $|v|\leq{V_DISPLAY_CLIP_MAX}$ m/s)'
    )
  else:
    v_series = None
    v_title = r'$v$ (need $\kappa$ or 23-column log)'

  if v_series is not None:
    axes[ax_idx].plot(ticks, v_series, color='teal')
  else:
    axes[ax_idx].text(
      0.5,
      0.5,
      'Add 23rd column (v) or log with κ',
      transform=axes[ax_idx].transAxes,
      ha='center',
      va='center',
      fontsize=FONT_SIZE_LABEL,
    )
  axes[ax_idx].set_ylabel(r'$v$, m/s', fontsize=FONT_SIZE_LABEL)
  axes[ax_idx].set_xlabel(
    r'$ticks$' if (kappa_full is None and dt_full is None) else None,
    fontsize=FONT_SIZE_LABEL,
  )
  axes[ax_idx].set_title(v_title, fontsize=24)
  axes[ax_idx].grid(True)
  axes[ax_idx].tick_params(axis='both', which='major', labelsize=FONT_SIZE_TICK)
  ax_idx += 1

  if kappa_full is not None:
    axes[ax_idx].plot(ticks, kappa_full, color='purple')
    axes[ax_idx].set_ylabel(r'$\kappa$', fontsize=FONT_SIZE_LABEL)
    axes[ax_idx].set_xlabel(r'$ticks$' if dt_full is None else None, fontsize=FONT_SIZE_LABEL)
    axes[ax_idx].set_title(r'$\kappa$ (curvature)', fontsize=24)
    axes[ax_idx].grid(True)
    axes[ax_idx].tick_params(axis='both', which='major', labelsize=FONT_SIZE_TICK)
    ax_idx += 1

  if dt_full is not None:
    axes[ax_idx].plot(ticks, dt_full, color='brown')
    axes[ax_idx].set_ylabel(r'$dt$, s', fontsize=FONT_SIZE_LABEL)
    axes[ax_idx].set_xlabel(r'$ticks$', fontsize=FONT_SIZE_LABEL)
    axes[ax_idx].set_title(r'$dt$', fontsize=24)
    axes[ax_idx].grid(True)
    axes[ax_idx].tick_params(axis='both', which='major', labelsize=FONT_SIZE_TICK)

  plt.tight_layout()
  plt.show()
  plt.close()


def parse_args():
  parser = argparse.ArgumentParser(
    description='Plot controller telemetry: dR, dR_dot, u, v, κ, dt.',
  )
  parser.add_argument('csv_path', help='Path to telemetry CSV')
  parser.add_argument('t1', type=int, help='Start tick (inclusive)')
  parser.add_argument('t2', type=int, help='End tick (exclusive)')
  return parser.parse_args()


def main():
  args = parse_args()
  data, n, t1, t2 = load_telemetry(args.csv_path, args.t1, args.t2)
  print(data.shape)

  arr = extract_arrays(data, n, t1, t2)
  print_dr_stats(
    arr['dR'],
    arr['R_min'],
    arr['rho_0'],
    arr['state'][-1],
    skip=0,
    linear_v_full=arr.get('linear_v_full'),
  )

  # First figure: only range [t1:t2] (slice in arr)
  plot_disk_and_path(arr, arr['R_min'])
  # Second figure: all data [0:n]
  plot_dr_drdot_u(
    arr['dR'], arr['dR_dot'], arr['u_full'], n,
    kappa_full=arr.get('kappa_full'),
    dt_full=arr.get('dt_full'),
    linear_v_full=arr.get('linear_v_full'),
  )


if __name__ == '__main__':
  main()
