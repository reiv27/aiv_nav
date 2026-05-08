"""
Plot controller telemetry from CSV: distance to equidistant (dR), dR_dot, control u.

Usage:
  python read_and_plot_csv.py <csv_path> <t1> <t2> [--curvature|--no-curvature]
Example:
  python read_and_plot_csv.py main_gazebo.csv 0 1090
  python read_and_plot_csv.py main_gazebo.csv 0 1090 --no-curvature
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
    'state': state, 'control_signal': control_signal,
    'x': x, 'y': y,
    'disk_x': disk_x, 'disk_y': disk_y,
    'closest_lidar_x': closest_lidar_x, 'closest_lidar_y': closest_lidar_y,
    'verA_x': verA_x, 'verA_y': verA_y,
    'closest_lidar_point_x': closest_lidar_point_x,
    'closest_lidar_point_y': closest_lidar_point_y,
  }


def print_dr_stats(dR, R_min, rho_0, state_last, skip=0):
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
      (disk_x[i], disk_y[i]), R_min,
      fill=False, edgecolor='blue', alpha=0.3, linewidth=0.5,
    )
    ax.add_patch(circle)

  ax.set_xlabel(r'$x$', fontsize=FONT_SIZE_LABEL)
  ax.set_ylabel(r'$y$', fontsize=FONT_SIZE_LABEL)
  ax.set_title(fr'Disk and path. $R = {R_min:.2f}$ m', fontsize=FONT_SIZE_TITLE)
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


def plot_dr_drdot_u(dR, dR_dot, u_full, n, kappa_full=None, dt_full=None, plot_curvature=True):
  """Second figure: dR, dR_dot, u, kappa, dt. Uses all data [0:n]. kappa, dt optional."""
  if dR_dot is None:
    dR_dot = np.gradient(dR, np.arange(n))
  ticks = np.arange(0, n)

  kappa_plot = kappa_full if plot_curvature else None

  n_axes = 3
  if kappa_plot is not None:
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
  axes[ax_idx].set_xlabel(r'$ticks$' if n_axes == 3 else None, fontsize=FONT_SIZE_LABEL)
  axes[ax_idx].set_title(r'$u$', fontsize=24)
  axes[ax_idx].grid(True)
  axes[ax_idx].tick_params(axis='both', which='major', labelsize=FONT_SIZE_TICK)
  ax_idx += 1

  if kappa_plot is not None:
    axes[ax_idx].plot(ticks, kappa_plot, color='purple')
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
    description='Plot controller telemetry: dR, dR_dot, u.',
  )
  parser.add_argument('csv_path', help='Path to telemetry CSV')
  parser.add_argument('t1', type=int, help='Start tick (inclusive)')
  parser.add_argument('t2', type=int, help='End tick (exclusive)')
  parser.add_argument(
    '--curvature',
    default=True,
    action=argparse.BooleanOptionalAction,
    help='Plot curvature κ when present in CSV (default: on). Use --no-curvature to omit.',
  )
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
  )

  # First figure: only range [t1:t2] (slice in arr)
  plot_disk_and_path(arr, arr['R_min'])
  # Second figure: all data [0:n]
  plot_dr_drdot_u(
    arr['dR'], arr['dR_dot'], arr['u_full'], n,
    kappa_full=arr.get('kappa_full'),
    dt_full=arr.get('dt_full'),
    plot_curvature=args.curvature,
  )


if __name__ == '__main__':
  main()
