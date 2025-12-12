import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# x,y,theta,R_min,rho_0,min_dist,closest_lidar_x,closest_lidar_y,disk_x,disk_y
data = np.loadtxt('controller_telemetry.csv', delimiter=',')
print(data.shape)
n = data.shape[0]
t1 = int(sys.argv[1])
t2 = int(sys.argv[2])
x = data[t1:t2, 0]
y = data[t1:t2, 1]
theta = data[t1:t2, 2]
R_min = data[0, 3]
rho_0 = data[0, 4]
min_dist = data[t1:t2, 5]
closest_lidar_x = data[t1:t2, 6]
closest_lidar_y = data[t1:t2, 7]
disk_x = data[t1:t2, 8]
disk_y = data[t1:t2, 9]
verA_x = data[t1:t2, 10]
verA_y = data[t1:t2, 11]
closest_lidar_point_x = data[t1:t2, 12]
closest_lidar_point_y = data[t1:t2, 13]
lidar_point_x = data[t1:t2, 14]
lidar_point_y = data[t1:t2, 15]
dR = data[:, 16]
state = data[t1:t2, 17]
control_signal = data[t1:t2, 18]

print(f"state: {state[-1]}")
print(f"R_min: {R_min}")
print(f"rho_0: {rho_0}")
# print(f"control signal: {control_signal[-1]}")
s = 50
print(f"max dR: {np.max(dR[s:])}")
print(f"min dR: {np.min(dR[s:])}")
print(f"mean dR: {np.mean(dR[s:])}")
print(f"std dR: {np.std(dR[s:])}")
print(f"max mode dR: {np.max(dR[s:]) - np.min(dR[s:])}")

plt.figure(figsize=(12, 12))
plt.scatter(x[0], y[0], s=100, color='black', label='Start')
plt.scatter(x[-1], y[-1], s=200, marker='*', color='green', label='End')
plt.plot(x, y, color='red', label='Robot Path')
plt.plot(closest_lidar_x, closest_lidar_y, color='orange', label='Closest Lidar Points')
plt.plot(disk_x, disk_y, color='blue', label='Disk')
plt.scatter(verA_x[-1], verA_y[-1], s=200, color='green', label='VerA')
plt.scatter(closest_lidar_point_x[-1], closest_lidar_point_y[-1], s=200, color='orange',
            label='Closest Lidar Point')
plt.scatter(lidar_point_x[-1], lidar_point_y[-1], s=100, color='red', label='Lidar Point')
print()
for i in range(len(disk_x)):
  if state[i] == 2.0:
    # print(f"dR: {dR[i]}")
    # print(f"i: {i}")
    # print(f"Control signal: {control_signal[i]}")
    continue
  circle = Circle((disk_x[i], disk_y[i]), R_min, 
                  fill=False, edgecolor='blue', alpha=0.3, linewidth=0.5)
  plt.gca().add_patch(circle)
# print()

temp = 50
circle = Circle((data[temp, 8], data[temp, 9]), R_min, 
                  fill=False, edgecolor='black', linewidth=1)
plt.gca().add_patch(circle)
plt.scatter(data[temp, 0], data[temp, 1], s=50, color='yellow')
plt.scatter(data[temp, 12], data[temp, 13], s=100, color='black')
plt.scatter(data[temp, 14], data[temp, 15], s=100, color='black')
print(data[temp, 0], data[temp, 1])
print(verA_x[temp], verA_y[temp])
print(data[temp, 12], data[temp, 13])
print(data[temp, 14], data[temp, 15])
print("\nControl signal: ", data[temp, 18])

plt.xlabel(r'$x$')
plt.ylabel(r'$y$')
plt.title('Debug Plot')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()
plt.close()

plt.plot(np.arange(0, n, 1), dR)
plt.xlabel(r'$ticks$', fontsize=16)
plt.ylabel(r'$d_R(t), m$', fontsize=16)
plt.title('Distance to equidistant', fontsize=18)
# plt.axis('equal')
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=14)
stats_text = f'max:  {np.max(dR[s:]):.3f}\n' \
             f'min:  {np.min(dR[s:]):.3f}\n' \
             f'mean: {np.mean(dR[s:]):.3f}\n' \
             f'std:  {np.std(dR[s:]):.3f}\n'
plt.text(0.98, 0.98, stats_text, transform=plt.gca().transAxes,
         fontsize=16, verticalalignment='top', horizontalalignment='right',
         bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
# plt.show()
# plt.close()