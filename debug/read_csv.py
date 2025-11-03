import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# x,y,theta,R_min,rho_0,min_dist,closest_lidar_x,closest_lidar_y,disk_x,disk_y
data = np.loadtxt('controller_telemetry.csv', delimiter=',')
print(data.shape)
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

rho = np.linalg.norm(np.array([disk_x, disk_y]) - np.array([closest_lidar_x, closest_lidar_y]), axis=0)
print(f"start: {x[-1]}, {y[-1]}")
print(f"VerA: {verA_x[-1]}, {verA_y[-1]}")
print(f"Closest Lidar Point: {closest_lidar_point_x[-1]}, {closest_lidar_point_y[-1]}")
print(f"Lidar Point: {lidar_point_x[-1]}, {lidar_point_y[-1]}")

plt.figure(figsize=(10, 10))
plt.scatter(x[0], y[0], s=100, color='black', label='Start')
plt.scatter(x[-1], y[-1], s=200, marker='*', color='green', label='End')
plt.plot(x, y, color='red', label='Robot Path')
plt.plot(closest_lidar_x, closest_lidar_y, color='orange', label='Closest Lidar Points')
plt.plot(disk_x, disk_y, color='blue', label='Disk')
plt.scatter(verA_x[-1], verA_y[-1], s=200, color='green', label='VerA')
plt.scatter(closest_lidar_point_x[-1], closest_lidar_point_y[-1], s=200, color='orange', label='Closest Lidar Point')
plt.scatter(lidar_point_x[-1], lidar_point_y[-1], s=100, color='red', label='Lidar Point')
for i in range(len(disk_x)):
  circle = Circle((disk_x[i], disk_y[i]), R_min, 
                  fill=False, edgecolor='blue', alpha=0.3, linewidth=0.5)
  plt.gca().add_patch(circle)
plt.xlabel(r'$x$')
plt.ylabel(r'$y$')
plt.title('Debug Plot')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()