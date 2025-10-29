import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# x,y,theta,R_min,rho_0,min_dist,closest_lidar_x,closest_lidar_y,disk_x,disk_y
data = np.loadtxt('controller_telemetry.csv', delimiter=',')
x = data[:, 0]
y = data[:, 1]
theta = data[:, 2]
R_min = data[0, 3]
rho_0 = data[0, 4]
min_dist = data[:, 5]
closest_lidar_x = data[:, 6]
closest_lidar_y = data[:, 7]
disk_x = data[:, 8]
disk_y = data[:, 9]

rho = np.linalg.norm(np.array([disk_x, disk_y]) - np.array([closest_lidar_x, closest_lidar_y]), axis=0)
print(R_min + rho_0)
print(rho)

plt.figure(figsize=(10, 10))
plt.scatter(x[0], y[0], s=100, color='black', label='Start')
plt.scatter(x[-1], y[-1], s=200, marker='*', color='green', label='End')
plt.plot(x, y, color='red', label='Robot Path')
plt.plot(closest_lidar_x, closest_lidar_y, color='orange', label='Closest Lidar Points')
plt.plot(disk_x, disk_y, color='blue', label='Disk')
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