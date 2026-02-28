# reactive_circumnav

ROS 2 package for reactive circumnavigation with 2D LiDAR.

## Requirements

- ROS 2 (Humble, Jazzy, or compatible)
- Dependencies: `rclcpp`, `sensor_msgs`, `geometry_msgs`, `message_filters`, `nav_msgs`, `tf2`, `tf2_geometry_msgs`, `visualization_msgs`, `std_msgs`

## Build

```bash
cd /path/to/ros2_ws
colcon build --packages-select reactive_circumnav
source install/setup.bash
```

## Run

### Using the launch file (recommended)

Loads the default config from `config/controller_config.yaml`:

```bash
ros2 launch reactive_circumnav reactive_circumnav_launch.py
```

### Using ros2 run

Pass a parameter file explicitly:

```bash
ros2 run reactive_circumnav reactive_circumnav --params-file /path/to/controller_config.yaml
```

With the installed config:

```bash
ros2 run reactive_circumnav reactive_circumnav --params-file $(ros2 pkg prefix reactive_circumnav)/share/reactive_circumnav/config/controller_config.yaml
```

## Topics

| Type     | Topic         | Message                        | Description                    |
|----------|---------------|--------------------------------|--------------------------------|
| Subscribed | `/odom` (default) | `nav_msgs/msg/Odometry`      | Robot pose and twist           |
| Subscribed | `/scan` (default) | `sensor_msgs/msg/LaserScan`   | 2D LiDAR ranges                |
| Published  | `cmd_vel`     | `geometry_msgs/msg/Twist`     | Velocity commands              |
| Published  | `debug_markers` | `visualization_msgs/msg/MarkerArray` | Debug visualization (optional) |

Topic names for odometry and scan are set by the `odom_topic` and `scan_topic` parameters.

## Parameters

Main parameters (see `config/controller_config.yaml`):

| Parameter              | Type   | Description                    |
|------------------------|--------|--------------------------------|
| `odom_topic`           | string | Odometry topic name            |
| `scan_topic`           | string | LaserScan topic name           |
| `telemetry_log_path`   | string | Path for CSV telemetry log (empty = disabled) |
| `linear_velocity`      | double | Forward speed                  |
| `angular_velocity`     | double | Max angular speed              |
| `rho_0`                | double | Desired distance to obstacle  |
| `R_epsilon`            | double | Safety margin                  |
| `R_vis`                | double | LiDAR visibility radius        |
| `resolution`           | int    | LiDAR resolution               |
| `lidar_angle_offset`   | double | LiDAR angle offset             |
| `window_size`          | int    | Moving average window          |
| `nu`                   | double | Control gain                   |
| `history_size`         | int    | History size                   |
| `curvature_points_size`| int    | Curvature estimation window    |

Other configs in `config/` (e.g. `real_test_3.yaml`, `real_test_gap_mode.yaml`) override a subset of these for different experiments.

## License

Apache-2.0
