#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
// Messages
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"
// Message Filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rmw/types.h>
// tf2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Controller
#include "aiv_nav/controller.hpp"

class AIVController : public rclcpp::Node
{
private:

  // Message filters trick: synchronize Odometry and LaserScan messages
  // using message_filters subscribers and synchronizer
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_sub_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>>> sync_;
  
  void publishDebugMarkers();
  
  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);

  Controller controller_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Twist cmd_vel_msg_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
  visualization_msgs::msg::MarkerArray debug_markers_msg_;
  

  // CSV logging
  std::ofstream log_file_;

public:
  AIVController() : Node("aiv_nav")
  {  
    // Configure subscribers through message_filters::Subscriber
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      this, "odom", rmw_qos_profile_sensor_data);
    
    scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
      this, "scan", rmw_qos_profile_sensor_data);

    // Policy ApproximateTime with queue size
    using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry,
        sensor_msgs::msg::LaserScan>;

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), *odom_sub_, *scan_sub_
    );

    // Allowed "window" of desynchronization by stamp (slop)
    sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(0.15)
    );

    // Callback on synchronized pair of messages
    sync_->registerCallback(
      std::bind(&AIVController::syncCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Debug markers publisher
    debug_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "debug_markers", 10);

    // Open log file
    log_file_.open(
      "/home/user/ros2_ws/src/aiv_nav/debug/controller_telemetry.csv",
      std::ios::out | std::ios::trunc
    );
    if (log_file_.is_open()) {
      RCLCPP_INFO_ONCE(this->get_logger(), "Logging to: debug/controller_telemetry.csv");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open log file!");
    }

    this->declare_parameter<double>("linear_velocity");
    this->declare_parameter<double>("angular_velocity");
    this->declare_parameter<double>("rho_0");
    this->declare_parameter<double>("R_epsilon");
    this->declare_parameter<double>("R_vis");
    this->declare_parameter<int>("resolution");
    this->declare_parameter<double>("lidar_angle_offset");
    this->declare_parameter<int>("window_size");
    this->declare_parameter<double>("nu");
    this->declare_parameter<int>("history_size");

    double linear_velocity = this->get_parameter("linear_velocity").as_double();
    double angular_velocity = this->get_parameter("angular_velocity").as_double();
    double rho_0 = this->get_parameter("rho_0").as_double();
    double R_epsilon = this->get_parameter("R_epsilon").as_double();
    double R_vis = this->get_parameter("R_vis").as_double();
    int resolution = this->get_parameter("resolution").as_int();
    double lidar_angle_offset = this->get_parameter("lidar_angle_offset").as_double();
    int window_size = this->get_parameter("window_size").as_int();
    double nu = this->get_parameter("nu").as_double();
    int history_size = this->get_parameter("history_size").as_int();

    RCLCPP_INFO(this->get_logger(), "linear_velocity: %f", linear_velocity);
    RCLCPP_INFO(this->get_logger(), "angular_velocity: %f", angular_velocity);
    RCLCPP_INFO(this->get_logger(), "rho_0: %f", rho_0);
    RCLCPP_INFO(this->get_logger(), "R_epsilon: %f", R_epsilon);
    RCLCPP_INFO(this->get_logger(), "R_vis: %f", R_vis);
    // RCLCPP_INFO(this->get_logger(), "resolution: %d", resolution);
    // RCLCPP_INFO(this->get_logger(), "lidar_angle_offset: %f", lidar_angle_offset);
    // RCLCPP_INFO(this->get_logger(), "window_size: %d", window_size);
    // RCLCPP_INFO(this->get_logger(), "nu: %f", nu);
    RCLCPP_INFO(this->get_logger(), "R_min: %f", linear_velocity / angular_velocity + R_epsilon);
    RCLCPP_INFO(this->get_logger(), "--------------------------------");
  
    controller_ = Controller(
      linear_velocity,
      angular_velocity,
      rho_0,
      R_epsilon,
      R_vis,
      resolution,
      lidar_angle_offset,
      window_size,
      nu,
      history_size
    );
    RCLCPP_INFO(this->get_logger(), "Controller initialized");
    RCLCPP_INFO(this->get_logger(), "--------------------------------");
  }
};

void AIVController::publishDebugMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto now = this->now();

  visualization_msgs::msg::Marker closest_lidar_marker;
  closest_lidar_marker.header.frame_id = "robot2/odom";
  closest_lidar_marker.header.stamp = now;
  closest_lidar_marker.ns = "closest_lidar";
  closest_lidar_marker.id = 0;
  closest_lidar_marker.type = visualization_msgs::msg::Marker::SPHERE;
  closest_lidar_marker.action = visualization_msgs::msg::Marker::ADD;
  closest_lidar_marker.pose.position.x = controller_.get_closest_lidar_point()[0];
  closest_lidar_marker.pose.position.y = controller_.get_closest_lidar_point()[1];
  closest_lidar_marker.pose.position.z = 0.0;
  closest_lidar_marker.pose.orientation.w = 1.0;
  closest_lidar_marker.scale.x = 0.1;
  closest_lidar_marker.scale.y = 0.1;
  closest_lidar_marker.scale.z = 0.1;
  closest_lidar_marker.color.r = 1.0;
  closest_lidar_marker.color.g = 0.0;
  closest_lidar_marker.color.b = 0.0;
  closest_lidar_marker.color.a = 1.0;
  marker_array.markers.push_back(closest_lidar_marker);
  
  visualization_msgs::msg::Marker disk_marker;
  disk_marker.header.frame_id = "robot2/odom";
  disk_marker.header.stamp = now;
  disk_marker.ns = "disk";
  disk_marker.id = 0;
  disk_marker.type = visualization_msgs::msg::Marker::SPHERE;
  disk_marker.action = visualization_msgs::msg::Marker::ADD;
  disk_marker.pose.position.x = controller_.get_disk_pose()[0];
  disk_marker.pose.position.y = controller_.get_disk_pose()[1];
  disk_marker.pose.position.z = 0.0;
  disk_marker.pose.orientation.w = 1.0;
  disk_marker.scale.x = 0.15;
  disk_marker.scale.y = 0.15;
  disk_marker.scale.z = 0.15;
  disk_marker.color.r = 0.0;
  disk_marker.color.g = 0.0;
  disk_marker.color.b = 1.0;
  disk_marker.color.a = 1.0;
  marker_array.markers.push_back(disk_marker);
  
  visualization_msgs::msg::Marker verA_marker;
  verA_marker.header.frame_id = "robot2/odom";
  verA_marker.header.stamp = now;
  verA_marker.ns = "verA";
  verA_marker.id = 0;
  verA_marker.type = visualization_msgs::msg::Marker::SPHERE;
  verA_marker.action = visualization_msgs::msg::Marker::ADD;
  verA_marker.pose.position.x = controller_.get_verA()[0];
  verA_marker.pose.position.y = controller_.get_verA()[1];
  verA_marker.pose.position.z = 0.0;
  verA_marker.pose.orientation.w = 1.0;
  verA_marker.scale.x = 0.12;
  verA_marker.scale.y = 0.12;
  verA_marker.scale.z = 0.12;
  verA_marker.color.r = 0.0;
  verA_marker.color.g = 1.0;
  verA_marker.color.b = 0.0;
  verA_marker.color.a = 1.0;
  marker_array.markers.push_back(verA_marker);
  
  visualization_msgs::msg::Marker gap1_marker;
  gap1_marker.header.frame_id = "robot2/odom";
  gap1_marker.header.stamp = now;
  gap1_marker.ns = "gap_point_1";
  gap1_marker.id = 0;
  gap1_marker.type = visualization_msgs::msg::Marker::SPHERE;
  gap1_marker.action = visualization_msgs::msg::Marker::ADD;
  gap1_marker.pose.position.x = controller_.get_gap_point_1()[0];
  gap1_marker.pose.position.y = controller_.get_gap_point_1()[1];
  gap1_marker.pose.position.z = 0.0;
  gap1_marker.pose.orientation.w = 1.0;
  gap1_marker.scale.x = 0.1;
  gap1_marker.scale.y = 0.1;
  gap1_marker.scale.z = 0.1;
  gap1_marker.color.r = 1.0;
  gap1_marker.color.g = 1.0;
  gap1_marker.color.b = 0.0;
  gap1_marker.color.a = 1.0;
  marker_array.markers.push_back(gap1_marker);
  
  visualization_msgs::msg::Marker gap2_marker;
  gap2_marker.header.frame_id = "robot2/odom";
  gap2_marker.header.stamp = now;
  gap2_marker.ns = "gap_point_2";
  gap2_marker.id = 0;
  gap2_marker.type = visualization_msgs::msg::Marker::SPHERE;
  gap2_marker.action = visualization_msgs::msg::Marker::ADD;
  gap2_marker.pose.position.x = controller_.get_gap_point_2()[0];
  gap2_marker.pose.position.y = controller_.get_gap_point_2()[1];
  gap2_marker.pose.position.z = 0.0;
  gap2_marker.pose.orientation.w = 1.0;
  gap2_marker.scale.x = 0.1;
  gap2_marker.scale.y = 0.1;
  gap2_marker.scale.z = 0.1;
  gap2_marker.color.r = 1.0;
  gap2_marker.color.g = 1.0;
  gap2_marker.color.b = 0.0;
  gap2_marker.color.a = 1.0;
  marker_array.markers.push_back(gap2_marker);
  
  visualization_msgs::msg::Marker triangle_marker;
  triangle_marker.header.frame_id = "robot2/odom";
  triangle_marker.header.stamp = now;
  triangle_marker.ns = "triangle_set";
  triangle_marker.id = 0;
  triangle_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  triangle_marker.action = visualization_msgs::msg::Marker::ADD;
  
  geometry_msgs::msg::Point p1, p2, p3;
  p1.x = controller_.get_verA()[0];
  p1.y = controller_.get_verA()[1];
  p1.z = 0.0;
  
  p2.x = controller_.get_gap_point_1()[0];
  p2.y = controller_.get_gap_point_1()[1];
  p2.z = 0.0;
  
  p3.x = controller_.get_gap_point_2()[0];
  p3.y = controller_.get_gap_point_2()[1];
  p3.z = 0.0;
  
  triangle_marker.points.push_back(p1);
  triangle_marker.points.push_back(p2);
  triangle_marker.points.push_back(p2);
  triangle_marker.points.push_back(p3);
  triangle_marker.points.push_back(p3);
  triangle_marker.points.push_back(p1);
  
  triangle_marker.scale.x = 0.05;
  triangle_marker.color.r = 0.0;
  triangle_marker.color.g = 0.5;
  triangle_marker.color.b = 1.0;
  triangle_marker.color.a = 0.8;
  marker_array.markers.push_back(triangle_marker);
  
  debug_markers_pub_->publish(marker_array);
}

void AIVController::syncCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
                                 const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan)
{
  // Convert odometry orientation to yaw
  tf2::Quaternion q;
  tf2::fromMsg(odom->pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  const double x = odom->pose.pose.position.x;
  const double y = odom->pose.pose.position.y;
  const double theta = yaw;
  std::vector<double> lidar_data(scan->ranges.begin(), scan->ranges.end());
  std::vector<double> robot_state = {x, y, theta};

  // Update controller with new robot state and lidar data
  controller_.update(robot_state, lidar_data);

  // Log telemetry data
  if (log_file_.is_open()) {
    log_file_ 
              << robot_state[0] << ","
              << robot_state[1] << ","
              << robot_state[2] << ","
              << controller_.get_R_min() << ","
              << controller_.get_rho_0() << ","
              << controller_.get_min_dist() << ","
              << controller_.get_closest_lidar_point()[0] << ","
              << controller_.get_closest_lidar_point()[1] << ","
              << controller_.get_disk_pose()[0] << ","
              << controller_.get_disk_pose()[1] << ","
              << controller_.get_verA()[0] << ","
              << controller_.get_verA()[1] << ","
              << controller_.get_closest_lidar_point()[0] << ","
              << controller_.get_closest_lidar_point()[1] << ","
              << controller_.get_lidar_points()[controller_.get_disk_min_arg()][0] << ","
              << controller_.get_lidar_points()[controller_.get_disk_min_arg()][1] << ","
              << controller_.get_dR_prev() << ","
              << static_cast<int>(controller_.state().state_name()) << ","
              << controller_.get_control_signal() << "\n";
  }
  
  // Debug output
  // RCLCPP_INFO(this->get_logger(), "State: %lf, %lf, %lf", x, y, theta);
  // RCLCPP_INFO(this->get_logger(), "Lidar ray 0: %lf", lidar_data[0]);
  RCLCPP_INFO(this->get_logger(), "Mode: %d", static_cast<int>(controller_.state().state_name()));
  // RCLCPP_INFO(this->get_logger(), "Lidar ray 0: %lf", lidar_data[0]);
  RCLCPP_INFO(this->get_logger(), "Lidar closest point: %lf, %lf\n", controller_.get_closest_lidar_point()[0], controller_.get_closest_lidar_point()[1]);
  // RCLCPP_INFO(this->get_logger(), "Control signal: %lf", controller_.get_control_signal());
  // RCLCPP_INFO(this->get_logger(), " ");

  // Publish control signal
  cmd_vel_msg_.linear.x = controller_.get_linear_velocity();
  cmd_vel_msg_.angular.z = controller_.get_control_signal();
  cmd_vel_pub_->publish(cmd_vel_msg_);

  // Publish debug markers
  publishDebugMarkers();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AIVController>());
  rclcpp::shutdown();
  return 0;
}