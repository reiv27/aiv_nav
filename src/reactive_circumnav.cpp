#include <fstream>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rmw/types.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "reactive_circumnav/controller.hpp"
#include "reactive_circumnav/controller_params.hpp"
#include "reactive_circumnav/debug_markers.hpp"

class ReactiveCircumnav : public rclcpp::Node
{
private:
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_sub_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      nav_msgs::msg::Odometry,
      sensor_msgs::msg::LaserScan>>> sync_;
  
  void publishDebugMarkers();
  
  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);

  Controller controller_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Twist cmd_vel_msg_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
  visualization_msgs::msg::MarkerArray debug_markers_msg_;
  
  std::ofstream log_file_;

public:
  ReactiveCircumnav() : Node("reactive_circumnav")
  {
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>(
      "telemetry_log_path",
      "/home/user/workspace/src/reactive_circumnav/results/controller_telemetry.csv");

    const std::string odom_topic = this->get_parameter("odom_topic").as_string();
    const std::string scan_topic = this->get_parameter("scan_topic").as_string();
    const std::string telemetry_log_path =
      this->get_parameter("telemetry_log_path").as_string();

    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      this, odom_topic, rmw_qos_profile_sensor_data);

    scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
      this, scan_topic, rmw_qos_profile_sensor_data);

    using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry,
        sensor_msgs::msg::LaserScan>;

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), *odom_sub_, *scan_sub_
    );

    sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(0.15)
    );

    sync_->registerCallback(
      std::bind(&ReactiveCircumnav::syncCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    debug_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "debug_markers", 10);

    if (!telemetry_log_path.empty()) {
      log_file_.open(telemetry_log_path, std::ios::out | std::ios::trunc);
      if (log_file_.is_open()) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Logging to: %s", telemetry_log_path.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", telemetry_log_path.c_str());
      }
    }

    reactive_circumnav::ControllerParams p =
      reactive_circumnav::load_controller_params(this);

    RCLCPP_INFO(this->get_logger(), "Mode C control type: %s", p.mode_c_control_type.c_str());
    RCLCPP_INFO(this->get_logger(), "linear_velocity: %f", p.linear_velocity);
    RCLCPP_INFO(this->get_logger(), "angular_velocity: %f", p.angular_velocity);
    RCLCPP_INFO(this->get_logger(), "rho_0: %f", p.rho_0);
    RCLCPP_INFO(this->get_logger(), "R_epsilon: %f", p.R_epsilon);
    RCLCPP_INFO(this->get_logger(), "R_vis: %f", p.R_vis);
    RCLCPP_INFO(this->get_logger(), "R_min: %f",
      p.linear_velocity / p.angular_velocity + p.R_epsilon);
    RCLCPP_INFO(this->get_logger(), "--------------------------------");

    controller_ = Controller(
      p.linear_velocity,
      p.angular_velocity,
      p.rho_0,
      p.R_epsilon,
      p.R_vis,
      p.resolution,
      p.lidar_angle_offset,
      p.window_size,
      p.nu,
      p.history_size,
      p.curvature_points_size,
      p.k1,
      p.k2,
      p.mode_c_control_type
    );
    RCLCPP_INFO(this->get_logger(), "Controller initialized");
    RCLCPP_INFO(this->get_logger(), "--------------------------------");
  }
};

void ReactiveCircumnav::publishDebugMarkers()
{
  auto marker_array = reactive_circumnav::build_debug_markers(
    controller_, this->now(), "odom");
  debug_markers_pub_->publish(marker_array);
}

void ReactiveCircumnav::syncCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
                                 const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan)
{
  tf2::Quaternion q;
  tf2::fromMsg(odom->pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  const double x = odom->pose.pose.position.x;
  const double y = odom->pose.pose.position.y;
  const double theta = yaw;
  std::vector<double> lidar_data(scan->ranges.begin(), scan->ranges.end());
  std::vector<double> robot_state = {x, y, theta};

  controller_.update(robot_state, lidar_data);

  if (log_file_.is_open()) {
    log_file_
      << robot_state[0] << "," << robot_state[1] << "," << robot_state[2] << ","
      << controller_.get_R_min() << ","
      << controller_.get_rho_0() << ","
      << controller_.get_min_dist() << ","
      << controller_.get_closest_lidar_point()[0] << ","
      << controller_.get_closest_lidar_point()[1] << ","
      << controller_.get_disk_pose()[0] << ","
      << controller_.get_disk_pose()[1] << ","
      << controller_.get_verA()[0] << "," << controller_.get_verA()[1] << ","
      << controller_.get_closest_lidar_point()[0] << ","
      << controller_.get_closest_lidar_point()[1] << ","
      << controller_.get_lidar_points()[controller_.get_disk_min_arg()][0] << ","
      << controller_.get_lidar_points()[controller_.get_disk_min_arg()][1] << ","
      << controller_.get_dR_prev() << ","
      << static_cast<int>(controller_.state().state_name()) << ","
      << controller_.get_control_signal() << ","
      << controller_.get_dR_dot() << ","
      << controller_.get_curvature() << ","
      << controller_.get_dt() << ","
      << controller_.get_linear_velocity() << "\n";
  }

  cmd_vel_msg_.linear.x = controller_.get_linear_velocity();
  cmd_vel_msg_.angular.z = controller_.get_control_signal();
  cmd_vel_pub_->publish(cmd_vel_msg_);

  publishDebugMarkers();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReactiveCircumnav>());
  rclcpp::shutdown();
  return 0;
}
