#include <memory>
#include <vector>
#include <fstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
// Messages
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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

using std::placeholders::_1;

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

  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);

  Controller controller_;


  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Twist cmd_vel_msg_;

  // CSV logging
  std::ofstream log_file_;
  int log_iteration_ = 0;
  bool log_header_written_ = false;

public:
  AIVController() : Node("aiv_nav")
  {  
    // Configure subscribers through message_filters::Subscriber
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      this, "/odom", rmw_qos_profile_sensor_data);
    
    scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
      this, "/scan", rmw_qos_profile_sensor_data);

    // Policy ApproximateTime with queue size
    using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry,
        sensor_msgs::msg::LaserScan>;

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), *odom_sub_, *scan_sub_
    );

    // Allowed "window" of desynchronization by stamp (slop)
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.15));

    // Callback on synchronized pair of messages
    sync_->registerCallback(
      std::bind(&AIVController::syncCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Open log file
    log_file_.open(
      "/home/user/projects/ros2_ws/src/aiv_nav/debug/controller_telemetry.csv",
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

    double linear_velocity = this->get_parameter("linear_velocity").as_double();
    double angular_velocity = this->get_parameter("angular_velocity").as_double();
    double rho_0 = this->get_parameter("rho_0").as_double();
    double R_epsilon = this->get_parameter("R_epsilon").as_double();
    double R_vis = this->get_parameter("R_vis").as_double();
    int resolution = this->get_parameter("resolution").as_int();
    double lidar_angle_offset = this->get_parameter("lidar_angle_offset").as_double();
    int window_size = this->get_parameter("window_size").as_int();
    double nu = this->get_parameter("nu").as_double();
  
    controller_ = Controller(
      linear_velocity,
      angular_velocity,
      rho_0,
      R_epsilon,
      R_vis,
      resolution,
      lidar_angle_offset,
      window_size,
      nu
    );
  }
};

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
  // state,x,y,theta,R_min,rho_0,min_dist,closest_lidar_x,closest_lidar_y,disk_x,disk_y
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
    log_file_.flush();
  }
  
  // Debug output
  RCLCPP_INFO(this->get_logger(), "Mode: %s", controller_.state().name().data());

  // Publish control signal
  cmd_vel_msg_.linear.x = controller_.get_linear_velocity();
  cmd_vel_msg_.angular.z = controller_.get_control_signal();
  cmd_vel_pub_->publish(cmd_vel_msg_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AIVController>());
  rclcpp::shutdown();
  return 0;
}