#include <memory>
#include <vector>

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

  // Message filters trick: synchronize Odometry and LaserScan messages using message_filters subscribers and synchronizer
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_sub_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>>> sync_;

  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);
  
  // Test pole values
  
  
  Controller controller_{ std::make_unique<ModeA>(), 0.0, 0.0, 0.0, 0.0, 12.0, 360 };

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Twist cmd_vel_msg_;

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
      SyncPolicy(20), *odom_sub_, *scan_sub_
    );

    // Allowed "window" of desynchronization by stamp (slop)
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.15));

    // Callback on synchronized pair of messages
    sync_->registerCallback(
      std::bind(&AIVController::syncCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
};

void AIVController::syncCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
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
  std::vector<double> new_robot_state = controller_.get_robot_state();
  // RCLCPP_INFO(this->get_logger(), "Robot state: %f, %f, %f", new_robot_state[0], new_robot_state[1], new_robot_state[2]);
  std::vector<double> new_lidar_data = controller_.get_lidar_data();
  std::vector<std::vector<double>> new_lidar_points = controller_.get_lidar_points();
  // RCLCPP_INFO(this->get_logger(), "Lidar data: %f", new_lidar_data[0]);
  RCLCPP_INFO(this->get_logger(), "Lidar points: %f, %f", new_lidar_points[0][0], new_lidar_points[0][1]);
  // RCLCPP_INFO(this->get_logger(), "Min distance: %f", controller_.get_min_dist());
  RCLCPP_INFO(this->get_logger(), "Current state: %s", controller_.state().name().data());

  // cmd_vel_pub_->publish(cmd_vel_msg_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AIVController>());
  rclcpp::shutdown();
  return 0;
}