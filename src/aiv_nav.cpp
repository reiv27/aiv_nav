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
  
  Controller controller_{ std::make_unique<ModeA>() };

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Twist cmd_vel_msg_;

public:
  AIVController() : Node("aiv_nav")
  {
    // Configure subscribers through message_filters::Subscriber
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      this, "/odom", rmw_qos_profile_sensor_data);                                  // ★
    
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

void AIVController::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan)
{
  //   RCLCPP_INFO(this->get_logger(), "Got msg: %f", msg.ranges[0]);
  float test_odom = odom->pose.pose.position.x;
  float test_scan = scan->ranges[0];
  std::cout << test_odom << std::endl;
  std::cout << test_scan << std::endl;
  controller_.update();
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