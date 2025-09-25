#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "aiv_nav/controller.hpp"

using std::placeholders::_1;

class AIVController : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  geometry_msgs::msg::Twist cmd_vel_msg_;

  void lidarCallback_(const sensor_msgs::msg::LaserScan& msg);

  Controller controller_{ std::make_unique<ModeA>() };

public:
  AIVController() : Node("aiv_nav")
  {
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&AIVController::lidarCallback_, this, _1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
};

void AIVController::lidarCallback_(const sensor_msgs::msg::LaserScan& msg)
{
//   RCLCPP_INFO(this->get_logger(), "Got msg: %f", msg.ranges[0]);
  std::vector<float> ranges = msg.ranges;
  controller_.update();
  RCLCPP_INFO(this->get_logger(), "Current state: %s", controller_.state().name().data());
  
  cmd_vel_msg_.linear.x = -0.5;
  cmd_vel_msg_.angular.z = 0.1;
  
  cmd_vel_pub_->publish(cmd_vel_msg_);
  
  RCLCPP_INFO(this->get_logger(), 
    "State: %s | Linear: %.2f m/s | Angular: %.2f rad/s", 
    controller_.state().name().data(),
    cmd_vel_msg_.linear.x,
    cmd_vel_msg_.angular.z);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AIVController>());
  rclcpp::shutdown();
  return 0;
}