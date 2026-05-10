// ROS 2 node: on each LaserScan, log header stamp and dt (s) since the previous scan.

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanDtLogger : public rclcpp::Node
{
public:
  ScanDtLogger()
  : rclcpp::Node("scan_dt_logger")
  {
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/robot2/scan");
    csv_path_ = this->declare_parameter<std::string>("csv_path", "scan_dt_log.csv");
    use_sensor_qos_ = this->declare_parameter<bool>("use_sensor_qos", true);

    out_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!out_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open csv_path: %s", csv_path_.c_str());
      throw std::runtime_error("scan_dt_logger: cannot open output file");
    }
    out_ << "stamp_sec,stamp_nanosec,dt_s\n";
    out_.flush();

    rclcpp::QoS qos(10);
    if (use_sensor_qos_) {
      qos = rclcpp::SensorDataQoS();
    }

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      qos,
      std::bind(&ScanDtLogger::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Logging LiDAR dt to '%s' (topic '%s')",
      csv_path_.c_str(),
      scan_topic_.c_str());
  }

  ~ScanDtLogger() override
  {
    if (out_.is_open()) {
      out_.close();
    }
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const rclcpp::Time arrival{this->get_clock()->now()};
    const int32_t sec = msg->header.stamp.sec;
    const uint32_t nanosec = msg->header.stamp.nanosec;

    if (have_prev_) {
      const double dt = (arrival - prev_arrival_).seconds();
      out_ << sec << "," << nanosec << "," << dt << "\n";
    } else {
      out_ << sec << "," << nanosec << ",nan\n";
    }
    out_.flush();

    prev_arrival_ = arrival;
    have_prev_ = true;
  }

  std::string scan_topic_;
  std::string csv_path_;
  bool use_sensor_qos_{true};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Time prev_arrival_;
  bool have_prev_{false};
  std::ofstream out_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ScanDtLogger>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("scan_dt_logger"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
