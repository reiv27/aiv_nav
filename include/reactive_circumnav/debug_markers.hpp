#ifndef REACTIVE_CIRCUMNAV_DEBUG_MARKERS_HPP
#define REACTIVE_CIRCUMNAV_DEBUG_MARKERS_HPP

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "reactive_circumnav/controller.hpp"

namespace reactive_circumnav
{

/**
 * @brief Build debug MarkerArray from controller state for visualization
 * @param controller Controller instance with current state
 * @param stamp Timestamp for marker headers
 * @param frame_id Frame id for marker headers (default "odom")
 * @return MarkerArray to publish
 */
visualization_msgs::msg::MarkerArray build_debug_markers(
  const Controller& controller,
  const rclcpp::Time& stamp,
  const std::string& frame_id = "odom");

}  // namespace reactive_circumnav

#endif  // REACTIVE_CIRCUMNAV_DEBUG_MARKERS_HPP
