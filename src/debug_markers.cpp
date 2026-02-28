#include "reactive_circumnav/debug_markers.hpp"

#include "geometry_msgs/msg/point.hpp"

namespace reactive_circumnav
{

namespace
{

visualization_msgs::msg::Marker make_sphere_marker(
  const std::string& frame_id,
  const rclcpp::Time& stamp,
  const std::string& ns,
  int id,
  double x, double y,
  double scale,
  float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = x;
  m.pose.position.y = y;
  m.pose.position.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  return m;
}

}  // namespace

visualization_msgs::msg::MarkerArray build_debug_markers(
  const Controller& controller,
  const rclcpp::Time& stamp,
  const std::string& frame_id)
{
  visualization_msgs::msg::MarkerArray marker_array;

  const auto& closest = controller.get_closest_lidar_point();
  marker_array.markers.push_back(
    make_sphere_marker(frame_id, stamp, "closest_lidar", 0,
      closest[0], closest[1], 0.1, 1.0f, 0.0f, 0.0f, 1.0f));

  const auto& disk = controller.get_disk_pose();
  marker_array.markers.push_back(
    make_sphere_marker(frame_id, stamp, "disk", 0,
      disk[0], disk[1], 0.15, 0.0f, 0.0f, 1.0f, 1.0f));

  const auto& verA = controller.get_verA();
  marker_array.markers.push_back(
    make_sphere_marker(frame_id, stamp, "verA", 0,
      verA[0], verA[1], 0.12, 0.0f, 1.0f, 0.0f, 1.0f));

  const auto& gap1 = controller.get_gap_point_1();
  marker_array.markers.push_back(
    make_sphere_marker(frame_id, stamp, "gap_point_1", 0,
      gap1[0], gap1[1], 0.1, 1.0f, 1.0f, 0.0f, 1.0f));

  const auto& gap2 = controller.get_gap_point_2();
  marker_array.markers.push_back(
    make_sphere_marker(frame_id, stamp, "gap_point_2", 0,
      gap2[0], gap2[1], 0.1, 1.0f, 1.0f, 0.0f, 1.0f));

  visualization_msgs::msg::Marker triangle_marker;
  triangle_marker.header.frame_id = frame_id;
  triangle_marker.header.stamp = stamp;
  triangle_marker.ns = "triangle_set";
  triangle_marker.id = 0;
  triangle_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  triangle_marker.action = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point p1, p2, p3;
  p1.x = verA[0];
  p1.y = verA[1];
  p1.z = 0.0;
  p2.x = gap1[0];
  p2.y = gap1[1];
  p2.z = 0.0;
  p3.x = gap2[0];
  p3.y = gap2[1];
  p3.z = 0.0;

  triangle_marker.points.push_back(p1);
  triangle_marker.points.push_back(p2);
  triangle_marker.points.push_back(p2);
  triangle_marker.points.push_back(p3);
  triangle_marker.points.push_back(p3);
  triangle_marker.points.push_back(p1);
  triangle_marker.scale.x = 0.05;
  triangle_marker.color.r = 0.0f;
  triangle_marker.color.g = 0.5f;
  triangle_marker.color.b = 1.0f;
  triangle_marker.color.a = 0.8f;
  marker_array.markers.push_back(triangle_marker);

  return marker_array;
}

}  // namespace reactive_circumnav
