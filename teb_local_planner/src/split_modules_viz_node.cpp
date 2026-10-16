/*********************************************************************
 * Publish one RViz scene per split TEB module.
 *********************************************************************/
#include "split_module_scenarios.hpp"

#include "teb_local_planner/detail/costmap_segment_clip.hpp"
#include "teb_local_planner/detail/edge_mode_env.hpp"
#include "teb_local_planner/detail/plan_corner_geometry.hpp"
#include "teb_local_planner/path_dynamic_wait.h"

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace
{

geometry_msgs::msg::Point point(double x, double y, double z = 0.05)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

visualization_msgs::msg::Marker lineStrip(
  const std::string & ns, int id, float r, float g, float b)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.06;
  marker.pose.orientation.w = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0f;
  return marker;
}

visualization_msgs::msg::Marker label(const std::string & ns, int id, double x, double y, const std::string & text)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.6;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.35;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  marker.text = text;
  return marker;
}

visualization_msgs::msg::Marker sphere(
  const std::string & ns, int id, double x, double y, double radius, float r, float g, float b)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = radius;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = radius * 2.0;
  marker.scale.y = radius * 2.0;
  marker.scale.z = radius * 2.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.9f;
  return marker;
}

visualization_msgs::msg::MarkerArray buildScenes(double time_s)
{
  visualization_msgs::msg::MarkerArray scenes;
  const auto plan_data = teb_local_planner::test_data::planWindow();
  const int corner = teb_local_planner::detail::lockedCornerIdx(
    plan_data.plan, 0, 120.0, 40.0);
  auto plan_line = lineStrip("plan_window", 0, 0.2f, 0.7f, 1.0f);
  for (const auto & pose : plan_data.plan) {
    plan_line.points.push_back(point(pose.pose.position.x, pose.pose.position.y));
  }
  scenes.markers.push_back(plan_line);
  scenes.markers.push_back(label(
    "plan_window", 1, plan_data.plan.front().pose.position.x, plan_data.plan.front().pose.position.y + 1.2,
    "plan_window corner=" + std::to_string(corner)));

  const auto clip = teb_local_planner::test_data::obstacleClip();
  int clip_id = 0;
  for (const auto & segment : clip.segments) {
    auto raw = lineStrip("obstacle_clip", clip_id++, 0.55f, 0.55f, 0.55f);
    raw.points.push_back(point(segment.x0, segment.y0 + 8.0));
    raw.points.push_back(point(segment.x1, segment.y1 + 8.0));
    scenes.markers.push_back(raw);
    if (!segment.kept) {
      continue;
    }
    double x0 = segment.x0;
    double y0 = segment.y0 + 8.0;
    double x1 = segment.x1;
    double y1 = segment.y1 + 8.0;
    teb_local_planner::detail::clipWorldSegmentToAlignedRect(
      x0, y0, x1, y1, segment.xmin, segment.ymin + 8.0, segment.xmax, segment.ymax + 8.0);
    auto clipped = lineStrip("obstacle_clip", clip_id++, 0.1f, 0.9f, 0.2f);
    clipped.points.push_back(point(x0, y0, 0.12));
    clipped.points.push_back(point(x1, y1, 0.12));
    scenes.markers.push_back(clipped);
  }
  scenes.markers.push_back(label("obstacle_clip", clip_id, 0.0, 12.0, "obstacle_clip"));

  const auto edge = teb_local_planner::test_data::edgeLine();
  auto wall = lineStrip("edge_line", 0, 1.0f, 0.4f, 0.1f);
  for (const auto & xy : edge.wall) {
    wall.points.push_back(point(xy.first, xy.second));
  }
  scenes.markers.push_back(wall);
  scenes.markers.push_back(sphere(
    "edge_line", 2, edge.far_vehicle_x, edge.far_vehicle_y, 0.25, 0.2f, 0.6f, 1.0f));
  scenes.markers.push_back(label("edge_line", 1, edge.wall.front().first, edge.wall.front().second + 1.0, "edge_line"));

  const auto follow = teb_local_planner::test_data::edgeFollow();
  auto primary = lineStrip("edge_following", 1, 0.2f, 0.8f, 0.9f);
  for (const auto & pose : follow.path) {
    primary.points.push_back(point(pose.pose.position.x, pose.pose.position.y + 16.0));
  }
  scenes.markers.push_back(primary);
  scenes.markers.push_back(label(
    "edge_following", 2, follow.path.front().pose.position.x, follow.path.front().pose.position.y + 17.2,
    "edge_following " + teb_local_planner::detail::normalizeEdgeModeEnv(follow.modes.front().raw.c_str())));

  const auto narrow_data = teb_local_planner::test_data::narrowPassage();
  auto narrow = lineStrip("narrow", 0, 1.0f, 0.45f, 0.0f);
  for (const auto & polygon_point : narrow_data.polygon.points) {
    narrow.points.push_back(point(polygon_point.x, polygon_point.y + 22.0));
  }
  if (!narrow_data.polygon.points.empty()) {
    narrow.points.push_back(point(
      narrow_data.polygon.points.front().x, narrow_data.polygon.points.front().y + 22.0));
  }
  scenes.markers.push_back(narrow);
  scenes.markers.push_back(sphere(
    "narrow", 2, narrow_data.inside_x, narrow_data.inside_y + 22.0, 0.15, 0.1f, 1.0f, 0.3f));

  const auto hold = teb_local_planner::test_data::pathHold();
  auto wait_line = lineStrip("path_hold", 0, 0.8f, 0.2f, 0.8f);
  for (const auto & pose : hold.plan) {
    wait_line.points.push_back(point(pose.pose.position.x, pose.pose.position.y + 28.0));
  }
  scenes.markers.push_back(wait_line);
  const auto moving = teb_local_planner::test_data::movingObstacleXY(hold, time_s);
  scenes.markers.push_back(sphere(
    "path_hold", 2, moving.first, moving.second + 28.0, hold.obstacle_radius,
    1.0f, 0.15f, 0.15f));
  scenes.markers.push_back(label("path_hold", 1, 0.0, 30.2, "path_hold moving obstacle"));

  const auto spin = teb_local_planner::test_data::rotation();
  const double yaw_span = std::abs(spin.goal_yaw_rad) + 0.4;
  const double yaw = std::fmod(time_s * 0.8, yaw_span);
  auto rotation = lineStrip("rotation", 0, 0.3f, 1.0f, 0.5f);
  rotation.points.push_back(point(0.0, 34.0));
  rotation.points.push_back(point(std::cos(yaw) * 1.2, 34.0 + std::sin(yaw) * 1.2));
  scenes.markers.push_back(rotation);
  scenes.markers.push_back(label("rotation", 1, 0.0, 35.6, "rotation"));

  const auto corner_data = teb_local_planner::test_data::corner();
  auto corner_box = lineStrip("corner", 0, 1.0f, 0.2f, 0.2f);
  for (const auto & xy : {point(0, 38), point(1.2, 38), point(1.2, 39), point(0, 39), point(0, 38)}) {
    corner_box.points.push_back(xy);
  }
  scenes.markers.push_back(corner_box);
  scenes.markers.push_back(label(
    "corner", 1, 0.0, 39.6,
    "corner samples=" + std::to_string(corner_data.samples.size())));
  return scenes;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("split_modules_viz");
  auto publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/teb_split_modules/markers", 10);
  const auto started = node->now();
  auto timer = node->create_wall_timer(std::chrono::milliseconds(200), [publisher, node, started]() {
    const double time_s = (node->now() - started).seconds();
    auto stamped = buildScenes(time_s);
    const auto now = node->now();
    for (auto & marker : stamped.markers) {
      marker.header.stamp = now;
    }
    publisher->publish(stamped);
  });
  RCLCPP_INFO(node->get_logger(), "拆分模块可视化已发布: /teb_split_modules/markers");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
