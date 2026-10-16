#include "main_flow_checks.hpp"

#include "teb_local_planner/pose_se2.h"
#include "teb_local_planner/robot_footprint_model.h"

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{

geometry_msgs::msg::Point point(double x, double y, double z = 0.05)
{
  geometry_msgs::msg::Point value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
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
  marker.scale.x = 0.05;
  marker.pose.orientation.w = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0f;
  marker.lifetime = rclcpp::Duration(0, 0);
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
  marker.lifetime = rclcpp::Duration(0, 0);
  return marker;
}

visualization_msgs::msg::Marker label(
  const std::string & ns, int id, double x, double y, double z, const std::string & text,
  float r, float g, float b, double scale = 0.38)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = scale;
  marker.color.a = 1.0f;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.text = text;
  marker.lifetime = rclcpp::Duration(0, 0);
  return marker;
}

visualization_msgs::msg::Marker arrow(
  int id, double x, double y, double yaw, float r, float g, float b)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "motion";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.07;
  marker.scale.y = 0.14;
  marker.scale.z = 0.14;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0f;
  marker.lifetime = rclcpp::Duration(0, 0);
  marker.points.push_back(point(x, y, 0.18));
  marker.points.push_back(point(x + 0.55 * std::cos(yaw), y + 0.55 * std::sin(yaw), 0.18));
  return marker;
}

visualization_msgs::msg::Marker erase(const std::string & ns, int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = ns;
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  return marker;
}

struct ViewShift
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

ViewShift shiftFor(const std::vector<teb_local_planner::test_data::ScenePose> & route)
{
  ViewShift shift;
  if (!route.empty()) {
    shift.x = route.front().x;
    shift.y = route.front().y;
    shift.yaw = route.front().yaw;
  }
  return shift;
}

void mapToView(
  double x, double y, double yaw, const ViewShift & shift,
  double & out_x, double & out_y, double & out_yaw)
{
  const double dx = x - shift.x;
  const double dy = y - shift.y;
  const double cosine = std::cos(shift.yaw);
  const double sine = std::sin(shift.yaw);
  out_x = cosine * dx + sine * dy;
  out_y = -sine * dx + cosine * dy;
  out_yaw = yaw - shift.yaw;
}

double pathLength(const std::vector<teb_local_planner::test_data::ScenePose> & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return length;
}

teb_local_planner::test_data::ScenePose poseAlong(
  const std::vector<teb_local_planner::test_data::ScenePose> & path, double progress)
{
  teb_local_planner::test_data::ScenePose pose;
  if (path.empty()) {
    return pose;
  }
  if (path.size() == 1 || progress <= 0.0) {
    return path.front();
  }
  if (progress >= 1.0) {
    return path.back();
  }
  std::vector<double> cumulative(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    cumulative[i] = cumulative[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  const double target = cumulative.back() * std::max(0.0, std::min(1.0, progress));
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (cumulative[i] + 1e-9 < target) {
      continue;
    }
    const double span = cumulative[i] - cumulative[i - 1];
    const double blend = span > 1e-6 ? (target - cumulative[i - 1]) / span : 1.0;
    pose.x = path[i - 1].x + blend * (path[i].x - path[i - 1].x);
    pose.y = path[i - 1].y + blend * (path[i].y - path[i - 1].y);
    double dyaw = path[i].yaw - path[i - 1].yaw;
    while (dyaw > M_PI) {
      dyaw -= 2.0 * M_PI;
    }
    while (dyaw < -M_PI) {
      dyaw += 2.0 * M_PI;
    }
    pose.yaw = path[i - 1].yaw + blend * dyaw;
    return pose;
  }
  return path.back();
}

const std::vector<teb_local_planner::test_data::ScenePose> & routeOf(
  const teb_local_planner::test_data::FlowScene & scene)
{
  if (scene.motion.size() >= 2) {
    return scene.motion;
  }
  if (scene.optimized.size() >= 2) {
    return scene.optimized;
  }
  return scene.reference;
}

visualization_msgs::msg::MarkerArray buildStatic(
  const teb_local_planner::test_data::FlowItem & flow, const ViewShift & shift)
{
  visualization_msgs::msg::MarkerArray scenes;
  const auto & scene = flow.scene;
  const auto & route = routeOf(scene);
  auto place = [&](double x, double y) {
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
    mapToView(x, y, 0.0, shift, vx, vy, vyaw);
    return point(vx, vy);
  };

  auto reference = lineStrip("reference", 0, 0.72f, 0.72f, 0.72f);
  reference.scale.x = 0.04;
  for (const auto & pose : scene.reference) {
    auto sample = place(pose.x, pose.y);
    sample.z = 0.02;
    reference.points.push_back(sample);
  }
  if (reference.points.size() >= 2) {
    scenes.markers.push_back(reference);
  } else {
    scenes.markers.push_back(erase("reference", 0));
  }

  auto optimized = lineStrip("teb_trajectory", 0, 0.15f, 0.55f, 1.0f);
  optimized.scale.x = 0.09;
  for (const auto & pose : scene.optimized) {
    auto sample = place(pose.x, pose.y);
    sample.z = 0.08;
    optimized.points.push_back(sample);
  }
  if (optimized.points.size() >= 2) {
    scenes.markers.push_back(optimized);
  } else {
    scenes.markers.push_back(erase("teb_trajectory", 0));
  }

  auto motion = lineStrip("motion", 0, 0.15f, 0.9f, 0.35f);
  motion.scale.x = 0.045;
  for (const auto & pose : route) {
    auto sample = place(pose.x, pose.y);
    sample.z = 0.2;
    motion.points.push_back(sample);
  }
  if (motion.points.size() >= 2) {
    scenes.markers.push_back(motion);
  } else {
    scenes.markers.push_back(erase("motion", 0));
  }

  constexpr int kArrowSlots = 10;
  if (route.size() >= 2) {
    for (int slot = 0; slot < kArrowSlots; ++slot) {
      const double progress = (static_cast<double>(slot) + 0.5) / static_cast<double>(kArrowSlots);
      const auto pose = poseAlong(route, progress);
      double vx = 0.0;
      double vy = 0.0;
      double vyaw = 0.0;
      mapToView(pose.x, pose.y, pose.yaw, shift, vx, vy, vyaw);
      scenes.markers.push_back(arrow(1 + slot, vx, vy, vyaw, 0.2f, 0.95f, 0.4f));
    }
  } else {
    for (int slot = 0; slot < kArrowSlots; ++slot) {
      scenes.markers.push_back(erase("motion", 1 + slot));
    }
  }

  constexpr int kObstacleSlots = 8;
  for (int slot = 0; slot < kObstacleSlots; ++slot) {
    if (slot < static_cast<int>(scene.obstacles.size())) {
      const auto & obstacle = scene.obstacles[static_cast<std::size_t>(slot)];
      double ox = 0.0;
      double oy = 0.0;
      double oyaw = 0.0;
      mapToView(obstacle.x, obstacle.y, 0.0, shift, ox, oy, oyaw);
      scenes.markers.push_back(sphere(
        "obstacles", slot, ox, oy, obstacle.radius, 0.95f, 0.18f, 0.12f));
      scenes.markers.push_back(label(
        "obstacles", 50 + slot, ox, oy + obstacle.radius + 0.15, 0.7,
        "障碍物 obstacle " + std::to_string(slot + 1), 1.0f, 0.45f, 0.35f, 0.32));
    } else {
      scenes.markers.push_back(erase("obstacles", slot));
      scenes.markers.push_back(erase("obstacles", 50 + slot));
    }
  }

  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  bool boxed = false;
  auto include = [&](double x, double y) {
    if (!boxed) {
      min_x = max_x = x;
      min_y = max_y = y;
      boxed = true;
      return;
    }
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
  };
  for (const auto & pose : scene.reference) {
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
    mapToView(pose.x, pose.y, pose.yaw, shift, vx, vy, vyaw);
    include(vx, vy);
  }
  for (const auto & pose : route) {
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
    mapToView(pose.x, pose.y, pose.yaw, shift, vx, vy, vyaw);
    include(vx, vy);
  }
  for (const auto & obstacle : scene.obstacles) {
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
    mapToView(obstacle.x, obstacle.y, 0.0, shift, vx, vy, vyaw);
    include(vx, vy);
  }
  if (!boxed) {
    include(0.0, 0.0);
  }

  if (!scene.reference.empty()) {
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
    mapToView(
      scene.reference.front().x, scene.reference.front().y, 0.0, shift, vx, vy, vyaw);
    scenes.markers.push_back(label(
      "reference", 1, vx, vy - 0.55, 0.55,
      "参考路径 reference", 0.85f, 0.85f, 0.85f, 0.36));
  } else {
    scenes.markers.push_back(erase("reference", 1));
  }
  if (!scene.optimized.empty()) {
    const auto & mid = scene.optimized[scene.optimized.size() / 2];
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
    mapToView(mid.x, mid.y, 0.0, shift, vx, vy, vyaw);
    scenes.markers.push_back(label(
      "teb_trajectory", 1, vx, vy + 0.7, 0.9,
      "TEB优化轨迹", 0.45f, 0.72f, 1.0f, 0.36));
  } else {
    scenes.markers.push_back(erase("teb_trajectory", 1));
  }
  if (!route.empty()) {
    double gx = 0.0;
    double gy = 0.0;
    double gyaw = 0.0;
    mapToView(route.back().x, route.back().y, 0.0, shift, gx, gy, gyaw);
    scenes.markers.push_back(label(
      "motion", 20, -0.2, 0.9, 0.7,
      "运动轨迹起点 start", 0.4f, 1.0f, 0.5f, 0.34));
    scenes.markers.push_back(label(
      "motion", 21, gx + 0.2, gy + 0.55, 0.7,
      "运动轨迹终点 goal", 0.4f, 1.0f, 0.5f, 0.34));
  } else {
    scenes.markers.push_back(erase("motion", 20));
    scenes.markers.push_back(erase("motion", 21));
  }

  const double legend_x = -6.5;
  const double legend_y = 6.0;
  (void)min_x;
  (void)max_x;
  (void)min_y;
  (void)max_y;
  scenes.markers.push_back(label(
    "main_status", 1, legend_x, legend_y, 1.6,
    "图例 legend\n灰线 reference 参考路径\n蓝线 TEB 优化轨迹\n绿线 motion 运动轨迹\n红球 obstacle 障碍物\n黄框 robot 参数轮廓",
    1.0f, 1.0f, 1.0f, 0.34));
  scenes.markers.push_back(label(
    "main_status", 0, legend_x, legend_y - 2.4, 1.2,
    flow.name + (flow.passed ? " 通过" : " 失败"),
    1.0f, 1.0f, 1.0f, 0.42));
  scenes.markers.push_back(label(
    "main_status", 2, legend_x, legend_y - 3.0, 0.9,
    flow.detail, 0.9f, 0.9f, 0.9f, 0.28));

  const double swatch_x0 = legend_x + 3.4;
  auto swatch = lineStrip("main_status", 10, 0.72f, 0.72f, 0.72f);
  swatch.points.push_back(point(swatch_x0, legend_y + 0.55, 0.4));
  swatch.points.push_back(point(swatch_x0 + 1.4, legend_y + 0.55, 0.4));
  scenes.markers.push_back(swatch);
  auto swatch_teb = lineStrip("main_status", 11, 0.15f, 0.55f, 1.0f);
  swatch_teb.scale.x = 0.09;
  swatch_teb.points.push_back(point(swatch_x0, legend_y + 0.05, 0.4));
  swatch_teb.points.push_back(point(swatch_x0 + 1.4, legend_y + 0.05, 0.4));
  scenes.markers.push_back(swatch_teb);
  auto swatch_motion = lineStrip("main_status", 12, 0.15f, 0.9f, 0.35f);
  swatch_motion.points.push_back(point(swatch_x0, legend_y - 0.45, 0.4));
  swatch_motion.points.push_back(point(swatch_x0 + 1.4, legend_y - 0.45, 0.4));
  scenes.markers.push_back(swatch_motion);
  scenes.markers.push_back(sphere("main_status", 13, swatch_x0 + 0.3, legend_y - 0.95, 0.22, 0.95f, 0.18f, 0.12f));
  scenes.markers.push_back(sphere("main_status", 14, swatch_x0 + 1.0, legend_y - 0.95, 0.18, 0.95f, 0.85f, 0.15f));
  return scenes;
}

visualization_msgs::msg::Marker footprintMarker(double x, double y, double yaw)
{
  std_msgs::msg::ColorRGBA color;
  color.r = 0.95f;
  color.g = 0.82f;
  color.b = 0.12f;
  color.a = 1.0f;
  std::vector<visualization_msgs::msg::Marker> shape;
  const auto & model = teb_local_planner::test_data::followPathConfig().robot_model;
  if (model) {
    model->visualizeRobot(teb_local_planner::PoseSE2(x, y, yaw), shape, color);
  }
  visualization_msgs::msg::Marker marker;
  if (!shape.empty()) {
    marker = shape.front();
  } else {
    marker = lineStrip("motion", 80, color.r, color.g, color.b);
  }
  marker.header.frame_id = "map";
  marker.ns = "motion";
  marker.id = 80;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.lifetime = rclcpp::Duration(0, 0);
  marker.scale.x = 0.08;
  marker.color = color;
  marker.pose.position.z = 0.0;
  for (auto & sample : marker.points) {
    sample.z = 0.3;
  }
  return marker;
}

visualization_msgs::msg::MarkerArray buildRobot(
  const std::vector<teb_local_planner::test_data::ScenePose> & route, double progress,
  const ViewShift & shift)
{
  visualization_msgs::msg::MarkerArray scenes;
  if (route.empty()) {
    scenes.markers.push_back(erase("motion", 80));
    scenes.markers.push_back(erase("motion", 81));
    scenes.markers.push_back(erase("motion", 82));
    return scenes;
  }
  const auto robot = poseAlong(route, progress);
  double vx = 0.0;
  double vy = 0.0;
  double vyaw = 0.0;
  mapToView(robot.x, robot.y, robot.yaw, shift, vx, vy, vyaw);
  scenes.markers.push_back(footprintMarker(vx, vy, vyaw));
  scenes.markers.push_back(erase("motion", 81));
  const int percent = static_cast<int>(std::lround(std::max(0.0, std::min(1.0, progress)) * 100.0));
  scenes.markers.push_back(label(
    "motion", 82, vx, vy - 1.1, 0.85,
    "robot 轮廓 " + std::to_string(percent) + "%",
    1.0f, 0.9f, 0.35f, 0.32));
  return scenes;
}

void stampAll(
  visualization_msgs::msg::MarkerArray & scenes, const rclcpp::Time & now)
{
  for (auto & marker : scenes.markers) {
    marker.header.stamp = now;
    marker.header.frame_id = "map";
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }
  auto node = std::make_shared<rclcpp::Node>("teb_main_flow_viz");
  auto publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/teb_main_flow/markers", rclcpp::QoS(1).transient_local());
  auto planner = std::make_unique<teb_local_planner::TebLocalPlannerROS>();
  teb_local_planner::TebSplitModuleTestAccess::setClock(*planner);
  teb_local_planner::TebSplitModuleTestAccess::loadFollowPathParams(*planner);
  const std::array<std::uint32_t, 4> seeds = {1u, 42u, 20261015u, 99u};
  auto cached = std::make_shared<std::array<std::vector<teb_local_planner::test_data::FlowItem>, 4>>();
  for (std::size_t i = 0; i < seeds.size(); ++i) {
    (*cached)[i] = teb_local_planner::test_data::runMainFlow(*planner, seeds[i]);
    for (const auto & flow_item : (*cached)[i]) {
      RCLCPP_INFO(
        node->get_logger(), "主流程逐项 [%s] %s | %s",
        flow_item.passed ? "通过" : "失败", flow_item.name.c_str(), flow_item.detail.c_str());
    }
  }

  struct Playback
  {
    std::size_t index{0};
    rclcpp::Time stage_start{0, 0, RCL_ROS_TIME};
    bool started{false};
    std::size_t shown_index{static_cast<std::size_t>(-1)};
  };
  auto playback = std::make_shared<Playback>();
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [publisher, node, cached, playback]() {
      const auto & items = (*cached)[0];
      if (items.empty()) {
        return;
      }
      const auto now = node->now();
      if (!playback->started) {
        playback->started = true;
        playback->stage_start = now;
      }
      const auto & current = items[playback->index % items.size()];
      const double current_length = std::max(pathLength(routeOf(current.scene)), 0.5);
      const double current_duration = std::max(8.0, current_length / 0.45);
      constexpr double kHoldSeconds = 2.0;
      double elapsed = (now - playback->stage_start).seconds();
      if (elapsed >= current_duration + kHoldSeconds) {
        playback->index = (playback->index + 1) % items.size();
        playback->stage_start = now;
        elapsed = 0.0;
      }
      const auto & flow = items[playback->index % items.size()];
      const auto & route = routeOf(flow.scene);
      const auto shift = shiftFor(route);
      const double duration = std::max(8.0, std::max(pathLength(route), 0.5) / 0.45);
      const double progress = std::max(0.0, std::min(1.0, elapsed / duration));
      if (playback->shown_index != playback->index) {
        playback->shown_index = playback->index;
        auto stamped = buildStatic(flow, shift);
        stampAll(stamped, now);
        publisher->publish(stamped);
      }
      auto robot = buildRobot(route, progress, shift);
      stampAll(robot, now);
      publisher->publish(robot);
    });
  (void)timer;
  RCLCPP_INFO(
    node->get_logger(),
    "主流程逐项可视化已发布: /teb_main_flow/markers （每项走完完整路径后切换）");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
