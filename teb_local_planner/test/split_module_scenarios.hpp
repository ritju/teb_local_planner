#pragma once

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace teb_local_planner
{
namespace test_data
{

inline geometry_msgs::msg::PoseStamped poseAt(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

class Rng
{
public:
  explicit Rng(std::uint32_t seed)
  : state_(seed == 0 ? 1u : seed)
  {
  }

  double uniform(double low, double high)
  {
    state_ = state_ * 1664525u + 1013904223u;
    const double unit = static_cast<double>(state_ >> 8) / static_cast<double>(1u << 24);
    return low + (high - low) * unit;
  }

  int uniformInt(int low, int high)
  {
    return low + static_cast<int>(uniform(0.0, static_cast<double>(high - low + 1)));
  }

private:
  std::uint32_t state_;
};

inline constexpr std::uint32_t kDefaultSeed = 20261015u;

struct PlanWindowData
{
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  int corner_index{0};
  double corner_angle_deg{90.0};
  double arc_to_corner_m{0.0};
  double arc_query_m{0.0};
  int arc_query_idx{0};
  double step_m{0.7};
};

inline int lastIndexAtArc(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan, double max_arc)
{
  double acc = 0.0;
  int idx = 0;
  for (int j = 0; j < static_cast<int>(plan.size()); ++j) {
    idx = j;
    if (j > 0) {
      acc += std::hypot(
        plan[static_cast<std::size_t>(j)].pose.position.x -
        plan[static_cast<std::size_t>(j - 1)].pose.position.x,
        plan[static_cast<std::size_t>(j)].pose.position.y -
        plan[static_cast<std::size_t>(j - 1)].pose.position.y);
    }
    if (acc + 1e-9 >= max_arc) {
      break;
    }
  }
  return idx;
}

inline PlanWindowData planWindow(std::uint32_t seed = kDefaultSeed)
{
  Rng rng(seed);
  PlanWindowData data;
  data.step_m = rng.uniform(0.55, 0.95);
  double x = rng.uniform(-4.0, 4.0);
  double y = rng.uniform(-4.0, 4.0);
  double heading = rng.uniform(-M_PI, M_PI);
  const double gentle = (rng.uniform(0.0, 1.0) < 0.5 ? -1.0 : 1.0) * rng.uniform(18.0, 28.0) * M_PI / 180.0;
  const double sharp_deg = rng.uniform(78.0, 102.0);
  const double sharp = (rng.uniform(0.0, 1.0) < 0.5 ? -1.0 : 1.0) * sharp_deg * M_PI / 180.0;

  auto advance = [&](int count) {
    for (int i = 0; i < count; ++i) {
      x += data.step_m * std::cos(heading);
      y += data.step_m * std::sin(heading);
      data.plan.push_back(poseAt(x, y, heading));
    }
  };

  data.plan.push_back(poseAt(x, y, heading));
  advance(4);
  heading += gentle;
  advance(3);
  data.corner_index = static_cast<int>(data.plan.size()) - 1;
  data.corner_angle_deg = 180.0 - sharp_deg;
  heading += sharp;
  advance(4);
  data.arc_to_corner_m = data.corner_index * data.step_m;
  data.arc_query_m = 2.5 * data.step_m;
  data.arc_query_idx = lastIndexAtArc(data.plan, data.arc_query_m);
  return data;
}

struct ClipSegment
{
  double x0{0.0};
  double y0{0.0};
  double x1{0.0};
  double y1{0.0};
  double xmin{0.0};
  double ymin{0.0};
  double xmax{1.0};
  double ymax{1.0};
  bool kept{true};
  double cx0{0.0};
  double cy0{0.0};
  double cx1{0.0};
  double cy1{0.0};
};

struct ClipData
{
  std::vector<ClipSegment> segments;
};

inline ClipData obstacleClip(std::uint32_t seed = kDefaultSeed)
{
  Rng rng(seed ^ 0x51u);
  ClipData data;
  const double xmin = rng.uniform(-2.0, 1.0);
  const double ymin = rng.uniform(-2.0, 1.0);
  const double width = rng.uniform(1.2, 3.4);
  const double height = rng.uniform(1.2, 3.0);
  const double xmax = xmin + width;
  const double ymax = ymin + height;
  const double y_mid = 0.5 * (ymin + ymax);
  const double x_mid = 0.5 * (xmin + xmax);
  const double overhang = rng.uniform(0.4, 1.6);

  ClipSegment horizontal;
  horizontal.xmin = xmin;
  horizontal.ymin = ymin;
  horizontal.xmax = xmax;
  horizontal.ymax = ymax;
  horizontal.y0 = y_mid;
  horizontal.y1 = y_mid;
  horizontal.x0 = xmin - overhang;
  horizontal.x1 = xmax + overhang * 0.7;
  horizontal.kept = true;
  horizontal.cx0 = xmin;
  horizontal.cy0 = y_mid;
  horizontal.cx1 = xmax;
  horizontal.cy1 = y_mid;
  data.segments.push_back(horizontal);

  ClipSegment vertical = horizontal;
  vertical.x0 = x_mid;
  vertical.x1 = x_mid;
  vertical.y0 = ymin - overhang;
  vertical.y1 = ymax + overhang;
  vertical.cx0 = x_mid;
  vertical.cy0 = ymin;
  vertical.cx1 = x_mid;
  vertical.cy1 = ymax;
  data.segments.push_back(vertical);

  ClipSegment diagonal = horizontal;
  const double pad = rng.uniform(0.3, 1.1);
  diagonal.x0 = xmin - pad;
  diagonal.y0 = ymin - pad * height / width;
  diagonal.x1 = xmax + pad;
  diagonal.y1 = ymax + pad * height / width;
  diagonal.cx0 = xmin;
  diagonal.cy0 = ymin;
  diagonal.cx1 = xmax;
  diagonal.cy1 = ymax;
  data.segments.push_back(diagonal);

  ClipSegment outside = horizontal;
  outside.kept = false;
  outside.x0 = xmax + 2.0;
  outside.y0 = ymax + 2.0;
  outside.x1 = xmax + 4.0 + rng.uniform(0.2, 1.0);
  outside.y1 = ymax + 3.0;
  data.segments.push_back(outside);
  return data;
}

struct EdgeLineData
{
  geometry_msgs::msg::PoseStamped robot;
  std::vector<std::pair<double, double>> wall;
  double far_vehicle_x{0.0};
  double far_vehicle_y{0.0};
};

inline EdgeLineData edgeLine(std::uint32_t seed = kDefaultSeed)
{
  Rng rng(seed ^ 0xA3u);
  EdgeLineData data;
  const double yaw = rng.uniform(-M_PI, M_PI);
  const double length = rng.uniform(6.0, 14.0);
  const int pieces = rng.uniformInt(5, 9);
  double x = rng.uniform(-1.0, 1.0);
  double y = rng.uniform(-1.0, 1.0);
  data.robot = poseAt(x - 0.8 * std::sin(yaw), y + 0.8 * std::cos(yaw), yaw);
  for (int i = 0; i <= pieces; ++i) {
    const double along = length * static_cast<double>(i) / static_cast<double>(pieces);
    const double wobble = 0.08 * std::sin(along * 1.7 + rng.uniform(0.0, 1.0));
    data.wall.emplace_back(
      x + along * std::cos(yaw) - wobble * std::sin(yaw),
      y + along * std::sin(yaw) + wobble * std::cos(yaw));
  }
  data.far_vehicle_x = x - rng.uniform(18.0, 35.0) * std::sin(yaw);
  data.far_vehicle_y = y + rng.uniform(18.0, 35.0) * std::cos(yaw);
  return data;
}

struct EdgeModeSample
{
  std::string raw;
  std::string normalized;
};

struct EdgeFollowData
{
  std::vector<geometry_msgs::msg::PoseStamped> path;
  double chord_m{1.0};
  double query_x{0.0};
  double query_y{0.0};
  double query_distance_m{1.0};
  double parallel_distance_m{0.8};
  std::vector<EdgeModeSample> modes;
};

inline EdgeFollowData edgeFollow(std::uint32_t seed = kDefaultSeed)
{
  Rng rng(seed ^ 0x17u);
  EdgeFollowData data;
  double x = rng.uniform(-2.0, 2.0);
  double y = rng.uniform(-2.0, 2.0);
  double heading = rng.uniform(-M_PI, M_PI);
  const int count = rng.uniformInt(8, 14);
  for (int i = 0; i < count; ++i) {
    data.path.push_back(poseAt(x, y, heading));
    heading += rng.uniform(-0.35, 0.35);
    const double step = rng.uniform(0.45, 1.15);
    x += step * std::cos(heading);
    y += step * std::sin(heading);
  }
  {
    const auto & tail = data.path.back().pose.position;
    const double extend = 3.0;
    data.path.push_back(poseAt(
      tail.x + extend * std::cos(heading),
      tail.y + extend * std::sin(heading),
      heading));
  }
  const auto & a = data.path.front().pose.position;
  const auto & b = data.path.back().pose.position;
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  data.chord_m = std::hypot(dx, dy);
  const double nx = -dy / data.chord_m;
  const double ny = dx / data.chord_m;
  data.query_distance_m = rng.uniform(0.6, 1.8);
  data.query_x = 0.5 * (a.x + b.x) + nx * data.query_distance_m;
  data.query_y = 0.5 * (a.y + b.y) + ny * data.query_distance_m;
  data.parallel_distance_m = rng.uniform(0.4, 1.5);
  data.modes = {
    {" Wall + Reference ", "wall+reference"},
    {"\tCURB + wall\t", "curb+wall"},
    {"  Both+WALL ", "both+wall"},
  };
  return data;
}

struct NarrowData
{
  geometry_msgs::msg::Polygon polygon;
  double inside_x{0.0};
  double inside_y{0.0};
  double outside_x{0.0};
  double outside_y{0.0};
  std::vector<geometry_msgs::msg::PoseStamped> forward_plan;
  std::vector<geometry_msgs::msg::PoseStamped> reverse_plan;
  double arc_query_m{1.2};
  int arc_query_idx{0};
};

inline NarrowData narrowPassage(std::uint32_t seed = kDefaultSeed)
{
  Rng rng(seed ^ 0xC1u);
  NarrowData data;
  const double cx = rng.uniform(-1.0, 1.0);
  const double cy = rng.uniform(-1.0, 1.0);
  const int count = rng.uniformInt(6, 9);
  const double offset = rng.uniform(0.0, 0.5);
  const double step = 2.0 * M_PI / static_cast<double>(count);
  std::vector<double> angles;
  for (int i = 0; i < count; ++i) {
    angles.push_back(offset + step * static_cast<double>(i) + rng.uniform(-0.08, 0.08));
  }
  std::sort(angles.begin(), angles.end());
  double max_radius = 0.0;
  double sx = 0.0;
  double sy = 0.0;
  for (double angle : angles) {
    const double radius = rng.uniform(1.4, 2.6);
    max_radius = std::max(max_radius, radius);
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(cx + radius * std::cos(angle));
    point.y = static_cast<float>(cy + radius * std::sin(angle));
    data.polygon.points.push_back(point);
    sx += point.x;
    sy += point.y;
  }
  (void)sx;
  (void)sy;
  data.inside_x = cx;
  data.inside_y = cy;
  const double out_angle = rng.uniform(0.0, 2.0 * M_PI);
  data.outside_x = cx + (max_radius + 3.0) * std::cos(out_angle);
  data.outside_y = cy + (max_radius + 3.0) * std::sin(out_angle);

  const double yaw = rng.uniform(-1.0, 1.0);
  const int samples = 12;
  const double spacing = rng.uniform(0.28, 0.45);
  for (int i = 0; i < samples; ++i) {
    data.forward_plan.push_back(poseAt(
      i * spacing * std::cos(yaw), i * spacing * std::sin(yaw), yaw));
    data.reverse_plan.push_back(poseAt(
      (samples - 1 - i) * spacing * std::cos(yaw),
      (samples - 1 - i) * spacing * std::sin(yaw),
      yaw));
  }
  data.arc_query_m = 3.5 * spacing;
  data.arc_query_idx = lastIndexAtArc(data.reverse_plan, data.arc_query_m);
  return data;
}

struct PathHoldData
{
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  double robot_x{0.0};
  double robot_y{0.0};
  double lookahead_m{3.0};
  double obstacle_speed_m_s{0.8};
  int obstacle_id{7};
  double obstacle_radius{0.32};
  double aside_x{0.0};
  double aside_y{0.0};
  double on_path_x{0.0};
  double on_path_y{0.0};
};

inline PathHoldData pathHold(std::uint32_t seed = kDefaultSeed)
{
  Rng rng(seed ^ 0x55u);
  PathHoldData data;
  data.lookahead_m = rng.uniform(2.4, 3.6);
  data.obstacle_speed_m_s = rng.uniform(0.6, 1.4);
  data.obstacle_id = rng.uniformInt(3, 40);
  data.obstacle_radius = rng.uniform(0.25, 0.45);
  const double amplitude = rng.uniform(0.6, 1.4);
  const double wavelength = rng.uniform(4.0, 7.0);
  const int count = 28;
  const double dx = 0.32;
  for (int i = 0; i < count; ++i) {
    const double x = i * dx;
    const double y = amplitude * std::sin(2.0 * M_PI * x / wavelength);
    const double dy = amplitude * std::cos(2.0 * M_PI * x / wavelength) * (2.0 * M_PI / wavelength) * dx;
    data.plan.push_back(poseAt(x, y, std::atan2(dy, dx)));
  }
  data.robot_x = data.plan.front().pose.position.x + 0.05;
  data.robot_y = data.plan.front().pose.position.y;
  const auto & on_path = data.plan[5].pose.position;
  data.on_path_x = on_path.x;
  data.on_path_y = on_path.y;
  data.aside_x = on_path.x;
  data.aside_y = on_path.y + 5.5;
  return data;
}

inline std::pair<double, double> movingObstacleXY(const PathHoldData & data, double time_s)
{
  const double arc_step = 0.32;
  const double length = arc_step * static_cast<double>(data.plan.size() - 1);
  const double period = std::max(4.0, 2.0 * length / data.obstacle_speed_m_s);
  double phase = std::fmod(std::max(0.0, time_s), period) / period;
  if (phase > 0.5) {
    phase = 1.0 - phase;
  }
  phase *= 2.0;
  const double distance = phase * length;
  const double index = distance / arc_step;
  const int i0 = std::min(static_cast<int>(index), static_cast<int>(data.plan.size()) - 2);
  const double blend = index - static_cast<double>(i0);
  const auto & a = data.plan[static_cast<std::size_t>(i0)].pose.position;
  const auto & b = data.plan[static_cast<std::size_t>(i0 + 1)].pose.position;
  return {
    a.x + blend * (b.x - a.x),
    a.y + blend * (b.y - a.y)};
}

struct RotationData
{
  std::vector<double> remaining_rad;
  double zero_remaining_rad{0.0};
  double goal_yaw_rad{1.2};
  double min_omega{0.05};
};

inline RotationData rotation(std::uint32_t seed = kDefaultSeed)
{
  Rng rng(seed ^ 0x77u);
  RotationData data;
  for (int i = 0; i < 6; ++i) {
    data.remaining_rad.push_back(rng.uniform(0.15, 2.4));
  }
  std::sort(data.remaining_rad.begin(), data.remaining_rad.end());
  data.goal_yaw_rad = rng.uniform(-2.6, 2.6);
  return data;
}

struct CostmapData
{
  int service_wait_ms{10};
  int join_wait_ms{1000};
  int task_count{4};
};

inline CostmapData costmap(std::uint32_t seed = kDefaultSeed)
{
  Rng rng(seed ^ 0x99u);
  CostmapData data;
  data.task_count = rng.uniformInt(3, 6);
  data.service_wait_ms = rng.uniformInt(8, 25);
  return data;
}

struct CornerSample
{
  unsigned char cost{0};
  bool lethal_blocks{false};
  bool both_blocks{false};
};

struct CornerData
{
  std::vector<CornerSample> samples;
};

inline CornerData corner(std::uint32_t seed = kDefaultSeed)
{
  Rng rng(seed ^ 0xB4u);
  CornerData data;
  std::vector<int> costs = {0, 50, 128, 200, 252, 253, 254, 255};
  for (int i = 0; i < 6; ++i) {
    costs.push_back(rng.uniformInt(0, 255));
  }
  for (int cost : costs) {
    CornerSample sample;
    sample.cost = static_cast<unsigned char>(cost);
    sample.lethal_blocks = cost == 254;
    sample.both_blocks = cost == 253 || cost == 254;
    data.samples.push_back(sample);
  }
  return data;
}

struct MainFlowPlanData
{
  nav_msgs::msg::Path path;
  std::size_t kept_count{0};
};

inline MainFlowPlanData mainFlowPlan(std::uint32_t seed = kDefaultSeed)
{
  const auto window = planWindow(seed);
  MainFlowPlanData data;
  data.path.header.frame_id = "map";
  data.path.poses = window.plan;
  data.kept_count = window.plan.size();
  geometry_msgs::msg::PoseStamped duplicate = window.plan.back();
  duplicate.pose.position.x += 1e-8;
  duplicate.pose.position.y += 1e-8;
  data.path.poses.push_back(duplicate);
  geometry_msgs::msg::PoseStamped invalid = poseAt(
    window.plan.back().pose.position.x + 1.0,
    window.plan.back().pose.position.y + 0.4,
    0.2);
  invalid.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
  data.path.poses.push_back(invalid);
  return data;
}

}  // namespace test_data
}  // namespace teb_local_planner
