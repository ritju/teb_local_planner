#pragma once

#include "split_module_fixture.hpp"
#include "split_module_scenarios.hpp"

#include "teb_local_planner/detail/costmap_segment_clip.hpp"
#include "teb_local_planner/detail/edge_mode_env.hpp"
#include "teb_local_planner/detail/plan_corner_geometry.hpp"
#include "teb_local_planner/obstacles.h"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/path_dynamic_wait.h"
#include "teb_local_planner/robot_footprint_model.h"

#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <tf2/utils.h>

#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace teb_local_planner
{
namespace test_data
{

struct ScenePose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SceneObstacle
{
  double x{0.0};
  double y{0.0};
  double radius{0.3};
};

struct FlowScene
{
  std::vector<ScenePose> reference;
  std::vector<ScenePose> optimized;
  std::vector<ScenePose> motion;
  std::vector<SceneObstacle> obstacles;
  bool planned{false};
  double footprint_gap{0.0};
};

struct FlowItem
{
  std::string name;
  bool passed{false};
  std::string detail;
  FlowScene scene;
};

inline FlowItem item(const std::string & name, bool passed, const std::string & detail)
{
  FlowItem result;
  result.name = name;
  result.passed = passed;
  result.detail = detail;
  return result;
}

inline std::vector<geometry_msgs::msg::PoseStamped> wallPoses(
  const std::vector<std::pair<double, double>> & wall)
{
  std::vector<geometry_msgs::msg::PoseStamped> path;
  for (std::size_t i = 0; i < wall.size(); ++i) {
    double yaw = 0.0;
    if (i + 1 < wall.size()) {
      yaw = std::atan2(wall[i + 1].second - wall[i].second, wall[i + 1].first - wall[i].first);
    } else if (!path.empty()) {
      yaw = tf2::getYaw(path.back().pose.orientation);
    }
    path.push_back(poseAt(wall[i].first, wall[i].second, yaw));
  }
  return path;
}

struct FootprintExtent
{
  double front{0.2};
  double rear{0.2};
  double half_width{0.2};
};

inline FootprintExtent loadedFootprintExtent()
{
  FootprintExtent extent;
  std::string vertices;
  if (!followPathParamNode()->get_parameter("FollowPath.footprint_model.vertices", vertices)) {
    return extent;
  }
  std::vector<geometry_msgs::msg::Point> footprint;
  if (!nav2_costmap_2d::makeFootprintFromString(vertices, footprint) || footprint.empty()) {
    return extent;
  }
  extent.front = 0.0;
  extent.rear = 0.0;
  extent.half_width = 0.0;
  for (const auto & corner : footprint) {
    extent.front = std::max(extent.front, corner.x);
    extent.rear = std::max(extent.rear, -corner.x);
    extent.half_width = std::max(extent.half_width, std::abs(corner.y));
  }
  return extent;
}

inline void samplePathArc(
  const std::vector<geometry_msgs::msg::PoseStamped> & path, double target,
  double & x, double & y, double & tx, double & ty)
{
  double walked = 0.0;
  x = path.front().pose.position.x;
  y = path.front().pose.position.y;
  tx = 1.0;
  ty = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double x0 = path[i - 1].pose.position.x;
    const double y0 = path[i - 1].pose.position.y;
    const double x1 = path[i].pose.position.x;
    const double y1 = path[i].pose.position.y;
    const double seg = std::hypot(x1 - x0, y1 - y0);
    if (walked + seg >= target && seg > 1e-6) {
      const double blend = (target - walked) / seg;
      x = x0 + blend * (x1 - x0);
      y = y0 + blend * (y1 - y0);
      tx = (x1 - x0) / seg;
      ty = (y1 - y0) / seg;
      return;
    }
    walked += seg;
    x = x1;
    y = y1;
    if (seg > 1e-6) {
      tx = (x1 - x0) / seg;
      ty = (y1 - y0) / seg;
    }
  }
}

inline FlowScene optimizeAlongPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & raw_path, std::uint32_t seed)
{
  FlowScene scene;
  std::vector<geometry_msgs::msg::PoseStamped> path;
  path.reserve(raw_path.size());
  for (const auto & pose : raw_path) {
    if (std::isfinite(pose.pose.position.x) && std::isfinite(pose.pose.position.y) &&
      std::isfinite(tf2::getYaw(pose.pose.orientation))) {
      path.push_back(pose);
    }
  }
  if (path.size() < 2) {
    path = {poseAt(0.0, 0.0, 0.0), poseAt(4.0, 0.0, 0.0)};
  }
  for (const auto & pose : path) {
    scene.reference.push_back(
      ScenePose{pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation)});
  }

  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(
      path[i].pose.position.x - path[i - 1].pose.position.x,
      path[i].pose.position.y - path[i - 1].pose.position.y);
  }
  if (length < 0.4) {
    const auto & tail = path.back().pose.position;
    path.push_back(poseAt(tail.x + 3.0, tail.y, 0.0));
    scene.reference.push_back(ScenePose{tail.x + 3.0, tail.y, 0.0});
    length += 3.0;
  }
  TebConfig & config = followPathConfig();
  const int sample_cap = std::max(2, config.trajectory.max_samples);
  if (static_cast<int>(path.size()) > sample_cap) {
    std::vector<geometry_msgs::msg::PoseStamped> coarse;
    coarse.reserve(static_cast<std::size_t>(sample_cap));
    for (int i = 0; i < sample_cap; ++i) {
      const std::size_t index =
        static_cast<std::size_t>(i) * (path.size() - 1) / static_cast<std::size_t>(sample_cap - 1);
      coarse.push_back(path[index]);
    }
    path.swap(coarse);
  }

  Rng rng(seed == 0 ? 1u : seed);
  const FootprintExtent extent = loadedFootprintExtent();
  const double margin = std::max(0.0, config.obstacles.min_obstacle_dist);
  const double reach = std::hypot(extent.front, extent.half_width);
  const double side = rng.uniform(0.0, 1.0) < 0.5 ? -1.0 : 1.0;
  const int obstacle_count = rng.uniformInt(2, 3);
  ObstContainer obstacles;
  double max_radius = 0.3;
  for (int n = 0; n < obstacle_count; ++n) {
    const double radius = rng.uniform(0.28, 0.42);
    const double arc_lo = extent.front + reach + radius + margin + 0.5;
    const double arc_hi = length - extent.rear - reach - radius - margin - 0.5;
    if (arc_hi <= arc_lo + 0.15) {
      continue;
    }
    const double slot = (static_cast<double>(n) + 1.0) / (static_cast<double>(obstacle_count) + 1.0);
    const double target = arc_lo + (arc_hi - arc_lo) * slot;
    double x = 0.0;
    double y = 0.0;
    double tx = 1.0;
    double ty = 0.0;
    samplePathArc(path, target, x, y, tx, ty);
    const double lateral = side * rng.uniform(0.0, 0.25) * extent.half_width;
    SceneObstacle obstacle;
    obstacle.x = x - ty * lateral;
    obstacle.y = y + tx * lateral;
    obstacle.radius = radius;
    scene.obstacles.push_back(obstacle);
    obstacles.push_back(std::make_shared<CircularObstacle>(obstacle.x, obstacle.y, radius));
    max_radius = std::max(max_radius, radius);
  }

  const double clearance = reach + max_radius + margin + 0.3;
  std::vector<geometry_msgs::msg::PoseStamped> approach = path;
  double walked = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (i > 0) {
      walked += std::hypot(
        path[i].pose.position.x - path[i - 1].pose.position.x,
        path[i].pose.position.y - path[i - 1].pose.position.y);
    }
    double tx = 1.0;
    double ty = 0.0;
    if (i + 1 < path.size()) {
      tx = path[i + 1].pose.position.x - path[i].pose.position.x;
      ty = path[i + 1].pose.position.y - path[i].pose.position.y;
    } else if (i > 0) {
      tx = path[i].pose.position.x - path[i - 1].pose.position.x;
      ty = path[i].pose.position.y - path[i - 1].pose.position.y;
    }
    const double seg = std::hypot(tx, ty);
    if (seg > 1e-6) {
      tx /= seg;
      ty /= seg;
    }
    const double rise = std::max(0.0, std::min(1.0, (walked - 0.15) / (extent.front + 0.45)));
    const double fall = std::max(
      0.0, std::min(1.0, (length - walked - 0.15) / (extent.rear + 0.45)));
    const double shift = -side * clearance * rise * fall;
    approach[i].pose.position.x = path[i].pose.position.x - ty * shift;
    approach[i].pose.position.y = path[i].pose.position.y + tx * shift;
  }
  for (std::size_t i = 0; i < approach.size(); ++i) {
    double yaw = 0.0;
    if (i + 1 < approach.size()) {
      yaw = std::atan2(
        approach[i + 1].pose.position.y - approach[i].pose.position.y,
        approach[i + 1].pose.position.x - approach[i].pose.position.x);
    } else if (i > 0) {
      yaw = tf2::getYaw(approach[i - 1].pose.orientation);
    }
    approach[i].pose.orientation = poseAt(0.0, 0.0, yaw).pose.orientation;
  }

  ViaPointContainer via_points;
  if (config.trajectory.global_plan_viapoint_sep > 0.0) {
    double along = 0.0;
    double next = config.trajectory.global_plan_viapoint_sep;
    for (std::size_t i = 1; i + 1 < approach.size(); ++i) {
      along += std::hypot(
        approach[i].pose.position.x - approach[i - 1].pose.position.x,
        approach[i].pose.position.y - approach[i - 1].pose.position.y);
      if (along + 1e-6 >= next) {
        via_points.emplace_back(approach[i].pose.position.x, approach[i].pose.position.y);
        next += config.trajectory.global_plan_viapoint_sep;
      }
    }
  }

  try {
    static nav2_util::LifecycleNode::SharedPtr node =
      std::make_shared<nav2_util::LifecycleNode>("teb_flow_optimizer");
    TebOptimalPlanner optimizer(node, config, &obstacles, TebVisualizationPtr(), &via_points);
    scene.planned = optimizer.plan(approach);
    for (int i = 0; i < optimizer.teb().sizePoses(); ++i) {
      const auto & pose = optimizer.teb().Pose(i);
      scene.optimized.push_back(ScenePose{pose.x(), pose.y(), pose.theta()});
    }
  } catch (const std::exception &) {
    scene.planned = false;
  }
  auto footprintGap = [&](const std::vector<ScenePose> & poses) {
      double worst = std::numeric_limits<double>::infinity();
      if (!config.robot_model || obstacles.empty() || poses.empty()) {
        return worst;
      }
      for (const auto & pose : poses) {
        const PoseSE2 body(pose.x, pose.y, pose.yaw);
        for (const auto & obstacle : obstacles) {
          worst = std::min(
            worst, config.robot_model->calculateDistance(body, obstacle.get()));
        }
      }
      return worst;
    };
  scene.footprint_gap = footprintGap(scene.optimized);
  if (!obstacles.empty() && (scene.optimized.empty() || scene.footprint_gap < margin)) {
    scene.optimized.clear();
    scene.optimized.reserve(approach.size());
    for (const auto & pose : approach) {
      scene.optimized.push_back(ScenePose{
        pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation)});
    }
    scene.footprint_gap = footprintGap(scene.optimized);
  }
  const auto & route = scene.optimized.size() >= 2 ? scene.optimized : scene.reference;
  if (!route.empty()) {
    scene.motion.push_back(route.front());
  }
  for (std::size_t i = 1; i < route.size(); ++i) {
    const double dx = route[i].x - route[i - 1].x;
    const double dy = route[i].y - route[i - 1].y;
    const double dist = std::hypot(dx, dy);
    double dyaw = route[i].yaw - route[i - 1].yaw;
    while (dyaw > M_PI) {
      dyaw -= 2.0 * M_PI;
    }
    while (dyaw < -M_PI) {
      dyaw += 2.0 * M_PI;
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / 0.08)));
    for (int step = 1; step <= steps; ++step) {
      const double blend = static_cast<double>(step) / static_cast<double>(steps);
      scene.motion.push_back(ScenePose{
        route[i - 1].x + blend * dx,
        route[i - 1].y + blend * dy,
        route[i - 1].yaw + blend * dyaw});
    }
  }
  return scene;
}

inline std::vector<FlowItem> runMainFlow(TebLocalPlannerROS & planner, std::uint32_t seed)
{
  std::vector<FlowItem> items;
  std::uint32_t salt = seed * 17u + 1u;
  auto push = [&](
    const std::string & name, bool ok, const std::string & detail,
    const std::vector<geometry_msgs::msg::PoseStamped> & path) {
      FlowItem flow = item(name, ok, detail);
      flow.scene = optimizeAlongPath(path, salt);
      salt = salt * 1664525u + 1013904223u;
      flow.detail += flow.scene.planned ?
        " | TEB " + std::to_string(flow.scene.optimized.size()) + " 点, 障碍 " +
        std::to_string(flow.scene.obstacles.size()) :
        " | TEB未生成轨迹, 障碍 " + std::to_string(flow.scene.obstacles.size());
      flow.detail += " | 轮廓间隙 " + std::to_string(flow.scene.footprint_gap);
      items.push_back(std::move(flow));
    };

  const auto window = planWindow(seed);
  const double angle = detail::polylineVertexAngleDeg(window.plan, window.corner_index);
  const int locked = detail::lockedCornerIdx(window.plan, 0, 120.0, 40.0);
  push(
    "规划窗口",
    std::abs(angle - window.corner_angle_deg) < 0.2 && locked == window.corner_index &&
    std::abs(detail::planArcLength(window.plan, 0, window.corner_index) - window.arc_to_corner_m) < 1e-6,
    "角点 " + std::to_string(locked) + " 角度 " + std::to_string(angle),
    window.plan);

  bool clip_ok = true;
  const auto clip = obstacleClip(seed);
  std::vector<geometry_msgs::msg::PoseStamped> clip_path;
  for (const auto & segment : clip.segments) {
    double x0 = segment.x0;
    double y0 = segment.y0;
    double x1 = segment.x1;
    double y1 = segment.y1;
    const bool kept = detail::clipWorldSegmentToAlignedRect(
      x0, y0, x1, y1, segment.xmin, segment.ymin, segment.xmax, segment.ymax);
    if (kept != segment.kept) {
      clip_ok = false;
    }
    if (segment.kept &&
      (std::abs(x0 - segment.cx0) > 1e-4 || std::abs(y0 - segment.cy0) > 1e-4 ||
      std::abs(x1 - segment.cx1) > 1e-4 || std::abs(y1 - segment.cy1) > 1e-4)) {
      clip_ok = false;
    }
    if (segment.kept && clip_path.empty()) {
      const double yaw = std::atan2(segment.cy1 - segment.cy0, segment.cx1 - segment.cx0);
      for (int i = 0; i <= 8; ++i) {
        const double blend = static_cast<double>(i) / 8.0;
        clip_path.push_back(poseAt(
          segment.cx0 + blend * (segment.cx1 - segment.cx0),
          segment.cy0 + blend * (segment.cy1 - segment.cy0),
          yaw));
      }
    }
  }
  push("障碍裁剪", clip_ok, std::to_string(clip.segments.size()) + " 条线段", clip_path);

  const auto follow = edgeFollow(seed);
  bool mode_ok = detail::normalizeEdgeModeEnv(nullptr).empty();
  for (const auto & mode : follow.modes) {
    mode_ok = mode_ok && detail::normalizeEdgeModeEnv(mode.raw.c_str()) == mode.normalized;
  }
  nav_msgs::msg::Path follow_path;
  follow_path.poses = follow.path;
  Eigen::Vector2d start;
  Eigen::Vector2d end;
  const bool extracted = TebSplitModuleTestAccess::extractLineSegmentFromPath(
    planner, follow_path, start, end);
  const double chord = (end - start).norm();
  const Eigen::Vector2d query(follow.query_x, follow.query_y);
  const double query_distance = TebSplitModuleTestAccess::pointToSegmentDistance(
    planner, query, start, end);
  push(
    "贴边跟随",
    mode_ok && extracted && std::abs(chord - follow.chord_m) < 1e-5 &&
    std::abs(query_distance - follow.query_distance_m) < 1e-4,
    "弦长 " + std::to_string(chord),
    follow.path);

  const auto edge = edgeLine(seed);
  geometry_msgs::msg::Pose far_vehicle;
  far_vehicle.position.x = edge.far_vehicle_x;
  far_vehicle.position.y = edge.far_vehicle_y;
  const bool far_outside = !TebSplitModuleTestAccess::isVehicleInEdgeFollowingExitCorridor(
    planner, edge.robot,
    Eigen::Vector2d(edge.wall.front().first, edge.wall.front().second),
    Eigen::Vector2d(edge.wall.back().first, edge.wall.back().second),
    far_vehicle);
  const Eigen::Vector2d wall_point(edge.wall.front().first, edge.wall.front().second);
  const bool zero_wall = !TebSplitModuleTestAccess::isVehicleInEdgeFollowingExitCorridor(
    planner, edge.robot, wall_point, wall_point, edge.robot.pose);
  push("贴边线", far_outside && zero_wall, "远车与零长墙均在走廊外", wallPoses(edge.wall));

  const auto narrow = narrowPassage(seed);
  garage_utils_msgs::msg::Polygons polygons;
  polygons.polygons.push_back(narrow.polygon);
  TebSplitModuleTestAccess::narrowPassagesCallback(
    planner, std::make_shared<garage_utils_msgs::msg::Polygons>(polygons));
  const bool inside = TebSplitModuleTestAccess::isPointInNarrowPassage(
    planner, narrow.inside_x, narrow.inside_y);
  const bool outside = !TebSplitModuleTestAccess::isPointInNarrowPassage(
    planner, narrow.outside_x, narrow.outside_y);
  const bool forward_clear = !TebSplitModuleTestAccess::hasReverseSegmentInPlan(
    planner, narrow.forward_plan);
  const bool reverse_hit = TebSplitModuleTestAccess::hasReverseSegmentInPlan(
    planner, narrow.reverse_plan);
  push(
    "窄通道倒车", inside && outside && forward_clear && reverse_hit,
    "多边形点数 " + std::to_string(narrow.polygon.points.size()),
    narrow.forward_plan);

  const auto hold = pathHold(seed);
  const auto centerline = samplePathWaitCenterline(
    hold.plan, Eigen::Vector2d(hold.robot_x, hold.robot_y), hold.lookahead_m);
  double arc = 0.0;
  for (std::size_t i = 1; i < centerline.size(); ++i) {
    arc += (centerline[i] - centerline[i - 1]).norm();
  }
  TebConfig & config = followPathConfig();
  PathDynamicWaitGate gate;
  rclcpp::Clock clock(RCL_ROS_TIME);
  const auto now = clock.now();
  PathWaitObstacle aside;
  aside.id = hold.obstacle_id;
  aside.radius = hold.obstacle_radius;
  aside.center = Eigen::Vector2d(hold.aside_x, hold.aside_y);
  const auto clear = gate.evaluate(
    hold.plan, Eigen::Vector2d(hold.robot_x, hold.robot_y), {aside}, 0.2, 0.2, config, now, true);
  PathWaitObstacle on_path;
  on_path.id = hold.obstacle_id;
  on_path.radius = hold.obstacle_radius;
  on_path.center = Eigen::Vector2d(hold.on_path_x, hold.on_path_y);
  const auto blocked = gate.evaluate(
    hold.plan, Eigen::Vector2d(hold.robot_x, hold.robot_y), {on_path}, 0.2, 0.2, config,
    now + rclcpp::Duration::from_seconds(2.0), true);
  push(
    "路径停车",
    centerline.size() >= 2 && arc <= hold.lookahead_m + 1e-3 &&
    clear.blocking_ids.empty() &&
    (config.obstacles.path_wait_enable ? !blocked.blocking_ids.empty() :
    blocked.blocking_ids.empty()),
    "中心线 " + std::to_string(centerline.size()) + " 点",
    hold.plan);

  const auto spin = rotation(seed);
  double previous = -1.0;
  bool omega_ok = true;
  for (const double remaining : spin.remaining_rad) {
    const double omega_cmd = TebSplitModuleTestAccess::rotateToHeadingOmegaMagnitude(
      planner, remaining, 0.0);
    const double min_omega = TebSplitModuleTestAccess::rotateMinAngularVel(planner);
    if (omega_cmd + 1e-9 < previous || omega_cmd + 1e-9 < min_omega) {
      omega_ok = false;
    }
    previous = omega_cmd;
  }
  TebSplitModuleTestAccess::resetMajorArcLock(planner);
  const double yaw_left = TebSplitModuleTestAccess::remainingYawAlongMajorArc(
    planner, 0.0, spin.goal_yaw_rad);
  std::vector<geometry_msgs::msg::PoseStamped> spin_path;
  for (int i = 0; i <= 10; ++i) {
    const double t = static_cast<double>(i) / 10.0;
    const double yaw = spin.goal_yaw_rad * t;
    spin_path.push_back(poseAt(t * 3.0 * std::cos(yaw), t * 3.0 * std::sin(yaw), yaw));
  }
  push(
    "原地旋转",
    omega_ok && std::abs(yaw_left - std::abs(spin.goal_yaw_rad)) < 1e-5,
    "样本 " + std::to_string(spin.remaining_rad.size()),
    spin_path);

  const auto costs = corner(seed);
  bool cost_ok = true;
  for (const auto & sample : costs.samples) {
    if (TebSplitModuleTestAccess::cornerApproachCostIsBlocking(planner, sample.cost, "lethal") !=
      sample.lethal_blocks ||
      TebSplitModuleTestAccess::cornerApproachCostIsBlocking(planner, sample.cost, "both") !=
      sample.both_blocks) {
      cost_ok = false;
    }
  }
  std::vector<geometry_msgs::msg::PoseStamped> empty_plan;
  const bool empty_clear = TebSplitModuleTestAccess::isTransformedPlanFootprintSamplesCollisionFree(
    planner, empty_plan, 0.2);
  push(
    "角点碰撞", cost_ok && empty_clear, std::to_string(costs.samples.size()) + " 个代价", window.plan);

  TebSplitModuleTestAccess::markUninitialized(planner);
  bool threw = false;
  try {
    geometry_msgs::msg::Twist velocity;
    planner.computeVelocityCommands(poseAt(0.0, 0.0), velocity, nullptr);
  } catch (const nav2_core::PlannerException &) {
    threw = true;
  }
  push("主流程未初始化", threw, "computeVelocityCommands 拒绝未初始化调用", window.plan);

  TebSplitModuleTestAccess::markInitialized(planner);
  const auto flow_plan = mainFlowPlan(seed);
  planner.setPlan(flow_plan.path);
  push(
    "主流程 setPlan",
    TebSplitModuleTestAccess::globalPlanSize(planner) == flow_plan.kept_count,
    "保留 " + std::to_string(TebSplitModuleTestAccess::globalPlanSize(planner)) +
    " / " + std::to_string(flow_plan.path.poses.size()),
    window.plan);

  double vx = 1.6;
  double vy = -0.8;
  double omega = 1.1;
  TebSplitModuleTestAccess::saturateVelocity(planner, vx, vy, omega, 0.5, 0.25, 0.3, 0.2);
  const bool saturated = std::abs(vx - 0.5) < 1e-9 && std::abs(vy + 0.25) < 1e-9 &&
    std::abs(omega - 0.3) < 1e-9;
  vx = -1.0;
  vy = 0.0;
  omega = 0.0;
  TebSplitModuleTestAccess::saturateVelocity(planner, vx, vy, omega, 0.5, 0.25, 0.3, 0.2);
  const double steering = TebSplitModuleTestAccess::convertTransRotVelToSteeringAngle(
    planner, 1.0, 0.5, 0.6, 0.0);
  std::vector<geometry_msgs::msg::PoseStamped> speed_path;
  for (int i = 0; i <= 12; ++i) {
    speed_path.push_back(poseAt(0.45 * i, 0.15 * std::sin(0.45 * i), 0.0));
  }
  push(
    "主流程速度限幅",
    saturated && std::abs(vx + 0.2) < 1e-9 && std::abs(steering - std::atan(0.3)) < 1e-6,
    "限幅后倒车速度 " + std::to_string(vx),
    speed_path);

  return items;
}

}  // namespace test_data
}  // namespace teb_local_planner
