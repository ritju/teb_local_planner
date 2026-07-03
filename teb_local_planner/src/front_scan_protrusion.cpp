/*********************************************************************
 * Front 2D lidar protrusion check for edge-following exit
 *********************************************************************/
#include "teb_local_planner/teb_local_planner_ros.h"

#include <teb_local_planner/wall_protrusion_check.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <algorithm>
#include <limits>

namespace teb_local_planner
{
namespace
{

void mergeWallProtrusionHit(
  WallProtrusionHit& best_hit,
  const WallProtrusionHit& candidate_hit)
{
  if (candidate_hit.exit_trigger &&
    (!best_hit.exit_trigger || candidate_hit.exit_best_pen > best_hit.exit_best_pen))
  {
    best_hit = candidate_hit;
  }
}

std::vector<Eigen::Vector2d> pathPosesToMapPoints(
  const std::vector<geometry_msgs::msg::PoseStamped>& path_poses)
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(path_poses.size());
  for (const auto& pose : path_poses) {
    points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  return points;
}

visualization_msgs::msg::Marker makeCorridorLineStripMarker(
  const WallMonitorCorridor& corridor,
  int marker_id,
  const std::string& ns,
  float r, float g, float b,
  const std::string& frame_id,
  const rclcpp::Time& stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.05;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.85;
  marker.pose.orientation.w = 1.0;

  if (!corridor.valid) {
    return marker;
  }

  Eigen::Vector2d corners[4];
  wallMonitorCorridorCorners(corridor, corners);
  for (int i = 0; i < 4; ++i) {
    geometry_msgs::msg::Point p;
    p.x = corners[i].x();
    p.y = corners[i].y();
    p.z = 0.05;
    marker.points.push_back(p);
  }
  geometry_msgs::msg::Point p0;
  p0.x = corners[0].x();
  p0.y = corners[0].y();
  p0.z = 0.05;
  marker.points.push_back(p0);
  return marker;
}

}  // namespace

void TebLocalPlannerROS::frontScanCallback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  if (!cfg_->wall_line.enable_front_scan_protrusion_check) {
    return;
  }
  std::lock_guard<std::mutex> lock(front_scan_mutex_);
  latest_front_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
}

WallProtrusionHit TebLocalPlannerROS::scanFrontScanProtrusionHit(
  const EffWallFrame& wall,
  const Eigen::Vector2d& robot_position,
  double robot_yaw,
  const std::string& map_frame,
  const WallMonitorCorridor* monitor_corridor)
{
  WallProtrusionHit best_hit;
  if (!cfg_->wall_line.enable_front_scan_protrusion_check) {
    return best_hit;
  }

  sensor_msgs::msg::LaserScan::SharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(front_scan_mutex_);
    scan = latest_front_scan_;
  }
  if (!scan || scan->ranges.empty()) {
    return best_hit;
  }

  const std::string& scan_frame = scan->header.frame_id;
  if (scan_frame.empty() || !tf_ || !costmap_ros_) {
    return best_hit;
  }

  const std::string& base_frame = costmap_ros_->getBaseFrameID();

  Eigen::Affine3d tf_base_from_scan = Eigen::Affine3d::Identity();
  Eigen::Affine3d tf_map_from_scan = Eigen::Affine3d::Identity();
  try {
    const geometry_msgs::msg::TransformStamped tf_base =
      tf_->lookupTransform(base_frame, scan_frame, tf2::TimePointZero);
    tf_base_from_scan = tf2::transformToEigen(tf_base);

    const geometry_msgs::msg::TransformStamped tf_map =
      tf_->lookupTransform(map_frame, scan_frame, tf2::TimePointZero);
    tf_map_from_scan = tf2::transformToEigen(tf_map);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_DEBUG_THROTTLE(
      logger_, *(clock_), 2000,
      "front_scan protrusion: TF failed (%s <- %s): %s",
      base_frame.c_str(), scan_frame.c_str(), ex.what());
    return best_hit;
  }

  const double min_range = cfg_->wall_line.front_scan_min_range_base_footprint;
  const double max_range = cfg_->wall_line.front_scan_max_range_base_footprint;
  if (max_range < min_range) {
    return best_hit;
  }
  const double min_r_sq = min_range * min_range;
  const double max_r_sq = max_range * max_range;

  const std::size_t beam_count = scan->ranges.size();
  for (std::size_t i = 0; i < beam_count; ++i) {
    const float range = scan->ranges[i];
    if (!std::isfinite(range) ||
      range < scan->range_min || range > scan->range_max)
    {
      continue;
    }

    const double angle = scan->angle_min +
      static_cast<double>(i) * scan->angle_increment;
    const Eigen::Vector3d p_scan(
      static_cast<double>(range) * std::cos(angle),
      static_cast<double>(range) * std::sin(angle),
      0.0);

    const Eigen::Vector3d p_base = tf_base_from_scan * p_scan;
    const double r_sq = p_base.x() * p_base.x() + p_base.y() * p_base.y();
    if (r_sq < min_r_sq || r_sq > max_r_sq) {
      continue;
    }

    const Eigen::Vector3d p_map = tf_map_from_scan * p_scan;
    mergeWallProtrusionHit(
      best_hit,
      checkWallProtrusionPoint(
        p_map.head<2>(),
        wall,
        robot_position,
        robot_yaw,
        *cfg_,
        monitor_corridor));
  }

  return best_hit;
}

WallProtrusionHit TebLocalPlannerROS::scanCostmapConverterProtrusionHit(
  const EffWallFrame& wall,
  const Eigen::Vector2d& robot_position,
  double robot_yaw,
  const WallMonitorCorridor* monitor_corridor)
{
  WallProtrusionHit best_hit;
  if (!costmap_converter_) {
    return best_hit;
  }

  const costmap_converter::ObstacleArrayConstPtr obstacles =
    costmap_converter_->getObstacles();
  if (!obstacles) {
    return best_hit;
  }

  for (const auto& obstacle : obstacles->obstacles) {
    const geometry_msgs::msg::Polygon& polygon = obstacle.polygon;
    if (polygon.points.size() <= 2) {
      continue;
    }

    for (const auto& point : polygon.points) {
      const Eigen::Vector2d vertex(point.x, point.y);
      mergeWallProtrusionHit(
        best_hit,
        checkWallProtrusionPoint(
          vertex, wall, robot_position, robot_yaw, *cfg_, monitor_corridor));
    }
  }

  return best_hit;
}

void TebLocalPlannerROS::updateEdgeMonitorCorridors(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const std::vector<geometry_msgs::msg::PoseStamped>& path_poses)
{
  protrusion_monitor_corridor_ = WallMonitorCorridor{};
  vehicle_monitor_corridor_ = WallMonitorCorridor{};

  if (wall_line_points_.size() < 2) {
    return;
  }

  const Eigen::Vector2d robot_position(
    robot_pose.pose.position.x, robot_pose.pose.position.y);
  EffWallFrame wall;
  if (!buildEffWallFrameFromSegment(
      wall_line_points_[0], wall_line_points_[1], robot_position, *cfg_, wall))
  {
    return;
  }

  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const std::vector<Eigen::Vector2d> path_points = pathPosesToMapPoints(path_poses);
  const auto& wl = cfg_->wall_line;

  protrusion_monitor_corridor_ = buildWallMonitorCorridor(
    wall, robot_position, robot_yaw, path_points,
    wl.wall_line_obstacle_along_wall_rear_margin,
    wl.wall_line_protrusion_exit_forward_max,
    0.0,
    wl.wall_line_protrusion_exit_lateral_max,
    *cfg_);

  vehicle_monitor_corridor_ = buildWallMonitorCorridor(
    wall, robot_position, robot_yaw, path_points,
    wl.vehicle_exit_corridor_rear_m,
    wl.vehicle_exit_corridor_front_m,
    wl.vehicle_exit_corridor_wall_inner_m,
    wl.vehicle_exit_corridor_wall_robot_side_m,
    *cfg_);
}

void TebLocalPlannerROS::publishMonitorCorridorMarkers(const std::string& frame_id)
{
  if (!monitor_corridor_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const rclcpp::Time stamp = clock_->now();

  marker_array.markers.push_back(makeCorridorLineStripMarker(
    protrusion_monitor_corridor_, 0, "protrusion_monitor_corridor",
    0.1f, 0.9f, 1.0f, frame_id, stamp));
  marker_array.markers.push_back(makeCorridorLineStripMarker(
    vehicle_monitor_corridor_, 1, "vehicle_monitor_corridor",
    1.0f, 0.4f, 0.9f, frame_id, stamp));

  monitor_corridor_marker_pub_->publish(marker_array);
  publishProtrudingObstacleMarkers(frame_id);
}

void TebLocalPlannerROS::publishProtrudingObstacleMarkers(const std::string& frame_id)
{
  if (!protruding_obstacle_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const rclcpp::Time stamp = clock_->now();
  const int current_count = static_cast<int>(protruding_obstacles_.size());

  if (current_count == 0) {
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = frame_id;
    delete_all.header.stamp = stamp;
    delete_all.ns = "protruding_obstacle_points";
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);
    last_published_protruding_obstacle_marker_count_ = 0;
    protruding_obstacle_marker_pub_->publish(marker_array);
    return;
  }

  for (int i = 0; i < current_count; ++i) {
    const ProtrudingObstacle& obst = protruding_obstacles_[static_cast<std::size_t>(i)];
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "protruding_obstacle_points";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = obst.position.x();
    marker.pose.position.y = obst.position.y();
    marker.pose.position.z = 0.08;
    marker.pose.orientation.w = 1.0;

    const double sphere_diameter =
      std::clamp(0.2 + 0.4 * obst.influence_radius, 0.2, 0.8);
    marker.scale.x = sphere_diameter;
    marker.scale.y = sphere_diameter;
    marker.scale.z = sphere_diameter;
    marker.color.r = 1.0f;
    marker.color.g = 0.45f;
    marker.color.b = 0.0f;
    marker.color.a = 0.9f;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker_array.markers.push_back(marker);
  }

  for (int i = current_count; i < last_published_protruding_obstacle_marker_count_; ++i) {
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = stamp;
    delete_marker.ns = "protruding_obstacle_points";
    delete_marker.id = i;
    delete_marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(delete_marker);
  }

  last_published_protruding_obstacle_marker_count_ = current_count;
  protruding_obstacle_marker_pub_->publish(marker_array);
}

void TebLocalPlannerROS::clearProtrudingObstacleMarkers(const std::string& frame_id)
{
  if (!protruding_obstacle_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker delete_all;
  delete_all.header.frame_id = frame_id;
  delete_all.header.stamp = clock_->now();
  delete_all.ns = "protruding_obstacle_points";
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);
  protruding_obstacle_marker_pub_->publish(marker_array);
  last_published_protruding_obstacle_marker_count_ = 0;
}

void TebLocalPlannerROS::clearEdgeMonitorCorridors(const std::string& frame_id)
{
  protrusion_monitor_corridor_ = WallMonitorCorridor{};
  vehicle_monitor_corridor_ = WallMonitorCorridor{};

  if (!monitor_corridor_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker delete_all;
  delete_all.header.frame_id = frame_id;
  delete_all.header.stamp = clock_->now();
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);
  monitor_corridor_marker_pub_->publish(marker_array);
}

bool TebLocalPlannerROS::protrusionBlocksEdgeEntryForWallSegment(
  const Eigen::Vector2d& wall_segment_start,
  const Eigen::Vector2d& wall_segment_end,
  const Eigen::Vector2d& robot_position)
{
  EffWallFrame wall;
  if (!buildEffWallFrameFromSegment(
      wall_segment_start, wall_segment_end, robot_position, *cfg_, wall))
  {
    return false;
  }

  pruneExpiredProtrudingObstacles();

  const double robot_yaw = robot_pose_.theta();

  WallProtrusionHit best_hit =
    scanCostmapConverterProtrusionHit(wall, robot_position, robot_yaw, nullptr);

  std::string map_frame = cfg_->map_frame;
  if (costmap_converter_) {
    const costmap_converter::ObstacleArrayConstPtr obstacles =
      costmap_converter_->getObstacles();
    if (obstacles && !obstacles->header.frame_id.empty()) {
      map_frame = obstacles->header.frame_id;
    }
  }
  mergeWallProtrusionHit(
    best_hit,
    scanFrontScanProtrusionHit(
      wall, robot_position, robot_yaw, map_frame, nullptr));

  if (!best_hit.exit_trigger) {
    return false;
  }

  recordProtrusionDetection(
    best_hit.exit_best_point, best_hit.exit_best_pen, "edge_entry_probe");
  RCLCPP_WARN_THROTTLE(
    logger_, *(clock_), 2000,
    "Edge following: 候选墙线附近检测到凸出障碍 (%.2f, %.2f), pen=%.2f, 拒绝进入贴边",
    best_hit.exit_best_point.x(), best_hit.exit_best_point.y(), best_hit.exit_best_pen);
  return true;
}

void TebLocalPlannerROS::checkFrontScanProtrusionExit(
  const EffWallFrame& wall,
  const Eigen::Vector2d& robot_position,
  const std::string& map_frame)
{
  if (!cfg_->wall_line.enable_front_scan_protrusion_check) {
    return;
  }

  pruneExpiredProtrudingObstacles();

  const double robot_yaw = robot_pose_.theta();
  const WallMonitorCorridor* corridor = protrusion_monitor_corridor_.valid ?
    &protrusion_monitor_corridor_ : nullptr;

  const WallProtrusionHit best_hit =
    scanFrontScanProtrusionHit(
      wall, robot_position, robot_yaw, map_frame, corridor);
  if (best_hit.exit_trigger) {
    recordProtrusionDetection(
      best_hit.exit_best_point, best_hit.exit_best_pen, "front_scan");
  }
}

void TebLocalPlannerROS::pruneExpiredProtrudingObstacles()
{
  const rclcpp::Time now = clock_->now();
  const double timeout = cfg_->wall_line.obstacle_protrusion_timeout;
  const std::size_t size_before = protruding_obstacles_.size();
  auto it = protruding_obstacles_.begin();
  while (it != protruding_obstacles_.end()) {
    if ((now - it->detection_time).seconds() > timeout) {
      it = protruding_obstacles_.erase(it);
    } else {
      ++it;
    }
  }
  if (protruding_obstacles_.size() != size_before) {
    const std::string frame_id = costmap_ros_ ?
      costmap_ros_->getGlobalFrameID() : cfg_->map_frame;
    publishProtrudingObstacleMarkers(frame_id);
  }
}

bool TebLocalPlannerROS::hasProtrudingObstacleInMonitorCorridor(
  const geometry_msgs::msg::PoseStamped& robot_pose)
{
  pruneExpiredProtrudingObstacles();

  if (!protrusion_monitor_corridor_.valid) {
    return !protruding_obstacles_.empty();
  }

  bool in_corridor = false;
  auto it = protruding_obstacles_.begin();
  while (it != protruding_obstacles_.end()) {
    if (isPointInWallMonitorCorridor(it->position, protrusion_monitor_corridor_)) {
      in_corridor = true;
      ++it;
    } else {
      RCLCPP_DEBUG(
        logger_,
        "Protruding obstacle erased at (%.2f, %.2f), left monitor corridor",
        it->position.x(), it->position.y());
      it = protruding_obstacles_.erase(it);
    }
  }
  const std::string frame_id = costmap_ros_ ?
    costmap_ros_->getGlobalFrameID() : robot_pose.header.frame_id;
  publishProtrudingObstacleMarkers(frame_id);
  (void)robot_pose;
  return in_corridor;
}

void TebLocalPlannerROS::recordProtrusionDetection(
  const Eigen::Vector2d& best_point,
  double best_pen,
  const char * log_context)
{
  pruneExpiredProtrudingObstacles();
  const rclcpp::Time now = clock_->now();

  for (auto& existing : protruding_obstacles_) {
    if ((existing.position - best_point).norm() < 0.5) {
      existing.detection_time = now;
      existing.influence_radius = std::max(existing.influence_radius, best_pen);
      RCLCPP_INFO(
        logger_,
        "[%s] Protruding obstacle already exists at (%.2f, %.2f), updating timestamp.",
        log_context, existing.position.x(), existing.position.y());
      const std::string frame_id = costmap_ros_ ?
        costmap_ros_->getGlobalFrameID() : cfg_->map_frame;
      publishProtrudingObstacleMarkers(frame_id);
      return;
    }
  }

  while (static_cast<int>(protruding_obstacles_.size()) >=
    cfg_->wall_line.obstacle_protrusion_max_stored)
  {
    protruding_obstacles_.erase(protruding_obstacles_.begin());
  }

  ProtrudingObstacle protruding_obst;
  protruding_obst.position = best_point;
  protruding_obst.influence_radius = best_pen;
  protruding_obst.detection_time = now;
  protruding_obstacles_.push_back(protruding_obst);
  RCLCPP_INFO(
    logger_,
    "[%s] Protruding obstacle recorded at (%.2f, %.2f), penetration=%.2f.",
    log_context, best_point.x(), best_point.y(), best_pen);
  const std::string frame_id = costmap_ros_ ?
    costmap_ros_->getGlobalFrameID() : cfg_->map_frame;
  publishProtrudingObstacleMarkers(frame_id);
}

}  // namespace teb_local_planner
