/*********************************************************************
 * Front 2D lidar protrusion check for edge-following exit
 *********************************************************************/
#include "teb_local_planner/teb_local_planner_ros.h"

#include <teb_local_planner/wall_protrusion_check.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <cmath>
#include <limits>

namespace teb_local_planner
{

void TebLocalPlannerROS::frontScanCallback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  if (!cfg_->wall_line.enable_front_scan_protrusion_check) {
    return;
  }
  std::lock_guard<std::mutex> lock(front_scan_mutex_);
  latest_front_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
}

void TebLocalPlannerROS::checkFrontScanProtrusionExit(
  const EffWallFrame& wall,
  const Eigen::Vector2d& robot_position,
  const std::string& map_frame)
{
  if (!cfg_->wall_line.enable_front_scan_protrusion_check) {
    return;
  }

  sensor_msgs::msg::LaserScan::SharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(front_scan_mutex_);
    scan = latest_front_scan_;
  }
  if (!scan || scan->ranges.empty()) {
    return;
  }

  const std::string& scan_frame = scan->header.frame_id;
  if (scan_frame.empty() || !tf_ || !costmap_ros_) {
    return;
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
    return;
  }

  const double min_range = cfg_->wall_line.front_scan_min_range_base_footprint;
  const double max_range = cfg_->wall_line.front_scan_max_range_base_footprint;
  if (max_range < min_range) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 5000,
      "front_scan protrusion: max_range (%.2f) < min_range (%.2f), skip",
      max_range, min_range);
    return;
  }
  const double min_r_sq = min_range * min_range;
  const double max_r_sq = max_range * max_range;

  WallProtrusionHit best_hit;
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

    const WallProtrusionHit point_hit = checkWallProtrusionPoint(
      p_map.head<2>(),
      p_base.head<2>(),
      true,
      wall,
      robot_position,
      *cfg_);
    if (point_hit.exit_trigger &&
      (!best_hit.exit_trigger || point_hit.exit_best_pen > best_hit.exit_best_pen))
    {
      best_hit = point_hit;
    }
  }

  if (best_hit.exit_trigger) {
    recordProtrusionDetection(
      best_hit.exit_best_point, best_hit.exit_best_pen, "front_scan");
  }
}

void TebLocalPlannerROS::recordProtrusionDetection(
  const Eigen::Vector2d& best_point,
  double best_pen,
  const char * log_context)
{
  const rclcpp::Time now = clock_->now();
  protrusion_detection_timestamps_.push_back(now);

  const rclcpp::Duration window_duration = rclcpp::Duration::from_seconds(
    cfg_->wall_line.obstacle_protrusion_confirm_window);
  while (!protrusion_detection_timestamps_.empty() &&
    (now - protrusion_detection_timestamps_.front()) > window_duration)
  {
    protrusion_detection_timestamps_.erase(protrusion_detection_timestamps_.begin());
  }

  RCLCPP_INFO(
    logger_, "[%s] protrusion_detection_timestamps_.size(): %ld",
    log_context, protrusion_detection_timestamps_.size());

  if (static_cast<int>(protrusion_detection_timestamps_.size()) <
    cfg_->wall_line.obstacle_protrusion_min_confirm_frames)
  {
    return;
  }

  bool is_duplicate = false;
  for (auto& existing : protruding_obstacles_) {
    if ((existing.position - best_point).norm() < 0.5) {
      existing.detection_time = now;
      is_duplicate = true;
      RCLCPP_INFO(
        logger_,
        "[%s] Protruding obstacle already exists at (%.2f, %.2f), updating timestamp.",
        log_context, existing.position.x(), existing.position.y());
      break;
    }
  }

  if (!is_duplicate) {
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
      "[%s] Protruding obstacle confirmed at (%.2f, %.2f), penetration=%.2f.",
      log_context, best_point.x(), best_point.y(), best_pen);
  }
  protrusion_detection_timestamps_.clear();
}

}  // namespace teb_local_planner
