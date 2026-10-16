/*********************************************************************
 * teb_edge_following.cpp
 * Split from teb_local_planner_ros.cpp. Behavior unchanged.
 *********************************************************************/
#include "teb_local_planner/teb_local_planner_ros.h"

#include <boost/algorithm/string.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <rclcpp/logging.hpp>
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include <future>
#include <thread>
#include <system_error>
#include <chrono>
#include <functional>
#include <string>
#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav_2d_utils/tf_help.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include "dwb_core/exceptions.hpp"
#include "dwb_critics/obstacle_footprint.hpp"
#include "dwb_critics/line_iterator.hpp"
#include "nav2_util/robot_utils.hpp"
#include "teb_local_planner/misc.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstring>
#include <exception>
#include <limits>

#include "teb_local_planner/detail/edge_mode_env.hpp"
#include "teb_local_planner/detail/plan_corner_geometry.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace teb_local_planner
{
using detail::normalizeEdgeModeEnv;

namespace {
rclcpp::Clock & edgeFollowingLogClock()
{
  
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

double pointToSegmentDistance2d(
  const Eigen::Vector2d& query_point,
  const Eigen::Vector2d& segment_start,
  const Eigen::Vector2d& segment_end)
{
  
  // 退化线段直接退化为点距；非退化则做投影并裁剪到 [0,1]。
  const Eigen::Vector2d segment_vector = segment_end - segment_start;
  const double segment_length_squared = segment_vector.squaredNorm();
  if (segment_length_squared < 1e-12) {
    return (query_point - segment_start).norm();
  }
  const double projected_parameter =
    (query_point - segment_start).dot(segment_vector) / segment_length_squared;
  const double clamped_parameter = std::max(0.0, std::min(1.0, projected_parameter));
  const Eigen::Vector2d projected_point = segment_start + clamped_parameter * segment_vector;
  return (query_point - projected_point).norm();
}

double segmentToSegmentDistance2d(
  const Eigen::Vector2d& first_segment_start,
  const Eigen::Vector2d& first_segment_end,
  const Eigen::Vector2d& second_segment_start,
  const Eigen::Vector2d& second_segment_end)
{
  
  // 采用“端点到对侧线段最短距离”的近似，满足当前贴边几何筛选需求。
  const double first_to_second_start =
    pointToSegmentDistance2d(first_segment_start, second_segment_start, second_segment_end);
  const double first_to_second_end =
    pointToSegmentDistance2d(first_segment_end, second_segment_start, second_segment_end);
  const double second_to_first_start =
    pointToSegmentDistance2d(second_segment_start, first_segment_start, first_segment_end);
  const double second_to_first_end =
    pointToSegmentDistance2d(second_segment_end, first_segment_start, first_segment_end);
  return std::min(
    std::min(first_to_second_start, first_to_second_end),
    std::min(second_to_first_start, second_to_first_end));
}

/**
 * @brief 在折线上找距 query 最近的相邻段，若弧长不足 target_length 则向两端扩展，
 *        返回扩展窗首尾顶点作为局部贴边线段。
 */
bool extractNearestExpandedPolylineSegment(
  const std::vector<Eigen::Vector2d>& polyline,
  const Eigen::Vector2d& query_point,
  const double target_length,
  Eigen::Vector2d& segment_start,
  Eigen::Vector2d& segment_end)
{
  if (polyline.size() < 2) {
    return false;
  }

  std::size_t best_segment_index = 0;
  double best_distance = std::numeric_limits<double>::max();
  for (std::size_t segment_index = 0; segment_index + 1 < polyline.size(); ++segment_index) {
    const double distance_to_segment = pointToSegmentDistance2d(
      query_point, polyline[segment_index], polyline[segment_index + 1]);
    if (distance_to_segment < best_distance) {
      best_distance = distance_to_segment;
      best_segment_index = segment_index;
    }
  }

  std::size_t left_index = best_segment_index;
  std::size_t right_index = best_segment_index + 1;
  double window_arc_length = (polyline[right_index] - polyline[left_index]).norm();
  const double expand_target =
    (target_length > 1e-6) ? target_length : window_arc_length;

  while (window_arc_length + 1e-9 < expand_target) {
    bool expanded = false;
    if (left_index > 0) {
      --left_index;
      window_arc_length += (polyline[left_index + 1] - polyline[left_index]).norm();
      expanded = true;
      if (window_arc_length + 1e-9 >= expand_target) {
        break;
      }
    }
    if (right_index + 1 < polyline.size()) {
      ++right_index;
      window_arc_length += (polyline[right_index] - polyline[right_index - 1]).norm();
      expanded = true;
    }
    if (!expanded) {
      break;
    }
  }

  segment_start = polyline[left_index];
  segment_end = polyline[right_index];
  return (segment_end - segment_start).norm() > 1e-6;
}

double pointToPolylineDistance2d(
  const Eigen::Vector2d& query_point,
  const std::vector<Eigen::Vector2d>& polyline)
{
  if (polyline.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (polyline.size() == 1) {
    return (query_point - polyline.front()).norm();
  }
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t segment_index = 0; segment_index + 1 < polyline.size(); ++segment_index) {
    best_distance = std::min(
      best_distance,
      pointToSegmentDistance2d(
        query_point, polyline[segment_index], polyline[segment_index + 1]));
  }
  return best_distance;
}

/**
 * @brief 点的正交投影是否落在折线某段上（未裁剪参数 t∈[0,1]）。
 *        若是，perp_distance 为窗内最小垂距。
 */
bool pointProjectsOntoPolyline(
  const Eigen::Vector2d& query_point,
  const std::vector<Eigen::Vector2d>& polyline,
  double& perp_distance)
{
  perp_distance = std::numeric_limits<double>::infinity();
  if (polyline.size() < 2) {
    return false;
  }
  bool projects_onto_polyline = false;
  for (std::size_t segment_index = 0; segment_index + 1 < polyline.size(); ++segment_index) {
    const Eigen::Vector2d& segment_start = polyline[segment_index];
    const Eigen::Vector2d& segment_end = polyline[segment_index + 1];
    const Eigen::Vector2d segment_vector = segment_end - segment_start;
    const double segment_length_squared = segment_vector.squaredNorm();
    if (segment_length_squared < 1e-12) {
      continue;
    }
    const double projected_parameter =
      (query_point - segment_start).dot(segment_vector) / segment_length_squared;
    if (projected_parameter < 0.0 || projected_parameter > 1.0) {
      continue;
    }
    const Eigen::Vector2d projected_point =
      segment_start + projected_parameter * segment_vector;
    const double distance = (query_point - projected_point).norm();
    projects_onto_polyline = true;
    perp_distance = std::min(perp_distance, distance);
  }
  return projects_onto_polyline;
}

/**
 * @brief 路径与折线重叠统计：仅正交投影落在折线上的点计入垂距；
 *        overlap_arc_length 为最长连续重叠段的路径弧长。
 * @return 路径非空且折线至少两点时为 true；无重叠时 max/mean 为 inf、overlap 为 0。
 */
bool computePathPolylineOverlapStats(
  const std::vector<geometry_msgs::msg::PoseStamped>& path_poses,
  const std::vector<Eigen::Vector2d>& polyline,
  double& max_perp_distance,
  double& mean_perp_distance,
  double& overlap_arc_length)
{
  max_perp_distance = 0.0;
  mean_perp_distance = 0.0;
  overlap_arc_length = 0.0;
  if (path_poses.empty() || polyline.size() < 2) {
    return false;
  }

  bool previous_point_overlaps = false;
  Eigen::Vector2d previous_point = Eigen::Vector2d::Zero();
  double current_overlap_run = 0.0;
  double longest_overlap_run = 0.0;
  double sum_perp_distance = 0.0;
  std::size_t overlap_point_count = 0;

  for (const auto& pose_stamped : path_poses) {
    const Eigen::Vector2d point(
      pose_stamped.pose.position.x, pose_stamped.pose.position.y);
    double perp_distance = 0.0;
    const bool point_overlaps = pointProjectsOntoPolyline(point, polyline, perp_distance);
    if (point_overlaps) {
      max_perp_distance = std::max(max_perp_distance, perp_distance);
      sum_perp_distance += perp_distance;
      ++overlap_point_count;
      if (previous_point_overlaps) {
        current_overlap_run += (point - previous_point).norm();
      }
    } else {
      longest_overlap_run = std::max(longest_overlap_run, current_overlap_run);
      current_overlap_run = 0.0;
    }
    previous_point_overlaps = point_overlaps;
    previous_point = point;
  }
  longest_overlap_run = std::max(longest_overlap_run, current_overlap_run);
  overlap_arc_length = longest_overlap_run;

  if (overlap_point_count == 0) {
    max_perp_distance = std::numeric_limits<double>::infinity();
    mean_perp_distance = std::numeric_limits<double>::infinity();
  } else {
    mean_perp_distance =
      sum_perp_distance / static_cast<double>(overlap_point_count);
  }
  return true;
}

bool pathPointsWithinPolylineDistance(
  const std::vector<geometry_msgs::msg::PoseStamped>& path_poses,
  const std::vector<Eigen::Vector2d>& polyline,
  const double max_allowed_distance,
  const double min_overlap_length,
  double& max_distance,
  double& mean_distance,
  double& overlap_arc_length)
{
  if (!computePathPolylineOverlapStats(
        path_poses, polyline, max_distance, mean_distance, overlap_arc_length))
  {
    return false;
  }
  return overlap_arc_length + 1e-9 >= min_overlap_length &&
         max_distance <= max_allowed_distance;
}

bool segmentsCloseForFusion(
  const Eigen::Vector2d& fusion_primary_segment_start,
  const Eigen::Vector2d& fusion_primary_segment_end,
  const Eigen::Vector2d& reference_segment_start,
  const Eigen::Vector2d& reference_segment_end,
  const double maximum_angle_degrees,
  const double maximum_segment_separation_meters)
{
  
  // 融合判定分三步：长度有效 -> 方向夹角 -> 段间距离。
  const Eigen::Vector2d primary_direction = fusion_primary_segment_end - fusion_primary_segment_start;
  const Eigen::Vector2d reference_direction = reference_segment_end - reference_segment_start;
  const double primary_segment_length = primary_direction.norm();
  const double reference_segment_length = reference_direction.norm();
  if (primary_segment_length < 1e-6 || reference_segment_length < 1e-6) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teb_local_planner"), edgeFollowingLogClock(), 2000,
                         "[segmentsCloseForFusion] Edge following:" 
                         "贴边参考线和墙线或路沿特征长度太短, 墙线或路沿特征长度: %.2f, 贴边参考线长度: %.2f", 
                         primary_segment_length, reference_segment_length);
    return false;
  }
  const double cosine_alignment =
    std::fabs(primary_direction.dot(reference_direction) / (primary_segment_length * reference_segment_length));
  const double angle_degrees =
    std::acos(std::min(1.0, cosine_alignment)) * 180.0 / M_PI;
  if (angle_degrees > maximum_angle_degrees) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teb_local_planner"), edgeFollowingLogClock(), 2000,
                         "[segmentsCloseForFusion] Edge following:"
                         "贴边参考线和墙线或路沿特征角度太大: %.2f, 最大允许角度: %.2f", 
                         angle_degrees, maximum_angle_degrees);
    return false;
  }
  const double segment_separation_meters = segmentToSegmentDistance2d(
    fusion_primary_segment_start,
    fusion_primary_segment_end,
    reference_segment_start,
    reference_segment_end);
  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("teb_local_planner"), edgeFollowingLogClock(), 2000,
                       "[segmentsCloseForFusion] Edge following:"
                       "贴边参考线与墙线或路沿特征距离: %.2f, 最大允许距离: %.2f",
                       segment_separation_meters,
                       maximum_segment_separation_meters);
  if (segment_separation_meters > maximum_segment_separation_meters) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teb_local_planner"), edgeFollowingLogClock(), 2000,
                         "[segmentsCloseForFusion] Edge following:"
                         "贴边参考线和墙线或路沿特征距离太大: %.2f, 最大允许距离: %.2f",
                         segment_separation_meters, maximum_segment_separation_meters);
    return false;
  }
  return true;
}

}  // namespace

void TebLocalPlannerROS::clearActiveReferencePair()
{
  
  active_pair_index_ = -1;
  has_active_reference_pair_ = false;
  active_mission_segment_ = nav_msgs::msg::Path();
  active_reference_path_ = nav_msgs::msg::Path();
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "未找到匹配的贴边参考路径，清空贴边参考路径");
}

void TebLocalPlannerROS::refreshActiveReferencePair()
{
  
  if (paired_mission_and_reference_path_cache_.paths.size() < 2 ||
    removed_plan_cache_.poses.empty())
  {
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "贴边参考路径缓存或多点路径为空，清空贴边参考路径");
    clearActiveReferencePair();
    return;
  }

  const geometry_msgs::msg::PoseStamped & front_pose = removed_plan_cache_.poses.front();
  const Eigen::Vector2d front_xy(front_pose.pose.position.x, front_pose.pose.position.y);
  int best_pair_index = -1;
  const double match_threshold_m = cfg_->wall_line.mission_segment_front_pose_match_threshold_m;

  for (std::size_t pair_index = 0;
    2 * pair_index + 1 < paired_mission_and_reference_path_cache_.paths.size();
    ++pair_index)
  {
    const auto & mission_segment = paired_mission_and_reference_path_cache_.paths[2 * pair_index];
    bool segment_matched = false;
    for (const auto & pose_stamped : mission_segment.poses) {
      const double dx = front_xy.x() - pose_stamped.pose.position.x;
      const double dy = front_xy.y() - pose_stamped.pose.position.y;
      if (std::hypot(dx, dy) < match_threshold_m) {
        segment_matched = true;
        break;
      }
    }
    if (segment_matched) {
      best_pair_index = static_cast<int>(pair_index);
      break;
    }
  }

  if (best_pair_index < 0) {
    clearActiveReferencePair();
    return;
  }

  const std::size_t mission_path_index = static_cast<std::size_t>(best_pair_index) * 2;
  active_pair_index_ = best_pair_index;
  active_mission_segment_ = paired_mission_and_reference_path_cache_.paths[mission_path_index];
  active_reference_path_ = paired_mission_and_reference_path_cache_.paths[mission_path_index + 1];
  has_active_reference_pair_ = true;
}

void TebLocalPlannerROS::pairedMissionAndReferencePathCallback(
  const capella_ros_msg::msg::LaneCenterPaths::ConstSharedPtr lane_center_paths_message)
{
  
  std::lock_guard<std::mutex> edge_mission_pair_mutex_lock(edge_mission_pair_mutex_);
  paired_mission_and_reference_path_cache_ = *lane_center_paths_message;
  if (paired_mission_and_reference_path_cache_.paths.empty()) {
    clearActiveReferencePair();
    RCLCPP_INFO(
      logger_,
      "收到空的 paired_mission_and_reference_path，已清除当前参考线对");
    return;
  }
  refreshActiveReferencePair();
}

void TebLocalPlannerROS::removedPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr removed_plan_message)
{
  
  std::lock_guard<std::mutex> edge_mission_pair_mutex_lock(edge_mission_pair_mutex_);
  removed_plan_cache_ = *removed_plan_message;
  refreshActiveReferencePair();
}

bool TebLocalPlannerROS::snapshotHasActiveReferencePair() const
{
  
  std::lock_guard<std::mutex> edge_mission_pair_mutex_lock(edge_mission_pair_mutex_);
  return has_active_reference_pair_;
}

nav_msgs::msg::Path TebLocalPlannerROS::snapshotActiveMissionSegment() const
{
  
  std::lock_guard<std::mutex> edge_mission_pair_mutex_lock(edge_mission_pair_mutex_);
  return active_mission_segment_;
}

std::vector<nav_msgs::msg::Path> TebLocalPlannerROS::snapshotActiveReferencePathList() const
{
  
  std::lock_guard<std::mutex> edge_mission_pair_mutex_lock(edge_mission_pair_mutex_);
  if (!has_active_reference_pair_) {
    return {};
  }
  return {active_reference_path_};
}

bool TebLocalPlannerROS::extractLineSegmentFromPath(
  const nav_msgs::msg::Path& path,
  Eigen::Vector2d& segment_start,
  Eigen::Vector2d& segment_end) const
{
  
  if (path.poses.size() < 2) {
    return false;
  }
  segment_start = Eigen::Vector2d(path.poses.front().pose.position.x, path.poses.front().pose.position.y);
  segment_end = Eigen::Vector2d(path.poses.back().pose.position.x, path.poses.back().pose.position.y);
  return (segment_end - segment_start).norm() > 1e-6;
}

double TebLocalPlannerROS::pointToSegmentDistance(
  const Eigen::Vector2d& query_point,
  const Eigen::Vector2d& segment_start,
  const Eigen::Vector2d& segment_end) const
{
  
  const Eigen::Vector2d segment_vector = segment_end - segment_start;
  const double segment_length_squared = segment_vector.squaredNorm();
  if (segment_length_squared < 1e-12) {
    return (query_point - segment_start).norm();
  }
  const double projected_parameter =
    (query_point - segment_start).dot(segment_vector) / segment_length_squared;
  const double clamped_parameter = std::max(0.0, std::min(1.0, projected_parameter));
  const Eigen::Vector2d projected_point = segment_start + clamped_parameter * segment_vector;
  return (query_point - projected_point).norm();
}

double TebLocalPlannerROS::segmentToSegmentDistance(
  const Eigen::Vector2d& first_segment_start,
  const Eigen::Vector2d& first_segment_end,
  const Eigen::Vector2d& second_segment_start,
  const Eigen::Vector2d& second_segment_end) const
{
  
  const double first_to_second_start =
    pointToSegmentDistance(first_segment_start, second_segment_start, second_segment_end);
  const double first_to_second_end =
    pointToSegmentDistance(first_segment_end, second_segment_start, second_segment_end);
  const double second_to_first_start =
    pointToSegmentDistance(second_segment_start, first_segment_start, first_segment_end);
  const double second_to_first_end =
    pointToSegmentDistance(second_segment_end, first_segment_start, first_segment_end);
  return std::min(
    std::min(first_to_second_start, first_to_second_end),
    std::min(second_to_first_start, second_to_first_end));
}

double TebLocalPlannerROS::segmentDirectionAngleDifferenceDeg(
  const Eigen::Vector2d& first_segment_start,
  const Eigen::Vector2d& first_segment_end,
  const Eigen::Vector2d& second_segment_start,
  const Eigen::Vector2d& second_segment_end) const
{
  
  const Eigen::Vector2d first_direction = first_segment_end - first_segment_start;
  const Eigen::Vector2d second_direction = second_segment_end - second_segment_start;
  if (first_direction.norm() < 1e-6 || second_direction.norm() < 1e-6) {
    return 180.0;
  }
  const double dot_product =
    first_direction.normalized().dot(second_direction.normalized());
  const double clamped_dot = std::max(-1.0, std::min(1.0, dot_product));
  return std::acos(std::abs(clamped_dot)) * 180.0 / M_PI;
}

bool TebLocalPlannerROS::shouldRunEdgeFollowingForTransformedPlan(
  const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan)
{
  
  if (transformed_plan.size() < 2) {
    paths_near_edge_hit_count_ = 0;
    paths_near_edge_miss_count_++;
    if (paths_near_edge_miss_count_ >= cfg_->wall_line.paths_near_edge_exit_miss_count) {
      paths_near_edge_active_ = false;
    }
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Edge following: 路径点数太少，退出贴边模式");
    return paths_near_edge_active_;
  }

  // 剩余路径过短退出贴边已在 runEdgeFollowingPathUpdate 按 global_plan_ 与
  // max_global_plan_lookahead_dist 检查；此处不再用局部窗弧长复查。

  if (!snapshotHasActiveReferencePair()) {
    paths_near_edge_hit_count_ = 0;
    paths_near_edge_miss_count_++;
    if (paths_near_edge_miss_count_ >= cfg_->wall_line.paths_near_edge_exit_miss_count) {
      paths_near_edge_active_ = false;
    }
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "Edge following: removed_plan front_pose 未匹配到 mission 段, miss=%d",
      paths_near_edge_miss_count_);
    return paths_near_edge_active_;
  }

  const nav_msgs::msg::Path active_mission_segment = snapshotActiveMissionSegment();
  if (active_mission_segment.poses.size() < 2) {
    paths_near_edge_hit_count_ = 0;
    paths_near_edge_miss_count_++;
    if (paths_near_edge_miss_count_ >= cfg_->wall_line.paths_near_edge_exit_miss_count) {
      paths_near_edge_active_ = false;
    }
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "Edge following: 激活 mission 段长度太短");
    return paths_near_edge_active_;
  }

  // 重叠窗 + 窗内 max 垂距：transformed_plan 与 mission 折线。
  std::vector<Eigen::Vector2d> mission_polyline;
  mission_polyline.reserve(active_mission_segment.poses.size());
  for (const auto& pose_stamped : active_mission_segment.poses) {
    mission_polyline.emplace_back(
      pose_stamped.pose.position.x, pose_stamped.pose.position.y);
  }
  double max_distance_to_mission = 0.0;
  double mean_distance_to_mission = 0.0;
  double overlap_arc_length_to_mission = 0.0;
  const bool distance_ok = pathPointsWithinPolylineDistance(
    transformed_plan,
    mission_polyline,
    cfg_->wall_line.paths_near_edge_match_distance_threshold,
    cfg_->wall_line.min_path_edge_overlap_length,
    max_distance_to_mission,
    mean_distance_to_mission,
    overlap_arc_length_to_mission);
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000,
    "Edge following: transformed_plan 到 mission 折线 max=%.2f mean=%.2f overlap=%.2f "
    "(距阈 %.2f 重叠阈 %.2f)",
    max_distance_to_mission,
    mean_distance_to_mission,
    overlap_arc_length_to_mission,
    cfg_->wall_line.paths_near_edge_match_distance_threshold,
    cfg_->wall_line.min_path_edge_overlap_length);

  if (distance_ok) {
    paths_near_edge_hit_count_++;
    paths_near_edge_miss_count_ = 0;
    if (paths_near_edge_hit_count_ >= cfg_->wall_line.paths_near_edge_enter_hit_count) {
      paths_near_edge_active_ = true;
    }
  } else {
    paths_near_edge_hit_count_ = 0;
    paths_near_edge_miss_count_++;
    if (paths_near_edge_miss_count_ >= cfg_->wall_line.paths_near_edge_exit_miss_count) {
      paths_near_edge_active_ = false;
    }
  }
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000,
    "Edge following: mission 距离门控 hit=%d miss=%d active=%s",
    paths_near_edge_hit_count_,
    paths_near_edge_miss_count_,
    paths_near_edge_active_ ? "true" : "false");
  return paths_near_edge_active_;
}

bool TebLocalPlannerROS::edgeFollowingEntryGuards(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const nav_msgs::msg::Path& input_path)
{
  
  if (input_path.poses.size() < 2) {
    if (wall_line_points_.size() > 0) {
      wall_line_points_.clear();
    }
    switchParameterMode(false);
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Edge following: 路径长度太短，退出贴边模式");
    return false;
  }

  bool has_near_vehicles = false;
  {
    std::lock_guard<std::mutex> global_vehicle_poses_mutex_lock(global_vehicle_poses_mutex_);
    const bool have_active_edge_segment = (wall_line_points_.size() >= 2);
    Eigen::Vector2d active_edge_segment_start;
    Eigen::Vector2d active_edge_segment_end;
    if (have_active_edge_segment) {
      active_edge_segment_start = wall_line_points_[0];
      active_edge_segment_end = wall_line_points_[1];
    }
    for (const auto& nearby_vehicle_pose : global_vehicle_poses_) {
      bool vehicle_in_edge_following_exit_corridor = false;
      if (have_active_edge_segment) {
        vehicle_in_edge_following_exit_corridor = isVehicleInEdgeFollowingExitCorridor(
          robot_pose, active_edge_segment_start, active_edge_segment_end, nearby_vehicle_pose.pose);
      } else {
        vehicle_in_edge_following_exit_corridor =
          distance_points2d(robot_pose.pose.position, nearby_vehicle_pose.pose.position) <
          cfg_->wall_line.close_vehicle_distance_threshold;
      }
      if (vehicle_in_edge_following_exit_corridor) {
        has_near_vehicles = true;
        break;
      }
    }
  }
  if (has_near_vehicles) {
    if (wall_line_points_.size() > 0) {
      wall_line_points_.clear();
    }
    switchParameterMode(false);
    fusion_primary_lock_until_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Edge following: 机器人附近有车辆，退出贴边模式");
    return false;
  }

  if (hasProtrudingObstacleInMonitorCorridor(robot_pose)) {
    if (wall_line_points_.size() > 0) {
      wall_line_points_.clear();
    }
    switchParameterMode(false);
    fusion_primary_lock_until_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Edge following: 机器人附近障碍物轮廓点凸出距离太大，退出贴边模式");
    return false;
  }
  return true;
}

bool TebLocalPlannerROS::trySelectWallSegmentFromCandidates(
  const std::vector<nav_msgs::msg::Path>& wall_candidates,
  const nav_msgs::msg::Path& input_path,
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const double parallel_tolerance_degrees,
  const double distance_tolerance_meters,
  Eigen::Vector2d& selected_edge_segment_start,
  Eigen::Vector2d& selected_edge_segment_end,
  double& minimum_average_distance_to_plan,
  double& robot_perpendicular_distance_to_edge_line)
{
  minimum_average_distance_to_plan = std::numeric_limits<double>::max();
  robot_perpendicular_distance_to_edge_line = std::numeric_limits<double>::max();
  const auto& input_path_start_position = input_path.poses.front().pose.position;
  const auto& input_path_end_position = input_path.poses.back().pose.position;
  const double input_path_delta_x = input_path_end_position.x - input_path_start_position.x;
  const double input_path_delta_y = input_path_end_position.y - input_path_start_position.y;
  const double input_path_segment_length = std::hypot(input_path_delta_x, input_path_delta_y);
  if (input_path_segment_length <= cfg_->wall_line.min_path_line_length) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectWallSegmentFromCandidates] Edge following:"
                        "路径长度太短: %.2f, 最小路径长度阈值: %.2f", 
                        input_path_segment_length, cfg_->wall_line.min_path_line_length);
    return false;
  }
  const double robot_yaw_radians = tf2::getYaw(robot_pose.pose.orientation);
  const double input_path_yaw_radians = std::atan2(input_path_delta_y, input_path_delta_x);
  double path_heading_minus_robot_heading = input_path_yaw_radians - robot_yaw_radians;
  while (path_heading_minus_robot_heading > M_PI) {
    path_heading_minus_robot_heading -= 2 * M_PI;
  }
  while (path_heading_minus_robot_heading < -M_PI) {
    path_heading_minus_robot_heading += 2 * M_PI;
  }
  if (std::fabs(path_heading_minus_robot_heading) > M_PI / 4) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectWallSegmentFromCandidates] Edge following:"
                        "机器人方向与路径方向夹角 %.2f 度,超过45度", 
                        path_heading_minus_robot_heading * 180.0 / M_PI);
    return false;
  }
  const double input_path_direction_x = input_path_delta_x / input_path_segment_length;
  const double input_path_direction_y = input_path_delta_y / input_path_segment_length;

  bool found_valid_wall_segment = false;
  double best_minimum_average_distance = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point best_candidate_wall_start_position;
  geometry_msgs::msg::Point best_candidate_wall_end_position;
  double best_robot_perpendicular_distance_to_wall = std::numeric_limits<double>::max();

  for (const auto& wall_candidate_path : wall_candidates) {
    if (wall_candidate_path.poses.size() < 2) {
      continue;
    }
    const auto& wall_segment_start_position = wall_candidate_path.poses.front().pose.position;
    const auto& wall_segment_end_position = wall_candidate_path.poses.back().pose.position;
    const double wall_segment_delta_x = wall_segment_end_position.x - wall_segment_start_position.x;
    const double wall_segment_delta_y = wall_segment_end_position.y - wall_segment_start_position.y;
    const double wall_segment_length = std::hypot(wall_segment_delta_x, wall_segment_delta_y);
    if (wall_segment_length < min_wall_line_length_) {
      RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectWallSegmentFromCandidates] Edge following:"
                        "墙线长度太短: %.2f, 最小墙线长度阈值: %.2f", 
                        wall_segment_length, min_wall_line_length_);
      continue;
    }
    const double wall_direction_x = wall_segment_delta_x / wall_segment_length;
    const double wall_direction_y = wall_segment_delta_y / wall_segment_length;
    const double direction_dot_product =
      input_path_direction_x * wall_direction_x + input_path_direction_y * wall_direction_y;
    const double absolute_cosine_parallelism = std::fabs(direction_dot_product);
    const double parallelism_cosine_threshold =
      std::cos(parallel_tolerance_degrees / 180.0 * M_PI);
    if (absolute_cosine_parallelism < parallelism_cosine_threshold) {
      RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectWallSegmentFromCandidates] Edge following:"
                         "墙线与路径不平行: %.2f 度, 平行度阈值: %.2f 度", 
                         absolute_cosine_parallelism * 180.0 / M_PI, parallel_tolerance_degrees);
      continue;
    }
    const double wall_line_implicit_x_coefficient = wall_segment_delta_y;
    const double wall_line_implicit_y_coefficient = -wall_segment_delta_x;
    const double wall_line_implicit_constant_term =
      wall_segment_end_position.x * wall_segment_start_position.y -
      wall_segment_start_position.x * wall_segment_end_position.y;
    const double line_equation_normalization = std::hypot(
      wall_line_implicit_x_coefficient, wall_line_implicit_y_coefficient);
    const double average_distance_path_to_wall_line =
      std::fabs(
        wall_line_implicit_x_coefficient * input_path_start_position.x +
        wall_line_implicit_y_coefficient * input_path_start_position.y +
        wall_line_implicit_constant_term) /
      (line_equation_normalization + 1e-18);
    const double robot_perpendicular_distance_to_wall_line =
      std::fabs(
        wall_line_implicit_x_coefficient * robot_pose.pose.position.x +
        wall_line_implicit_y_coefficient * robot_pose.pose.position.y +
        wall_line_implicit_constant_term) /
      (line_equation_normalization + 1e-18);
    if (average_distance_path_to_wall_line <= distance_tolerance_meters &&
        average_distance_path_to_wall_line < best_minimum_average_distance) {
      found_valid_wall_segment = true;
      best_minimum_average_distance = average_distance_path_to_wall_line;
      best_candidate_wall_start_position = wall_segment_start_position;
      best_candidate_wall_end_position = wall_segment_end_position;
      best_robot_perpendicular_distance_to_wall = robot_perpendicular_distance_to_wall_line;
    }
  }
  if (!found_valid_wall_segment) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[trySelectWallSegmentFromCandidates] Edge following:"
                         "没有找到合适的墙线");
    return false;
  }
  selected_edge_segment_start = Eigen::Vector2d(
    best_candidate_wall_start_position.x, best_candidate_wall_start_position.y);
  selected_edge_segment_end = Eigen::Vector2d(
    best_candidate_wall_end_position.x, best_candidate_wall_end_position.y);
  minimum_average_distance_to_plan = best_minimum_average_distance;
  robot_perpendicular_distance_to_edge_line = best_robot_perpendicular_distance_to_wall;
  RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[trySelectWallSegmentFromCandidates] Edge following:"
                      "选择的墙线起点: (%.2f, %.2f), 终点: (%.2f, %.2f), 平均距离: %.2f, 机器人到墙线距离: %.2f", 
                       selected_edge_segment_start.x(), selected_edge_segment_start.y(), 
                       selected_edge_segment_end.x(), selected_edge_segment_end.y(), 
                       minimum_average_distance_to_plan, robot_perpendicular_distance_to_edge_line);
  return true;
}

bool TebLocalPlannerROS::trySelectSegmentFromTwoPointPath(
  const nav_msgs::msg::Path& two_point_line_path,
  const nav_msgs::msg::Path& input_path,
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const double parallel_tolerance_degrees,
  const double distance_tolerance_meters,
  Eigen::Vector2d& selected_edge_segment_start,
  Eigen::Vector2d& selected_edge_segment_end,
  double& minimum_average_distance_to_plan,
  double& robot_perpendicular_distance_to_edge_line)
{
  (void)parallel_tolerance_degrees;  // 只距离、不平行：保留参数以兼容调用方。
  minimum_average_distance_to_plan = std::numeric_limits<double>::max();
  robot_perpendicular_distance_to_edge_line = std::numeric_limits<double>::max();
  if (two_point_line_path.poses.size() < 2 || input_path.poses.size() < 2) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectSegmentFromTwoPointPath] Edge following:"
                         "两点线路径或输入路径数量小于2, 路径点数: %zu, 输入路径点数: %zu",
                         two_point_line_path.poses.size(), input_path.poses.size());
    return false;
  }
  const auto& input_path_start_position = input_path.poses.front().pose.position;
  const auto& input_path_end_position = input_path.poses.back().pose.position;
  const double input_path_delta_x = input_path_end_position.x - input_path_start_position.x;
  const double input_path_delta_y = input_path_end_position.y - input_path_start_position.y;
  const double input_path_segment_length = std::hypot(input_path_delta_x, input_path_delta_y);
  if (input_path_segment_length <= cfg_->wall_line.min_path_line_length) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectSegmentFromTwoPointPath] Edge following:"
                        "路径长度太短: %.2f, 最小路径长度阈值: %.2f", 
                        input_path_segment_length, cfg_->wall_line.min_path_line_length);
    return false;
  }
  const double robot_yaw_radians = tf2::getYaw(robot_pose.pose.orientation);
  const double input_path_yaw_radians = std::atan2(input_path_delta_y, input_path_delta_x);
  double path_heading_minus_robot_heading = input_path_yaw_radians - robot_yaw_radians;
  while (path_heading_minus_robot_heading > M_PI) {
    path_heading_minus_robot_heading -= 2 * M_PI;
  }
  while (path_heading_minus_robot_heading < -M_PI) {
    path_heading_minus_robot_heading += 2 * M_PI;
  }
  if (std::fabs(path_heading_minus_robot_heading) > M_PI / 4) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectSegmentFromTwoPointPath] Edge following:"
                         "机器人方向与路径方向夹角为 %.2f 度，超过45度", 
                         path_heading_minus_robot_heading * 180.0 / M_PI);
    return false;
  }

  const auto& edge_line_start_position = two_point_line_path.poses.front().pose.position;
  const auto& edge_line_end_position = two_point_line_path.poses.back().pose.position;
  const Eigen::Vector2d edge_start(
    edge_line_start_position.x, edge_line_start_position.y);
  const Eigen::Vector2d edge_end(
    edge_line_end_position.x, edge_line_end_position.y);
  const double edge_line_segment_length = (edge_end - edge_start).norm();
  if (edge_line_segment_length < min_wall_line_length_) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectSegmentFromTwoPointPath] Edge following:"
                         "墙线长度太短: %.2f, 最小墙线长度阈值: %.2f", 
                         edge_line_segment_length, min_wall_line_length_);
    return false;
  }

  const std::vector<Eigen::Vector2d> edge_polyline{edge_start, edge_end};
  double max_distance_path_to_edge = 0.0;
  double mean_distance_path_to_edge = 0.0;
  double overlap_arc_length_path_to_edge = 0.0;
  if (!pathPointsWithinPolylineDistance(
        input_path.poses,
        edge_polyline,
        distance_tolerance_meters,
        cfg_->wall_line.min_path_edge_overlap_length,
        max_distance_path_to_edge,
        mean_distance_path_to_edge,
        overlap_arc_length_path_to_edge))
  {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectSegmentFromTwoPointPath] Edge following:"
                         "路径到边线 max=%.2f mean=%.2f overlap=%.2f, 距阈: %.2f 重叠阈: %.2f",
                         max_distance_path_to_edge, mean_distance_path_to_edge,
                         overlap_arc_length_path_to_edge,
                         distance_tolerance_meters,
                         cfg_->wall_line.min_path_edge_overlap_length);
    return false;
  }
  const Eigen::Vector2d robot_xy(
    robot_pose.pose.position.x, robot_pose.pose.position.y);
  const double robot_distance_to_edge =
    pointToPolylineDistance2d(robot_xy, edge_polyline);

  selected_edge_segment_start = edge_start;
  selected_edge_segment_end = edge_end;
  minimum_average_distance_to_plan = mean_distance_path_to_edge;
  robot_perpendicular_distance_to_edge_line = robot_distance_to_edge;
  return true;
}

bool TebLocalPlannerROS::trySelectBestReferencePathFromList(
  const std::vector<nav_msgs::msg::Path>& reference_path_candidates,
  const nav_msgs::msg::Path& input_path,
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const double parallel_tolerance_degrees,
  const double distance_tolerance_meters,
  Eigen::Vector2d& selected_edge_segment_start,
  Eigen::Vector2d& selected_edge_segment_end,
  double& minimum_average_distance_to_plan,
  double& robot_perpendicular_distance_to_edge_line)
{
  (void)parallel_tolerance_degrees;  // 只距离、不平行：保留参数以兼容调用方。

  bool found_any_matching_reference_segment = false;
  double best_minimum_average_distance_to_plan = std::numeric_limits<double>::max();
  Eigen::Vector2d best_reference_segment_start;
  Eigen::Vector2d best_reference_segment_end;
  double best_robot_perpendicular_distance_to_edge_line = 0.0;
  const Eigen::Vector2d robot_xy(
    robot_pose.pose.position.x, robot_pose.pose.position.y);

  if (input_path.poses.size() < 2) {
    return false;
  }
  const auto& input_path_start_position = input_path.poses.front().pose.position;
  const auto& input_path_end_position = input_path.poses.back().pose.position;
  const double input_path_segment_length = std::hypot(
    input_path_end_position.x - input_path_start_position.x,
    input_path_end_position.y - input_path_start_position.y);
  if (input_path_segment_length <= cfg_->wall_line.min_path_line_length) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "[trySelectBestReferencePathFromList] Edge following: 路径长度太短: %.2f",
      input_path_segment_length);
    return false;
  }
  const double robot_yaw_radians = tf2::getYaw(robot_pose.pose.orientation);
  const double input_path_yaw_radians = std::atan2(
    input_path_end_position.y - input_path_start_position.y,
    input_path_end_position.x - input_path_start_position.x);
  double path_heading_minus_robot_heading = input_path_yaw_radians - robot_yaw_radians;
  while (path_heading_minus_robot_heading > M_PI) {
    path_heading_minus_robot_heading -= 2 * M_PI;
  }
  while (path_heading_minus_robot_heading < -M_PI) {
    path_heading_minus_robot_heading += 2 * M_PI;
  }
  if (std::fabs(path_heading_minus_robot_heading) > M_PI / 4) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "[trySelectBestReferencePathFromList] Edge following: 机器人与路径夹角过大: %.2f 度",
      path_heading_minus_robot_heading * 180.0 / M_PI);
    return false;
  }

  for (const auto& raw_reference_path : reference_path_candidates) {
    if (raw_reference_path.poses.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        logger_, *(clock_), 2000,
        "[trySelectBestReferencePathFromList] Edge following: 贴边参考线点数小于2, 点数: %zu",
        raw_reference_path.poses.size());
      continue;
    }
    try {
      geometry_msgs::msg::TransformStamped transform_map_from_reference_path_frame;
      const bool needs_tf =
        !raw_reference_path.header.frame_id.empty() &&
        raw_reference_path.header.frame_id != cfg_->map_frame;
      if (needs_tf) {
        transform_map_from_reference_path_frame = tf_->lookupTransform(
          cfg_->map_frame,
          raw_reference_path.header.frame_id,
          tf2::TimePointZero,
          tf2::durationFromSec(0.5));
      }

      std::vector<Eigen::Vector2d> reference_polyline_in_map;
      reference_polyline_in_map.reserve(raw_reference_path.poses.size());
      for (const auto& raw_pose : raw_reference_path.poses) {
        geometry_msgs::msg::PoseStamped pose_in_map = raw_pose;
        if (needs_tf) {
          tf2::doTransform(
            pose_in_map, pose_in_map, transform_map_from_reference_path_frame);
        }
        reference_polyline_in_map.emplace_back(
          pose_in_map.pose.position.x, pose_in_map.pose.position.y);
      }

      // 重叠窗 + 窗内 max 垂距：input_path 与 reference 折线。
      double max_distance_to_reference = 0.0;
      double mean_distance_to_reference = 0.0;
      double overlap_arc_length_to_reference = 0.0;
      if (!pathPointsWithinPolylineDistance(
            input_path.poses,
            reference_polyline_in_map,
            distance_tolerance_meters,
            cfg_->wall_line.min_path_edge_overlap_length,
            max_distance_to_reference,
            mean_distance_to_reference,
            overlap_arc_length_to_reference))
      {
        RCLCPP_INFO_THROTTLE(
          logger_, *(clock_), 2000,
          "[trySelectBestReferencePathFromList] Edge following: "
          "路径到参考折线 max=%.2f mean=%.2f overlap=%.2f (距阈 %.2f 重叠阈 %.2f)",
          max_distance_to_reference, mean_distance_to_reference,
          overlap_arc_length_to_reference,
          distance_tolerance_meters,
          cfg_->wall_line.min_path_edge_overlap_length);
        continue;
      }

      double robot_distance_to_reference = 0.0;
      if (pointProjectsOntoPolyline(
            robot_xy, reference_polyline_in_map, robot_distance_to_reference))
      {
        if (robot_distance_to_reference > distance_tolerance_meters) {
          continue;
        }
      } else {
        robot_distance_to_reference =
          pointToPolylineDistance2d(robot_xy, reference_polyline_in_map);
      }

      Eigen::Vector2d local_segment_start;
      Eigen::Vector2d local_segment_end;
      if (!extractNearestExpandedPolylineSegment(
            reference_polyline_in_map,
            robot_xy,
            min_wall_line_length_,
            local_segment_start,
            local_segment_end))
      {
        RCLCPP_WARN_THROTTLE(
          logger_, *(clock_), 2000,
          "[trySelectBestReferencePathFromList] Edge following: 无法从贴边参考折线提取局部段");
        continue;
      }
      if ((local_segment_end - local_segment_start).norm() < min_wall_line_length_) {
        continue;
      }

      RCLCPP_INFO_THROTTLE(
        logger_, *(clock_), 2000,
        "[trySelectBestReferencePathFromList] Edge following: 局部贴边段 "
        "(%.2f, %.2f)->(%.2f, %.2f), path->ref max=%.2f mean=%.2f overlap=%.2f",
        local_segment_start.x(), local_segment_start.y(),
        local_segment_end.x(), local_segment_end.y(),
        max_distance_to_reference, mean_distance_to_reference,
        overlap_arc_length_to_reference);

      if (mean_distance_to_reference < best_minimum_average_distance_to_plan) {
        best_minimum_average_distance_to_plan = mean_distance_to_reference;
        best_reference_segment_start = local_segment_start;
        best_reference_segment_end = local_segment_end;
        best_robot_perpendicular_distance_to_edge_line = robot_distance_to_reference;
        found_any_matching_reference_segment = true;
      }
    } catch (const tf2::TransformException& transform_exception) {
      RCLCPP_INFO_THROTTLE(
        logger_, *(clock_), 2000, "[trySelectBestReferencePathFromList] Edge following: 贴边参考线TF跳过: %s", 
        transform_exception.what());
      continue;
    }
  }
  if (!found_any_matching_reference_segment) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectBestReferencePathFromList] Edge following:"
                         "没有找到合适的贴边参考线");
    return false;
  }
  selected_edge_segment_start = best_reference_segment_start;
  selected_edge_segment_end = best_reference_segment_end;
  minimum_average_distance_to_plan = best_minimum_average_distance_to_plan;
  robot_perpendicular_distance_to_edge_line = best_robot_perpendicular_distance_to_edge_line;
  return true;
}

double TebLocalPlannerROS::computeWallLineDistWeightFromRobotDistance(
  const double robot_perpendicular_distance_to_edge_line) const
{
  
  const double base_weight = weight_wall_line_dist_;
  const double min_dist = cfg_->wall_line.min_wall_dist;
  const double tolerance = cfg_->wall_line.distance_tolerance;
  const double delta = robot_perpendicular_distance_to_edge_line - min_dist;
  constexpr double k_eps = 1e-5;

  if (delta < -k_eps) {
    const double scale_lower = cfg_->wall_line.wall_line_dist_weight_scale_under_min_lower;
    const double scale_upper = cfg_->wall_line.wall_line_dist_weight_scale_under_min_upper;
    const double clamp_lo = std::min(scale_lower, scale_upper);
    const double clamp_hi = std::max(scale_lower, scale_upper);
    if (min_dist < k_eps) {
      return base_weight * clamp_lo;
    }
    const double ratio = robot_perpendicular_distance_to_edge_line / min_dist;
    const double scale = std::max(clamp_lo, std::min(clamp_hi, ratio));
    return base_weight * scale;
  }

  if (delta <= k_eps) {
    return base_weight;
  }

  const double band = tolerance - min_dist;
  if (std::fabs(band) <= k_eps) {
    return base_weight;
  }

  const double scale_at_tolerance = cfg_->wall_line.wall_line_dist_weight_scale_at_distance_tolerance;
  const double weight_at_tolerance = base_weight * scale_at_tolerance;
  const double weight_at_min = base_weight;
  const double interpolated =
    (weight_at_tolerance - weight_at_min) / band * delta + weight_at_min;
  const double weight_lo = std::min(weight_at_min, weight_at_tolerance);
  const double weight_hi = std::max(weight_at_min, weight_at_tolerance);
  return std::min(weight_hi, std::max(weight_lo, interpolated));
}

void TebLocalPlannerROS::updateScheduledWallDistWeights(
  const double robot_perpendicular_distance_to_edge_line)
{
  cfg_->optim.weight_wall_line_dist =
    computeWallLineDistWeightFromRobotDistance(robot_perpendicular_distance_to_edge_line);
  if (weight_wall_line_dist_ > 1e-9 && weight_wall_side_pull_ > 0.0) {
    const double scale = cfg_->optim.weight_wall_line_dist / weight_wall_line_dist_;
    cfg_->optim.weight_wall_side_pull = weight_wall_side_pull_ * scale;
  }
}

void TebLocalPlannerROS::applyWallLineSegmentAndVisual(
  const Eigen::Vector2d& wall_line_segment_start,
  const Eigen::Vector2d& wall_line_segment_end,
  const nav_msgs::msg::Path& input_path,
  const double minimum_average_distance_to_plan,
  const double robot_perpendicular_distance_to_edge_line)
{
  
  if (protrusionBlocksEdgeEntryForWallSegment(
      wall_line_segment_start, wall_line_segment_end, robot_pose_.position()))
  {
    return;
  }

  if (wall_line_points_.size() > 0) {
    wall_line_points_.clear();
  }
  wall_line_points_.push_back(wall_line_segment_start);
  wall_line_points_.push_back(wall_line_segment_end);
  updateScheduledWallDistWeights(robot_perpendicular_distance_to_edge_line);
  RCLCPP_INFO_THROTTLE(
    logger_, *(clock_), 2000,
    "[applyWallLineSegmentAndVisual] Edge following: 机器人到贴边线距离 %.3f m, 贴边权重: %.2f, 侧隙Pull: %.2f",
    robot_perpendicular_distance_to_edge_line, cfg_->optim.weight_wall_line_dist,
    cfg_->optim.weight_wall_side_pull);
  (void)minimum_average_distance_to_plan;
  wall_line_update_time_ = clock_->now();
  std_msgs::msg::Float32 edge_distance_message;
  edge_distance_message.data = static_cast<float>(robot_perpendicular_distance_to_edge_line);
  edge_distance_publisher_->publish(edge_distance_message);
  switchParameterMode(true);

  visualization_msgs::msg::Marker wall_line_marker_message;
  wall_line_marker_message.ns = "teb_local_planner";
  wall_line_marker_message.id = 0;
  wall_line_marker_message.type = visualization_msgs::msg::Marker::LINE_LIST;
  wall_line_marker_message.action = visualization_msgs::msg::Marker::ADD;
  wall_line_marker_message.scale.x = 0.1f;
  wall_line_marker_message.color.r = 1.0f;
  wall_line_marker_message.color.g = 1.0f;
  wall_line_marker_message.color.b = 0.0f;
  wall_line_marker_message.color.a = 1.0f;
  geometry_msgs::msg::Point marker_line_start_point;
  geometry_msgs::msg::Point marker_line_end_point;
  marker_line_start_point.x = wall_line_segment_start.x();
  marker_line_start_point.y = wall_line_segment_start.y();
  marker_line_start_point.z = 0.0;
  marker_line_end_point.x = wall_line_segment_end.x();
  marker_line_end_point.y = wall_line_segment_end.y();
  marker_line_end_point.z = 0.0;
  wall_line_marker_message.points.push_back(marker_line_start_point);
  wall_line_marker_message.points.push_back(marker_line_end_point);
  wall_line_marker_message.header.stamp = clock_->now();
  wall_line_marker_message.header.frame_id = input_path.header.frame_id;
  wall_line_marker_publisher_->publish(wall_line_marker_message);
}

void TebLocalPlannerROS::mergeFusionPrimaryWithReference(
  const Eigen::Vector2d& fusion_primary_segment_start,
  const Eigen::Vector2d& fusion_primary_segment_end,
  const bool fusion_primary_segment_valid,
  const double fusion_primary_minimum_average_distance_to_plan,
  const double fusion_primary_robot_perpendicular_distance_to_edge,
  const Eigen::Vector2d& reference_segment_start,
  const Eigen::Vector2d& reference_segment_end,
  const bool reference_segment_valid,
  const double reference_minimum_average_distance_to_plan,
  const double reference_robot_perpendicular_distance_to_edge,
  const nav_msgs::msg::Path& input_path)
{
  
  // 1) 读取融合门限与锁定参数。
  const rclcpp::Time current_clock_time = clock_->now();
  const double fusion_lock_duration_s =
    cfg_->wall_line.fusion_primary_lock_duration;
  const double reference_match_max_angle_deg =
    cfg_->wall_line.reference_match_max_angle_deg;
  const double reference_match_max_distance_m =
    cfg_->wall_line.reference_match_max_distance_m;

  if (!fusion_primary_segment_valid) {
    fusion_primary_lock_until_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  }

  // 2) 主来源与参考线几何接近时，刷新 primary 锁定窗口。
  const bool segments_geometrically_close =
    fusion_primary_segment_valid && reference_segment_valid &&
    segmentsCloseForFusion(
      fusion_primary_segment_start,
      fusion_primary_segment_end,
      reference_segment_start,
      reference_segment_end,
      reference_match_max_angle_deg,
      reference_match_max_distance_m);
  if (segments_geometrically_close) {
    fusion_primary_lock_until_ =
      current_clock_time +
      rclcpp::Duration::from_seconds(fusion_lock_duration_s);
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, 
                         "[mergeFusionPrimaryWithReference] Edge following:"
                         "贴边参考线和墙线或路沿特征接近");
  }

  // 3) 依据“几何接近/锁定中/参考不可用”决定是否优先主来源。
  bool use_primary_segment = false;
  if (fusion_primary_segment_valid) {
    if (segments_geometrically_close) {
      use_primary_segment = true;
    } else if (
      fusion_primary_lock_until_.nanoseconds() != 0u &&
      current_clock_time < fusion_primary_lock_until_) {
      use_primary_segment = true;
    } else if (!reference_segment_valid) {
      use_primary_segment = true;
    }
  }

  if (use_primary_segment) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, 
                         "[mergeFusionPrimaryWithReference] Edge following:"
                         "使用提取的墙线或是路沿作为贴边路径");
    applyWallLineSegmentAndVisual(
      fusion_primary_segment_start,
      fusion_primary_segment_end,
      input_path,
      fusion_primary_minimum_average_distance_to_plan,
      fusion_primary_robot_perpendicular_distance_to_edge);
    cacheReferenceLineHold(
      fusion_primary_segment_start,
      fusion_primary_segment_end,
      fusion_primary_minimum_average_distance_to_plan,
      fusion_primary_robot_perpendicular_distance_to_edge,
      current_clock_time);
    return;
  }

  // 4) 主来源不可用时，回退到参考线。
  if (reference_segment_valid) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, 
                         "[mergeFusionPrimaryWithReference] Edge following:"
                         "使用贴边参考线作为贴边路径");
    applyWallLineSegmentAndVisual(
      reference_segment_start,
      reference_segment_end,
      input_path,
      reference_minimum_average_distance_to_plan,
      reference_robot_perpendicular_distance_to_edge);
    cacheReferenceLineHold(
      reference_segment_start,
      reference_segment_end,
      reference_minimum_average_distance_to_plan,
      reference_robot_perpendicular_distance_to_edge,
      current_clock_time);
    return;
  }

  // 5) 参考线也不可用时，在超时窗口内可复用上一条缓存参考线。
  const double reference_no_valid_path_timeout_seconds =
    cfg_->wall_line.reference_no_valid_path_timeout;
  if (reference_line_hold_.size() == 2 &&
      reference_line_last_success_time_.nanoseconds() != 0u &&
      (current_clock_time - reference_line_last_success_time_).seconds() <
        reference_no_valid_path_timeout_seconds) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, 
                         "[mergeFusionPrimaryWithReference] Edge following:"
                         "使用缓存的贴边参考线作为贴边路径");
    applyWallLineSegmentAndVisual(
      reference_line_hold_[0],
      reference_line_hold_[1],
      input_path,
      reference_line_hold_minimum_average_distance_to_plan_,
      reference_line_hold_robot_perpendicular_distance_to_edge_line_);
    return;
  }

  // 6) 全部不可用，统一退出贴边模式。
  exitEdgeFollowingMode(
    "mergeFusionPrimaryWithReference",
    "未找到合适的贴边路径，清空缓存，退出贴边模式");
}

void TebLocalPlannerROS::exitEdgeFollowingMode(
  const char * log_context,
  const char * reason)
{
  
  // 退出动作必须原子一致：清线 -> 关模式 -> 打日志。
  if (!wall_line_points_.empty()) {
    wall_line_points_.clear();
  }
  reference_line_hold_.clear();
  const std::string map_frame = costmap_ros_ ?
    costmap_ros_->getGlobalFrameID() : cfg_->map_frame;
  clearEdgeMonitorCorridors(map_frame);
  switchParameterMode(false);
  RCLCPP_INFO_THROTTLE(
    logger_,
    *(clock_),
    2000,
    "[%s] Edge following: %s",
    log_context != nullptr ? log_context : "exitEdgeFollowingMode",
    reason != nullptr ? reason : "退出贴边模式");
}

void TebLocalPlannerROS::cacheReferenceLineHold(
  const Eigen::Vector2d& segment_start,
  const Eigen::Vector2d& segment_end,
  const double minimum_average_distance_to_plan,
  const double robot_perpendicular_distance_to_edge_line,
  const rclcpp::Time& cache_time)
{
  
  // 统一缓存“最近一次可用贴边线段”及其质量指标，供后续 fallback 使用。
  reference_line_hold_.clear();
  reference_line_hold_.push_back(segment_start);
  reference_line_hold_.push_back(segment_end);
  reference_line_hold_minimum_average_distance_to_plan_ = minimum_average_distance_to_plan;
  reference_line_hold_robot_perpendicular_distance_to_edge_line_ =
    robot_perpendicular_distance_to_edge_line;
  reference_line_last_success_time_ = cache_time;
}

void TebLocalPlannerROS::pruneExpiredCurbLinePath()
{
  
  // 路沿数据是时效数据，超时后清空，避免用旧线段误导贴边。
  std::lock_guard<std::mutex> curb_line_mutex_lock(curb_line_mutex_);
  if (curb_line_path_.poses.empty()) {
    return;
  }
  const auto seconds_since_curb_path_update = clock_->now() - curb_line_update_time_;
  if (seconds_since_curb_path_update.seconds() > keep_wall_line_time_) {
    curb_line_path_.poses.clear();
  }
}

void TebLocalPlannerROS::applyReferenceLineHoldFallbackOrExit(
  const nav_msgs::msg::Path& input_path)
{
  
  // 参考线失效后的兜底策略：先尝试缓存，再退出模式。
  const rclcpp::Time current_clock_time = clock_->now();
  const double reference_no_valid_path_timeout_s =
    cfg_->wall_line.reference_no_valid_path_timeout;
  if (reference_line_hold_.size() == 2 &&
      reference_line_last_success_time_.nanoseconds() != 0u &&
      (current_clock_time - reference_line_last_success_time_).seconds() <
        reference_no_valid_path_timeout_s) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000,
      "[applyReferenceLineHoldFallbackOrExit] Edge following: 使用缓存的参考路径作为贴边路径"
      "参考路径起点: (%.2f, %.2f), 参考路径终点: (%.2f, %.2f), 参考路径最小平均距离: %.2f, 参考路径与机器人垂直距离: %.2f",
      reference_line_hold_[0].x(), reference_line_hold_[0].y(),
      reference_line_hold_[1].x(), reference_line_hold_[1].y(),
      reference_line_hold_minimum_average_distance_to_plan_,
      reference_line_hold_robot_perpendicular_distance_to_edge_line_);
    applyWallLineSegmentAndVisual(
      reference_line_hold_[0],
      reference_line_hold_[1],
      input_path,
      reference_line_hold_minimum_average_distance_to_plan_,
      reference_line_hold_robot_perpendicular_distance_to_edge_line_);
    return;
  }
  exitEdgeFollowingMode(
    "applyReferenceLineHoldFallbackOrExit",
    "参考路径缓存失效，退出贴边模式");
}

void TebLocalPlannerROS::updateReferenceLineVec(
  const std::vector<nav_msgs::msg::Path>& reference_path_list,
  nav_msgs::msg::Path& input_path,
  const double parallel_tolerance_degrees,
  const double distance_tolerance_meters,
  const geometry_msgs::msg::PoseStamped& robot_pose)
{
  
  // 参考线模式：只在参考路径集合中做选线，不融合墙线/路沿。
  if (!edgeFollowingEntryGuards(robot_pose, input_path)) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[updateReferenceLineVec] Edge following:"
                "进入贴边模式检查失败");
    return;
  }
  // 先从候选参考路径中选择最佳线段，再统一更新可视化与缓存。
  Eigen::Vector2d selected_reference_segment_start;
  Eigen::Vector2d selected_reference_segment_end;
  double selected_min_avg_dist_to_plan = 0.0;
  double selected_robot_dist_to_edge = 0.0;
  const bool selected_reference_ok = trySelectBestReferencePathFromList(
    reference_path_list,
    input_path,
    robot_pose,
    parallel_tolerance_degrees,
    distance_tolerance_meters,
    selected_reference_segment_start,
    selected_reference_segment_end,
    selected_min_avg_dist_to_plan,
    selected_robot_dist_to_edge);
  const rclcpp::Time current_clock_time = clock_->now();
  if (selected_reference_ok) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[updateReferenceLineVec] Edge following: 选择参考路径起点: (%.2f, %.2f),"
                        "参考路径终点: (%.2f, %.2f), 参考路径最小平均距离: %.2f, 参考路径与机器人垂直距离: %.2f",
                        selected_reference_segment_start.x(), selected_reference_segment_start.y(), 
                        selected_reference_segment_end.x(), selected_reference_segment_end.y(), 
                        selected_min_avg_dist_to_plan, selected_robot_dist_to_edge);
    applyWallLineSegmentAndVisual(
      selected_reference_segment_start,
      selected_reference_segment_end,
      input_path,
      selected_min_avg_dist_to_plan,
      selected_robot_dist_to_edge);
    cacheReferenceLineHold(
      selected_reference_segment_start,
      selected_reference_segment_end,
      selected_min_avg_dist_to_plan,
      selected_robot_dist_to_edge,
      current_clock_time);
    return;
  }
  applyReferenceLineHoldFallbackOrExit(input_path);
}

void TebLocalPlannerROS::runEdgeFollowingPathUpdate(
  std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
  const geometry_msgs::msg::PoseStamped& robot_pose)
{
  
  // 总调度入口：预裁剪输入路径后，按 edge_mode 分发到不同贴边来源。
  if (cfg_->optim.weight_wall_line_dist <= 0.0) {
    return;
  }
  const std::string edge_mode = normalizeEdgeModeEnv(detail::useCurbOrWallEnv());
  if (edge_mode.empty()) {
    return;
  }

  if (transformed_plan.size() < 2) {
    exitEdgeFollowingMode(
      "runEdgeFollowingPathUpdate",
      "转换路径点数太少，退出贴边模式");
    return;
  }

  // 剩余全局路径过短则退出贴边，用 max_global_plan_lookahead_dist 作为离墙跑道。
  // 不看 transformed_plan：局部窗会被 lookahead/costmap/角点截断，剩余仍长时不应因此退出。
  const double remaining_exit_length = cfg_->trajectory.max_global_plan_lookahead_dist;
  if (remaining_exit_length > 0.0) {
    double remaining_global_arc_m = 0.0;
    if (global_plan_.size() >= 2) {
      for (size_t plan_index = 1; plan_index < global_plan_.size(); ++plan_index) {
        remaining_global_arc_m += distance_points2d(
          global_plan_[plan_index - 1].pose.position,
          global_plan_[plan_index].pose.position);
        if (remaining_global_arc_m >= remaining_exit_length) {
          break;
        }
      }
    }
    if (remaining_global_arc_m < remaining_exit_length) {
      paths_near_edge_hit_count_ = 0;
      paths_near_edge_active_ = false;
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 2000,
        "Edge following: 全局路径剩余弧长 %.2f < max_global_plan_lookahead_dist %.2f, 退出贴边模式",
        remaining_global_arc_m,
        remaining_exit_length);
      exitEdgeFollowingMode(
        "runEdgeFollowingPathUpdate",
        "全局路径剩余弧长不足，退出贴边模式");
      return;
    }
  }

  // 将 transformed_plan 按弧长截断到 transform_path_line_length，仅作为本拍贴边匹配窗。
  double accumulated_dist_m = 0.0;
  unsigned int input_path_end_idx =
    static_cast<unsigned int>(transformed_plan.size() - 1);
  for (unsigned int segment_end_index = 1; segment_end_index < transformed_plan.size();
       segment_end_index++) {
    accumulated_dist_m += distance_points2d(
      transformed_plan[segment_end_index - 1].pose.position,
      transformed_plan[segment_end_index].pose.position);
    if (accumulated_dist_m >= cfg_->wall_line.transform_path_line_length) {
      input_path_end_idx = segment_end_index;
      break;
    }
  }
  nav_msgs::msg::Path input_path;
  input_path.header = transformed_plan.at(0).header;
  input_path.poses =
    std::vector<geometry_msgs::msg::PoseStamped>(transformed_plan.begin(),
      transformed_plan.begin() +
        static_cast<std::ptrdiff_t>(input_path_end_idx + 1));
  transformed_path->publish(input_path);

  updateEdgeMonitorCorridors(robot_pose, input_path.poses);

  struct MonitorCorridorGuard
  {
    TebLocalPlannerROS * self;
    const geometry_msgs::msg::PoseStamped& robot_pose;
    const nav_msgs::msg::Path& input_path;
    ~MonitorCorridorGuard()
    {
      self->updateEdgeMonitorCorridors(robot_pose, input_path.poses);
      self->publishMonitorCorridorMarkers(input_path.header.frame_id);
    }
  } monitor_corridor_guard{this, robot_pose, input_path};

  const bool edge_mode_includes_wall =
    (edge_mode.find("wall") != std::string::npos);
  const bool edge_mode_includes_curb =
    (edge_mode.find("curb") != std::string::npos);
  const bool edge_mode_includes_reference =
    (edge_mode.find("reference") != std::string::npos);

  // 参考线相关模式先做“路径靠边可行性”门控，失败则直接退出贴边。
  if (edge_mode_includes_reference) {
    if (!shouldRunEdgeFollowingForTransformedPlan(input_path.poses)) {
      exitEdgeFollowingMode(
        "runEdgeFollowingPathUpdate",
        "不能找到路径靠近边缘，退出贴边模式");
      return;
    }
  }

  // paired_mission_and_reference_path：上层在进入导航前发布（TRANSIENT_LOCAL），
  // 此处不得因超时而清空缓存；几何与 TF 使用在 trySelectBestReferencePathFromList 中按当前时间刷新。

  // 单来源模式：wall / curb / reference
  if (edge_mode == "wall") {
    if (!wall_line_ptr_) {
      return;
    }
    const std::vector<nav_msgs::msg::Path> path_from_wall_line =
      wall_line_ptr_->get_compare_result(input_path);
    updateWallLineVec(
      path_from_wall_line,
      input_path,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      robot_pose);
    return;
  }
  if (edge_mode == "curb") {
    pruneExpiredCurbLinePath();
    updateCurbLineVec(
      curb_line_path_,
      input_path,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      robot_pose);
    return;
  }
  if (edge_mode == "reference") {
    const std::vector<nav_msgs::msg::Path> active_reference_path_list =
      snapshotActiveReferencePathList();
    updateReferenceLineVec(
      active_reference_path_list,
      input_path,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      robot_pose);
    return;
  }
  // 融合模式 A：墙线 + 参考线（无路沿）
  if (edge_mode_includes_wall && edge_mode_includes_reference && !edge_mode_includes_curb) {
    // 融合模式：主来源优先使用墙线（若可用），再与参考路径按几何约束融合。
    if (!wall_line_ptr_) {
      std::vector<nav_msgs::msg::Path> active_reference_paths =
        snapshotActiveReferencePathList();
      Eigen::Vector2d selected_reference_segment_start;
      Eigen::Vector2d selected_reference_segment_end;
      double reference_min_avg_dist = 0.0;
      double reference_robot_dist_to_edge = 0.0;
      if (!edgeFollowingEntryGuards(robot_pose, input_path)) {
        return;
      }
      const bool reference_selected = trySelectBestReferencePathFromList(
        active_reference_paths,
        input_path,
        robot_pose,
        cfg_->wall_line.parallel_tolerance,
        cfg_->wall_line.distance_tolerance,
        selected_reference_segment_start,
        selected_reference_segment_end,
        reference_min_avg_dist,
        reference_robot_dist_to_edge);
      if (reference_selected) {
        applyWallLineSegmentAndVisual(
          selected_reference_segment_start,
          selected_reference_segment_end,
          input_path,
          reference_min_avg_dist,
          reference_robot_dist_to_edge);
        cacheReferenceLineHold(
          selected_reference_segment_start,
          selected_reference_segment_end,
          reference_min_avg_dist,
          reference_robot_dist_to_edge,
          clock_->now());
      } else {
        applyReferenceLineHoldFallbackOrExit(input_path);
      }
      return;
    }
    if (!edgeFollowingEntryGuards(robot_pose, input_path)) {
      return;
    }
    const std::vector<nav_msgs::msg::Path> wall_line_candidate_paths =
      wall_line_ptr_->get_compare_result(input_path);
    Eigen::Vector2d primary_segment_start;
    Eigen::Vector2d primary_segment_end;
    double primary_min_avg_dist = 0.0;
    double primary_robot_dist_to_edge = 0.0;
    const bool primary_segment_valid = trySelectWallSegmentFromCandidates(
      wall_line_candidate_paths,
      input_path,
      robot_pose,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      primary_segment_start,
      primary_segment_end,
      primary_min_avg_dist,
      primary_robot_dist_to_edge);
    std::vector<nav_msgs::msg::Path> active_reference_paths = snapshotActiveReferencePathList();
    Eigen::Vector2d reference_segment_start;
    Eigen::Vector2d reference_segment_end;
    double reference_min_avg_dist = 0.0;
    double reference_robot_dist_to_edge = 0.0;
    const bool reference_segment_valid = trySelectBestReferencePathFromList(
      active_reference_paths,
      input_path,
      robot_pose,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      reference_segment_start,
      reference_segment_end,
      reference_min_avg_dist,
      reference_robot_dist_to_edge);
    mergeFusionPrimaryWithReference(
      primary_segment_start,
      primary_segment_end,
      primary_segment_valid,
      primary_min_avg_dist,
      primary_robot_dist_to_edge,
      reference_segment_start,
      reference_segment_end,
      reference_segment_valid,
      reference_min_avg_dist,
      reference_robot_dist_to_edge,
      input_path);
    return;
  }
  // 融合模式 B：路沿 + 参考线（无墙线）
  if (edge_mode_includes_curb && edge_mode_includes_reference && !edge_mode_includes_wall) {
    pruneExpiredCurbLinePath();
    if (!edgeFollowingEntryGuards(robot_pose, input_path)) {
      RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[runEdgeFollowingPathUpdate] Edge following:" 
                          "贴边检查失败, 退出贴边模式");
      return;
    }
    nav_msgs::msg::Path curb_line_path_snapshot;
    {
      std::lock_guard<std::mutex> curb_line_mutex_lock(curb_line_mutex_);
      curb_line_path_snapshot = curb_line_path_;
    }
    Eigen::Vector2d primary_curb_segment_start;
    Eigen::Vector2d primary_curb_segment_end;
    double primary_curb_min_avg_dist = 0.0;
    double primary_curb_robot_dist_to_edge = 0.0;
    const bool primary_curb_segment_valid = trySelectSegmentFromTwoPointPath(
      curb_line_path_snapshot,
      input_path,
      robot_pose,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      primary_curb_segment_start,
      primary_curb_segment_end,
      primary_curb_min_avg_dist,
      primary_curb_robot_dist_to_edge);
    std::vector<nav_msgs::msg::Path> active_reference_paths = snapshotActiveReferencePathList();
    Eigen::Vector2d reference_segment_start;
    Eigen::Vector2d reference_segment_end;
    double reference_min_avg_dist = 0.0;
    double reference_robot_dist_to_edge = 0.0;
    const bool reference_segment_valid = trySelectBestReferencePathFromList(
      active_reference_paths,
      input_path,
      robot_pose,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      reference_segment_start,
      reference_segment_end,
      reference_min_avg_dist,
      reference_robot_dist_to_edge);
    mergeFusionPrimaryWithReference(
      primary_curb_segment_start,
      primary_curb_segment_end,
      primary_curb_segment_valid,
      primary_curb_min_avg_dist,
      primary_curb_robot_dist_to_edge,
      reference_segment_start,
      reference_segment_end,
      reference_segment_valid,
      reference_min_avg_dist,
      reference_robot_dist_to_edge,
      input_path);
    return;
  }

  RCLCPP_WARN_THROTTLE(
    logger_,
    *(clock_),
    5000,
    "USE_CURB_OR_WALL=\"%s\" (规范化 \"%s\") 不是支持的贴边模式",
    detail::useCurbOrWallEnv() != nullptr ? detail::useCurbOrWallEnv() : "(null)",
    edge_mode.c_str());
}


}  // namespace teb_local_planner
