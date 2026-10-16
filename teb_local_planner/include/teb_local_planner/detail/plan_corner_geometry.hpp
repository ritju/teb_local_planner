#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "teb_local_planner/misc.h"

namespace teb_local_planner
{
namespace detail
{
inline double yawAbsDiff(const double yaw_a, const double yaw_b)
{
  double d = yaw_b - yaw_a;
  while (d > M_PI) {
    d -= 2.0 * M_PI;
  }
  while (d < -M_PI) {
    d += 2.0 * M_PI;
  }
  return std::fabs(d);
}

// Interior polyline angle at idx using vertices idx-span, idx, idx+span (same as corner detection).
// Straight ≈ 180°, sharp turn is smaller. Returns 180 if the vertex cannot be evaluated.
inline double polylineVertexAngleDeg(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const int idx,
  const int span = 1)
{
  const int n = static_cast<int>(plan.size());
  const int ia = idx - span;
  const int ic = idx + span;
  if (span < 1 || ia < 0 || ic >= n || idx < 0 || idx >= n) {
    return 180.0;
  }
  const auto & p0 = plan[static_cast<size_t>(idx)].pose.position;
  const auto & p1 = plan[static_cast<size_t>(ia)].pose.position;
  const auto & p2 = plan[static_cast<size_t>(ic)].pose.position;
  const double dx_a = p0.x - p1.x;
  const double dy_a = p0.y - p1.y;
  const double dx_b = p2.x - p0.x;
  const double dy_b = p2.y - p0.y;
  const double dx_c = p2.x - p1.x;
  const double dy_c = p2.y - p1.y;
  const double length_a = std::sqrt(dx_a * dx_a + dy_a * dy_a);
  const double length_b = std::sqrt(dx_b * dx_b + dy_b * dy_b);
  const double length_c = std::sqrt(dx_c * dx_c + dy_c * dy_c);
  constexpr double min_segment_length = 1e-6;
  if (length_a < min_segment_length || length_b < min_segment_length) {
    return 180.0;
  }
  double cos_theta =
    (length_a * length_a + length_b * length_b - length_c * length_c) /
    (2.0 * length_a * length_b);
  cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
  return std::acos(cos_theta) * 180.0 / M_PI;
}

inline int lastIdxWithinArc(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const int start_idx,
  const double max_arc)
{
  const int n = static_cast<int>(plan.size());
  if (start_idx < 0 || start_idx >= n) {
    return start_idx;
  }
  if (max_arc < 0.0) {
    return n - 1;
  }
  double acc = 0.0;
  int idx = start_idx;
  for (int j = start_idx; j < n; ++j) {
    idx = j;
    if (j > start_idx) {
      acc += distance_points2d(
        plan[static_cast<size_t>(j - 1)].pose.position,
        plan[static_cast<size_t>(j)].pose.position);
    }
    if (acc + 1e-9 >= max_arc) {
      break;
    }
  }
  return idx;
}

inline int findFirstSharpCornerIdx(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const int start_idx,
  const double thresh_deg,
  const double max_arc)
{
  const int n = static_cast<int>(plan.size());
  if (start_idx < 0 || start_idx >= n) {
    return -1;
  }
  double acc = 0.0;
  for (int j = start_idx + 1; j < n; ++j) {
    acc += distance_points2d(
      plan[static_cast<size_t>(j - 1)].pose.position,
      plan[static_cast<size_t>(j)].pose.position);
    if (max_arc >= 0.0 && acc > max_arc + 1e-9) {
      break;
    }
    if (polylineVertexAngleDeg(plan, j) < thresh_deg) {
      return j;
    }
  }
  return -1;
}

inline int findCurrentRowStartIdx(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const int start_idx,
  const double thresh_deg)
{
  const int n = static_cast<int>(plan.size());
  if (start_idx < 0 || start_idx >= n) {
    return start_idx;
  }
  for (int j = start_idx - 1; j >= 0; --j) {
    if (polylineVertexAngleDeg(plan, j) < thresh_deg) {
      return std::min(j + 1, start_idx);
    }
  }
  return 0;
}

inline double planArcLength(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const int from_idx,
  const int to_idx)
{
  const int n = static_cast<int>(plan.size());
  if (from_idx < 0 || to_idx < from_idx || to_idx >= n) {
    return 0.0;
  }
  double acc = 0.0;
  for (int j = from_idx + 1; j <= to_idx; ++j) {
    acc += distance_points2d(
      plan[static_cast<size_t>(j - 1)].pose.position,
      plan[static_cast<size_t>(j)].pose.position);
  }
  return acc;
}

inline int findEuclideanClosestPlanIdx(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const double sq_dist_threshold,
  const double max_accum_closest_search)
{
  const int n = static_cast<int>(plan.size());
  if (n <= 0) {
    return 0;
  }
  int i = 0;
  double sq_dist = 1e10;
  bool robot_reached = false;
  double accum_path_from_plan_start = 0.0;
  for (int j = 0; j < n; ++j) {
    if (j > 0) {
      accum_path_from_plan_start += distance_points2d(
        plan[static_cast<size_t>(j - 1)].pose.position,
        plan[static_cast<size_t>(j)].pose.position);
    }
    if (max_accum_closest_search > 1e-9 &&
      accum_path_from_plan_start > max_accum_closest_search + 1e-9) {
      break;
    }
    const double x_diff = robot_pose.pose.position.x - plan[static_cast<size_t>(j)].pose.position.x;
    const double y_diff = robot_pose.pose.position.y - plan[static_cast<size_t>(j)].pose.position.y;
    const double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
    if (new_sq_dist > sq_dist_threshold) {
      break;
    }
    if (robot_reached && new_sq_dist > sq_dist) {
      break;
    }
    if (new_sq_dist < sq_dist) {
      sq_dist = new_sq_dist;
      i = j;
      if (sq_dist < 0.25) {
        robot_reached = true;
      }
    }
  }
  return i;
}

inline int clampClosestIdxBeforeUnprunedCorner(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const int closest_idx,
  const double thresh_deg)
{
  const int n = static_cast<int>(plan.size());
  if (closest_idx <= 0 || closest_idx >= n) {
    return std::max(0, std::min(closest_idx, std::max(0, n - 1)));
  }
  int first_corner = -1;
  for (int j = 1; j < closest_idx; ++j) {
    if (polylineVertexAngleDeg(plan, j) < thresh_deg) {
      first_corner = j;
      break;
    }
  }
  if (first_corner < 0) {
    return closest_idx;
  }
  int i = 0;
  double sq_dist = 1e10;
  for (int j = 0; j <= first_corner; ++j) {
    const double x_diff =
      robot_pose.pose.position.x - plan[static_cast<size_t>(j)].pose.position.x;
    const double y_diff =
      robot_pose.pose.position.y - plan[static_cast<size_t>(j)].pose.position.y;
    const double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
    if (new_sq_dist < sq_dist) {
      sq_dist = new_sq_dist;
      i = j;
    }
  }
  return i;
}

inline int lockedCornerIdx(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const int closest_idx,
  const double thresh_deg,
  const double max_arc)
{
  int search_start = closest_idx;
  if (closest_idx > 0 &&
      polylineVertexAngleDeg(plan, closest_idx) < thresh_deg) {
    search_start = closest_idx - 1;
  }
  return findFirstSharpCornerIdx(plan, search_start, thresh_deg, max_arc);
}

}  // namespace detail
}  // namespace teb_local_planner
