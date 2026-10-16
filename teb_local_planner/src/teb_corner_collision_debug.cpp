/*********************************************************************
 * teb_corner_collision_debug.cpp
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


using nav2_util::declare_parameter_if_not_declared;

namespace teb_local_planner
{
namespace {
const char * footprintCostName(double cost)
{
  using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  using nav2_costmap_2d::LETHAL_OBSTACLE;
  using nav2_costmap_2d::NO_INFORMATION;
  const int c = static_cast<int>(std::lround(cost));
  if (c == static_cast<int>(LETHAL_OBSTACLE)) {
    return "LETHAL";
  }
  if (c == static_cast<int>(INSCRIBED_INFLATED_OBSTACLE)) {
    return "INSCRIBED";
  }
  if (c == static_cast<int>(NO_INFORMATION)) {
    return "NO_INFORMATION";
  }
  if (c < 0) {
    return "INVALID";
  }
  return "COST";
}

double pathLengthToIndex(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const size_t end_index)
{
  
  if (plan.empty()) {
    return 0.0;
  }
  const size_t last_seg = std::min(end_index, plan.size() - 1);
  double len = 0.0;
  for (size_t i = 0; i < last_seg; ++i) {
    const double dx = plan[i + 1].pose.position.x - plan[i].pose.position.x;
    const double dy = plan[i + 1].pose.position.y - plan[i].pose.position.y;
    len += std::hypot(dx, dy);
  }
  return len;
}

/** 弧长 dist 处：取已驶过的最后一个离散路径点下标 k（cum[k]≤dist），朝向用 plan[k]（无 slerp） */
size_t waypointIndexOwningArcLength(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const size_t end_index,
  const double dist)
{
  
  if (plan.empty() || end_index >= plan.size()) {
    return 0;
  }
  if (dist <= 0.0) {
    return 0;
  }
  double cum_at_k = 0.0;
  size_t k = 0;
  while (k + 1 <= end_index) {
    const double dx = plan[k + 1].pose.position.x - plan[k].pose.position.x;
    const double dy = plan[k + 1].pose.position.y - plan[k].pose.position.y;
    const double seg = std::hypot(dx, dy);
    if (cum_at_k + seg <= dist + 1e-9) {
      cum_at_k += seg;
      ++k;
    } else {
      break;
    }
  }
  return k;
}

bool interpolatePoseAlongPlanToIndex(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const size_t end_index,
  const double distance_along,
  geometry_msgs::msg::PoseStamped * out)
{
  
  if (!out || plan.empty() || end_index >= plan.size()) {
    return false;
  }
  if (distance_along <= 0.0) {
    *out = plan[0];
    return true;
  }
  const double total = pathLengthToIndex(plan, end_index);
  const double dist = std::min(std::max(0.0, distance_along), total);
  if (dist >= total - 1e-9) {
    *out = plan[end_index];
    return true;
  }
  double accumulated = 0.0;
  for (size_t i = 0; i < end_index; ++i) {
    const double ax = plan[i].pose.position.x;
    const double ay = plan[i].pose.position.y;
    const double bx = plan[i + 1].pose.position.x;
    const double by = plan[i + 1].pose.position.y;
    const double dx = bx - ax;
    const double dy = by - ay;
    const double seg_len = std::hypot(dx, dy);
    if (seg_len < 1e-9) {
      continue;
    }
    if (accumulated + seg_len >= dist - 1e-9) {
      const double t = (dist - accumulated) / seg_len;
      out->header = plan[i].header;
      out->pose.position.x = ax + t * dx;
      out->pose.position.y = ay + t * dy;
      out->pose.position.z =
        plan[i].pose.position.z + t * (plan[i + 1].pose.position.z - plan[i].pose.position.z);
      const size_t orient_idx = waypointIndexOwningArcLength(plan, end_index, dist);
      out->pose.orientation = plan[orient_idx].pose.orientation;
      return true;
    }
    accumulated += seg_len;
  }
  *out = plan[end_index];
  return true;
}

}  // namespace

bool TebLocalPlannerROS::orientedFootprintAnyVertexOutsideLocalCostmap(
  double x, double y, double theta,
  const std::vector<geometry_msgs::msg::Point> & footprint_poly) const
{
  
  if (!costmap_ || footprint_poly.size() < 2) {
    return true;
  }
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  unsigned int mx = 0, my = 0;
  for (const auto & p : footprint_poly) {
    const double wx = x + (p.x * c - p.y * s);
    const double wy = y + (p.x * s + p.y * c);
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      return true;
    }
  }
  return false;
}

bool TebLocalPlannerROS::cornerApproachCostIsBlocking(
  unsigned char cost,
  const std::string& cost_mode) const
{
  using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  using nav2_costmap_2d::LETHAL_OBSTACLE;

  if (cost_mode == "lethal") {
    return cost == LETHAL_OBSTACLE;
  }
  if (cost_mode == "inscribed") {
    return cost == INSCRIBED_INFLATED_OBSTACLE;
  }
  return cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE;
}

double TebLocalPlannerROS::cornerApproachCheckMargin() const
{
  if (!cfg_) {
    return 0.0;
  }
  const double configured = cfg_->trajectory.corner_approach_check_margin;
  if (configured < 0.0) {
    return cfg_->obstacles.min_obstacle_dist;
  }
  return configured;
}

std::vector<geometry_msgs::msg::Point> TebLocalPlannerROS::cornerApproachFootprintSpec() const
{
  std::vector<geometry_msgs::msg::Point> footprint =
    (footprint_spec_.size() >= 3) ?
    footprint_spec_ :
    (costmap_ros_ ? costmap_ros_->getRobotFootprint() :
    std::vector<geometry_msgs::msg::Point>{});

  const double margin = cornerApproachCheckMargin();
  if (margin > 1e-9 && footprint.size() >= 3) {
    nav2_costmap_2d::padFootprint(footprint, margin);
  }
  return footprint;
}

bool TebLocalPlannerROS::checkCornerFootprintPoseBlocked(
  const geometry_msgs::msg::Pose2D& pose2d,
  const std::string& cost_mode,
  unsigned char& hit_cost_out) const
{
  if (!costmap_model_) {
    return false;
  }

  const std::vector<geometry_msgs::msg::Point> corner_footprint =
    cornerApproachFootprintSpec();
  if (corner_footprint.size() < 3) {
    return false;
  }

  try {
    const unsigned char cost = costmap_model_->scorePose(
      pose2d, dwb_critics::getOrientedFootprint(pose2d, corner_footprint));
    if (cornerApproachCostIsBlocking(cost, cost_mode)) {
      hit_cost_out = cost;
      publishCollisionFootprintDebug(
        "collision_corner", "", pose2d.x, pose2d.y, pose2d.theta,
        corner_footprint, static_cast<double>(cost));
      return true;
    }
  } catch (const dwb_core::IllegalTrajectoryException& e) {
    if (std::strcmp(e.what(), "Trajectory Hits Obstacle.") == 0) {
      const unsigned char lethal = nav2_costmap_2d::LETHAL_OBSTACLE;
      if (cornerApproachCostIsBlocking(lethal, cost_mode)) {
        hit_cost_out = lethal;
        publishCollisionFootprintDebug(
          "collision_corner", "", pose2d.x, pose2d.y, pose2d.theta,
          corner_footprint, static_cast<double>(lethal));
        return true;
      }
    }
  }
  return false;
}

bool TebLocalPlannerROS::checkCornerApproachRayBlocked(
  const geometry_msgs::msg::Pose2D& robot_pose_local,
  const geometry_msgs::msg::Pose2D& corner_pose_local,
  double sample_spacing,
  const std::string& cost_mode,
  unsigned char& hit_cost_out,
  int* blocked_sample_index_out)
{
  const std::vector<geometry_msgs::msg::Pose2D> sample_poses =
    buildCornerApproachCheckPoses(
      robot_pose_local, corner_pose_local, true, sample_spacing);

  for (std::size_t i = 0; i < sample_poses.size(); ++i) {
    if (checkCornerFootprintPoseBlocked(sample_poses[i], cost_mode, hit_cost_out)) {
      if (blocked_sample_index_out != nullptr) {
        *blocked_sample_index_out = static_cast<int>(i);
      }
      return true;
    }
  }
  clearCollisionFootprintDebug("collision_corner", "");
  return false;
}

std::vector<geometry_msgs::msg::Pose2D> TebLocalPlannerROS::buildCornerApproachCheckPoses(
  const geometry_msgs::msg::Pose2D& robot_pose_local,
  const geometry_msgs::msg::Pose2D& corner_pose_local,
  const bool ray_check_enable,
  const double sample_spacing) const
{
  std::vector<geometry_msgs::msg::Pose2D> sample_poses;
  const double dx = corner_pose_local.x - robot_pose_local.x;
  const double dy = corner_pose_local.y - robot_pose_local.y;
  const double direction_to_corner = std::atan2(dy, dx);

  geometry_msgs::msg::Pose2D sample_pose;
  sample_pose.theta = direction_to_corner;

  if (!ray_check_enable) {
    sample_pose.x = corner_pose_local.x;
    sample_pose.y = corner_pose_local.y;
    sample_poses.push_back(sample_pose);
    return sample_poses;
  }

  const double segment_len = std::hypot(dx, dy);
  if (segment_len <= 1e-6) {
    sample_pose.x = corner_pose_local.x;
    sample_pose.y = corner_pose_local.y;
    sample_poses.push_back(sample_pose);
    return sample_poses;
  }

  const double spacing = std::max(1e-3, sample_spacing);
  std::vector<double> arc_samples;
  for (double s = 0.0; s < segment_len; s += spacing) {
    arc_samples.push_back(s);
  }
  if (arc_samples.empty() || std::abs(arc_samples.back() - segment_len) > 1e-3) {
    arc_samples.push_back(segment_len);
  }

  sample_poses.reserve(arc_samples.size());
  for (const double s : arc_samples) {
    const double t = s / segment_len;
    sample_pose.x = robot_pose_local.x + t * dx;
    sample_pose.y = robot_pose_local.y + t * dy;
    sample_poses.push_back(sample_pose);
  }
  return sample_poses;
}

void TebLocalPlannerROS::publishCornerApproachFootprintMarkers(
  const std::string& frame_id,
  const geometry_msgs::msg::Pose2D& robot_pose_local,
  const geometry_msgs::msg::Pose2D& corner_pose_local,
  const bool ray_check_enable,
  const double sample_spacing,
  const int blocked_sample_index,
  const unsigned char hit_cost)
{
  if (!corner_approach_footprint_marker_pub_ ||
    !cfg_->trajectory.corner_approach_footprint_marker_enable)
  {
    return;
  }

  const std::vector<geometry_msgs::msg::Point> footprint_spec =
    cornerApproachFootprintSpec();
  if (footprint_spec.size() < 3) {
    return;
  }

  const std::vector<geometry_msgs::msg::Pose2D> sample_poses =
    buildCornerApproachCheckPoses(
      robot_pose_local, corner_pose_local, ray_check_enable, sample_spacing);
  if (sample_poses.empty()) {
    clearCornerApproachFootprintMarkers(frame_id);
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const rclcpp::Time stamp = clock_->now();
  int marker_id = 0;

  if (ray_check_enable) {
    visualization_msgs::msg::Marker ray_marker;
    ray_marker.header.frame_id = frame_id;
    ray_marker.header.stamp = stamp;
    ray_marker.ns = "corner_approach_ray";
    ray_marker.id = 0;
    ray_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ray_marker.action = visualization_msgs::msg::Marker::ADD;
    ray_marker.scale.x = 0.04;
    ray_marker.color.r = 1.0f;
    ray_marker.color.g = 0.85f;
    ray_marker.color.b = 0.0f;
    ray_marker.color.a = 0.9f;
    ray_marker.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point p0;
    p0.x = robot_pose_local.x;
    p0.y = robot_pose_local.y;
    p0.z = 0.03;
    geometry_msgs::msg::Point p1;
    p1.x = corner_pose_local.x;
    p1.y = corner_pose_local.y;
    p1.z = 0.03;
    ray_marker.points.push_back(p0);
    ray_marker.points.push_back(p1);
    marker_array.markers.push_back(ray_marker);
    marker_id = 1;
  }

  const bool corner_only = !ray_check_enable && sample_poses.size() == 1;
  for (std::size_t i = 0; i < sample_poses.size(); ++i) {
    const dwb_critics::Footprint oriented_footprint =
      dwb_critics::getOrientedFootprint(sample_poses[i], footprint_spec);
    if (oriented_footprint.size() < 3) {
      continue;
    }

    visualization_msgs::msg::Marker footprint_marker;
    footprint_marker.header.frame_id = frame_id;
    footprint_marker.header.stamp = stamp;
    footprint_marker.ns = corner_only ?
      "corner_approach_corner_footprint" : "corner_approach_ray_footprint";
    footprint_marker.id = marker_id + static_cast<int>(i);
    footprint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    footprint_marker.action = visualization_msgs::msg::Marker::ADD;
    footprint_marker.scale.x = 0.03;
    footprint_marker.pose.orientation.w = 1.0;

    const bool is_blocked_sample =
      static_cast<int>(i) == blocked_sample_index && hit_cost != 0;
    if (is_blocked_sample) {
      footprint_marker.color.r = 1.0f;
      footprint_marker.color.g = 0.1f;
      footprint_marker.color.b = 0.1f;
    } else if (corner_only) {
      footprint_marker.color.r = 0.2f;
      footprint_marker.color.g = 0.7f;
      footprint_marker.color.b = 1.0f;
    } else {
      footprint_marker.color.r = 0.1f;
      footprint_marker.color.g = 0.9f;
      footprint_marker.color.b = 0.3f;
    }
    footprint_marker.color.a = 0.85f;

    for (const auto& pt : oriented_footprint) {
      geometry_msgs::msg::Point mp;
      mp.x = pt.x;
      mp.y = pt.y;
      mp.z = 0.02;
      footprint_marker.points.push_back(mp);
    }
    footprint_marker.points.push_back(footprint_marker.points.front());
    marker_array.markers.push_back(footprint_marker);

    if (is_blocked_sample) {
      visualization_msgs::msg::Marker text;
      text.header.frame_id = frame_id;
      text.header.stamp = stamp;
      text.ns = "corner_approach_hit_cost";
      text.id = 0;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = sample_poses[i].x;
      text.pose.position.y = sample_poses[i].y;
      text.pose.position.z = 0.45;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.22;
      text.color.r = 1.0f;
      text.color.g = 0.85f;
      text.color.b = 0.1f;
      text.color.a = 1.0f;
      text.text = std::string("corner cost=") + std::to_string(static_cast<int>(hit_cost)) +
        " " + footprintCostName(static_cast<double>(hit_cost));
      marker_array.markers.push_back(text);
    }
  }

  const int current_marker_count = marker_id + static_cast<int>(sample_poses.size());
  for (int id = current_marker_count; id < last_published_corner_footprint_marker_count_; ++id) {
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = stamp;
    delete_marker.ns = "corner_approach_ray_footprint";
    delete_marker.id = id;
    delete_marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(delete_marker);

    visualization_msgs::msg::Marker delete_corner_marker = delete_marker;
    delete_corner_marker.ns = "corner_approach_corner_footprint";
    marker_array.markers.push_back(delete_corner_marker);
  }

  last_published_corner_footprint_marker_count_ = current_marker_count;
  corner_approach_footprint_marker_pub_->publish(marker_array);
}

void TebLocalPlannerROS::clearCornerApproachFootprintMarkers(const std::string& frame_id)
{
  if (!corner_approach_footprint_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const rclcpp::Time stamp = clock_->now();

  visualization_msgs::msg::Marker delete_ray;
  delete_ray.header.frame_id = frame_id;
  delete_ray.header.stamp = stamp;
  delete_ray.ns = "corner_approach_ray";
  delete_ray.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_ray);

  visualization_msgs::msg::Marker delete_ray_fp;
  delete_ray_fp.header.frame_id = frame_id;
  delete_ray_fp.header.stamp = stamp;
  delete_ray_fp.ns = "corner_approach_ray_footprint";
  delete_ray_fp.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_ray_fp);

  visualization_msgs::msg::Marker delete_corner_fp;
  delete_corner_fp.header.frame_id = frame_id;
  delete_corner_fp.header.stamp = stamp;
  delete_corner_fp.ns = "corner_approach_corner_footprint";
  delete_corner_fp.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_corner_fp);

  visualization_msgs::msg::Marker delete_hit_text;
  delete_hit_text.header.frame_id = frame_id;
  delete_hit_text.header.stamp = stamp;
  delete_hit_text.ns = "corner_approach_hit_cost";
  delete_hit_text.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_hit_text);

  corner_approach_footprint_marker_pub_->publish(marker_array);
  last_published_corner_footprint_marker_count_ = 0;
}

std::string TebLocalPlannerROS::collisionDebugFrameId(const std::string & preferred_frame) const
{
  if (!preferred_frame.empty()) {
    return preferred_frame;
  }
  if (costmap_ros_) {
    return costmap_ros_->getGlobalFrameID();
  }
  return cfg_ ? cfg_->map_frame : std::string("map");
}

void TebLocalPlannerROS::clearCollisionFootprintDebug(
  const char * ns, const std::string & frame_id) const
{
  if (!collision_footprint_marker_pub_ || ns == nullptr) {
    return;
  }
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker del;
  del.header.frame_id = collisionDebugFrameId(frame_id);
  del.header.stamp = clock_->now();
  del.ns = ns;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(del);
  collision_footprint_marker_pub_->publish(marker_array);
}

void TebLocalPlannerROS::publishCollisionFootprintDebug(
  const char * ns,
  const std::string & frame_id,
  const double x, const double y, const double yaw,
  const std::vector<geometry_msgs::msg::Point> & footprint_poly,
  const double cost) const
{
  const int cost_i = static_cast<int>(std::lround(cost));
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 500,
    "碰撞检查失败 [%s] cost=%d (%s) pose=(%.3f, %.3f, %.3f rad)",
    ns ? ns : "?", cost_i, footprintCostName(cost), x, y, yaw);

  if (!collision_footprint_marker_pub_ || ns == nullptr || footprint_poly.size() < 3) {
    return;
  }

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = x;
  pose2d.y = y;
  pose2d.theta = yaw;
  const dwb_critics::Footprint oriented =
    dwb_critics::getOrientedFootprint(pose2d, footprint_poly);
  if (oriented.size() < 3) {
    return;
  }

  const std::string frame = collisionDebugFrameId(frame_id);
  const rclcpp::Time stamp = clock_->now();
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker del;
  del.header.frame_id = frame;
  del.header.stamp = stamp;
  del.ns = ns;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(del);

  visualization_msgs::msg::Marker footprint_marker;
  footprint_marker.header.frame_id = frame;
  footprint_marker.header.stamp = stamp;
  footprint_marker.ns = ns;
  footprint_marker.id = 0;
  footprint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  footprint_marker.action = visualization_msgs::msg::Marker::ADD;
  footprint_marker.scale.x = 0.04;
  footprint_marker.pose.orientation.w = 1.0;
  footprint_marker.color.r = 1.0f;
  footprint_marker.color.g = 0.15f;
  footprint_marker.color.b = 0.05f;
  footprint_marker.color.a = 0.95f;
  double cx = 0.0;
  double cy = 0.0;
  for (const auto & pt : oriented) {
    geometry_msgs::msg::Point mp;
    mp.x = pt.x;
    mp.y = pt.y;
    mp.z = 0.04;
    footprint_marker.points.push_back(mp);
    cx += pt.x;
    cy += pt.y;
  }
  footprint_marker.points.push_back(footprint_marker.points.front());
  cx /= static_cast<double>(oriented.size());
  cy /= static_cast<double>(oriented.size());
  marker_array.markers.push_back(footprint_marker);

  visualization_msgs::msg::Marker text;
  text.header.frame_id = frame;
  text.header.stamp = stamp;
  text.ns = ns;
  text.id = 1;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.pose.position.x = cx;
  text.pose.position.y = cy;
  text.pose.position.z = 0.45;
  text.pose.orientation.w = 1.0;
  text.scale.z = 0.22;
  text.color.r = 1.0f;
  text.color.g = 0.85f;
  text.color.b = 0.1f;
  text.color.a = 1.0f;
  text.text = std::string(ns) + " cost=" + std::to_string(cost_i) + " " + footprintCostName(cost);
  marker_array.markers.push_back(text);

  collision_footprint_marker_pub_->publish(marker_array);
}

bool TebLocalPlannerROS::isTransformedPlanFootprintSamplesCollisionFree(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const double sample_spacing_m) const
{
  
  if (!rotation_collision_checker_ || plan.empty()) {
    return true;
  }
  const size_t end_idx = plan.size() - 1;

  const std::vector<geometry_msgs::msg::Point> & footprint_poly =
    (footprint_spec_.size() >= 3) ? footprint_spec_ : costmap_ros_->getRobotFootprint();

  const std::string& cost_mode = cfg_->rotation.path_footprint_check_cost_mode;
  const std::string debug_frame = collisionDebugFrameId(plan.front().header.frame_id);
  constexpr const char * kCollisionNs = "collision_path_sample";

  const auto pose_footprint_clear = [&](double px, double py, double pyaw) -> bool {
    if (orientedFootprintAnyVertexOutsideLocalCostmap(px, py, pyaw, footprint_poly)) {
      return true;
    }
    const double fc = rotation_collision_checker_->footprintCostAtPose(
      px, py, pyaw, footprint_poly);
    if (cornerApproachCostIsBlocking(static_cast<unsigned char>(fc), cost_mode)) {
      publishCollisionFootprintDebug(kCollisionNs, debug_frame, px, py, pyaw, footprint_poly, fc);
      return false;
    }
    return true;
  };

  const double total_len = pathLengthToIndex(plan, end_idx);
  double forward_cap = cfg_->trajectory.max_global_plan_lookahead_dist;
  if (forward_cap <= 1e-9) {
    forward_cap = total_len;
  }
  const double sample_len = std::min(total_len, forward_cap);
  double spacing = std::max(1e-3, sample_spacing_m);
  if (sample_len > 1e-9) {
    spacing = std::max(spacing, sample_len / 150.0);
  }

  std::vector<double> dists;
  for (double d = 0.0; d < sample_len + 1e-9; d += spacing) {
    dists.push_back(std::min(d, sample_len));
  }
  if (dists.empty()) {
    dists.push_back(0.0);
  }
  if (sample_len > 1e-9 &&
      std::abs(dists.back() - sample_len) > 1e-3) {
    dists.push_back(sample_len);
  }

  geometry_msgs::msg::PoseStamped sampled;
  for (const double d : dists) {
    if (!interpolatePoseAlongPlanToIndex(plan, end_idx, d, &sampled)) {
      continue;
    }
    const double yaw = tf2::getYaw(sampled.pose.orientation);
    if (!pose_footprint_clear(sampled.pose.position.x, sampled.pose.position.y, yaw)) {
      return false;
    }
  }
  clearCollisionFootprintDebug(kCollisionNs, debug_frame);
  return true;
}

}  // namespace teb_local_planner
