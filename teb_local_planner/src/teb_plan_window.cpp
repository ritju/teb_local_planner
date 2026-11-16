/*********************************************************************
 * teb_plan_window.cpp
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
#include "dwb_critics/obstacle_footprint.hpp"
#include "nav2_util/robot_utils.hpp"
#include "teb_local_planner/misc.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <exception>
#include <limits>
#include <utility>
#include <map>
#include <sstream>

#include "teb_local_planner/detail/plan_corner_geometry.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace teb_local_planner
{
using detail::yawAbsDiff;
using detail::polylineVertexAngleDeg;
using detail::lastIdxWithinArc;
using detail::findFirstSharpCornerIdx;
using detail::findCurrentRowStartIdx;
using detail::planArcLength;
using detail::findEuclideanClosestPlanIdx;
using detail::clampClosestIdxBeforeUnprunedCorner;
using detail::lockedCornerIdx;


namespace {
bool footprintFullyInsideCostmap(
  const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint)
{
  if (footprint.size() < 3) {
    return false;
  }
  for (const auto & point : footprint) {
    unsigned int x, y;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !map.worldToMap(point.x, point.y, x, y)) {
      return false;
    }
  }
  return true;
}

// Rasterize the visible intersection, including the interior, rather than only
// the outline. Leaving the map is not itself an occupied observation.
std::vector<nav2_costmap_2d::MapLocation> visibleFootprintCells(
  nav2_costmap_2d::Costmap2D & map,
  std::vector<geometry_msgs::msg::Point> polygon)
{
  std::vector<nav2_costmap_2d::MapLocation> cells;
  if (polygon.size() < 3 || map.getSizeInCellsX() == 0 || map.getSizeInCellsY() == 0) {
    return cells;
  }
  for (const auto & p : polygon) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      return cells;
    }
  }
  const double min_x = map.getOriginX();
  const double min_y = map.getOriginY();
  const double max_x = std::nextafter(min_x + map.getSizeInCellsX() * map.getResolution(), min_x);
  const double max_y = std::nextafter(min_y + map.getSizeInCellsY() * map.getResolution(), min_y);
  // Sutherland-Hodgman clipping against the four sides of the map rectangle.
  for (int side = 0; side < 4 && !polygon.empty(); ++side) {
    const bool use_x = side < 2;
    const bool lower = side % 2 == 0;
    const double limit = use_x ? (lower ? min_x : max_x) : (lower ? min_y : max_y);
    auto coordinate = [use_x](const geometry_msgs::msg::Point & p) {return use_x ? p.x : p.y;};
    auto inside = [&](const geometry_msgs::msg::Point & p) {
      return lower ? coordinate(p) >= limit : coordinate(p) <= limit;
    };
    std::vector<geometry_msgs::msg::Point> clipped;
    auto previous = polygon.back();
    for (const auto & point : polygon) {
      const bool previous_inside = inside(previous);
      const bool point_inside = inside(point);
      if (previous_inside != point_inside) {
        const double ratio = (limit - coordinate(previous)) /
          (coordinate(point) - coordinate(previous));
        auto crossing = previous;
        crossing.x += ratio * (point.x - previous.x);
        crossing.y += ratio * (point.y - previous.y);
        if (use_x) {crossing.x = limit;} else {crossing.y = limit;}
        clipped.push_back(crossing);
      }
      if (point_inside) {clipped.push_back(point);}
      previous = point;
    }
    polygon = std::move(clipped);
  }
  if (polygon.size() < 3) {
    return cells;
  }
  std::vector<nav2_costmap_2d::MapLocation> grid_polygon;
  for (const auto & point : polygon) {
    nav2_costmap_2d::MapLocation cell;
    int x, y;
    map.worldToMapEnforceBounds(point.x, point.y, x, y);
    // Subtraction/division can round the clipped upper boundary back to size.
    // Clamp explicitly even after worldToMapEnforceBounds to keep edge cells valid.
    cell.x = static_cast<unsigned int>(std::clamp(x, 0, static_cast<int>(map.getSizeInCellsX()) - 1));
    cell.y = static_cast<unsigned int>(std::clamp(y, 0, static_cast<int>(map.getSizeInCellsY()) - 1));
    grid_polygon.push_back(cell);
  }
  map.convexFillCells(grid_polygon, cells);
  return cells;
}

bool blockingCost(unsigned char cost)
{
  return cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::NO_INFORMATION;
}

}  // namespace

bool TebLocalPlannerROS::pruneArrivedLockedCorner(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (global_plan_.size() < 3 || cfg_ == nullptr || tf_ == nullptr) {
    return false;
  }

  geometry_msgs::msg::PoseStamped robot_in_plan;
  try {
    geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(
      global_plan_.front().header.frame_id,
      robot_pose.header.frame_id,
      tf2::TimePointZero,
      tf2::durationFromSec(0.5));
    tf2::doTransform(robot_pose, robot_in_plan, global_to_plan_transform);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "到达角点裁剪: 机器人位姿变换到路径坐标系失败: %s", ex.what());
    return false;
  }

  double sq_dist_threshold = 1e10;
  if (costmap_ != nullptr) {
    double dist_threshold = std::max(
      costmap_->getSizeInCellsX() * costmap_->getResolution() / 2.0,
      costmap_->getSizeInCellsY() * costmap_->getResolution() / 2.0);
    dist_threshold *= 0.85;
    sq_dist_threshold = dist_threshold * dist_threshold;
  }

  const double thresh_deg = static_cast<double>(cfg_->trajectory.theta_threshold);
  const double search_arc = cfg_->trajectory.max_global_plan_lookahead_dist > 1e-9
    ? (cfg_->trajectory.max_global_plan_lookahead_dist + 1.0)
    : -1.0;
  const int closest_raw = findEuclideanClosestPlanIdx(
    global_plan_, robot_in_plan, sq_dist_threshold,
    cfg_->trajectory.transform_global_plan_closest_search_max_accum_dist);
  const int closest_idx = clampClosestIdxBeforeUnprunedCorner(
    global_plan_, robot_in_plan, closest_raw, thresh_deg);
  const int corner_idx = lockedCornerIdx(
    global_plan_, closest_idx, thresh_deg, search_arc);
  const int n_plan = static_cast<int>(global_plan_.size());
  const double arrive_dist = cfg_->trajectory.last_corner_record_distance;
  const double arc_to_c = (corner_idx >= 0)
    ? planArcLength(global_plan_, closest_idx, corner_idx)
    : -1.0;
  const double angle_deg = (corner_idx >= 0)
    ? polylineVertexAngleDeg(global_plan_, corner_idx)
    : 180.0;

  const char * skip_reason = nullptr;
  bool arrived = false;
  if (corner_idx < 0) {
    skip_reason = "no_corner";
  } else if (corner_idx >= n_plan - 1) {
    skip_reason = "no_residual";
  } else if (arrive_dist <= 1e-9) {
    skip_reason = "arrive_dist_disabled";
  } else if (arc_to_c > arrive_dist + 1e-9) {
    skip_reason = "arc_to_c";
  } else {
    arrived = true;
  }

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000,
    "[lock_corner] 锁定角点 closest_raw=%d closest=%d C=%d 角度=%.2f 弧长到角点=%.3f "
    "到达阈值=%.3f 已到达=%d 原因=%s",
    closest_raw, closest_idx, corner_idx, angle_deg, arc_to_c, arrive_dist,
    arrived ? 1 : 0, skip_reason != nullptr ? skip_reason : "ok");

  if (!arrived) {
    return false;
  }

  global_plan_.erase(
    global_plan_.begin(),
    global_plan_.begin() + corner_idx + 1);
  RCLCPP_INFO(
    logger_,
    "到达角点裁剪: 角点 idx=%d，删除 [0,%d]，剩余 %zu 点，弧长到角点=%.3f m",
    corner_idx, corner_idx, global_plan_.size(), arc_to_c);
  return true;
}

 bool TebLocalPlannerROS::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, std::vector<geometry_msgs::msg::PoseStamped>& global_plan, double dist_behind_robot, double max_prune_dist)
 {
   
   if (global_plan.empty())
     return true;
   
   try
   {
     // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
     // Use TimePointZero to get the latest available transform to avoid extrapolation errors
     // when global_pose.header.stamp is in the future (especially important for low-frequency transforms like map->odom at 10Hz)
     geometry_msgs::msg::PoseStamped robot;
     try {
       // Get transform using TimePointZero to avoid extrapolation
       geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(
                   global_plan.front().header.frame_id,
                   global_pose.header.frame_id,
                   tf2::TimePointZero,
                   tf2::durationFromSec(0.5));
       
       // Apply transform manually to avoid using global_pose.header.stamp
       tf2::doTransform(global_pose, robot, global_to_plan_transform);
     } catch (const tf2::ExtrapolationException& ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "路径裁剪 TF 外推失败，本周期跳过裁剪: %s", ex.what());
       // If TimePointZero fails with extrapolation, skip pruning this cycle rather than using future time
       // This prevents further extrapolation errors
       return true;
     } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "路径裁剪 TF 查询失败，本周期跳过裁剪: %s", ex.what());
       return true;
     }
     
     double dist_thresh_sq = dist_behind_robot*dist_behind_robot;    
     // iterate plan until a pose close the robot is found
     std::vector<geometry_msgs::msg::PoseStamped>::iterator it = global_plan.begin();
     std::vector<geometry_msgs::msg::PoseStamped>::iterator erase_end = it;
     double accum_dist = 0;
     double min_dist_threshold = std::numeric_limits<double>::max();
     const bool limit_prune_search = (max_prune_dist > 0.0);
     while (it != global_plan.end() && (!limit_prune_search || accum_dist < max_prune_dist))
     {
       double dx = robot.pose.position.x - it->pose.position.x;
       double dy = robot.pose.position.y - it->pose.position.y;
       double dist_sq = dx * dx + dy * dy;
       if (it != global_plan.begin())
       {
         double ddx = it->pose.position.x - (it-1)->pose.position.x;
         double ddy = it->pose.position.y - (it-1)->pose.position.y;
         accum_dist += std::sqrt(ddx*ddx + ddy*ddy);
       }
       if (dist_sq < min_dist_threshold)
       {
         min_dist_threshold = dist_sq;
         erase_end = it;
         if (dist_sq < dist_thresh_sq)
         {
           break;
         }
       }
       if (dist_sq < min_dist_threshold)
       {
         // If close enough, additionally require plan pose orientation to align with robot orientation
         // If yaw misalignment is too large, keep searching forward until we find a close pose with aligned yaw
         double robot_yaw = tf2::getYaw(robot.pose.orientation);
         double pose_yaw = tf2::getYaw(it->pose.orientation);
         double yaw_diff = pose_yaw - robot_yaw;
         while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
         while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

         if (std::fabs(yaw_diff) <= cfg_->trajectory.prune_angle_threshold * M_PI / 180.0)
         {
           erase_end = it;
           break;
         }
         // else: do not break, continue searching forward
       }
       ++it;
       // ++count_it;
     }
 
     if (erase_end == global_plan.end())
       return false;
     
     if (erase_end != global_plan.begin())
       global_plan.erase(global_plan.begin(), erase_end);
   }
   catch (const tf2::TransformException& ex)
   {
     RCLCPP_DEBUG(logger_, "无法裁剪路径，缺少坐标变换: %s\n", ex.what());
     return false;
   }
   return true;
 }

std::vector<geometry_msgs::msg::Point> TebLocalPlannerROS::lateralPaddedFootprintSpec() const
{
  std::vector<geometry_msgs::msg::Point> footprint = footprint_spec_;
  if (footprint.size() < 3) {
    return footprint;
  }
  const double margin = (cfg_ != nullptr) ? cfg_->obstacles.min_obstacle_dist : 0.0;
  if (margin <= 1e-9) {
    return footprint;
  }
  for (auto & p : footprint) {
    if (p.y > 1e-9) {
      p.y += margin;
    } else if (p.y < -1e-9) {
      p.y -= margin;
    }
  }
  return footprint;
}

bool TebLocalPlannerROS::globalPlanPoseLateralPaddedFootprintFreeInControllerFrame(
  const geometry_msgs::msg::PoseStamped& pose_plan_frame,
  const geometry_msgs::msg::TransformStamped& plan_to_global_transform,
  bool require_full_observation) const
{
  geometry_msgs::msg::PoseStamped pose_global;
  tf2::doTransform(pose_plan_frame, pose_global, plan_to_global_transform);
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = pose_global.pose.position.x;
  pose2d.y = pose_global.pose.position.y;
  pose2d.theta = tf2::getYaw(pose_global.pose.orientation);
  const dwb_critics::Footprint oriented_footprint =
    dwb_critics::getOrientedFootprint(pose2d, lateralPaddedFootprintSpec());
  if (require_full_observation) {
    return orientedFootprintFullyObservedAndFree(oriented_footprint);
  }
  return orientedFootprintNavigableOnPartialCostmap(pose2d, oriented_footprint);
}

bool TebLocalPlannerROS::orientedFootprintFullyObservedAndFree(
  const std::vector<geometry_msgs::msg::Point> & footprint) const
{
  if (!costmap_ || !footprintFullyInsideCostmap(*costmap_, footprint)) {
    return false;
  }
  const auto cells = visibleFootprintCells(*costmap_, footprint);
  return !cells.empty() && std::none_of(cells.begin(), cells.end(), [&](const auto & cell) {
    return blockingCost(costmap_->getCost(cell.x, cell.y));
  });
}

bool TebLocalPlannerROS::orientedFootprintNavigableOnPartialCostmap(
  const geometry_msgs::msg::Pose2D & pose2d_controller_frame,
  const std::vector<geometry_msgs::msg::Point> & oriented_footprint) const
{
  (void)pose2d_controller_frame;
  if (!costmap_) {
    return true;
  }
  const auto cells = visibleFootprintCells(*costmap_, oriented_footprint);
  return std::none_of(cells.begin(), cells.end(), [&](const auto & cell) {
    return blockingCost(costmap_->getCost(cell.x, cell.y));
  });
}

bool TebLocalPlannerROS::selectGoalStandoffAlongPlan(
  const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
  const geometry_msgs::msg::TransformStamped& plan_to_global_transform,
  int first_candidate_idx,
  const std::function<bool(int)> & candidate_is_safe,
  int& standoff_idx_out,
  geometry_msgs::msg::PoseStamped& effective_last_plan_pose) const
{
  standoff_idx_out = -1;
  goal_standoff_force_hold_ = false;
  if (global_plan.empty()) {
    goal_standoff_latched_ = false;
    goal_standoff_plan_idx_ = -1;
    effective_last_plan_pose = geometry_msgs::msg::PoseStamped();
    return false;
  }

  effective_last_plan_pose = global_plan.back();
  // Replans and pruning can change indices, but must never replace the locked
  // physical parking point. A missing/blocked point means an immediate hold.
  if (goal_standoff_latched_) {
    goal_standoff_plan_idx_ = -1;
    for (int idx = first_candidate_idx; idx < static_cast<int>(global_plan.size()); ++idx) {
      const auto & candidate = global_plan[static_cast<size_t>(idx)];
      if (candidate.header.frame_id == goal_standoff_pose_plan_.header.frame_id &&
          std::hypot(candidate.pose.position.x - goal_standoff_pose_plan_.pose.position.x,
            candidate.pose.position.y - goal_standoff_pose_plan_.pose.position.y) < 1e-3 &&
          std::fabs(angles::shortest_angular_distance(tf2::getYaw(candidate.pose.orientation),
            tf2::getYaw(goal_standoff_pose_plan_.pose.orientation))) < 1e-3) {
        goal_standoff_plan_idx_ = idx;
        break;
      }
    }
    standoff_idx_out = goal_standoff_plan_idx_;
    tf2::doTransform(goal_standoff_pose_plan_, goal_standoff_pose_global_, plan_to_global_transform);
    effective_last_plan_pose = goal_standoff_pose_plan_;
    if (standoff_idx_out < 0) {
      goal_standoff_failure_reason_ = "locked_point_missing";
      std::ostringstream detail;
      detail << "locked=(" << goal_standoff_pose_global_.pose.position.x << ","
             << goal_standoff_pose_global_.pose.position.y << ") frame="
             << goal_standoff_pose_global_.header.frame_id
             << " first_candidate_idx=" << first_candidate_idx
             << " position_tol=0.001 yaw_tol=0.001";
      goal_standoff_failure_detail_ = detail.str();
      goal_standoff_force_hold_ = true;
    } else {
      goal_standoff_force_hold_ = !candidate_is_safe(standoff_idx_out);
    }
    return !goal_standoff_force_hold_;
  }

  // One initial search per occupied-goal episode, including the no-result case.
  if (goal_standoff_search_done_) {
    return false;
  }
  goal_standoff_search_done_ = true;
  if (cfg_->trajectory.enable_occupied_goal_standoff_search) {
    for (int idx = static_cast<int>(global_plan.size()) - 2; idx >= first_candidate_idx; --idx) {
      if (!candidate_is_safe(idx)) {
        continue;
      }
      goal_standoff_latched_ = true;
      goal_standoff_plan_idx_ = idx;
      standoff_idx_out = idx;
      effective_last_plan_pose = global_plan[static_cast<size_t>(idx)];
      goal_standoff_pose_plan_ = effective_last_plan_pose;
      tf2::doTransform(effective_last_plan_pose, goal_standoff_pose_global_, plan_to_global_transform);
      RCLCPP_INFO(logger_, "终点占用: 锁定沿路径停车点 idx=%d", idx);
      return true;
    }
  }

  goal_standoff_plan_idx_ = -1;
  goal_standoff_failure_reason_ = cfg_->trajectory.enable_occupied_goal_standoff_search ?
    "no_safe_candidate" : "search_disabled";
  RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000,
    "终点占用: 未找到安全替代终点");
  return false;
}

bool TebLocalPlannerROS::transformGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
                   const geometry_msgs::msg::PoseStamped& global_pose, const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                   std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::msg::TransformStamped* tf_plan_to_global,
                   int* prune_row_from_idx, int* prune_row_to_idx, PathWindowKeyPoints* key_points)
 {
   if (prune_row_from_idx) {
     *prune_row_from_idx = -1;
   }
   if (prune_row_to_idx) {
     *prune_row_to_idx = -1;
   }
   if (key_points) {
     *key_points = PathWindowKeyPoints();
   }
   
   // this method is a slightly modified version of base_local_planner/goal_functions.h
 
 
   transformed_plan.clear();
 
   try 
   {
     if (global_plan.empty())
     {
      RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "收到的全局路径长度为 0");
       *current_goal_idx = 0;
       return false;
     }
     
     const geometry_msgs::msg::PoseStamped& plan_pose = global_plan.front();
     geometry_msgs::msg::TransformStamped plan_to_global_transform;
     try {
       plan_to_global_transform = tf_->lookupTransform(
                   global_frame,
                   plan_pose.header.frame_id,
                   tf2::TimePointZero,
                   tf2::durationFromSec(0.5));
     } catch (const tf2::ExtrapolationException& ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "全局路径 TF 外推失败，尝试最新时间（%s → %s）: %s",
        plan_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
       // Retry with TimePointZero but without specifying source time (uses latest available)
       try {
         plan_to_global_transform = tf_->lookupTransform(
                     global_frame,
                     plan_pose.header.frame_id,
                     tf2::TimePointZero);
       } catch (const tf2::TransformException& ex2) {
        RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "全局路径 TF 重试失败: %s", ex2.what());
         throw nav2_core::PlannerException(
           std::string("Could not transform the global plan to the frame of the controller: ") + ex2.what()
         );
       }
     } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "全局路径 TF 查询失败: %s", ex.what());
       throw nav2_core::PlannerException(
         std::string("Could not transform the global plan to the frame of the controller: ") + ex.what()
       );
     }
     
     geometry_msgs::msg::PoseStamped robot_pose;
     try {
       // Get transform using TimePointZero to avoid extrapolation
       geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(
                   plan_pose.header.frame_id,
                   global_pose.header.frame_id,
                   tf2::TimePointZero,
                   tf2::durationFromSec(0.5));
       
       // Apply transform manually to avoid using global_pose.header.stamp
       tf2::doTransform(global_pose, robot_pose, global_to_plan_transform);
     } catch (const tf2::ExtrapolationException& ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "机器人位姿 TF 外推失败，正在重试: %s", ex.what());
       // Retry with TimePointZero but without specifying source time
       try {
         geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(
                     plan_pose.header.frame_id,
                     global_pose.header.frame_id,
                     tf2::TimePointZero);
         tf2::doTransform(global_pose, robot_pose, global_to_plan_transform);
       } catch (const tf2::TransformException& ex2) {
        RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "机器人位姿 TF 重试失败: %s", ex2.what());
         throw nav2_core::PlannerException(
           std::string("Could not transform the global plan to the frame of the controller: ") + ex2.what()
         );
       }
     } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "机器人位姿 TF 查询失败: %s", ex.what());
       throw nav2_core::PlannerException(
         std::string("Could not transform the global plan to the frame of the controller: ") + ex.what()
       );
     }
 
     std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap_->getMutex());

     //we'll discard points on the plan that are outside the local costmap
     double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                      costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
     dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                            // located on the border of the local costmap
     
 
     int i = 0;
     double sq_dist_threshold = dist_threshold * dist_threshold;

    const double corner_thresh_deg =
      static_cast<double>(cfg_->trajectory.theta_threshold);
    i = findEuclideanClosestPlanIdx(
      global_plan, robot_pose, sq_dist_threshold,
      cfg_->trajectory.transform_global_plan_closest_search_max_accum_dist);
    i = clampClosestIdxBeforeUnprunedCorner(
      global_plan, robot_pose, i, corner_thresh_deg);

    const int n_plan = static_cast<int>(global_plan.size());
    const int global_goal_idx = n_plan - 1;
    geometry_msgs::msg::PoseStamped effective_last_plan_pose = global_plan.back();
    int standoff_idx = -1;
    auto planPoseAt = [&](int idx) -> const geometry_msgs::msg::PoseStamped & {
      return global_plan[static_cast<size_t>(idx)];
    };

    geometry_msgs::msg::PoseStamped newer_pose;

    const int segment_start_idx = i;

    const double costmap_size_x =
      costmap.getSizeInCellsX() * costmap.getResolution();
    const double costmap_size_y =
      costmap.getSizeInCellsY() * costmap.getResolution();
    const double costmap_span = std::max(costmap_size_x, costmap_size_y);
    const double span_scale =
      cfg_->trajectory.transform_global_plan_costmap_span_scale;
    const bool has_span_limit = span_scale > 1e-9;
    const double arc_limit = has_span_limit
      ? (span_scale * costmap_span)
      : std::numeric_limits<double>::infinity();
    const double corner_search_arc = has_span_limit ? arc_limit : -1.0;

    const int corner_idx = findFirstSharpCornerIdx(
      global_plan, segment_start_idx, corner_thresh_deg, corner_search_arc);
    const int span_end_idx = lastIdxWithinArc(
      global_plan, segment_start_idx, corner_search_arc);
    const int row_end_idx = (corner_idx >= 0) ? corner_idx : span_end_idx;
    const bool has_path_after_row = row_end_idx < global_goal_idx;

    double clip_length = max_plan_length;
    if (has_span_limit) {
      if (clip_length > 1e-9) {
        clip_length = std::min(clip_length, arc_limit);
      } else {
        clip_length = arc_limit;
      }
    }
    const int path_lookahead_idx = lastIdxWithinArc(
      global_plan, segment_start_idx,
      (clip_length > 1e-9) ? clip_length : -1.0);
    const int lookahead_idx = std::min(path_lookahead_idx, row_end_idx);

    bool corner_in_horizon = false;
    if (corner_idx >= 0) {
      double acc_c = 0.0;
      for (int j = segment_start_idx + 1; j <= corner_idx && j < n_plan; ++j) {
        acc_c += distance_points2d(
          global_plan[static_cast<size_t>(j - 1)].pose.position,
          global_plan[static_cast<size_t>(j)].pose.position);
      }
      const double horizon = (clip_length > 1e-9)
        ? (clip_length + 1.0)
        : std::numeric_limits<double>::infinity();
      corner_in_horizon = acc_c <= horizon + 1e-9;
    }

    const double to_end_dist =
      cfg_->trajectory.transformed_plan_collision_pose_to_end_distance;
    const double front_dist =
      cfg_->trajectory.transformed_plan_collision_pose_safe_distance_front;
    const double back_dist =
      cfg_->trajectory.transformed_plan_collision_pose_safe_distance_back;

    int end_idx = lookahead_idx;
    int last_occ = -1;
    int safe_distance_terminal_idx = -1;
    int lock_mode = 0;  // 0=none, 1=geometric, 2=occupancy
    bool need_prune_remaining_row = false;
    bool prune_include_start = false;
    std::vector<char> occupied;
    std::vector<double> arc_from_start;

    const int row_count = row_end_idx - segment_start_idx + 1;
    occupied.assign(static_cast<size_t>(std::max(row_count, 1)), 0);
    arc_from_start.assign(static_cast<size_t>(std::max(row_count, 1)), 0.0);
    for (int j = segment_start_idx + 1; j <= row_end_idx; ++j) {
      arc_from_start[static_cast<size_t>(j - segment_start_idx)] =
        arc_from_start[static_cast<size_t>(j - 1 - segment_start_idx)] +
        distance_points2d(
          global_plan[static_cast<size_t>(j - 1)].pose.position,
          global_plan[static_cast<size_t>(j)].pose.position);
    }
    bool goal_scanned = false;
    bool goal_has_visible_footprint = false;
    std::vector<geometry_msgs::msg::Point> goal_blocking_cells;
    auto scan_goal = [&]() {
      geometry_msgs::msg::PoseStamped goal;
      tf2::doTransform(global_plan.back(), goal, plan_to_global_transform);
      geometry_msgs::msg::Pose2D pose;
      pose.x = goal.pose.position.x;
      pose.y = goal.pose.position.y;
      pose.theta = tf2::getYaw(goal.pose.orientation);
      const auto footprint = dwb_critics::getOrientedFootprint(pose, lateralPaddedFootprintSpec());
      const auto cells = visibleFootprintCells(*costmap_, footprint);
      goal_has_visible_footprint = !cells.empty();
      for (const auto & cell : cells) {
        if (blockingCost(costmap_->getCost(cell.x, cell.y))) {
          geometry_msgs::msg::Point point;
          costmap_->mapToWorld(cell.x, cell.y, point.x, point.y);
          goal_blocking_cells.push_back(point);
        }
      }
      goal_scanned = true;
    };
    for (int j = segment_start_idx; j <= row_end_idx; ++j) {
      if (j == global_goal_idx) {
        scan_goal();
        occupied[static_cast<size_t>(j - segment_start_idx)] = goal_blocking_cells.empty() ? 0 : 1;
      } else {
        occupied[static_cast<size_t>(j - segment_start_idx)] =
          globalPlanPoseLateralPaddedFootprintFreeInControllerFrame(
            planPoseAt(j), plan_to_global_transform) ? 0 : 1;
      }
    }
    auto row_pose_free = [&](const int idx) -> bool {
      if (idx < segment_start_idx || idx > row_end_idx) {
        return false;
      }
      return occupied[static_cast<size_t>(idx - segment_start_idx)] == 0;
    };
    int last_occ_in_window = -1;
    for (int j = lookahead_idx; j >= segment_start_idx; --j) {
      if (occupied[static_cast<size_t>(j - segment_start_idx)]) {
        last_occ_in_window = j;
        break;
      }
    }
    for (int j = row_end_idx; j >= segment_start_idx; --j) {
      if (occupied[static_cast<size_t>(j - segment_start_idx)]) {
        last_occ = j;
        break;
      }
    }

    const double arc_look =
      arc_from_start[static_cast<size_t>(lookahead_idx - segment_start_idx)];
    const bool occ_near_tail =
      last_occ_in_window >= 0 && to_end_dist > 1e-9 &&
      (arc_look -
        arc_from_start[static_cast<size_t>(last_occ_in_window - segment_start_idx)]) +
        1e-9 < to_end_dist;

    auto candidate_clearances = [&](const int k) {
      const int ki = k - segment_start_idx;
      const double arc_k = arc_from_start[static_cast<size_t>(ki)];
      double front_clear = std::numeric_limits<double>::infinity();
      double back_clear = std::numeric_limits<double>::infinity();
      for (int j = segment_start_idx; j <= row_end_idx; ++j) {
        if (!occupied[static_cast<size_t>(j - segment_start_idx)]) {
          continue;
        }
        const double arc_j =
          arc_from_start[static_cast<size_t>(j - segment_start_idx)];
        if (j > k) {
          front_clear = std::min(front_clear, arc_j - arc_k);
        } else if (j < k) {
          back_clear = std::min(back_clear, arc_k - arc_j);
        }
      }
      return std::make_pair(front_clear, back_clear);
    };
    auto candidate_is_safe = [&](const int k) -> bool {
      if (occupied[static_cast<size_t>(k - segment_start_idx)]) {
        return false;
      }
      const auto clearances = candidate_clearances(k);
      return !(front_dist > 1e-9 && clearances.first + 1e-9 < front_dist) &&
             !(back_dist > 1e-9 && clearances.second + 1e-9 < back_dist);
    };


    if (occ_near_tail) {
      lock_mode = 2;
      const double row_arc =
        arc_from_start[static_cast<size_t>(row_end_idx - segment_start_idx)];
      const double free_tail = (last_occ < row_end_idx)
        ? (row_arc - arc_from_start[static_cast<size_t>(last_occ - segment_start_idx)])
        : 0.0;
      if (free_tail + 1e-9 >= to_end_dist && to_end_dist > 1e-9) {
        int tail_idx = last_occ;
        double acc = 0.0;
        for (int j = last_occ + 1; j <= row_end_idx; ++j) {
          acc += distance_points2d(
            global_plan[static_cast<size_t>(j - 1)].pose.position,
            global_plan[static_cast<size_t>(j)].pose.position);
          tail_idx = j;
          if (acc + 1e-9 >= to_end_dist) {
            break;
          }
        }
        end_idx = tail_idx;
      } else {
        int safe_idx = -1;
        if (front_dist > 1e-9 || back_dist > 1e-9) {
          for (int j = row_end_idx; j >= segment_start_idx; --j) {
            if (candidate_is_safe(j)) {
              safe_idx = j;
              break;
            }
          }
        }
        if (safe_idx >= 0) {
          end_idx = safe_idx;
          safe_distance_terminal_idx = safe_idx;
          const double arrive_dist =
            cfg_->trajectory.transformed_plan_row_terminal_arrive_distance;
          const double remain_to_safe = (safe_idx >= segment_start_idx)
            ? arc_from_start[static_cast<size_t>(safe_idx - segment_start_idx)]
            : 0.0;
          const double eucl_to_safe = distance_points2d(
            robot_pose.pose.position,
            global_plan[static_cast<size_t>(safe_idx)].pose.position);
          const bool arrived =
            (arrive_dist > 1e-9) &&
            (segment_start_idx >= safe_idx ||
             remain_to_safe <= arrive_dist + 1e-9 ||
             eucl_to_safe <= arrive_dist + 1e-9);
          if (arrived && has_path_after_row) {
            need_prune_remaining_row = true;
            prune_include_start = true;
          }
        } else {
          need_prune_remaining_row = true;
          prune_include_start = has_path_after_row;
          end_idx = segment_start_idx;
          for (int j = row_end_idx; j >= segment_start_idx; --j) {
            if (!occupied[static_cast<size_t>(j - segment_start_idx)]) {
              end_idx = j;
              break;
            }
          }
        }
      }
      end_idx = std::min(end_idx, row_end_idx);
    } else if (corner_in_horizon && last_occ_in_window < 0) {
      int last_occ_to_corner = -1;
      for (int j = corner_idx; j >= segment_start_idx; --j) {
        if (occupied[static_cast<size_t>(j - segment_start_idx)]) {
          last_occ_to_corner = j;
          break;
        }
      }
      if (last_occ_to_corner < 0) {
        lock_mode = 1;
        end_idx = corner_idx;
      }
    }

    {
      if (!row_pose_free(end_idx)) {
        int found_idx = -1;
        for (int j = end_idx + 1; j <= row_end_idx; ++j) {
          if (row_pose_free(j)) {
            found_idx = j;
            break;
          }
        }
        if (found_idx < 0) {
          for (int j = end_idx - 1; j >= segment_start_idx; --j) {
            if (row_pose_free(j)) {
              found_idx = j;
              break;
            }
          }
        }
        if (found_idx >= 0) {
          RCLCPP_INFO_THROTTLE(
            logger_, *(clock_), 2000,
            "transformGlobalPlan: 侧向膨胀后行终点 %d -> %d (边距=%.3f m)",
            end_idx, found_idx,
            cfg_ != nullptr ? cfg_->obstacles.min_obstacle_dist : 0.0);
          end_idx = found_idx;
        } else {
          RCLCPP_WARN_THROTTLE(
            logger_, *(clock_), 2000,
            "transformGlobalPlan: 当前行 [%d,%d] 无侧向膨胀后的空闲终点，准备裁剪",
            segment_start_idx, row_end_idx);
          need_prune_remaining_row = true;
          prune_include_start = has_path_after_row;
          end_idx = segment_start_idx;
        }
      }
    }

    // Include any forward extension made by ordinary row/window logic. Gate
    // before the occupied-goal retreat, so shrinking to a stop cannot disable it.
    const int candidate_window_end = std::max(lookahead_idx, end_idx);
    const bool window_contains_global_goal = candidate_window_end == global_goal_idx ||
      (corner_in_horizon && corner_idx == global_goal_idx);
    if (cfg_->trajectory.enable_occupied_goal_stop &&
        (window_contains_global_goal || occupied_goal_active_)) {
      if (!goal_scanned) {scan_goal();}
      const bool original_goal_free = goal_blocking_cells.empty();
      if (!original_goal_free) {
        occupied_goal_active_ = true;
        occupied_goal_clear_active_ = false;
        // Remember observed blockers, not the entire goal footprint. A blocker
        // leaving the rolling map must not masquerade as a cleared obstacle.
        occupied_goal_blocking_cells_.insert(occupied_goal_blocking_cells_.end(),
          goal_blocking_cells.begin(), goal_blocking_cells.end());
        auto key = [](const geometry_msgs::msg::Point & point) {
          return std::make_pair(std::llround(point.x * 1e6), std::llround(point.y * 1e6));
        };
        std::sort(occupied_goal_blocking_cells_.begin(), occupied_goal_blocking_cells_.end(),
          [&](const auto & a, const auto & b) {return key(a) < key(b);});
        auto end = std::unique(occupied_goal_blocking_cells_.begin(), occupied_goal_blocking_cells_.end(),
          [&](const auto & a, const auto & b) {return key(a) == key(b);});
        occupied_goal_blocking_cells_.erase(end, occupied_goal_blocking_cells_.end());
      } else if (occupied_goal_active_) {
        const bool observed_blockers_clear = goal_has_visible_footprint &&
          !occupied_goal_blocking_cells_.empty() && std::all_of(
            occupied_goal_blocking_cells_.begin(), occupied_goal_blocking_cells_.end(),
            [&](const auto & point) {
              unsigned int x, y;
              return costmap_->worldToMap(point.x, point.y, x, y) &&
                     !blockingCost(costmap_->getCost(x, y));
            });
        if (!observed_blockers_clear) {
          occupied_goal_clear_active_ = false;
        } else {
          const auto now = clock_->now();
          if (!occupied_goal_clear_active_) {
            occupied_goal_clear_active_ = true;
            occupied_goal_near_clear_since_ = now;
          }
          const double clear_duration = std::max(0.0,
            cfg_->trajectory.occupied_goal_clear_duration);
          if ((now - occupied_goal_near_clear_since_).seconds() >= clear_duration) {
            resetOccupiedGoalState();
            if (planner_) {
              planner_->clearPlanner();
            }
            RCLCPP_INFO(logger_, "原终点连续安全 %.3fs，取消停车等待计时并恢复原终点", clear_duration);
          }
        }
      }
      if (occupied_goal_active_) {
        std::map<std::string, int> rejected_candidates;
        auto safe_standoff = [&](int idx) {
          auto reject = [&](const char * reason, const std::string & detail) {
            goal_standoff_failure_reason_ = reason;
            goal_standoff_failure_detail_ = detail;
            ++rejected_candidates[reason];
            return false;
          };
          // An existing terminal may be outside a temporarily shortened horizon;
          // keep its identity and only plan the current visible prefix to it.
          if (idx < segment_start_idx || idx > row_end_idx ||
              (!goal_standoff_latched_ && idx > candidate_window_end)) {
            return reject("outside_candidate_range", "idx=" + std::to_string(idx) +
              " row_start=" + std::to_string(segment_start_idx) +
              " row_end=" + std::to_string(row_end_idx) +
              " window_end=" + std::to_string(candidate_window_end));
          }
          geometry_msgs::msg::PoseStamped candidate;
          tf2::doTransform(planPoseAt(idx), candidate, plan_to_global_transform);
          geometry_msgs::msg::Pose2D pose;
          pose.x = candidate.pose.position.x;
          pose.y = candidate.pose.position.y;
          pose.theta = tf2::getYaw(candidate.pose.orientation);
          const auto footprint = dwb_critics::getOrientedFootprint(pose, lateralPaddedFootprintSpec());
          std::ostringstream detail;
          detail << "candidate=(" << pose.x << "," << pose.y << "," << pose.theta << ")"
                 << " frame=" << global_frame;
          if (!footprintFullyInsideCostmap(*costmap_, footprint)) {
            detail << " map_origin=(" << costmap_->getOriginX() << "," << costmap_->getOriginY()
                   << ") map_size=(" << costmap_->getSizeInCellsX() * costmap_->getResolution()
                   << "," << costmap_->getSizeInCellsY() * costmap_->getResolution() << ")";
            return reject("footprint_outside_costmap", detail.str());
          }
          const auto clearances = candidate_clearances(idx);
          detail << " front_clear=" << clearances.first << "/" << front_dist
                 << " back_clear=" << clearances.second << "/" << back_dist;
          const auto cells = visibleFootprintCells(*costmap_, footprint);
          if (cells.empty()) {
            return reject("footprint_unobservable", detail.str());
          }
          for (const auto & cell : cells) {
            const auto cost = costmap_->getCost(cell.x, cell.y);
            if (blockingCost(cost)) {
              double wx, wy;
              costmap_->mapToWorld(cell.x, cell.y, wx, wy);
              detail << " blocking_cell=(" << wx << "," << wy << ") cost=" << int(cost);
              return reject(cost == nav2_costmap_2d::NO_INFORMATION ?
                "footprint_unknown" : "footprint_collision", detail.str());
            }
          }
          if (idx == segment_start_idx && idx + 1 < n_plan) {
            const auto & point = planPoseAt(idx).pose.position;
            const auto & next = planPoseAt(idx + 1).pose.position;
            const double projection = (robot_pose.pose.position.x - point.x) * (next.x - point.x) +
              (robot_pose.pose.position.y - point.y) * (next.y - point.y);
            const double stop_dist = std::max(0.05, cfg_->trajectory.goal_standoff_stop_distance);
            if (projection > 1e-6 && std::hypot(robot_pose.pose.position.x - point.x,
                robot_pose.pose.position.y - point.y) > stop_dist) {
              return reject("locked_point_passed", detail.str());
            }
          }
          if (front_dist > 1e-9 && clearances.first + 1e-9 < front_dist) {
            return reject("front_clearance_insufficient", detail.str());
          }
          if (back_dist > 1e-9 && clearances.second + 1e-9 < back_dist) {
            return reject("back_clearance_insufficient", detail.str());
          }
          goal_standoff_failure_reason_ = "valid";
          goal_standoff_failure_detail_ = detail.str();
          return true;
        };
        selectGoalStandoffAlongPlan(global_plan, plan_to_global_transform,
          segment_start_idx, safe_standoff, standoff_idx, effective_last_plan_pose);
        if (!goal_standoff_latched_ && !rejected_candidates.empty()) {
          std::ostringstream summary;
          summary << "rejected_candidates:";
          for (const auto & rejection : rejected_candidates) {
            summary << " " << rejection.first << "=" << rejection.second;
          }
          summary << "; last_candidate: " << goal_standoff_failure_detail_;
          goal_standoff_failure_detail_ = summary.str();
        }
        if (goal_standoff_last_logged_reason_ != goal_standoff_failure_reason_) {
          if (goal_standoff_failure_reason_ == "valid") {
            RCLCPP_INFO(logger_, "终点占用停车点校验通过: idx=%d reason=valid %s",
              standoff_idx, goal_standoff_failure_detail_.c_str());
          } else {
            RCLCPP_WARN(logger_, "终点占用停车点校验失败: idx=%d reason=%s %s",
              standoff_idx, goal_standoff_failure_reason_.c_str(),
              goal_standoff_failure_detail_.c_str());
          }
          goal_standoff_last_logged_reason_ = goal_standoff_failure_reason_;
        }
      }
    } else {
      resetOccupiedGoalState();
    }
    occupied_goal_no_standoff_ = occupied_goal_active_ &&
      (!goal_standoff_latched_ || goal_standoff_force_hold_);
    occupied_goal_dist_m_ = occupied_goal_active_ ? std::hypot(
      robot_pose.pose.position.x - global_plan.back().pose.position.x,
      robot_pose.pose.position.y - global_plan.back().pose.position.y) :
      std::numeric_limits<double>::infinity();

    // 停车等待期间保留完整 global_plan_，终点释放后可继续原路径。
    if (occupied_goal_active_) {
      need_prune_remaining_row = false;
      prune_include_start = false;
      safe_distance_terminal_idx = -1;
    }

    if (standoff_idx >= 0) {
      end_idx = std::min(candidate_window_end, standoff_idx);
      if (!goal_standoff_force_hold_) {
        safe_distance_terminal_idx = standoff_idx;
      }
      if (prune_row_from_idx && prune_row_to_idx &&
          *prune_row_from_idx >= 0 && *prune_row_to_idx >= standoff_idx)
      {
        *prune_row_to_idx = standoff_idx - 1;
        if (*prune_row_from_idx > *prune_row_to_idx) {
          *prune_row_from_idx = -1;
          *prune_row_to_idx = -1;
        }
      }
    }

    auto publish_decision_marker = [&](const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & pub,
                                       const std::string & ns,
                                       const geometry_msgs::msg::PoseStamped * pose,
                                       const float r, const float g, const float b) {
      if (!pub) {
        return;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = global_frame;
      marker.header.stamp = clock_->now();
      marker.ns = ns;
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = pose ? visualization_msgs::msg::Marker::ADD :
        visualization_msgs::msg::Marker::DELETE;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.28;
      marker.scale.y = 0.28;
      marker.scale.z = 0.28;
      if (pose) {
        marker.pose = pose->pose;
        marker.pose.position.z += 0.15;
        marker.color.a = 0.95;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
      }
      pub->publish(marker);
    };

    geometry_msgs::msg::PoseStamped safe_terminal_pose;
    if (safe_distance_terminal_idx >= 0 && safe_distance_terminal_idx < n_plan) {
      tf2::doTransform(
        planPoseAt(safe_distance_terminal_idx), safe_terminal_pose, plan_to_global_transform);
      publish_decision_marker(
        safe_terminal_marker_pub_, "transformed_plan_safe_terminal", &safe_terminal_pose,
        0.1F, 0.95F, 0.2F);
      double front_clear = std::numeric_limits<double>::infinity();
      double back_clear = std::numeric_limits<double>::infinity();
      const int safe_offset = safe_distance_terminal_idx - segment_start_idx;
      const double safe_arc = arc_from_start[static_cast<size_t>(safe_offset)];
      for (int j = segment_start_idx; j <= row_end_idx; ++j) {
        if (!occupied[static_cast<size_t>(j - segment_start_idx)]) {
          continue;
        }
        const double occ_arc = arc_from_start[static_cast<size_t>(j - segment_start_idx)];
        if (j > safe_distance_terminal_idx) {
          front_clear = std::min(front_clear, occ_arc - safe_arc);
        } else if (j < safe_distance_terminal_idx) {
          back_clear = std::min(back_clear, safe_arc - occ_arc);
        }
      }
      RCLCPP_INFO_THROTTLE(
        logger_, *(clock_), 2000,
        "安全终点 marker: idx=%d pose=(%.3f, %.3f) front_clear=%.3f/%.3f "
        "back_clear=%.3f/%.3f frame=%s",
        safe_distance_terminal_idx, safe_terminal_pose.pose.position.x,
        safe_terminal_pose.pose.position.y, front_clear, front_dist,
        back_clear, back_dist, safe_terminal_pose.header.frame_id.c_str());
    } else {
      publish_decision_marker(
        safe_terminal_marker_pub_, "transformed_plan_safe_terminal", nullptr,
        0.1F, 0.95F, 0.2F);
    }

    geometry_msgs::msg::PoseStamped occupied_goal_stop_pose;
    const bool has_occupied_goal_stop = occupied_goal_no_standoff_ && n_plan > 0;
    if (has_occupied_goal_stop) {
      // 立即停车的可视化点就是当前机器人位置，不再计算终点前的预设停车点。
      occupied_goal_stop_pose = robot_pose;
      geometry_msgs::msg::PoseStamped occupied_goal_stop_global;
      tf2::doTransform(
        occupied_goal_stop_pose, occupied_goal_stop_global, plan_to_global_transform);
      publish_decision_marker(
        occupied_goal_stop_marker_pub_, "last_occupied_goal_stop",
        &occupied_goal_stop_global, 1.0F, 0.15F, 0.1F);
      RCLCPP_INFO_THROTTLE(
        logger_, *(clock_), 2000,
        "终点占用立即停车 marker: goal_dist=%.3f m "
        "pose=(%.3f, %.3f) frame=%s reason=%s %s",
        occupied_goal_dist_m_,
        occupied_goal_stop_global.pose.position.x,
        occupied_goal_stop_global.pose.position.y,
        occupied_goal_stop_global.header.frame_id.c_str(),
        goal_standoff_failure_reason_.c_str(), goal_standoff_failure_detail_.c_str());
    } else {
      publish_decision_marker(
        occupied_goal_stop_marker_pub_, "last_occupied_goal_stop", nullptr,
        1.0F, 0.15F, 0.1F);
    }

    for (int k = segment_start_idx; k <= end_idx; ++k) {
      tf2::doTransform(planPoseAt(k), newer_pose, plan_to_global_transform);
      transformed_plan.push_back(newer_pose);
    }

    if (need_prune_remaining_row && prune_row_from_idx && prune_row_to_idx) {
      // 最后一行禁止从行首整段删除，且不能把全局最后一个点剔掉
      if (!has_path_after_row) {
        prune_include_start = false;
      }
      int from_idx = segment_start_idx + 1;
      if (prune_include_start) {
        from_idx = findCurrentRowStartIdx(
          global_plan, segment_start_idx, corner_thresh_deg);
      }
      int to_idx = row_end_idx;
      if (!has_path_after_row) {
        to_idx = global_goal_idx - 1;
      }
      if (from_idx <= to_idx && to_idx >= 0) {
        *prune_row_from_idx = from_idx;
        *prune_row_to_idx = to_idx;
      }
    }

    i = end_idx + 1;
    RCLCPP_INFO_THROTTLE(
      logger_, *(clock_), 2000,
      "[transformGlobalPlan]: 前视=%.3f 跨度上限=%.3f 裁剪长度=%.3f 角点=%d "
      "在视界内=%d 锁定=%d 行终点=%d 最后占用=%d 窗口终点=%d 裁剪剩余行=%d "
      "裁剪从=%d 裁剪到=%d",
      max_plan_length, has_span_limit ? arc_limit : -1.0, clip_length,
      corner_idx, corner_in_horizon ? 1 : 0, lock_mode,
      row_end_idx, last_occ, end_idx, need_prune_remaining_row ? 1 : 0,
      prune_row_from_idx ? *prune_row_from_idx : -1,
      prune_row_to_idx ? *prune_row_to_idx : -1);

    if (key_points)
    {
      *key_points = PathWindowKeyPoints();
      key_points->frame_id = global_frame;
      key_points->lock_mode = lock_mode;
      key_points->closest_idx = segment_start_idx;
      key_points->corner_idx = corner_idx;
      key_points->end_idx = end_idx;
      key_points->last_occ_idx = last_occ;
      key_points->row_end_idx = row_end_idx;
      key_points->prune_from_idx = prune_row_from_idx ? *prune_row_from_idx : -1;
      key_points->prune_to_idx = prune_row_to_idx ? *prune_row_to_idx : -1;
      auto fill_key = [&](const int idx, geometry_msgs::msg::Point & out) -> bool {
        if (idx < 0 || idx >= n_plan) {
          return false;
        }
        geometry_msgs::msg::PoseStamped pose_out;
        tf2::doTransform(planPoseAt(idx), pose_out, plan_to_global_transform);
        out = pose_out.pose.position;
        if (!pose_out.header.frame_id.empty()) {
          key_points->frame_id = pose_out.header.frame_id;
        }
        return true;
      };
      key_points->has_closest = fill_key(segment_start_idx, key_points->closest);
      key_points->has_corner = fill_key(corner_idx, key_points->corner);
      key_points->has_terminal = fill_key(end_idx, key_points->terminal);
      key_points->has_last_occ = fill_key(last_occ, key_points->last_occ);
      const bool row_end_distinct =
        row_end_idx >= 0 && row_end_idx != corner_idx && row_end_idx != end_idx;
      key_points->has_row_end =
        row_end_distinct && fill_key(row_end_idx, key_points->row_end);
      key_points->has_prune_from = fill_key(key_points->prune_from_idx, key_points->prune_from);
      key_points->has_prune_to = fill_key(key_points->prune_to_idx, key_points->prune_to);
    }
         
     // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
     // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
     if (transformed_plan.empty())
     {
       tf2::doTransform(effective_last_plan_pose, newer_pose, plan_to_global_transform);
       transformed_plan.push_back(newer_pose);
       
       // Return the index of the current goal point (inside the distance threshold)
       if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
     }
     else
     {
       if (current_goal_idx) {
         *current_goal_idx = i - 1;
       }
     }
     
     // Return the transformation from the global plan to the global planning frame if desired
     if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
   }
   catch(tf2::LookupException& ex)
   {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "无可用坐标变换: %s", ex.what());
     return false;
   }
   catch(tf2::ConnectivityException& ex)
   {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "TF 连通性错误: %s", ex.what());
     return false;
   }
   catch(tf2::ExtrapolationException& ex)
   {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "TF 外推错误: %s", ex.what());
     if (global_plan.size() > 0)
      RCLCPP_ERROR_THROTTLE(
        logger_, *clock_, 1000,
        "TF 外推上下文: 全局坐标系=%s，路径点数=%u，路径坐标系=%s",
        global_frame.c_str(), static_cast<unsigned>(global_plan.size()),
        global_plan[0].header.frame_id.c_str());
 
     return false;
   }
 
   return true;
 }
 
 
     
       
       
 double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan, const geometry_msgs::msg::PoseStamped& local_goal,
                     int current_goal_idx, const geometry_msgs::msg::TransformStamped& tf_plan_to_global, int moving_average_length) const
 {
   
   int n = (int)global_plan.size();
   
   // check if we are near the global goal already
   if (current_goal_idx > n-moving_average_length-2)
   {
     if (current_goal_idx >= n-1) // we've exactly reached the goal
     {
       return tf2::getYaw(local_goal.pose.orientation);
     }
     else
     {
 //      tf::Quaternion global_orientation;
 //      tf::quaternionMsgToTF(global_plan.back().pose.orientation, global_orientation);
 //      return  tf2::getYaw(tf_plan_to_global.getRotation() *  global_orientation );
 
         tf2::Quaternion global_orientation, tf_plan_to_global_orientation;
         tf2::fromMsg(global_plan.back().pose.orientation, global_orientation);
         tf2::fromMsg(tf_plan_to_global.transform.rotation, tf_plan_to_global_orientation);
 
         return tf2::getYaw(tf_plan_to_global_orientation * global_orientation);
     }     
   }
   
   // reduce number of poses taken into account if the desired number of poses is not available
   moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
   
   std::vector<double> candidates;
   geometry_msgs::msg::PoseStamped tf_pose_k = local_goal;
   geometry_msgs::msg::PoseStamped tf_pose_kp1;
   
   int range_end = current_goal_idx + moving_average_length;
   for (int i = current_goal_idx; i < range_end; ++i)
   {
     // Transform pose of the global plan to the planning frame
     const geometry_msgs::msg::PoseStamped& pose = global_plan.at(i+1);
     tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);
       
     // calculate yaw angle  
     candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
               tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
     
     if (i<range_end-1) 
       tf_pose_k = tf_pose_kp1;
   }
   return average_angles(candidates);
 }

}  // namespace teb_local_planner
