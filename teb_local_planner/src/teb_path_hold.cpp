/*********************************************************************
 * teb_path_hold.cpp
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

#include "teb_local_planner/path_dynamic_wait.h"

using nav2_util::declare_parameter_if_not_declared;

namespace teb_local_planner
{
void TebLocalPlannerROS::pathWaitObstacleCB(
  const costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obst_msg)
{
  std::lock_guard<std::mutex> l(path_wait_obst_mutex_);
  path_wait_obstacle_msg_ = *obst_msg;
  path_wait_receive_time_ = clock_->now();
  path_wait_msg_valid_ = true;
}

bool TebLocalPlannerROS::collectPathWaitObstacles(
  std::vector<PathWaitObstacle> & out,
  bool & message_valid)
{
  out.clear();
  message_valid = false;
  costmap_converter_msgs::msg::ObstacleArrayMsg msg;
  rclcpp::Time recv;
  {
    std::lock_guard<std::mutex> l(path_wait_obst_mutex_);
    if (!path_wait_msg_valid_)
    {
      return false;
    }
    msg = path_wait_obstacle_msg_;
    recv = path_wait_receive_time_;
  }

  const double timeout = cfg_->obstacles.path_wait_msg_timeout;
  if (timeout > 0.0 && (clock_->now() - recv).seconds() > timeout)
  {
    return false;
  }
  message_valid = true;

  Eigen::Affine3d obstacle_to_map = Eigen::Affine3d::Identity();
  const std::string src_frame = msg.header.frame_id.empty() ? cfg_->map_frame : msg.header.frame_id;
  if (src_frame != cfg_->map_frame)
  {
    try
    {
      geometry_msgs::msg::TransformStamped tf_msg = tf_->lookupTransform(
        cfg_->map_frame, src_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
      obstacle_to_map = tf2::transformToEigen(tf_msg);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "PATH 停车门控: 动态障碍 TF %s→%s 失败: %s",
        src_frame.c_str(), cfg_->map_frame.c_str(), ex.what());
      return false;
    }
  }

  out.reserve(msg.obstacles.size());
  for (const auto & src : msg.obstacles)
  {
    PathWaitObstacle dst;
    dst.id = static_cast<int>(src.id);
    auto toMap = [&](const geometry_msgs::msg::Point32 & p) {
      Eigen::Vector3d v(p.x, p.y, p.z);
      return (obstacle_to_map * v).head<2>();
    };
    if (src.polygon.points.size() >= 3)
    {
      dst.polygon.reserve(src.polygon.points.size());
      Eigen::Vector2d acc = Eigen::Vector2d::Zero();
      for (const auto & p : src.polygon.points)
      {
        const Eigen::Vector2d xy = toMap(p);
        dst.polygon.push_back(xy);
        acc += xy;
      }
      dst.center = acc / static_cast<double>(dst.polygon.size());
      dst.radius = 0.0;
    }
    else if (!src.polygon.points.empty())
    {
      dst.center = toMap(src.polygon.points.front());
      dst.radius = std::max(0.0, static_cast<double>(src.radius));
      if (src.polygon.points.size() == 2)
      {
        dst.polygon.push_back(dst.center);
        dst.polygon.push_back(toMap(src.polygon.points.back()));
      }
    }
    else
    {
      continue;
    }
    out.push_back(std::move(dst));
  }
  return true;
}

bool TebLocalPlannerROS::tryPathWaitHold(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & velocity,
  geometry_msgs::msg::TwistStamped & cmd_vel)
{
  if (!cfg_->obstacles.path_wait_enable)
  {
    return false;
  }

  std::vector<PathWaitObstacle> obsts;
  bool message_valid = false;
  collectPathWaitObstacles(obsts, message_valid);

  const double inscribed = cfg_->robot_model ? cfg_->robot_model->getInscribedRadius()
                                            : robot_inscribed_radius_;
  const double r_foot = computePathWaitFootprintHalfWidth(footprint_spec_, inscribed);
  const double min_obs = std::max(cfg_->obstacles.min_obstacle_dist, normal_min_obstacle_dist_);
  const Eigen::Vector2d robot_xy(robot_pose.pose.position.x, robot_pose.pose.position.y);

  const PathWaitResult result = path_dynamic_wait_gate_.evaluate(
    global_plan_, robot_xy, obsts, r_foot, min_obs, *cfg_, clock_->now(), message_valid);

  if (cfg_->obstacles.publish_path_wait_debug && path_wait_debug_marker_pub_)
  {
    visualization_msgs::msg::MarkerArray ma;
    const std::string frame_id = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : cfg_->map_frame;
    buildPathWaitDebugMarkers(ma, frame_id, clock_->now(), result);
    path_wait_debug_marker_pub_->publish(ma);
  }

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 1000,
    "[PathWait] mode=%s reason=%s hw=%.3f n_dyn=%zu blocking=%s waiting=%s "
    "latched=%s wait=%.2fs rem=%.2fs msg_valid=%d",
    pathWaitModeName(result.mode),
    result.reason.c_str(),
    result.half_width,
    obsts.size(),
    joinPathWaitIds(result.blocking_ids).c_str(),
    joinPathWaitIds(result.waiting_ids).c_str(),
    joinPathWaitIds(result.latched_ids).c_str(),
    result.elapsed_wait_s,
    result.remaining_wait_s,
    message_valid ? 1 : 0);

  if (result.mode != PathWaitMode::Wait)
  {
    return false;
  }

  if (result.previous_mode != PathWaitMode::Wait && planner_)
  {
    planner_->clearPlanner();
  }

  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  const double dt = std::max(1e-3, control_duration_);
  auto approach_zero = [dt](double v, double acc) {
    const double a = std::max(1e-6, acc);
    const double step = a * dt;
    if (std::abs(v) <= step)
    {
      return 0.0;
    }
    return v - std::copysign(step, v);
  };
  cmd_vel.twist.linear.x = approach_zero(velocity.linear.x, cfg_->robot.acc_lim_x);
  cmd_vel.twist.linear.y = approach_zero(velocity.linear.y, cfg_->robot.acc_lim_y);
  cmd_vel.twist.angular.z = approach_zero(velocity.angular.z, cfg_->robot.acc_lim_theta);
  last_cmd_ = cmd_vel.twist;

  if (visualization_)
  {
    std::vector<geometry_msgs::msg::PoseStamped> stay;
    geometry_msgs::msg::PoseStamped p = robot_pose;
    p.header.stamp = clock_->now();
    stay.push_back(p);
    visualization_->publishLocalPlan(stay);
    visualization_->publishGlobalPlan(global_plan_);
  }
  if (global_plan_pub_ && !global_plan_.empty())
  {
    nav_msgs::msg::Path global_plan_msg;
    global_plan_msg.header.stamp = clock_->now();
    global_plan_msg.header.frame_id = global_plan_.front().header.frame_id;
    global_plan_msg.poses = global_plan_;
    global_plan_pub_->publish(global_plan_msg);
  }
  return true;
}

bool TebLocalPlannerROS::tryGoalStandoffHold(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & velocity,
  geometry_msgs::msg::TwistStamped & cmd_vel)
{
  if (!goal_standoff_latched_) {
    goal_standoff_hold_active_ = false;
    return false;
  }

  bool should_hold = goal_standoff_force_hold_;
  if (!should_hold) {
    if (goal_standoff_pose_global_.header.frame_id != robot_pose.header.frame_id) {
      goal_standoff_hold_active_ = false;
      return false;
    }
    const double dist = std::hypot(
      robot_pose.pose.position.x - goal_standoff_pose_global_.pose.position.x,
      robot_pose.pose.position.y - goal_standoff_pose_global_.pose.position.y);
    const double xy_tol = std::max(0.05, cfg_->goal_tolerance.xy_goal_tolerance);
    should_hold = dist <= xy_tol;
  }
  if (!should_hold) {
    goal_standoff_hold_active_ = false;
    return false;
  }

  if (!goal_standoff_hold_active_ && planner_) {
    planner_->clearPlanner();
    RCLCPP_INFO(logger_, "终点退让: 到达退让点，改为零速度保持");
  }
  goal_standoff_hold_active_ = true;

  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  const double dt = std::max(1e-3, control_duration_);
  auto approach_zero = [dt](double speed, double acc) {
    const double step = std::max(1e-6, acc) * dt;
    if (std::abs(speed) <= step) {
      return 0.0;
    }
    return speed - std::copysign(step, speed);
  };
  cmd_vel.twist.linear.x = approach_zero(velocity.linear.x, cfg_->robot.acc_lim_x);
  cmd_vel.twist.linear.y = approach_zero(velocity.linear.y, cfg_->robot.acc_lim_y);
  cmd_vel.twist.angular.z = approach_zero(velocity.angular.z, cfg_->robot.acc_lim_theta);
  last_cmd_ = cmd_vel.twist;
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 1000,
    "终点退让: 零速度保持 (force=%s)",
    goal_standoff_force_hold_ ? "是" : "否");
  return true;
}

bool TebLocalPlannerROS::tryOccupiedGoalNearHold(
  const geometry_msgs::msg::Twist & velocity,
  geometry_msgs::msg::TwistStamped & cmd_vel)
{
  const double dist_limit = cfg_->trajectory.last_occupied_goal_dist;
  if (dist_limit <= 0.0) {
    occupied_goal_near_hold_active_ = false;
    return false;
  }

  constexpr double kReleaseDelaySec = 0.4;
  const rclcpp::Time now = clock_->now();
  const bool within_dist = occupied_goal_no_standoff_ && occupied_goal_dist_m_ < dist_limit;
  if (within_dist) {
    occupied_goal_near_clear_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    if (!occupied_goal_near_hold_active_) {
      occupied_goal_near_hold_active_ = true;
      occupied_goal_near_hold_since_ = now;
      if (planner_) {
        planner_->clearPlanner();
      }
      RCLCPP_INFO(
        logger_,
        "终点占用且无退让点，base 距终点 %.3f m < %.3f m，零速度",
        occupied_goal_dist_m_, dist_limit);
    }
  } else if (occupied_goal_near_hold_active_) {
    if (occupied_goal_near_clear_since_.nanoseconds() == 0) {
      occupied_goal_near_clear_since_ = now;
    }
    const double clear_elapsed = (now - occupied_goal_near_clear_since_).seconds();
    if (clear_elapsed >= kReleaseDelaySec) {
      occupied_goal_near_hold_active_ = false;
      occupied_goal_near_hold_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
      occupied_goal_near_clear_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
      RCLCPP_INFO(logger_, "终点占用近距停车解除");
      return false;
    }
  } else {
    return false;
  }

  const double timeout = cfg_->trajectory.last_occupied_goal_hold_timeout;
  if (timeout > 0.0 && occupied_goal_near_hold_since_.nanoseconds() > 0) {
    const double held = (now - occupied_goal_near_hold_since_).seconds();
    if (held >= timeout) {
      occupied_goal_near_hold_active_ = false;
      occupied_goal_near_hold_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
      occupied_goal_near_clear_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
      RCLCPP_ERROR(
        logger_,
        "终点占用且无退让点，近距零速度已持续 %.1f s，本次控制失败",
        held);
      throw nav2_core::PlannerException(
        "终点被占用且无退让点，近距零速度超时");
    }
  }

  cmd_vel.header.stamp = now;
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  const double dt = std::max(1e-3, control_duration_);
  auto approach_zero = [dt](double speed, double acc) {
    const double step = std::max(1e-6, acc) * dt;
    if (std::abs(speed) <= step) {
      return 0.0;
    }
    return speed - std::copysign(step, speed);
  };
  cmd_vel.twist.linear.x = approach_zero(velocity.linear.x, cfg_->robot.acc_lim_x);
  cmd_vel.twist.linear.y = approach_zero(velocity.linear.y, cfg_->robot.acc_lim_y);
  cmd_vel.twist.angular.z = approach_zero(velocity.angular.z, cfg_->robot.acc_lim_theta);
  last_cmd_ = cmd_vel.twist;
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 1000,
    "终点占用近距停车: dist=%.3f m limit=%.3f m",
    occupied_goal_dist_m_, dist_limit);
  return true;
}

}  // namespace teb_local_planner
