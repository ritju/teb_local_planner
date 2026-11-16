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
#include "teb_local_planner/occupied_goal_braking.hpp"

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

  if (cfg_->debug_mode.load(std::memory_order_relaxed)) {
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
  }

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
  if (goal_standoff_pose_global_.header.frame_id == robot_pose.header.frame_id) {
    const double dist = std::hypot(
      robot_pose.pose.position.x - goal_standoff_pose_global_.pose.position.x,
      robot_pose.pose.position.y - goal_standoff_pose_global_.pose.position.y);
    const double stop_dist = std::max(0.05, cfg_->trajectory.goal_standoff_stop_distance);
    should_hold = should_hold ||
      (dist <= stop_dist && std::hypot(velocity.linear.x, velocity.linear.y) <= 0.05);
  }
  if (!should_hold) {
    goal_standoff_hold_active_ = false;
    return false;
  }

  goal_standoff_hold_active_ = true;
  stopForOccupiedGoal(robot_pose, velocity, cmd_vel);
  return true;
}

bool TebLocalPlannerROS::isOccupiedGoalBrakingPathSafe(
  const std::vector<geometry_msgs::msg::PoseStamped> & path) const
{
  if (!costmap_ || !costmap_model_ || footprint_spec_.size() < 3 || path.empty()) {
    return false;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap_->getMutex());
  const double spacing = std::max(0.005, costmap_->getResolution() * 0.5);
  auto previous = path.front();
  for (const auto & pose : path) {
    if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
        !std::isfinite(tf2::getYaw(pose.pose.orientation))) {
      return false;
    }
    const double yaw0 = tf2::getYaw(previous.pose.orientation);
    const double dyaw = angles::shortest_angular_distance(yaw0,
      tf2::getYaw(pose.pose.orientation));
    const double distance = std::hypot(pose.pose.position.x - previous.pose.position.x,
      pose.pose.position.y - previous.pose.position.y);
    const int count = std::max(1, static_cast<int>(std::ceil(
      std::max(distance / spacing, std::fabs(dyaw) / 0.05))));
    for (int j = 0; j <= count; ++j) {
      const double ratio = static_cast<double>(j) / count;
      geometry_msgs::msg::Pose2D sample;
      sample.x = previous.pose.position.x + ratio *
        (pose.pose.position.x - previous.pose.position.x);
      sample.y = previous.pose.position.y + ratio *
        (pose.pose.position.y - previous.pose.position.y);
      sample.theta = yaw0 + ratio * dyaw;
      const auto footprint = dwb_critics::getOrientedFootprint(sample, lateralPaddedFootprintSpec());
      for (const auto & vertex : footprint) {
        unsigned int mx, my;
        if (!costmap_->worldToMap(vertex.x, vertex.y, mx, my)) {
          return false;
        }
      }
      if (!orientedFootprintFullyObservedAndFree(footprint)) {
        return false;
      }
    }
    previous = pose;
  }
  return true;
}

void TebLocalPlannerROS::resetOccupiedGoalState()
{
  occupied_goal_active_ = false;
  occupied_goal_timed_out_ = false;
  occupied_goal_no_standoff_ = false;
  occupied_goal_dist_m_ = std::numeric_limits<double>::infinity();
  occupied_goal_near_hold_active_ = false;
  occupied_goal_near_hold_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  occupied_goal_near_clear_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  occupied_goal_clear_active_ = false;
  occupied_goal_blocking_cells_.clear();
  goal_standoff_latched_ = false;
  goal_standoff_search_done_ = false;
  goal_standoff_plan_idx_ = -1;
  goal_standoff_force_hold_ = false;
  goal_standoff_failure_reason_ = "none";
  goal_standoff_failure_detail_.clear();
  goal_standoff_last_logged_reason_.clear();
  goal_standoff_hold_active_ = false;
}

void TebLocalPlannerROS::publishOccupiedGoalStopPrediction(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  auto start = robot_pose;
  start.header.stamp = clock_->now();
  nav_msgs::msg::Path path;
  path.header = start.header;
  path.poses.push_back(start);
  teb_msgs::msg::TrajectoryMsg trajectory;
  trajectory.header = start.header;
  teb_msgs::msg::TrajectoryPointMsg first;
  first.pose = start.pose;
  first.velocity = velocity;
  trajectory.trajectory.push_back(first);

  const double vx = velocity.linear.x, vy = velocity.linear.y, wz = velocity.angular.z;
  const bool finite_velocity = std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz);
  if (!finite_velocity) {
    // Do not advertise an invalid measurement as a stationary, safe prediction.
    path.poses.clear();
    trajectory.trajectory.clear();
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000,
      "终点占用立即停车: reason=invalid_measured_velocity，无法生成制动预测，保持零速");
  } else if (std::hypot(vx, vy) > 1e-6 || std::fabs(wz) > 1e-6) {
    const double ax = cfg_->robot.acc_lim_x;
    const double ay = cfg_->robot.acc_lim_y;
    const double aw = cfg_->robot.acc_lim_theta;
    auto valid_axis = [](double speed, double decel) {
      return std::fabs(speed) <= 1e-6 || (std::isfinite(decel) && decel > 0.0);
    };
    const bool valid_deceleration = valid_axis(vx, ax) && valid_axis(vy, ay) && valid_axis(wz, aw);
    const double delay = std::isfinite(control_duration_) ? std::max(1e-3, control_duration_) : 0.1;
    auto stopping_time = [](double speed, double decel) {
      return std::fabs(speed) <= 1e-6 ? 0.0 : std::fabs(speed) / decel;
    };
    // Invalid deceleration cannot define a stopping distance. Publish a clearly
    // diagnosed constant-velocity preview instead of collapsing to one point.
    const double horizon = valid_deceleration ? delay + std::max({
      stopping_time(vx, ax), stopping_time(vy, ay), stopping_time(wz, aw)}) : 2.0;
    if (!valid_deceleration) {
      RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000,
        "终点占用立即停车: reason=invalid_prediction_deceleration "
        "acc=(%.3f,%.3f,%.3f)，发布 2s 匀速运动预测，保持零速", ax, ay, aw);
    }
    const double spacing = costmap_ ? std::max(0.005, costmap_->getResolution() * 0.5) : 0.025;
    const double step = std::min({0.05, spacing / std::max(1e-6, std::hypot(vx, vy)),
      0.025 / std::max(1e-6, std::fabs(wz))});
    auto component = [&](double speed, double decel, double t) {
      if (!valid_deceleration || t <= delay || std::fabs(speed) <= 1e-6) {
        return speed;
      }
      return std::copysign(std::max(0.0, std::fabs(speed) - decel * (t - delay)), speed);
    };
    auto pose = start;
    double yaw = tf2::getYaw(start.pose.orientation);
    double t = 0.0;
    // Bound work for malformed/extreme configurations. A truncated preview
    // retains nonzero terminal velocity and is explicitly diagnosed below.
    for (size_t i = 0; t < horizon && i < 10000; ++i) {
      double next = std::min(horizon, t + step);
      if (valid_deceleration && t < delay) {
        next = std::min(next, delay);
      }
      const double dt = next - t;
      if (dt <= 0.0) {break;}
      const double ux = component(vx, ax, next);
      const double uy = component(vy, ay, next);
      const double uw = component(wz, aw, next);
      const double mx = (component(vx, ax, t) + ux) * 0.5;
      const double my = (component(vy, ay, t) + uy) * 0.5;
      const double mw = (component(wz, aw, t) + uw) * 0.5;
      const double middle_yaw = yaw + mw * dt * 0.5;
      pose.pose.position.x += (mx * std::cos(middle_yaw) - my * std::sin(middle_yaw)) * dt;
      pose.pose.position.y += (mx * std::sin(middle_yaw) + my * std::cos(middle_yaw)) * dt;
      yaw += mw * dt;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation = tf2::toMsg(orientation);
      path.poses.push_back(pose);
      teb_msgs::msg::TrajectoryPointMsg point;
      point.pose = pose.pose;
      point.velocity.linear.x = ux;
      point.velocity.linear.y = uy;
      point.velocity.angular.z = uw;
      point.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
        rclcpp::Duration::from_seconds(next));
      trajectory.trajectory.push_back(point);
      t = next;
    }
    if (t < horizon) {
      RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000,
        "终点占用制动预测达到采样上限: preview=%.3fs required=%.3fs，保持零速", t, horizon);
    }
    if (!isOccupiedGoalBrakingPathSafe(path.poses)) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "终点占用立即停车: 制动预测范围存在碰撞、未知或出图，保留预测供监控并保持零速");
    }
  }
  if (visualization_) {
    visualization_->deferLocalPlanPublish(false);
    visualization_->publishLocalPlan(path.poses);
  }
  if (occupied_goal_braking_path_pub_) {
    occupied_goal_braking_path_pub_->publish(path);
  }
  if (occupied_goal_braking_trajectory_pub_) {
    occupied_goal_braking_trajectory_pub_->publish(trajectory);
  }
}

void TebLocalPlannerROS::stopForOccupiedGoal(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & velocity,
  geometry_msgs::msg::TwistStamped & cmd_vel)
{
  cmd_vel.header = robot_pose.header;
  cmd_vel.header.stamp = clock_->now();
  if (costmap_ros_) {
    cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  }
  cmd_vel.twist = geometry_msgs::msg::Twist();
  last_cmd_ = cmd_vel.twist;
  if (planner_) {
    planner_->clearPlanner();
  }
  publishOccupiedGoalStopPrediction(robot_pose, velocity);
  checkOccupiedGoalWaitTimeout(true);
}

void TebLocalPlannerROS::checkOccupiedGoalWaitTimeout(bool waiting)
{
  if (occupied_goal_timed_out_) {
    throw nav2_core::OccupiedGoalTimeout("终点占用停车等待超时");
  }
  if (!occupied_goal_active_) {
    occupied_goal_near_hold_active_ = false;
    return;
  }
  const auto now = clock_->now();
  if (waiting && !occupied_goal_near_hold_active_) {
    occupied_goal_near_hold_active_ = true;
    occupied_goal_near_hold_since_ = now;
  }
  const double timeout = cfg_->trajectory.last_occupied_goal_hold_timeout;
  if (occupied_goal_near_hold_active_ && timeout > 0.0 &&
      (now - occupied_goal_near_hold_since_).seconds() >= timeout) {
    occupied_goal_timed_out_ = true;
    last_cmd_ = geometry_msgs::msg::Twist();
    throw nav2_core::OccupiedGoalTimeout("终点占用停车等待超时");
  }
}

void TebLocalPlannerROS::applyGoalStandoffBraking(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & velocity,
  geometry_msgs::msg::TwistStamped & cmd_vel)
{
  if (!visualization_) {
    if (goal_standoff_latched_) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "替代终点制动: reason=visualization_unavailable，立即零速并发布制动预测");
      stopForOccupiedGoal(robot_pose, velocity, cmd_vel);
    }
    return;
  }
  if (!goal_standoff_latched_) {
    nav_msgs::msg::Path empty;
    empty.header.stamp = clock_->now();
    empty.header.frame_id = robot_pose.header.frame_id;
    if (occupied_goal_braking_path_pub_) {
      occupied_goal_braking_path_pub_->publish(empty);
    }
    if (occupied_goal_braking_trajectory_pub_) {
      teb_msgs::msg::TrajectoryMsg empty_trajectory;
      empty_trajectory.header = empty.header;
      occupied_goal_braking_trajectory_pub_->publish(empty_trajectory);
    }
    return;
  }
  const auto plan = visualization_->lastLocalPlan();
  if (plan.size() < 2 || plan.front().header.frame_id != robot_pose.header.frame_id ||
      plan.back().header.frame_id != goal_standoff_pose_global_.header.frame_id) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "替代终点制动: reason=%s poses=%zu robot_frame=%s stop_frame=%s "
      "plan_front_frame=%s plan_back_frame=%s，停车等待",
      plan.size() < 2 ? "optimized_plan_missing" : "optimized_plan_frame_mismatch", plan.size(),
      robot_pose.header.frame_id.c_str(), goal_standoff_pose_global_.header.frame_id.c_str(),
      plan.empty() ? "" : plan.front().header.frame_id.c_str(),
      plan.empty() ? "" : plan.back().header.frame_id.c_str());
    stopForOccupiedGoal(robot_pose, velocity, cmd_vel);
    return;
  }
  if (std::hypot(plan.back().pose.position.x - goal_standoff_pose_global_.pose.position.x,
        plan.back().pose.position.y - goal_standoff_pose_global_.pose.position.y) > 0.25) {
    // The optimized endpoint does not match the locked stop. Hold rather than
    // allowing an unchecked command to drive beyond the intended terminal.
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "替代终点制动: reason=optimized_endpoint_mismatch "
      "endpoint=(%.3f,%.3f) stop=(%.3f,%.3f) tolerance=0.250，停车等待",
      plan.back().pose.position.x, plan.back().pose.position.y,
      goal_standoff_pose_global_.pose.position.x, goal_standoff_pose_global_.pose.position.y);
    stopForOccupiedGoal(robot_pose, velocity, cmd_vel);
    return;
  }

  double remaining = std::hypot(
    robot_pose.pose.position.x - plan.front().pose.position.x,
    robot_pose.pose.position.y - plan.front().pose.position.y);
  for (size_t i = 1; i < plan.size(); ++i) {
    remaining += std::hypot(plan[i].pose.position.x - plan[i - 1].pose.position.x,
      plan[i].pose.position.y - plan[i - 1].pose.position.y);
  }
  const double stop_offset = std::max(0.05, cfg_->trajectory.goal_standoff_stop_distance);
  const double available = std::max(0.0, remaining - stop_offset);
  const double decel = cfg_->robot.acc_lim_x;
  if (!std::isfinite(decel) || decel <= 0.0) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000,
      "替代终点制动: reason=invalid_deceleration acc_lim_x=%.3f，停车等待", decel);
    stopForOccupiedGoal(robot_pose, velocity, cmd_vel);
    return;
  }
  // 以已配置的 controller_frequency 对应控制周期作为一次指令响应延迟。
  const double delay = std::max(1e-3, control_duration_);
  const double measured_speed = std::hypot(velocity.linear.x, velocity.linear.y);
  // 解 d = v*T + v²/(2*a)，得到当前距离允许的最大速度。
  const double safe_speed = occupiedGoalSafeSpeed(available, decel, delay);
  const double required = occupiedGoalStoppingDistance(measured_speed, decel, delay);
  if (required > available + 0.02) {
    cmd_vel.twist = geometry_msgs::msg::Twist();
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000,
      "替代终点制动距离不足: available=%.3f required=%.3f v=%.3f reason=insufficient_braking_distance",
      available, required, measured_speed);
  } else {
    const double commanded = std::hypot(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y);
    if (commanded > safe_speed && commanded > 1e-6) {
      const double ratio = safe_speed / commanded;
      cmd_vel.twist.linear.x *= ratio;
      cmd_vel.twist.linear.y *= ratio;
      if (!cfg_->robot.cmd_angle_instead_rotvel) {
        cmd_vel.twist.angular.z *= ratio;
      }
    }
  }

  // 从优化后的 TEB 路径截取保守停车距离，作为监控器检查的制动预测。
  const double projected_speed = std::max(measured_speed,
    std::hypot(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y));
  const double projected_arc = occupiedGoalStoppingDistance(projected_speed, decel, delay);
  std::vector<geometry_msgs::msg::PoseStamped> braking_path;
  auto start = robot_pose;
  start.header.stamp = clock_->now();
  braking_path.push_back(start);
  double arc = 0.0;
  auto previous = start;
  for (const auto & pose : plan) {
    const double segment = std::hypot(pose.pose.position.x - previous.pose.position.x,
      pose.pose.position.y - previous.pose.position.y);
    if (segment < 1e-6) {
      previous = pose;
      continue;
    }
    const double take = std::min(segment, std::max(0.0, projected_arc - arc));
    const int samples = std::max(1, static_cast<int>(std::ceil(take / 0.1)));
    tf2::Quaternion q0, q1;
    tf2::fromMsg(previous.pose.orientation, q0);
    tf2::fromMsg(pose.pose.orientation, q1);
    for (int j = 1; j <= samples; ++j) {
      const double ratio = (take / segment) * static_cast<double>(j) / samples;
      auto sample = pose;
      sample.pose.position.x = previous.pose.position.x + ratio *
        (pose.pose.position.x - previous.pose.position.x);
      sample.pose.position.y = previous.pose.position.y + ratio *
        (pose.pose.position.y - previous.pose.position.y);
      sample.pose.orientation = tf2::toMsg(q0.slerp(q1, ratio));
      braking_path.push_back(sample);
    }
    if (arc + segment >= projected_arc) {
      break;
    }
    arc += segment;
    previous = pose;
  }
  // Always expose the complete optimized path to collision_monitor while decelerating.
  // The braking prediction has its own topics and must not hide upcoming obstacles.
  visualization_->publishLocalPlan(plan);
  if (!isOccupiedGoalBrakingPathSafe(braking_path)) {
    cmd_vel.twist = geometry_msgs::msg::Twist();
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 500,
      "替代终点制动范围内存在近障碍、未知或出图，立即输出零速度 reason=braking_corridor_unsafe");
  }
  checkOccupiedGoalWaitTimeout(std::hypot(cmd_vel.twist.linear.x,
    cmd_vel.twist.linear.y) < 1e-6 && std::fabs(cmd_vel.twist.angular.z) < 1e-6);
  nav_msgs::msg::Path path_msg;
  path_msg.header = start.header;
  path_msg.poses = braking_path;
  if (occupied_goal_braking_path_pub_) {
    occupied_goal_braking_path_pub_->publish(path_msg);
  }
  if (occupied_goal_braking_trajectory_pub_) {
    teb_msgs::msg::TrajectoryMsg trajectory;
    trajectory.header = path_msg.header;
    double distance = 0.0;
    for (size_t i = 0; i < braking_path.size(); ++i) {
      if (i > 0) {
        distance += std::hypot(
          braking_path[i].pose.position.x - braking_path[i - 1].pose.position.x,
          braking_path[i].pose.position.y - braking_path[i - 1].pose.position.y);
      }
      teb_msgs::msg::TrajectoryPointMsg point;
      point.pose = braking_path[i].pose;
      double t = 0.0;
      double speed = projected_speed;
      if (projected_speed > 1e-6) {
        const double response_distance = projected_speed * delay;
        if (distance <= response_distance) {
          t = distance / projected_speed;
        } else {
          const double brake_distance = distance - response_distance;
          speed = std::sqrt(std::max(0.0,
            projected_speed * projected_speed - 2.0 * decel * brake_distance));
          t = delay + (projected_speed - speed) / decel;
          point.acceleration.linear.x =
            -std::copysign(decel, velocity.linear.x != 0.0 ? velocity.linear.x :
              cmd_vel.twist.linear.x);
        }
      }
      point.velocity.linear.x = std::copysign(speed,
        velocity.linear.x != 0.0 ? velocity.linear.x : cmd_vel.twist.linear.x);
      point.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
        rclcpp::Duration::from_seconds(t));
      trajectory.trajectory.push_back(point);
    }
    occupied_goal_braking_trajectory_pub_->publish(trajectory);
  }
}

bool TebLocalPlannerROS::tryOccupiedGoalNearHold(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & velocity,
  geometry_msgs::msg::TwistStamped & cmd_vel)
{
  if (occupied_goal_timed_out_) {
    throw nav2_core::OccupiedGoalTimeout("终点占用停车等待超时");
  }
  if (!occupied_goal_active_) {
    occupied_goal_near_hold_active_ = false;
    return false;
  }

  bool holding = false;
  if (goal_standoff_latched_) {
    holding = tryGoalStandoffHold(robot_pose, velocity, cmd_vel);
  } else {
    stopForOccupiedGoal(robot_pose, velocity, cmd_vel);
    holding = true;
  }

  // Replanning or recovery of the same terminal cannot extend a running deadline.
  checkOccupiedGoalWaitTimeout(holding);
  return holding;
}

}  // namespace teb_local_planner
