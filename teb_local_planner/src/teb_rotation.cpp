/*********************************************************************
 * teb_rotation.cpp
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
void TebLocalPlannerROS::resetMajorArcLock()
{
  major_arc_active_ = false;
  major_arc_rotdir_ = RotType::none;
  major_arc_progress_valid_ = false;
}

void TebLocalPlannerROS::captureMajorArcProgress(double rem_lock)
{
  major_arc_progress_x_ = robot_pose_.x();
  major_arc_progress_y_ = robot_pose_.y();
  major_arc_progress_rem_ = rem_lock;
  major_arc_progress_time_ = clock_->now();
  major_arc_progress_valid_ = true;
}

double TebLocalPlannerROS::remainingYawAlongMajorArc(double yaw_now, double yaw_goal) const
{
  const double d = g2o::normalize_theta(yaw_goal - yaw_now);
  if (major_arc_rotdir_ == RotType::left) {
    return (d >= 0.0) ? d : d + 2.0 * M_PI;
  }
  if (major_arc_rotdir_ == RotType::right) {
    return (d <= 0.0) ? -d : 2.0 * M_PI - d;
  }
  return std::abs(d);
}

bool TebLocalPlannerROS::checkMajorArcStagnationUnlock(double yaw_goal)
{
  const double timeout = cfg_->rotation.major_arc_stagnation_timeout;
  if (!major_arc_active_ || major_arc_rotdir_ == RotType::none || timeout <= 0.0) {
    return false;
  }

  const double rem = remainingYawAlongMajorArc(robot_pose_.theta(), yaw_goal);
  if (!major_arc_progress_valid_) {
    captureMajorArcProgress(rem);
    return false;
  }

  const double yaw_thr = cfg_->rotation.major_arc_stagnation_yaw;
  const double xy_thr = cfg_->rotation.major_arc_stagnation_xy;
  const double dx = robot_pose_.x() - major_arc_progress_x_;
  const double dy = robot_pose_.y() - major_arc_progress_y_;
  const double dist = std::hypot(dx, dy);
  const double rem_drop = major_arc_progress_rem_ - rem;
  const bool yaw_progress = yaw_thr > 0.0 && rem_drop >= yaw_thr;
  const bool xy_progress = xy_thr > 0.0 && dist >= xy_thr;
  if (yaw_progress || xy_progress) {
    captureMajorArcProgress(rem);
    return false;
  }

  const double elapsed = (clock_->now() - major_arc_progress_time_).seconds();
  if (elapsed < timeout) {
    return false;
  }

  RCLCPP_WARN(
    logger_,
    "优弧停滞解锁 rem=%.3f rad drop=%.3f rad dist=%.3f m dt=%.2f s rotdir=%s",
    rem, rem_drop, dist, elapsed,
    (major_arc_rotdir_ == RotType::left) ? "left" : "right");
  resetMajorArcLock();
  return true;
}

std::vector<geometry_msgs::msg::PoseStamped> TebLocalPlannerROS::buildMajorArcOrientedPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan, double d_long) const
{
  if (plan.size() < 2) {
    return plan;
  }

  auto yaw_of = [](const geometry_msgs::msg::PoseStamped & p) {
    return tf2::getYaw(p.pose.orientation);
  };
  auto set_yaw = [](geometry_msgs::msg::PoseStamped & p, double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, g2o::normalize_theta(yaw));
    p.pose.orientation = tf2::toMsg(q);
  };

  const double yaw0 = yaw_of(plan.front());
  const double yawg = yaw_of(plan.back());

  std::vector<double> s(plan.size(), 0.0);
  for (size_t i = 1; i < plan.size(); ++i) {
    const double dx = plan[i].pose.position.x - plan[i - 1].pose.position.x;
    const double dy = plan[i].pose.position.y - plan[i - 1].pose.position.y;
    s[i] = s[i - 1] + std::hypot(dx, dy);
  }
  const double path_len = std::max(s.back(), 1e-6);

  constexpr double kMaxYawStep = 0.4;
  const int n_yaw = std::max(2, static_cast<int>(std::ceil(std::abs(d_long) / kMaxYawStep)) + 1);
  const int n = std::max(n_yaw, static_cast<int>(plan.size()));

  std::vector<geometry_msgs::msg::PoseStamped> out;
  out.reserve(static_cast<size_t>(n));
  for (int k = 0; k < n; ++k) {
    const double t = (n <= 1) ? 1.0 : static_cast<double>(k) / static_cast<double>(n - 1);
    const double s_target = t * path_len;
    size_t i = 1;
    while (i + 1 < s.size() && s[i] < s_target) {
      ++i;
    }
    const double s0 = s[i - 1];
    const double s1 = s[i];
    const double u = (s1 - s0 > 1e-9) ? (s_target - s0) / (s1 - s0) : 1.0;
    geometry_msgs::msg::PoseStamped p = plan[i];
    p.pose.position.x = plan[i - 1].pose.position.x +
      u * (plan[i].pose.position.x - plan[i - 1].pose.position.x);
    p.pose.position.y = plan[i - 1].pose.position.y +
      u * (plan[i].pose.position.y - plan[i - 1].pose.position.y);
    set_yaw(p, yaw0 + t * d_long);
    out.push_back(p);
  }
  out.front() = plan.front();
  set_yaw(out.front(), yaw0);
  out.back() = plan.back();
  set_yaw(out.back(), yawg);
  return out;
}

 bool TebLocalPlannerROS::shouldRotateInPlace(
  const geometry_msgs::msg::Twist & velocity,
  const std::vector<geometry_msgs::msg::PoseStamped> & transformed_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  
  // Note: do NOT gate rotation on current angular velocity (intentionally removed)

  const rclcpp::Time now = clock_->now();
  auto stampRotationAnchor = [&]() {
    last_rotation_pose_ = robot_pose;
    last_rotation_pose_time_ = now;
  };

  // Check if plan is not empty
  if (transformed_plan.empty())
  {
    // RCLCPP_INFO_THROTTLE(
    //   logger_, *clock_, 2000,
    //   "原地转: shouldRotateInPlace=false，transformed_plan 为空");
    if (was_inplace_rotation_active_) {
      stampRotationAnchor();
    }
    was_inplace_rotation_active_ = false;
    return false;
  }

  // 线速度模长只依赖 odom，提前算好供门控与日志复用。超阈值时一律不原地转，可在 lookahead 前早退以省算力。
  // const double linear_vel = std::hypot(velocity.linear.x, velocity.linear.y);
  if (std::abs(velocity.linear.x) >= cfg_->rotation.linear_vel_threshold)
  {
    // RCLCPP_INFO_THROTTLE(
    //   logger_, *clock_, 500,
    //   "原地转: shouldRotateInPlace=false，线速度 gate（v_xy=%.3f >= %.3f）",
    //   velocity.linear.x, cfg_->rotation.linear_vel_threshold);
    if (was_inplace_rotation_active_) {
      stampRotationAnchor();
    }
    was_inplace_rotation_active_ = false;
    return false;
  }

  // Find target pose based on forward lookahead distance
  // Start from the first pose (robot position) and find the first pose that is at least forward_lookahead_distance away
  size_t target_idx = transformed_plan.size() - 1;
  double accumulated_distance = 0.0;

  for (size_t i = 1; i < transformed_plan.size(); ++i)
  {
    double dx = transformed_plan[i].pose.position.x - transformed_plan[i-1].pose.position.x;
    double dy = transformed_plan[i].pose.position.y - transformed_plan[i-1].pose.position.y;
    accumulated_distance += std::sqrt(dx * dx + dy * dy);

    if (accumulated_distance >= cfg_->rotation.forward_lookahead_distance)
    {
      target_idx = i;
      break;
    }
  }

  const geometry_msgs::msg::PoseStamped * const target_pose = &transformed_plan[target_idx];
  const double rotation_limit_duration = cfg_->rotation.rotation_limit_duration;
  const double rotation_limit_distance = cfg_->rotation.rotation_limit_distance;

  auto normalizeAngle = [](double angle) -> double
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  };

  // Calculate angle difference between robot orientation and target pose orientation
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double target_yaw = tf2::getYaw(target_pose->pose.orientation);
  double angular_distance = normalizeAngle(target_yaw - robot_yaw);

  // 距离门比的是机器人相对上次原地转锚点的位移，而不是 lookahead 路点。
  // 正在进行的原地转跳过限流，避免同一段转向被自己刚写入的锚点掐断。
  if (
    !was_inplace_rotation_active_ &&
    rotation_limit_duration > 0.0 &&
    rotation_limit_distance > 0.0 &&
    last_rotation_pose_time_.nanoseconds() > 0 &&
    !last_rotation_pose_.header.frame_id.empty() &&
    last_rotation_pose_.header.frame_id == robot_pose.header.frame_id)
  {
    const double elapsed = (now - last_rotation_pose_time_).seconds();
    if (elapsed <= rotation_limit_duration)
    {
      const double dx = robot_pose.pose.position.x - last_rotation_pose_.pose.position.x;
      const double dy = robot_pose.pose.position.y - last_rotation_pose_.pose.position.y;
      const double robot_dist = std::hypot(dx, dy);
      if (robot_dist <= rotation_limit_distance)
      {
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, 500,
          "原地转: shouldRotateInPlace=false，rotation_limit 生效 "
          "(dt=%.2f <= %.2f, robot_dist=%.3f <= %.3f)",
          elapsed, rotation_limit_duration, robot_dist, rotation_limit_distance);
        return false;
      }
    }
  }

  // 离局部路径过远时不对齐到前瞻切向。侧向切入由 TEB 完成，避免先转到路径方向再被优化轨迹转回去。
  // 距离是点到 transformed_plan 各边的最短距离（垂足限制在线段上）。本函数调用时路径起点尚未改写成机器人位姿。
  const double max_lateral = cfg_->rotation.approach_max_lateral_dist;
  if (max_lateral > 0.0 && transformed_plan.size() >= 2)
  {
    const double robot_x = robot_pose.pose.position.x;
    const double robot_y = robot_pose.pose.position.y;
    double min_lateral = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < transformed_plan.size(); ++i)
    {
      const double edge_start_x = transformed_plan[i - 1].pose.position.x;
      const double edge_start_y = transformed_plan[i - 1].pose.position.y;
      const double edge_end_x = transformed_plan[i].pose.position.x;
      const double edge_end_y = transformed_plan[i].pose.position.y;
      const double edge_dx = edge_end_x - edge_start_x;
      const double edge_dy = edge_end_y - edge_start_y;
      const double edge_length_sq = edge_dx * edge_dx + edge_dy * edge_dy;
      double edge_distance;
      if (edge_length_sq < 1e-12) {
        edge_distance = std::hypot(robot_x - edge_start_x, robot_y - edge_start_y);
      } else {
        const double projection = std::clamp(
          ((robot_x - edge_start_x) * edge_dx + (robot_y - edge_start_y) * edge_dy) / edge_length_sq,
          0.0, 1.0);
        edge_distance = std::hypot(
          robot_x - (edge_start_x + projection * edge_dx),
          robot_y - (edge_start_y + projection * edge_dy));
      }
      if (edge_distance < min_lateral) {
        min_lateral = edge_distance;
      }
    }
    if (min_lateral > max_lateral)
    {
      if (was_inplace_rotation_active_) {
        stampRotationAnchor();
      }
      was_inplace_rotation_active_ = false;
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 500,
        "原地转: shouldRotateInPlace=false，横向距离否决 (lateral=%.3f > %.3f m)",
        min_lateral, max_lateral);
      return false;
    }
  }

  // Check if angle difference exceeds threshold
  if (std::abs(angular_distance) <= cfg_->rotation.angle_threshold)
  {
    // 仅在“原地转状态退出”的那一拍刷新锚点，避免直行阶段反复刷新导致误限流。
    if (was_inplace_rotation_active_) {
      stampRotationAnchor();
    }
    was_inplace_rotation_active_ = false;

    // RCLCPP_INFO_THROTTLE(
    //   logger_, *clock_, 500,
    //   "原地转: shouldRotateInPlace=false，方向已对齐 |dtheta|=%.3f rad <= angle_threshold %.3f rad",
    //   std::abs(angular_distance), cfg_->rotation.angle_threshold);
    return false;
  }

  const bool rotate_in_place_clear =
    checkRotateToHeadingCollisionNominal(angular_distance, robot_pose, velocity);
  const bool path_check_enable = cfg_->rotation.path_footprint_check_enable;
  const bool path_footprint_clear = !path_check_enable ||
    isTransformedPlanFootprintSamplesCollisionFree(
      transformed_plan, cfg_->rotation.path_footprint_sample_spacing);
  const bool collision_ok = rotate_in_place_clear && path_footprint_clear;
  if (!collision_ok) {
    stampRotationAnchor();
    was_inplace_rotation_active_ = false;
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 500,
      "原地转: shouldRotateInPlace=false，碰障预检未通过 |dtheta|=%.3f rad、w_odom=%.3f、v_xy=%.3f "
      "(原地扫掠=%s、路径采样=%s)",
      std::abs(angular_distance), velocity.angular.z, std::abs(velocity.linear.x),
      rotate_in_place_clear ? "通过" : "未通过",
      path_check_enable ? (path_footprint_clear ? "通过" : "未通过") : "关闭");
  } else {
    if (!was_inplace_rotation_active_) {
      stampRotationAnchor();
    }
    was_inplace_rotation_active_ = true;
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 500,
      "原地转: shouldRotateInPlace=true，碰障预检通过 |dtheta|=%.3f rad、v_xy=%.3f、w_odom=%.3f、forward_lookahead=%.2f m",
      std::abs(angular_distance), std::abs(velocity.linear.x), velocity.angular.z,
      cfg_->rotation.forward_lookahead_distance);
  }
  return collision_ok;
}

double TebLocalPlannerROS::rotateToHeadingOmegaMagnitude(
  const double remaining_angle_rad, const double omega_current_abs) const
{
  
  const double wn = cfg_->rotation.rotate_to_heading_angular_vel;
  const double wm = cfg_->rotation.rotate_min_angular_vel;
  const double a = std::max(1e-6, cfg_->rotation.max_angular_accel);
  const double r = std::max(0.0, remaining_angle_rad);
  const double w_sqrt = std::min(wn, std::max(wm, std::sqrt(wm * wm + 2.0 * a * r)));

  const double manual_blend = cfg_->rotation.decel_blend_start_rad;
  double blend;
  if (manual_blend > 1e-9) {
    blend = manual_blend;
  } else {
    // 自动：从 wn 刹到 wm 所需角程 (wn²−wm²)/(2α)；从当前 |ω| 刹到 wm 为 (ω²−wm²)/(2α)。取较大者并乘扩展系数，使减速早于 sqrt 末端。
    constexpr double kAutoExtend = 3.0;
    const double r_from_wn = (wn * wn - wm * wm) / (2.0 * a);
    const double w_cap = std::min(std::max(0.0, std::abs(omega_current_abs)), wn);
    const double r_from_w =
      (w_cap > wm + 1e-9) ? ((w_cap * w_cap - wm * wm) / (2.0 * a)) : 0.0;
    blend = kAutoExtend * std::max(r_from_wn, r_from_w);
    blend = std::clamp(blend, 1e-3, M_PI);
  }

  const double w_linear_cap = std::min(wn, wn * r / blend);
  return std::max(wm, std::min(wn, std::min(w_sqrt, w_linear_cap)));
}

bool TebLocalPlannerROS::isRotationCollisionFreeDecel(
  const geometry_msgs::msg::PoseStamped & pose,
  double omega_direction_sign,
  double rotation_magnitude_rad,
  double initial_omega_z) const
{
  
  static_cast<void>(initial_omega_z);
  if (rotation_magnitude_rad < 1e-9) {
    clearCollisionFootprintDebug("collision_rotate_sweep", pose.header.frame_id);
    return true;
  }
  if (!rotation_collision_checker_) {
    return true;
  }

  const double dir = (omega_direction_sign >= 0.0) ? 1.0 : -1.0;
  const double yaw0 = tf2::getYaw(pose.pose.orientation);
  const std::vector<geometry_msgs::msg::Point> & footprint_poly =
    (footprint_spec_.size() >= 3) ? footprint_spec_ : costmap_ros_->getRobotFootprint();
  const std::string& cost_mode = cfg_->rotation.path_footprint_check_cost_mode;
  const std::string debug_frame = collisionDebugFrameId(pose.header.frame_id);
  constexpr const char * kCollisionNs = "collision_rotate_sweep";

  auto footprint_ok = [&](double y) -> bool {
    while (y > M_PI) y -= 2.0 * M_PI;
    while (y < -M_PI) y += 2.0 * M_PI;
    // 与 isTrajectoryFeasible 一致优先 footprint_spec_，避免 getRobotFootprint() 与贴边/动态轮廓不一致导致误报
    const double footprint_cost = rotation_collision_checker_->footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y, y, footprint_poly);
    if (cornerApproachCostIsBlocking(static_cast<unsigned char>(footprint_cost), cost_mode)) {
      publishCollisionFootprintDebug(
        kCollisionNs, debug_frame,
        pose.pose.position.x, pose.pose.position.y, y, footprint_poly, footprint_cost);
      return false;
    }
    return true;
  };

  if (!footprint_ok(yaw0)) {
    return false;
  }

  // 沿弧均匀采样 yaw，取代按 ω 积分步进：后者易数值不收敛，且步进路径与「整段旋转扫掠」不完全一致
  constexpr double kSampleStepRad = M_PI / 36.0;  // 5°
  const int n_samples =
    std::min(256, std::max(8, static_cast<int>(std::ceil(rotation_magnitude_rad / kSampleStepRad)) + 1));
  for (int i = 0; i <= n_samples; ++i) {
    const double fraction = static_cast<double>(i) / static_cast<double>(n_samples);
    const double y = yaw0 + dir * fraction * rotation_magnitude_rad;
    if (!footprint_ok(y)) {
      return false;
    }
  }
  clearCollisionFootprintDebug(kCollisionNs, debug_frame);
  return true;
}

std::vector<geometry_msgs::msg::PoseStamped> TebLocalPlannerROS::buildInPlaceRotationPredictedPath(
  const geometry_msgs::msg::PoseStamped & pose,
  double omega_direction_sign,
  double rotation_magnitude_rad) const
{
  
  std::vector<geometry_msgs::msg::PoseStamped> path;
  if (rotation_magnitude_rad < 1e-9) {
    path.push_back(pose);
    path.front().header.stamp = clock_->now();
    return path;
  }

  const double dir = (omega_direction_sign >= 0.0) ? 1.0 : -1.0;
  const double yaw0 = tf2::getYaw(pose.pose.orientation);
  constexpr double kSampleStepRad = M_PI / 36.0;  // 5°，与 isRotationCollisionFreeDecel 一致
  const int n_samples =
    std::min(256, std::max(8, static_cast<int>(std::ceil(rotation_magnitude_rad / kSampleStepRad)) + 1));

  path.reserve(static_cast<size_t>(n_samples) + 1);
  const rclcpp::Time now = clock_->now();
  for (int i = 0; i <= n_samples; ++i) {
    const double fraction = static_cast<double>(i) / static_cast<double>(n_samples);
    double y = yaw0 + dir * fraction * rotation_magnitude_rad;
    while (y > M_PI) {
      y -= 2.0 * M_PI;
    }
    while (y < -M_PI) {
      y += 2.0 * M_PI;
    }

    geometry_msgs::msg::PoseStamped p = pose;
    p.header.stamp = now;
    p.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), y));
    path.push_back(p);
  }
  return path;
}

void TebLocalPlannerROS::publishInPlaceRotationLocalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  double omega_direction_sign,
  double rotation_magnitude_rad) const
{
  
  if (!visualization_) {
    return;
  }
  const auto path =
    buildInPlaceRotationPredictedPath(pose, omega_direction_sign, rotation_magnitude_rad);
  if (path.empty()) {
    return;
  }
  visualization_->publishLocalPlan(path);
}

bool TebLocalPlannerROS::checkRotateToHeadingCollisionNominal(
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  
  const double abs_angular_distance = std::abs(angular_distance_to_heading);
  const double clockwise_angle = abs_angular_distance;
  const double counterclockwise_angle = 2.0 * M_PI - abs_angular_distance;
  const double wn = std::max(1e-6, cfg_->rotation.rotate_to_heading_angular_vel);
  const double clockwise_time = clockwise_angle / wn;
  const double counterclockwise_time = counterclockwise_angle / wn;
  const bool use_clockwise = (clockwise_time <= counterclockwise_time);
  double angular_vel_sign = (angular_distance_to_heading > 0.0) ? 1.0 : -1.0;
  if (!use_clockwise)
  {
    angular_vel_sign = -angular_vel_sign;
  }

  double test_angular_distance = use_clockwise ? clockwise_angle : counterclockwise_angle;
  const double primary_sign = angular_vel_sign;
  const double primary_mag = test_angular_distance;
  if (isRotationCollisionFreeDecel(pose, angular_vel_sign, test_angular_distance, velocity.angular.z)) {
    return true;
  }

  angular_vel_sign = -angular_vel_sign;
  test_angular_distance = use_clockwise ? counterclockwise_angle : clockwise_angle;
  if (isRotationCollisionFreeDecel(pose, angular_vel_sign, test_angular_distance, velocity.angular.z)) {
    // RCLCPP_INFO_THROTTLE(
    //   logger_, *clock_, 500,
    //   "原地转: collision_nominal 首选向 sign=%.0f、rot_mag=%.3f rad 未通过，备选向 sign=%.0f、rot_mag=%.3f rad 通过 "
    //   "（|dtheta|=%.3f、use_clockwise=%d、w_odom=%.3f）",
    //   primary_sign,
    //   primary_mag,
    //   angular_vel_sign,
    //   test_angular_distance,
    //   abs_angular_distance,
    //   use_clockwise ? 1 : 0,
    //   velocity.angular.z);
    return true;
  }
  return false;
}

bool TebLocalPlannerROS::computeRotateToHeadingCommand(
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  geometry_msgs::msg::TwistStamped & cmd_vel)
{
  
  const double abs_angular_distance = std::abs(angular_distance_to_heading);
  const double clockwise_angle = abs_angular_distance;
  const double counterclockwise_angle = 2.0 * M_PI - abs_angular_distance;
  const double wn = std::max(1e-6, cfg_->rotation.rotate_to_heading_angular_vel);
  const double clockwise_time = clockwise_angle / wn;
  const double counterclockwise_time = counterclockwise_angle / wn;

  const bool use_clockwise = (clockwise_time <= counterclockwise_time);
  double angular_vel_sign = (angular_distance_to_heading > 0.0) ? 1.0 : -1.0;
  if (!use_clockwise)
  {
    angular_vel_sign = -angular_vel_sign;
  }

  auto fill_cmd = [&](double av_sign, double rot_mag) {
    const double w_des_mag = rotateToHeadingOmegaMagnitude(rot_mag, std::abs(velocity.angular.z));
    const double w_star = av_sign * w_des_mag;
    const double alpha = std::max(1e-6, cfg_->rotation.max_angular_accel);
    const double dt = control_duration_;
    // 原地转斜坡起点：不要直接用 odom/上一帧 TEB 的大角速度。与 w_star 反向时从 0 起爬；
    // 同向时幅值不超过 rotate_to_heading_angular_vel，否则 180° 对向时会长期像「TEB 在控角速度」。
    const double w_odom = velocity.angular.z;
    double w_prev = 0.0;
    if (std::abs(w_odom) > 1e-9) {
      const bool same_sign = (w_odom > 0.0) == (w_star > 0.0);
      if (same_sign) {
        w_prev = std::copysign(std::min(std::abs(w_odom), wn), w_star);
      }
    }
    double w_cmd = w_prev + std::clamp(w_star - w_prev, -alpha * dt, alpha * dt);
    // 原逻辑：只要 |w_cmd|>0 就 max(|w_cmd|, wm)。减速时斜坡会把 ω 降到 wm 以下，再被 max 顶回 wm，
    // 在「即将对准目标」阶段会像末段突然加速。仅「从接近静止起步」时用 wm 克服执行器死区。
    if (std::fabs(w_cmd) >= 1e-6 && std::fabs(w_prev) < 1e-6 &&
      std::fabs(w_cmd) < cfg_->rotation.rotate_min_angular_vel)
    {
      w_cmd = std::copysign(cfg_->rotation.rotate_min_angular_vel, w_cmd > 0.0 ? 1.0 : -1.0);
    }
    cmd_vel.header = pose.header;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = w_cmd;
  };

  double test_angular_distance = use_clockwise ? clockwise_angle : counterclockwise_angle;
  const double cmd_primary_sign = angular_vel_sign;
  const double cmd_primary_mag = test_angular_distance;
  if (isRotationCollisionFreeDecel(pose, angular_vel_sign, test_angular_distance, velocity.angular.z))
  {
    fill_cmd(angular_vel_sign, test_angular_distance);
    publishInPlaceRotationLocalPlan(pose, angular_vel_sign, test_angular_distance);
    return true;
  }

  angular_vel_sign = -angular_vel_sign;
  test_angular_distance = use_clockwise ? counterclockwise_angle : clockwise_angle;
  if (isRotationCollisionFreeDecel(pose, angular_vel_sign, test_angular_distance, velocity.angular.z))
  {
    fill_cmd(angular_vel_sign, test_angular_distance);
    publishInPlaceRotationLocalPlan(pose, angular_vel_sign, test_angular_distance);
    // RCLCPP_INFO_THROTTLE(
    //   logger_, *clock_, 500,
    //   "原地转: computeRotate 首选 sign=%.0f、rot_mag=%.3f rad 未通过，备选 sign=%.0f、rot_mag=%.3f rad 已填 cmd "
    //   "（|dtheta|=%.3f、use_clockwise=%d、w_odom=%.3f、cmd_w=%.3f）",
    //   cmd_primary_sign,
    //   cmd_primary_mag,
    //   angular_vel_sign,
    //   test_angular_distance,
    //   abs_angular_distance,
    //   use_clockwise ? 1 : 0,
    //   velocity.angular.z,
    //   cmd_vel.twist.angular.z);
    return true;
  }

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 500,
    "原地转: 两旋转方向碰障预检均失败（computeRotateToHeadingCommand），|dtheta|=%.3f rad，use_clockwise=%d，w_odom=%.3f",
    abs_angular_distance,
    use_clockwise ? 1 : 0,
    velocity.angular.z);
  return false;
}

bool TebLocalPlannerROS::isRotationCollisionFree(
  const geometry_msgs::msg::TwistStamped & cmd_vel,
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose)
{
  
  double initial_yaw = tf2::getYaw(pose.pose.orientation);
  double abs_angular_distance = std::abs(angular_distance_to_heading);
  double abs_angular_vel = std::abs(cmd_vel.twist.angular.z);
  const std::vector<geometry_msgs::msg::Point> footprint_poly = costmap_ros_->getRobotFootprint();
  constexpr const char * kCollisionNs = "collision_rotate_legacy";
  const std::string debug_frame = collisionDebugFrameId(pose.header.frame_id);

  auto pose_blocked = [&](double yaw, double footprint_cost) -> bool {
    using namespace nav2_costmap_2d;  // NOLINT
    if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
      publishCollisionFootprintDebug(
        kCollisionNs, debug_frame,
        pose.pose.position.x, pose.pose.position.y, yaw, footprint_poly, footprint_cost);
      return true;
    }
    if (footprint_cost >= static_cast<double>(MAX_NON_OBSTACLE))
    {
      publishCollisionFootprintDebug(
        kCollisionNs, debug_frame,
        pose.pose.position.x, pose.pose.position.y, yaw, footprint_poly, footprint_cost);
      return true;
    }
    return false;
  };
  
  if (abs_angular_vel < 1e-6)
  {
    double footprint_cost = rotation_collision_checker_->footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y,
      initial_yaw, footprint_poly);
    if (pose_blocked(initial_yaw, footprint_cost)) {
      return false;
    }
    clearCollisionFootprintDebug(kCollisionNs, debug_frame);
    return true;
  }
  
  // Calculate total rotation time needed
  double total_rotation_time = abs_angular_distance / abs_angular_vel;
  int num_steps = static_cast<int>(std::ceil(total_rotation_time / control_duration_));
  
  // Ensure at least one step (check current and target)
  if (num_steps < 1)
  {
    num_steps = 1;
  }
  
  // Check collision at each step from current orientation to target orientation
  for (int step = 0; step <= num_steps; ++step)
  {
    double simulated_time = step * control_duration_;
    double rotated_angle = abs_angular_vel * simulated_time;
    
    // Clamp rotated_angle to not exceed the target
    if (rotated_angle >= abs_angular_distance)
    {
      rotated_angle = abs_angular_distance;
    }
    
    // Calculate current yaw based on rotation direction
    double current_yaw = initial_yaw;
    if (cmd_vel.twist.angular.z > 0)
    {
      current_yaw += rotated_angle;
    }
    else
    {
      current_yaw -= rotated_angle;
    }
    
    // Normalize current_yaw to [-PI, PI]
    while (current_yaw > M_PI) current_yaw -= 2.0 * M_PI;
    while (current_yaw < -M_PI) current_yaw += 2.0 * M_PI;
    
    double footprint_cost = rotation_collision_checker_->footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y,
      current_yaw, footprint_poly);
    
    if (pose_blocked(current_yaw, footprint_cost)) {
      return false;
    }
    
    // If we've reached the target, break
    if (rotated_angle >= abs_angular_distance)
    {
      break;
    }
  }
  
  clearCollisionFootprintDebug(kCollisionNs, debug_frame);
  return true;  // No collision detected
}

}  // namespace teb_local_planner
