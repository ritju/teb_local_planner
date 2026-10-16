/*********************************************************************
 * teb_narrow_backward.cpp
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

#include "teb_local_planner/detail/plan_corner_geometry.hpp"
#include "teb_local_planner/detail/async_wait.hpp"

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

using detail::kBackgroundPollSlice;
using detail::joinThreadQuietly;
using detail::detachThreadQuietly;
using detail::waitForFutureInterruptible;

// =============================================================================
// 狭窄通道局部规划：订阅 /narrow_passages，按需临时放宽倒车 TEB 参数
// =============================================================================

void TebLocalPlannerROS::updateNarrowPassageTebSettings(const bool enable_narrow_teb)
{
  
  if (enable_narrow_teb) {
    if (!narrow_teb_settings_applied_) {
      applyNarrowPassageTebSettings();
    }
  } else if (narrow_teb_settings_applied_) {
    restoreNarrowPassageTebSettings();
  }
}

void TebLocalPlannerROS::enableBackwardCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(enable_backward_mutex_);
  enable_backward_cmd_ = msg->data;
  enable_backward_cmd_received_ = true;
  RCLCPP_INFO(
    logger_,
    "/enable_backward: %s（%s）",
    msg->data ? "true" : "false",
    msg->data ? "强制倒车友好 TEB 参数" : "回退窄通道/倒车段判断");
}

void TebLocalPlannerROS::narrowPassagesCallback(
  const garage_utils_msgs::msg::Polygons::SharedPtr msg)
{
  
  {
    std::lock_guard<std::mutex> lock(narrow_polygons_mutex_);
    narrow_polygons_received_ = true;
    narrow_polygons_ = msg->polygons;
    if (narrow_polygons_.empty()) {
      latched_narrow_passage_ = false;
      RCLCPP_WARN(
        logger_,
        "狭窄通道: TEB 收到空的 /narrow_passages，切换为非窄通道模式");
    } else {
      RCLCPP_INFO(
        logger_,
        "狭窄通道: TEB 更新 /narrow_passages，多边形数量=%zu",
        narrow_polygons_.size());
    }
  }
  publishNarrowPassagesMarkers(msg->polygons);
}

void TebLocalPlannerROS::publishNarrowPassagesMarkers(
  const std::vector<geometry_msgs::msg::Polygon> & polygons)
{
  
  if (!narrow_passages_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker delete_all_marker;
  delete_all_marker.header.frame_id = cfg_->map_frame;
  delete_all_marker.ns = "narrow_passages";
  delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_all_marker);

  const rclcpp::Time marker_stamp = clock_->now();
  for (std::size_t polygon_index = 0; polygon_index < polygons.size(); ++polygon_index) {
    const auto & polygon = polygons[polygon_index];
    if (polygon.points.size() < 2) {
      continue;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = cfg_->map_frame;
    marker.header.stamp = marker_stamp;
    marker.ns = "narrow_passages";
    marker.id = static_cast<int>(polygon_index);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.06;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.0f;
    marker.color.a = 0.9f;

    marker.points.reserve(polygon.points.size() + 1);
    for (const auto & polygon_point : polygon.points) {
      geometry_msgs::msg::Point point;
      point.x = polygon_point.x;
      point.y = polygon_point.y;
      point.z = 0.1;
      marker.points.push_back(point);
    }
    const auto & first_point = polygon.points.front();
    geometry_msgs::msg::Point closing_point;
    closing_point.x = first_point.x;
    closing_point.y = first_point.y;
    closing_point.z = 0.1;
    marker.points.push_back(closing_point);
    marker_array.markers.push_back(marker);
  }

  narrow_passages_marker_pub_->publish(marker_array);
}

bool TebLocalPlannerROS::narrowPolygonsAvailable() const
{
  
  std::lock_guard<std::mutex> lock(narrow_polygons_mutex_);
  return narrow_polygons_received_ && !narrow_polygons_.empty();
}

bool TebLocalPlannerROS::isPointInNarrowPassage(const double x, const double y) const
{
  
  std::lock_guard<std::mutex> lock(narrow_polygons_mutex_);
  for (const auto & polygon : narrow_polygons_) {
    if (polygon.points.size() < 3) {
      continue;
    }
    bool inside = false;
    size_t j = polygon.points.size() - 1;
    for (size_t i = 0; i < polygon.points.size(); ++i) {
      const auto & pi = polygon.points[i];
      const auto & pj = polygon.points[j];
      const bool intersect =
        ((pi.y > y) != (pj.y > y)) &&
        (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y + 1e-9) + pi.x);
      if (intersect) {
        inside = !inside;
      }
      j = i;
    }
    if (inside) {
      return true;
    }
  }
  return false;
}

bool TebLocalPlannerROS::isFootprintFullyOutsideNarrowPassages(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  
  std::lock_guard<std::mutex> lock(narrow_polygons_mutex_);
  if (!narrow_polygons_received_ || narrow_polygons_.empty()) {
    return true;
  }
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);
  const nav2_costmap_2d::Footprint footprint = costmap_ros_->getRobotFootprint();
  for (const auto & pt : footprint) {
    const double wx = pose.pose.position.x + pt.x * cos_y - pt.y * sin_y;
    const double wy = pose.pose.position.y + pt.x * sin_y + pt.y * cos_y;
    for (const auto & polygon : narrow_polygons_) {
      if (polygon.points.size() < 3) {
        continue;
      }
      bool inside = false;
      size_t j = polygon.points.size() - 1;
      for (size_t i = 0; i < polygon.points.size(); ++i) {
        const auto & pi = polygon.points[i];
        const auto & pj = polygon.points[j];
        const bool intersect =
          ((pi.y > wy) != (pj.y > wy)) &&
          (wx < (pj.x - pi.x) * (wy - pi.y) / (pj.y - pi.y + 1e-9) + pi.x);
        if (intersect) {
          inside = !inside;
        }
        j = i;
      }
      if (inside) {
        return false;
      }
    }
  }
  return true;
}

void TebLocalPlannerROS::updateNarrowPassageLatch(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  
  const bool prev_latched = latched_narrow_passage_;
  if (!narrowPolygonsAvailable()) {
    latched_narrow_passage_ = false;
    if (prev_latched) {
      RCLCPP_INFO(
        logger_,
        "狭窄通道: TEB 无有效多边形，Latch 由 true 置 false");
    }
    return;
  }
  if (isPointInNarrowPassage(robot_pose.pose.position.x, robot_pose.pose.position.y)) {
    latched_narrow_passage_ = true;
    if (!prev_latched) {
      RCLCPP_INFO(
        logger_,
        "狭窄通道: TEB base_link (%.2f, %.2f) 进入窄通道，Latch 置 true",
        robot_pose.pose.position.x, robot_pose.pose.position.y);
    }
    return;
  }
  if (isFootprintFullyOutsideNarrowPassages(robot_pose)) {
    latched_narrow_passage_ = false;
    if (prev_latched) {
      RCLCPP_INFO(
        logger_,
        "狭窄通道: TEB footprint 完全离开窄通道，Latch 由 true 置 false");
    }
  }
}

bool TebLocalPlannerROS::hasReverseSegmentAhead() const
{
  if (global_plan_.size() < 3 || cfg_ == nullptr) {
    return false;
  }
  const int start_idx = 0;
  const double lookahead = cfg_->trajectory.reverse_segment_lookahead_dist;
  const int end_idx = (lookahead > 1e-9)
    ? lastIdxWithinArc(global_plan_, start_idx, lookahead)
    : static_cast<int>(global_plan_.size()) - 1;
  if (end_idx - start_idx + 1 < 3) {
    return false;
  }
  const std::vector<geometry_msgs::msg::PoseStamped> window(
    global_plan_.begin() + start_idx,
    global_plan_.begin() + end_idx + 1);
  return hasReverseSegmentInPlan(window);
}

bool TebLocalPlannerROS::hasReverseSegmentInPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan) const
{
  // 跳过首段：从 plan[1] 起做非重叠扩窗，按倒车弧长 / 判定区间总弧长判定
  if (plan.size() < 3) {
    return false;
  }

  const double min_seg_len = cfg_->trajectory.reverse_segment_min_segment_length_m;
  const double min_angle_deg = cfg_->trajectory.reverse_segment_min_angle_deg;
  const double ratio_thresh = std::clamp(
    cfg_->trajectory.reverse_segment_arc_length_ratio, 0.0, 1.0);

  // 判定区间总弧长：相邻路点折线长 plan[1] → plan.back()
  double plan_arc_length = 0.0;
  for (size_t k = 1; k + 1 < plan.size(); ++k) {
    plan_arc_length += std::hypot(
      plan[k + 1].pose.position.x - plan[k].pose.position.x,
      plan[k + 1].pose.position.y - plan[k].pose.position.y);
  }
  if (plan_arc_length < min_seg_len || ratio_thresh <= 0.0) {
    return false;
  }

  const double reverse_arc_needed = ratio_thresh * plan_arc_length;
  double reverse_arc_length = 0.0;
  double consumed_arc_length = 0.0;

  size_t i = 1;
  while (i + 1 < plan.size()) {
    // 弦长过短则向前扩窗至 i+2, i+3, ...
    size_t j = i + 1;
    double dx = plan[j].pose.position.x - plan[i].pose.position.x;
    double dy = plan[j].pose.position.y - plan[i].pose.position.y;
    double chord_length = std::hypot(dx, dy);
    while (chord_length < min_seg_len && j + 1 < plan.size()) {
      ++j;
      dx = plan[j].pose.position.x - plan[i].pose.position.x;
      dy = plan[j].pose.position.y - plan[i].pose.position.y;
      chord_length = std::hypot(dx, dy);
    }
    if (chord_length < min_seg_len) {
      break;
    }

    // 用路径折线弧长计入占比（与 plan_arc_length 同口径）；夹角用弦方向
    double window_arc_length = 0.0;
    for (size_t k = i; k < j; ++k) {
      window_arc_length += std::hypot(
        plan[k + 1].pose.position.x - plan[k].pose.position.x,
        plan[k + 1].pose.position.y - plan[k].pose.position.y);
    }

    const double yaw = tf2::getYaw(plan[i].pose.orientation);
    const double dot = dx * std::cos(yaw) + dy * std::sin(yaw);
    const double cos_angle = std::clamp(dot / chord_length, -1.0, 1.0);
    const double heading_segment_angle_deg = std::acos(cos_angle) * 180.0 / M_PI;
    if (heading_segment_angle_deg >= min_angle_deg) {
      reverse_arc_length += window_arc_length;
      RCLCPP_DEBUG(
        logger_,
        "狭窄通道: 倒车段 plan[%zu]->[%zu], angle=%.2f deg, "
        "rev_arc=%.3f/%.3f (need=%.3f)",
        i, j, heading_segment_angle_deg,
        reverse_arc_length, plan_arc_length, reverse_arc_needed);
      if (reverse_arc_length >= reverse_arc_needed) {
        return true;
      }
    }

    consumed_arc_length += window_arc_length;
    // 剩余全算倒车仍不够阈值 → 提前判假
    if (reverse_arc_length + (plan_arc_length - consumed_arc_length) < reverse_arc_needed) {
      return false;
    }
    i = j;
  }
  return false;
}

void TebLocalPlannerROS::publishBackwardMode(const bool backward)
{
  
  if (!backward_mode_pub_) {
    return;
  }
  if (backward_mode_has_published_ && backward_mode_published_ == backward) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = backward;
  backward_mode_pub_->publish(msg);
  backward_mode_published_ = backward;
  backward_mode_has_published_ = true;
  RCLCPP_INFO(
    logger_,
    "/backward_mode 已发布: %s",
    backward ? "true (倒车)" : "false (前进)");
}

void TebLocalPlannerROS::resetBackwardModePublicationState(const bool publish_false)
{
  
  backward_exit_false_count_ = 0;
  backward_exit_window_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  if (publish_false) {
    publishBackwardMode(false);
    backward_mode_initial_sent_ = true;
  } else {
    backward_mode_published_ = false;
    backward_mode_has_published_ = false;
    backward_mode_initial_sent_ = false;
  }
}

void TebLocalPlannerROS::enterDualArcReverseLatch()
{
  const bool newly = !dual_arc_reverse_active_;
  dual_arc_reverse_active_ = true;
  dual_arc_reverse_enter_time_ = clock_->now();
  dual_arc_reverse_forward_count_ = 0;
  dual_arc_reverse_fail_count_ = 0;
  if (newly) {
    RCLCPP_INFO(logger_, "双弧失败: 进入倒车友好权重 latch");
  } else {
    RCLCPP_INFO(logger_, "双弧失败: 超时 probe 仍不可行，重新 latch");
  }
}

void TebLocalPlannerROS::exitDualArcReverseLatch()
{
  if (!dual_arc_reverse_active_) {
    return;
  }
  dual_arc_reverse_active_ = false;
  dual_arc_reverse_forward_count_ = 0;
  dual_arc_reverse_fail_count_ = 0;
  RCLCPP_INFO(logger_, "双弧失败: 退出倒车友好权重 latch");
}

void TebLocalPlannerROS::resetDualArcReverseLatch()
{
  dual_arc_reverse_active_ = false;
  dual_arc_reverse_forward_count_ = 0;
  dual_arc_reverse_fail_count_ = 0;
  dual_arc_reverse_enter_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void TebLocalPlannerROS::updateBackwardModePublication(const bool reverse_segment)
{
  
  if (!backward_mode_initial_sent_) {
    publishBackwardMode(false);
    backward_mode_initial_sent_ = true;
  }

  if (!backward_mode_published_) {
    if (reverse_segment) {
      publishBackwardMode(true);
      backward_exit_false_count_ = 0;
      backward_exit_window_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    return;
  }

  if (reverse_segment) {
    backward_exit_false_count_ = 0;
    backward_exit_window_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return;
  }

  const rclcpp::Time now = clock_->now();
  const int required_false_frames = std::max(1, cfg_->trajectory.backward_check_num);
  const double check_duration_s = std::max(0.0, cfg_->trajectory.backward_check_duration);

  if (backward_exit_false_count_ == 0) {
    backward_exit_window_start_ = now;
    backward_exit_false_count_ = 1;
    return;
  }

  const double elapsed_s = (now - backward_exit_window_start_).seconds();
  if (elapsed_s > check_duration_s) {
    backward_exit_window_start_ = now;
    backward_exit_false_count_ = 1;
    RCLCPP_DEBUG(
      logger_,
      "/backward_mode 退出防抖超时 (%.3fs > %.3fs)，计数清零重来",
      elapsed_s, check_duration_s);
    return;
  }

  ++backward_exit_false_count_;
  if (backward_exit_false_count_ >= static_cast<size_t>(required_false_frames)) {
    publishBackwardMode(false);
    backward_exit_false_count_ = 0;
    backward_exit_window_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
}

void TebLocalPlannerROS::applyNarrowPassageTebSettings()
{
  
  if (narrow_teb_settings_applied_) {
    return;
  }
  normal_weight_kinematics_forward_drive_ = cfg_->optim.weight_kinematics_forward_drive;
  normal_delete_detours_backwards_ = cfg_->hcp.delete_detours_backwards;
  normal_allow_init_with_backwards_motion_ = cfg_->trajectory.allow_init_with_backwards_motion;
  cfg_->optim.weight_kinematics_forward_drive =
    cfg_->optim.weight_kinematics_forward_drive_in_narrow_passages;
  cfg_->hcp.delete_detours_backwards = false;
  cfg_->trajectory.allow_init_with_backwards_motion = true;
  narrow_teb_settings_applied_ = true;
  RCLCPP_INFO(
    logger_,
    "狭窄通道: TEB 应用倒车友好参数 "
    "(forward_drive %.2f->%.2f, delete_detours_backwards=false, "
    "allow_init_with_backwards_motion=true)",
    normal_weight_kinematics_forward_drive_,
    cfg_->optim.weight_kinematics_forward_drive_in_narrow_passages);
}

void TebLocalPlannerROS::restoreNarrowPassageTebSettings()
{
  
  if (!narrow_teb_settings_applied_) {
    return;
  }
  cfg_->optim.weight_kinematics_forward_drive = normal_weight_kinematics_forward_drive_;
  cfg_->hcp.delete_detours_backwards = normal_delete_detours_backwards_;
  cfg_->trajectory.allow_init_with_backwards_motion = normal_allow_init_with_backwards_motion_;
  narrow_teb_settings_applied_ = false;
  RCLCPP_DEBUG(
    logger_,
    "狭窄通道: TEB 已恢复正常参数 (forward_drive=%.2f)",
    normal_weight_kinematics_forward_drive_);
}

 
 void TebLocalPlannerROS::switchParameterMode(bool enable_edge_mode)
 {
   
   if (!initialized_ || !nh_.lock()) {
     return;
   }

   auto node = nh_.lock();

   // Always enforce max_vel_x in edge mode, since speed limit logic may overwrite it each cycle
   if (enable_edge_mode) {
     cfg_->robot.max_vel_x = cfg_->wall_line.edge_max_vel_x;
   }

   // Avoid redundant parameter updates - MUST be first!
   if (enable_edge_mode == is_edge_following_mode_) {
     return;
   }

   try {
    if (enable_edge_mode) {
       // Switch to edge-following mode
       RCLCPP_INFO(logger_, "切换到贴边模式参数");
       cfg_->optim.weight_optimaltime = cfg_->wall_line.edge_weight_optimaltime;
       cfg_->obstacles.min_obstacle_dist = cfg_->wall_line.edge_min_obstacle_dist;
       cfg_->optim.weight_inflation = cfg_->wall_line.edge_weight_inflation;
       cfg_->robot.acc_lim_theta = cfg_->wall_line.edge_acc_lim_theta;
       cfg_->robot.max_vel_theta = cfg_->wall_line.edge_max_vel_theta;

       // Set footprint vertices
       node->set_parameter(rclcpp::Parameter(name_ + "." + "footprint_model.vertices", cfg_->wall_line.edge_footprint_vertices));
       is_edge_following_mode_ = true;
     } else {
       // Switch to normal mode
       RCLCPP_INFO(logger_, "切换到正常模式参数");
       cfg_->optim.weight_optimaltime = normal_weight_optimaltime_;
       cfg_->obstacles.min_obstacle_dist = normal_min_obstacle_dist_;
       cfg_->optim.weight_inflation = normal_weight_inflation_;
       cfg_->robot.acc_lim_theta = cfg_max_angular_acc_;
       cfg_->robot.max_vel_theta = cfg_max_angular_vel_;
       // Set footprint vertices
       node->set_parameter(rclcpp::Parameter(name_ + "." + "footprint_model.vertices", normal_footprint_vertices_));
       is_edge_following_mode_ = false;
     }

    if (cfg_->wall_line.switch_static_layer) {
      bool desired_state = !enable_edge_mode;
      desired_static_layer_state_ = desired_state;

      if (desired_state == current_static_layer_state_.load()) {
        RCLCPP_INFO(logger_, "static_layer.enabled 已是目标状态 %d，跳过服务调用", desired_state ? 1 : 0);
      } else if (desired_state) {
        spawnBackgroundTask([this]() {
          auto start = std::chrono::steady_clock::now();
          auto delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(cfg_->wall_line.static_layer_enable_delay));

          while (isBackgroundWorkAllowed() &&
                 std::chrono::steady_clock::now() - start < delay_ms) {
            if (!desired_static_layer_state_.load()) {
              RCLCPP_DEBUG(logger_, "延迟开启 static_layer 已取消：目标状态已变化");
              return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }

          if (isBackgroundWorkAllowed() && desired_static_layer_state_.load()) {
            setStaticLayerEnabled(true, true);
          }
        });
      } else {
        setStaticLayerEnabled(false, false);
      }
    }
    
    if (cfg_->wall_line.switch_local_footprint) {
      bool desired_footprint_state = enable_edge_mode;
      desired_local_footprint_state_ = desired_footprint_state;

      if (desired_footprint_state == current_local_footprint_state_.load()) {
        RCLCPP_INFO(logger_, "局部 footprint 已是目标状态 %d，跳过服务调用", desired_footprint_state ? 1 : 0);
      } else if (!desired_footprint_state) {
        // 退出贴边：等待大轮廓无碰撞后再恢复（见 updatePendingNormalFootprintRestore）
        armPendingNormalFootprintRestore();
      } else {
        clearPendingNormalFootprintRestore();
        setLocalFootprintEnabled(true, false);
      }
    }

    if (cfg_->wall_line.switch_global_footprint) {
      bool desired_footprint_state = enable_edge_mode;
      desired_global_footprint_state_ = desired_footprint_state;

      if (desired_footprint_state == current_global_footprint_state_.load()) {
        RCLCPP_INFO(logger_, "全局 footprint 已是目标状态 %d，跳过服务调用", desired_footprint_state ? 1 : 0);
      } else if (!desired_footprint_state) {
        armPendingNormalFootprintRestore();
      } else {
        clearPendingNormalFootprintRestore();
        setGlobalFootprintEnabled(true, false);
      }
    }
    // Delayed max_vel_x restoration when switching to normal mode
    if (!enable_edge_mode) {
      desired_normal_vel_x_restore_ = true;
      spawnBackgroundTask([this]() {
        auto start = std::chrono::steady_clock::now();
        auto delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(cfg_->wall_line.static_layer_enable_delay));

        while (isBackgroundWorkAllowed() &&
               std::chrono::steady_clock::now() - start < delay_ms) {
          if (!desired_normal_vel_x_restore_.load()) {
            RCLCPP_DEBUG(logger_, "延迟恢复 max_vel_x 已取消：已切回贴边模式");
            return;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!isBackgroundWorkAllowed() || !desired_normal_vel_x_restore_.load()) {
          return;
        }
        {
            std::lock_guard<std::mutex> l(speed_limit_mutex_);
            if (has_speed_limit_) {
              // Ensure non-negative limit and cap by robot's nominal base maximum
              const double limit = std::max(0.0, speed_limit_linear_x_);
              if (std::isfinite(limit) && speed_limit_linear_x_ > 0) {
                if (limit < safe_linear_speed_limit_ && cfg_->robot.max_vel_x != limit) {
                  cfg_->robot.max_vel_x = std::min(limit, safe_linear_speed_limit_);
                  RCLCPP_INFO(logger_, "切回正常模式时应用速度限制，当前最大线速度: %.2f m/s", cfg_->robot.max_vel_x);
                } else if (limit >= safe_linear_speed_limit_ && cfg_->robot.max_vel_x != safe_linear_speed_limit_) {
                  cfg_->robot.max_vel_x = safe_linear_speed_limit_;
                  RCLCPP_INFO(logger_, "切回正常模式时收到的速度限制大于安全上限，已钳到: %.2f m/s", safe_linear_speed_limit_);
                }
              } else if (!std::isfinite(limit) || limit <= 0) {
                cfg_->robot.max_vel_x = cfg_max_vel_x_;
                RCLCPP_INFO_THROTTLE(logger_, *(clock_), 5000, "切回正常模式时速度限制无效 (非有限或 <=0)，当前速度上限: %.2f m/s", cfg_->robot.max_vel_x);
              }
            } else {
              cfg_->robot.max_vel_x = cfg_max_vel_x_;
              RCLCPP_INFO(logger_, "切回正常模式时未设置速度限制，恢复正常 max_vel_x: %.2f m/s", cfg_max_vel_x_);
            }
        }
      });
    } else {
      desired_normal_vel_x_restore_ = false;
    }
   } catch (const std::exception& ex) {
     RCLCPP_WARN(logger_, "切换参数模式失败: %s", ex.what());
   }
 }

}  // namespace teb_local_planner
