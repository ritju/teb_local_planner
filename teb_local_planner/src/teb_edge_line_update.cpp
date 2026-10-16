/*********************************************************************
 * teb_edge_line_update.cpp
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

using nav2_util::declare_parameter_if_not_declared;

namespace teb_local_planner
{
bool TebLocalPlannerROS::isVehicleInEdgeFollowingExitCorridor(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const Eigen::Vector2d& wall_w0,
  const Eigen::Vector2d& wall_w1,
  const geometry_msgs::msg::Pose& vehicle_pose)
{
  (void)robot_pose;
  (void)wall_w0;
  (void)wall_w1;

  if (vehicle_monitor_corridor_.valid) {
    return isPointInWallMonitorCorridor(
      Eigen::Vector2d(vehicle_pose.position.x, vehicle_pose.position.y),
      vehicle_monitor_corridor_);
  }

  const Eigen::Vector2d robot_position(
    robot_pose.pose.position.x, robot_pose.pose.position.y);
  EffWallFrame wall;
  if (!buildEffWallFrameFromSegment(wall_w0, wall_w1, robot_position, *cfg_, wall)) {
    return false;
  }

  const WallMonitorCorridor fallback = buildWallMonitorCorridor(
    wall,
    robot_position,
    tf2::getYaw(robot_pose.pose.orientation),
    {},
    cfg_->wall_line.vehicle_exit_corridor_rear_m,
    cfg_->wall_line.vehicle_exit_corridor_front_m,
    cfg_->wall_line.vehicle_exit_corridor_wall_inner_m,
    cfg_->wall_line.vehicle_exit_corridor_wall_robot_side_m,
    *cfg_);

  if (!fallback.valid) {
    return false;
  }

  return isPointInWallMonitorCorridor(
    Eigen::Vector2d(vehicle_pose.position.x, vehicle_pose.position.y),
    fallback);
}

void TebLocalPlannerROS::updateWallLineVec(
   const std::vector<nav_msgs::msg::Path>& wall_line, 
   nav_msgs::msg::Path& input_path,
   const double parallel_tolerance,
   const double distance_tolerance,
   const geometry_msgs::msg::PoseStamped& robot_pose)
 {
   
   std::lock_guard<std::mutex> l(update_wall_line_mutex_);
   if (input_path.poses.size() < 2)
   {
     if(wall_line_points_.size() > 0) wall_line_points_.clear();
     RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "Edge following: 路径点数太少(%zu)，退出贴边模式，清空贴边配置",
                 input_path.poses.size());
     switchParameterMode(false);
     return;
   }

   // 贴边时：车辆在「方向前后 × 墙法向内外」走廊内则退出（有墙段用走廊；否则退回圆形距离阈值）
   bool has_near_vehicles = false;
   {
     std::lock_guard<std::mutex> veh_lock(global_vehicle_poses_mutex_);
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 机器人附近车辆数量: %ld", global_vehicle_poses_.size());
     const bool have_wall_segment = (wall_line_points_.size() >= 2);
     Eigen::Vector2d w0, w1;
     if (have_wall_segment) {
       w0 = wall_line_points_[0];
       w1 = wall_line_points_[1];
       RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 墙线起点: (%.2f, %.2f), 终点: (%.2f, %.2f)", w0.x(), w0.y(), w1.x(), w1.y());
     }
     for (const auto& vehicle : global_vehicle_poses_) {
       bool in_exit_region = false;
       if (have_wall_segment) {
         in_exit_region = isVehicleInEdgeFollowingExitCorridor(robot_pose, w0, w1, vehicle.pose);
       } else {
         in_exit_region = distance_points2d(robot_pose.pose.position, vehicle.pose.position) <
           cfg_->wall_line.close_vehicle_distance_threshold;
         RCLCPP_INFO(logger_, "Edge following: 车辆距离机器人太近：%.2f m，小于阈值：%.2f m", 
                     distance_points2d(robot_pose.pose.position, vehicle.pose.position), 
                     cfg_->wall_line.close_vehicle_distance_threshold);
       }
       if (in_exit_region) {
         if (have_wall_segment) {
           RCLCPP_INFO(logger_,
             "Edge following: 车辆在贴边检查区域范围内，退出贴边模式");
         } else {
           RCLCPP_INFO(logger_,
             "Edge following: 车辆距离机器人太近：%.2f m，小于阈值：%.2f m，退出贴边模式",
             distance_points2d(robot_pose.pose.position, vehicle.pose.position),
             cfg_->wall_line.close_vehicle_distance_threshold);
         }
         has_near_vehicles = true;
         break;
       }
     }
   }
   if (has_near_vehicles)
   {
    if(wall_line_points_.size() > 0) wall_line_points_.clear();
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "Edge following: 机器人附近有车辆，退出贴边模式，清空贴边配置");
    switchParameterMode(false);
    return;
   }

   // Check for protruding obstacles that prohibit entering edge-following mode
   if (hasProtrudingObstacleInMonitorCorridor(robot_pose))
   {
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 机器人附近有障碍物，退出贴边模式，清空贴边配置");
     if(wall_line_points_.size() > 0) wall_line_points_.clear();
     switchParameterMode(false);
     return;
   }

    // 提取机器人当前朝向（从四元数转换为偏航角）
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    // 提取输入路径的首尾点
    const auto& start_pose = input_path.poses.front().pose.position;
    const auto& end_pose = input_path.poses.back().pose.position;
    
    // 计算路径方向向量（从起点指向终点）
    const double dx_path = end_pose.x - start_pose.x;
    const double dy_path = end_pose.y - start_pose.y;
    const double path_length = std::hypot(dx_path, dy_path);
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 路径长度: %.2f", path_length);
    if (path_length > cfg_->wall_line.min_path_line_length) {
      // 计算路径方向角度（相对于世界坐标系）
      const double path_yaw = std::atan2(dy_path, dx_path);
      
      // 计算机器人朝向与路径方向的夹角
      double angle_diff = path_yaw - robot_yaw;
      
      // 归一化角度到[-π, π]范围
      while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
      
      // 检查夹角是否超出±45度范围
      if (std::fabs(angle_diff) > M_PI / 4) {
        if(wall_line_points_.size() > 0) wall_line_points_.clear();
        RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "Edge following: 机器人方向与路径方向夹角太大: %.2f，退出贴边模式，清空贴边配置",
                    angle_diff * 180.0 / M_PI);
        switchParameterMode(false);
        return;
      }
    } else {
      if(wall_line_points_.size() > 0) wall_line_points_.clear();
      RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "Edge following: 路径长度太短(%.2f)，退出贴边模式，清空贴边配置", path_length);
      switchParameterMode(false);
      return;
    }
    const double dir_input_x = dx_path / path_length;
    const double dir_input_y = dy_path / path_length;

    // 遍历所有墙线
    bool found_valid_wall = false;
    double min_distance = std::numeric_limits<double>::max();
    nav_msgs::msg::Path best_wall_path;
    geometry_msgs::msg::Point best_wall_start, best_wall_end;
    double best_robot_to_edge_distance = std::numeric_limits<double>::max();
    for(const auto& wall_path : wall_line) 
    {
      if(wall_path.poses.size() < 2) continue; // 无效墙线
      
      // 提取墙线端点
      const auto& wall_start = wall_path.poses.front().pose.position;
      const auto& wall_end = wall_path.poses.back().pose.position;
      
      // 计算墙线方向向量
      const double dx_wall = wall_end.x - wall_start.x;
      const double dy_wall = wall_end.y - wall_start.y;
      const double wall_length = std::hypot(dx_wall, dy_wall);
      if(wall_length < min_wall_line_length_) continue; // 无效墙线
      
      const double dir_wall_x = dx_wall / wall_length;
      const double dir_wall_y = dy_wall / wall_length;
      
      // 计算平行度（余弦值）
      const double dot_product = dir_input_x * dir_wall_x + dir_input_y * dir_wall_y;
      const double cos_theta = std::fabs(dot_product);
      const double parallel_threshold = std::cos(parallel_tolerance / 180 * M_PI);
      
      // 检查平行度是否满足要求
      if(cos_theta < parallel_threshold) continue;
      
      // 计算垂直距离
      const double A = dy_wall;
      const double B = -dx_wall;
      const double C = wall_end.x * wall_start.y - wall_start.x * wall_end.y;
      
      const double numerator = std::fabs(A * start_pose.x + B * start_pose.y + C);
      const double denominator = std::hypot(A, B);
      const double avg_distance = numerator / denominator;
      const double robot_to_edge_distance =
        std::fabs(A * robot_pose.pose.position.x + B * robot_pose.pose.position.y + C) / denominator;
      
      RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 墙线与路径平行度: %.2f, 距离: %.2f", 
                           acos(cos_theta) * 180.0 / M_PI, avg_distance);
      RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 机器人到墙线距离: %.2f", 
                           robot_to_edge_distance);
      
      // 记录满足平行度且距离最小的墙线
      if (avg_distance <= distance_tolerance && avg_distance < min_distance) 
      {
        found_valid_wall = true;
        min_distance = avg_distance;
        best_robot_to_edge_distance = robot_to_edge_distance;
        best_wall_path = wall_path;
        best_wall_start = wall_start;
        best_wall_end = wall_end;
      } else {
        RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 墙线与路径平行度不满足要求: %.2f, 距离: %.2f", 
                             acos(cos_theta) * 180.0 / M_PI, avg_distance);
      }
    }

    // 5. 如果找到最佳匹配的墙线，更新配置
    if (found_valid_wall) 
    {
      const Eigen::Vector2d candidate_wall_start(best_wall_start.x, best_wall_start.y);
      const Eigen::Vector2d candidate_wall_end(best_wall_end.x, best_wall_end.y);
      const Eigen::Vector2d robot_pos(
        robot_pose.pose.position.x, robot_pose.pose.position.y);
      if (protrusionBlocksEdgeEntryForWallSegment(
          candidate_wall_start, candidate_wall_end, robot_pos))
      {
        return;
      }

      if(wall_line_points_.size() > 0) wall_line_points_.clear();
      wall_line_points_.emplace_back(Eigen::Vector2d(best_wall_start.x, best_wall_start.y));
      wall_line_points_.emplace_back(Eigen::Vector2d(best_wall_end.x, best_wall_end.y));
      updateScheduledWallDistWeights(best_robot_to_edge_distance);
      wall_line_update_time_ = clock_->now();
      std_msgs::msg::Float32 distance;
      distance.data = static_cast<float>(best_robot_to_edge_distance);
      edge_distance_publisher_->publish(distance);
      
      // Switch to edge-following mode parameters
      switchParameterMode(true);
      
      RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 选择最佳墙线距离: %.2f", min_distance);
      RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 最小障碍物距离: %.2f, 最优时间权重: %.2f, 墙线距离权重: %.2f, 侧隙Pull: %.2f", 
                           cfg_->obstacles.min_obstacle_dist, cfg_->optim.weight_optimaltime, 
                           cfg_->optim.weight_wall_line_dist, cfg_->optim.weight_wall_side_pull);
      
      // 发布可视化标记
      visualization_msgs::msg::Marker marker_msg;
      marker_msg.ns = "teb_local_planner";
      marker_msg.id = 0;
      marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker_msg.action = visualization_msgs::msg::Marker::ADD;
      marker_msg.scale.x = 0.1;
      marker_msg.color.r = 1.0;
      marker_msg.color.g = 1.0;
      marker_msg.color.b = 0.0;
      marker_msg.color.a = 1.0;
      
      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = best_wall_start.x;
      start_point.y = best_wall_start.y;
      start_point.z = 0.0;
      
      end_point.x = best_wall_end.x;
      end_point.y = best_wall_end.y;
      end_point.z = 0.0;
      
      marker_msg.points.push_back(start_point);
      marker_msg.points.push_back(end_point);
      
      marker_msg.header.stamp = clock_->now();
      marker_msg.header.frame_id = input_path.header.frame_id;
      
      wall_line_marker_publisher_->publish(marker_msg);
    }
    else 
    {
      RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "Edge following: 没有找到合适的墙线，准备退出贴边模式，清空贴边配置");
      // 6. 如果没有找到合适的墙线，检查是否需要清除现有配置
      if(wall_line_points_.size() > 0)
      {
        auto time_diff = clock_->now() - wall_line_update_time_;
        if (time_diff.seconds() > keep_wall_line_time_)
        {
          wall_line_points_.clear();
          // Switch back to normal mode parameters
          switchParameterMode(false);
        }
      }
    }
 }
 
 void TebLocalPlannerROS::updateCurbLineVec(
   const nav_msgs::msg::Path curb_line, 
   nav_msgs::msg::Path& input_path,
   const double parallel_tolerance,
   const double distance_tolerance,
   const geometry_msgs::msg::PoseStamped& robot_pose)
 {
   
   std::lock_guard<std::mutex> l(update_curb_line_mutex_);

   // 如果输入路径长度小于2，则清空wall_line_points_并退出
   if (input_path.poses.size() < 2) {
     if(wall_line_points_.size() > 0) wall_line_points_.clear();
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 路径点数太少(%zu)，退出贴边模式，清空贴边配置",
                input_path.poses.size());
    switchParameterMode(false);
    return;
   }

   // 贴边时：车辆在「方向前后 × 墙线法向内外」走廊内则退出（与 updateWallLineVec 逻辑一致）
   bool has_near_vehicles = false;
   {
     std::lock_guard<std::mutex> veh_lock(global_vehicle_poses_mutex_);
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 机器人附近车辆数量: %ld", 
                          global_vehicle_poses_.size());
     const bool have_wall_segment = (wall_line_points_.size() >= 2);
     Eigen::Vector2d w0, w1;
     if (have_wall_segment) {
       w0 = wall_line_points_[0];
       w1 = wall_line_points_[1];
     }
     for (const auto& vehicle : global_vehicle_poses_) {
       bool in_exit_region = false;
       if (have_wall_segment) {
         in_exit_region = isVehicleInEdgeFollowingExitCorridor(robot_pose, w0, w1, vehicle.pose);
       } else {
         in_exit_region = distance_points2d(robot_pose.pose.position, vehicle.pose.position) <
           cfg_->wall_line.close_vehicle_distance_threshold;
       }
       if (in_exit_region) {
         if (have_wall_segment) {
           RCLCPP_INFO(logger_,
             "[updateCurbLineVec] Edge following: 车辆在贴边检查区域范围内，退出贴边模式");
         } else {
           RCLCPP_INFO(logger_,
             "[updateCurbLineVec] Edge following: 车辆距离机器人太近：%.2f m，小于阈值：%.2f m，退出贴边模式",
             distance_points2d(robot_pose.pose.position, vehicle.pose.position),
             cfg_->wall_line.close_vehicle_distance_threshold);
         }
         has_near_vehicles = true;
         break;
       }
     }
   }
   if (has_near_vehicles)
   {
    if(wall_line_points_.size() > 0) wall_line_points_.clear();
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 机器人附近有车辆，退出贴边模式，清空贴边配置");
    switchParameterMode(false);
    return;
   }

   // Check for protruding obstacles that prohibit entering edge-following mode
   if (hasProtrudingObstacleInMonitorCorridor(robot_pose))
   {
     RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 机器人附近有障碍物，退出贴边模式，清空贴边配置");
     if(wall_line_points_.size() > 0) wall_line_points_.clear();
     switchParameterMode(false);
     return;
   }

    // 提取机器人当前朝向（从四元数转换为偏航角）
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    
    // 提取输入路径的首尾点
    const auto& start_pose = input_path.poses.front().pose.position;
    const auto& end_pose = input_path.poses.back().pose.position;
    
    // 计算路径方向向量（从起点指向终点）
    const double dx_path = end_pose.x - start_pose.x;
    const double dy_path = end_pose.y - start_pose.y;
    const double path_length = std::hypot(dx_path, dy_path);
    // 如果路径长度小于min_wall_line_length_，则清空wall_line_points_并退出
    if (path_length > cfg_->wall_line.min_path_line_length) {
      // 计算路径方向角度（相对于世界坐标系）
      const double path_yaw = std::atan2(dy_path, dx_path);
      
      // 计算机器人朝向与路径方向的夹角
      double angle_diff = path_yaw - robot_yaw;
      
      // 归一化角度到[-π, π]范围
      while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
      
      // 检查夹角是否超出±45度范围,如果超出则清空wall_line_points_并退出
      if (std::fabs(angle_diff) > M_PI / 4) {
        if(wall_line_points_.size() > 0) wall_line_points_.clear();
        RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 机器人方向与路径方向夹角太大: %.2f，退出贴边模式，清空贴边配置",
                    angle_diff * 180.0 / M_PI);
        switchParameterMode(false);
        return;
      }
    } else {
      // 6. 如果没有找到合适的墙线，检查是否需要清除现有配置
      if(wall_line_points_.size() > 0) wall_line_points_.clear();
      switchParameterMode(false);
      return;
    }
   
   const double dir_input_x = dx_path / path_length;
   const double dir_input_y = dy_path / path_length;
 
   // 3. 遍历所有墙线
   bool found_valid_wall = false;
   double min_distance = std::numeric_limits<double>::max();
   nav_msgs::msg::Path best_wall_path;
   geometry_msgs::msg::Point best_wall_start, best_wall_end;
   double best_robot_to_edge_distance = std::numeric_limits<double>::max();
   if(curb_line.poses.size() > 1)
   {
     // 4. 提取墙线端点
     const auto& wall_start = curb_line.poses.front().pose.position;
     const auto& wall_end = curb_line.poses.back().pose.position;
     
     // 5. 计算墙线方向向量
     const double dx_wall = wall_end.x - wall_start.x;
     const double dy_wall = wall_end.y - wall_start.y;
     const double wall_length = std::hypot(dx_wall, dy_wall);
     if(wall_length > min_wall_line_length_)
     {
       RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 墙线长度: %.2f !", wall_length);
       const double dir_wall_x = dx_wall / wall_length;
       const double dir_wall_y = dy_wall / wall_length;
       
       // 计算平行度（余弦值）
       const double dot_product = dir_input_x * dir_wall_x + dir_input_y * dir_wall_y;
       const double cos_theta = std::fabs(dot_product);
       const double parallel_threshold = std::abs(std::cos(parallel_tolerance / 180 * M_PI));
       if (cos_theta <= 1.0)
       {
         RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 马路边沿与路径夹角: %.2f !", std::acos(cos_theta) / M_PI * 180);
       }
       // 检查平行度是否满足要求
       if(cos_theta > parallel_threshold)
       {
         // 计算垂直距离
         const double A = dy_wall;
         const double B = -dx_wall;
         const double C = wall_end.x * wall_start.y - wall_start.x * wall_end.y;
         
         const double numerator = std::fabs(A * start_pose.x + B * start_pose.y + C);
         const double denominator = std::hypot(A, B);
         const double avg_distance = numerator / denominator;
         
         RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 墙线与路径夹角: %.2f, 平均距离: %.2f",
                              cos_theta * 180.0 / M_PI, avg_distance);

         const double robot_to_edge_distance =
           std::fabs(A * robot_pose.pose.position.x + B * robot_pose.pose.position.y + C) / denominator;
         RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 机器人与贴边线的距离: %.2f", 
                              robot_to_edge_distance);
         
         // 记录满足平行度且距离最小的墙线
         if (avg_distance <= distance_tolerance && avg_distance < min_distance) 
         {
           found_valid_wall = true;
           min_distance = avg_distance;
           best_robot_to_edge_distance = robot_to_edge_distance;
           best_wall_path = curb_line;
           best_wall_start = wall_start;
           best_wall_end = wall_end;
         }
       }
     }
   }
 
   // 5. 如果找到最佳匹配的墙线，更新配置
   if (found_valid_wall) 
   {
     const Eigen::Vector2d candidate_wall_start(best_wall_start.x, best_wall_start.y);
     const Eigen::Vector2d candidate_wall_end(best_wall_end.x, best_wall_end.y);
     const Eigen::Vector2d robot_pos(
       robot_pose.pose.position.x, robot_pose.pose.position.y);
     if (protrusionBlocksEdgeEntryForWallSegment(
         candidate_wall_start, candidate_wall_end, robot_pos))
     {
       return;
     }

     if(wall_line_points_.size() > 0) wall_line_points_.clear();
     wall_line_points_.emplace_back(Eigen::Vector2d(best_wall_start.x, best_wall_start.y));
     wall_line_points_.emplace_back(Eigen::Vector2d(best_wall_end.x, best_wall_end.y));
     
     updateScheduledWallDistWeights(best_robot_to_edge_distance);
     wall_line_update_time_ = clock_->now();
     std_msgs::msg::Float32 distance;
     distance.data = static_cast<float>(best_robot_to_edge_distance);
     edge_distance_publisher_->publish(distance);
     
     // Switch to edge-following mode parameters
     switchParameterMode(true);
     
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 选择最佳墙线距离: %.2f", min_distance);
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 最小障碍物距离: %.2f, 最优时间权重: %.2f, 墙线距离权重: %.2f, 侧隙Pull: %.2f", 
                          cfg_->obstacles.min_obstacle_dist, 
                          cfg_->optim.weight_optimaltime, cfg_->optim.weight_wall_line_dist,
                          cfg_->optim.weight_wall_side_pull);
     
     // 发布可视化标记
     visualization_msgs::msg::Marker marker_msg;
     marker_msg.ns = "teb_local_planner";
     marker_msg.id = 0;
     marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
     marker_msg.action = visualization_msgs::msg::Marker::ADD;
     marker_msg.scale.x = 0.1;
     marker_msg.color.r = 1.0;
     marker_msg.color.g = 1.0;
     marker_msg.color.b = 0.0;
     marker_msg.color.a = 1.0;
     
     geometry_msgs::msg::Point start_point, end_point;
     start_point.x = best_wall_start.x;
     start_point.y = best_wall_start.y;
     start_point.z = 0.0;
     
     end_point.x = best_wall_end.x;
     end_point.y = best_wall_end.y;
     end_point.z = 0.0;
     
     marker_msg.points.push_back(start_point);
     marker_msg.points.push_back(end_point);
     
     marker_msg.header.stamp = clock_->now();
     marker_msg.header.frame_id = input_path.header.frame_id;
     
     wall_line_marker_publisher_->publish(marker_msg);
   }
   else 
   {
     RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 没有找到合适的墙线，准备退出贴边/清空贴边配置");
     // 6. 如果没有找到合适的墙线，检查是否需要清除现有配置
     if(wall_line_points_.size() > 0)
     {
       auto time_diff = clock_->now() - wall_line_update_time_;
       if (time_diff.seconds() > keep_wall_line_time_)
       {
         wall_line_points_.clear();
         RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000,
                     "[updateCurbLineVec] Edge following: 超过 keep_wall_line_time(%.2fs)，清空贴边配置并退出贴边模式",
                     keep_wall_line_time_);
         
         // Switch back to normal mode parameters
         switchParameterMode(false);
       }
     }
   }
 }
 
 
 void TebLocalPlannerROS::curb_line_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
 {
   
   std::lock_guard<std::mutex> l(curb_line_mutex_);
 
   curb_line_path_.poses.clear();
   if (msg->poses.size() == 0) {return;}
   // 获取当前时间戳
   rclcpp::Time current_time = clock_->now();
 
   if (msg->poses.size() > 1)
   {
     try {
       // 转换路径的起点和终点到map坐标系
       geometry_msgs::msg::PoseStamped start_pose_transformed;
       geometry_msgs::msg::PoseStamped end_pose_transformed;
       double curb_line_length = distance_points2d(msg->poses.front().pose.position, msg->poses.back().pose.position);
       if (curb_line_length < min_wall_line_length_)
       {
         return;
       }
       
       // 转换起点
       if (msg->header.frame_id != cfg_->map_frame) {
         try {
           geometry_msgs::msg::TransformStamped transform_start = tf_->lookupTransform(
             cfg_->map_frame, 
             msg->header.frame_id,
             tf2::TimePointZero,
             tf2::durationFromSec(0.5));
           tf2::doTransform(msg->poses.front(), start_pose_transformed, transform_start);
         } catch (const tf2::ExtrapolationException& ex) {
           RCLCPP_WARN(logger_, "curbLinePathCallback: 时间戳异常，重新转换起点: %s", ex.what());
           try {
             geometry_msgs::msg::TransformStamped transform_start = tf_->lookupTransform(
               cfg_->map_frame, 
               msg->header.frame_id,
               tf2::TimePointZero);
             tf2::doTransform(msg->poses.front(), start_pose_transformed, transform_start);
           } catch (const tf2::TransformException& ex2) {
             RCLCPP_ERROR(logger_, "curbLinePathCallback: 转换起点失败: %s", ex2.what());
             return; // Skip this message if transform fails
           }
         } catch (const tf2::TransformException& ex) {
           RCLCPP_ERROR(logger_, "curbLinePathCallback: 转换起点异常: %s", ex.what());
           return; // Skip this message if transform fails
         }
       } else {
         start_pose_transformed = msg->poses.front();
       }
       
       // 转换终点
       if (msg->header.frame_id != cfg_->map_frame) {
         try {
           geometry_msgs::msg::TransformStamped transform_end = tf_->lookupTransform(
             cfg_->map_frame, 
             msg->header.frame_id,
             tf2::TimePointZero,
             tf2::durationFromSec(0.5));
           tf2::doTransform(msg->poses.back(), end_pose_transformed, transform_end);
         } catch (const tf2::ExtrapolationException& ex) {
           RCLCPP_WARN(logger_, "curbLinePathCallback: 转换终点异常: %s, 重新转换", ex.what());
           try {
             geometry_msgs::msg::TransformStamped transform_end = tf_->lookupTransform(
               cfg_->map_frame, 
               msg->header.frame_id,
               tf2::TimePointZero);
             tf2::doTransform(msg->poses.back(), end_pose_transformed, transform_end);
           } catch (const tf2::TransformException& ex2) {
             RCLCPP_ERROR(logger_, "curbLinePathCallback: 转换终点失败: %s", ex2.what());
             return; // Skip this message if transform fails
           }
         } catch (const tf2::TransformException& ex) {
           RCLCPP_ERROR(logger_, "curbLinePathCallback: 转换终点异常: %s", ex.what());
           return; // Skip this message if transform fails
         }
       } else {
         end_pose_transformed = msg->poses.back();
       }
       
       // 更新转换后的位姿的时间戳和坐标系
       start_pose_transformed.header.stamp = current_time;
       start_pose_transformed.header.frame_id = cfg_->map_frame;
       end_pose_transformed.header.stamp = current_time;
       end_pose_transformed.header.frame_id = cfg_->map_frame;
       
       // 添加到路径中
       curb_line_path_.poses.emplace_back(start_pose_transformed);
       curb_line_path_.poses.emplace_back(end_pose_transformed);

       // 更新curb_line更新时间戳
       curb_line_update_time_ = clock_->now();
     }
     catch (tf2::TransformException &ex) {
       RCLCPP_WARN(logger_, 
                   "坐标变换失败: %s, 不更新curb_line_path_", ex.what());
     }
   }
 }
       
 //Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
 //{
 //  Eigen::Vector2d vel;
 //  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
 //  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
 //  return vel;
 //}
       
       

}  // namespace teb_local_planner
