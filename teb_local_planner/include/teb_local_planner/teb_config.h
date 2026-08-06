/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef TEB_CONFIG_H_
#define TEB_CONFIG_H_

#include <nav2_util/lifecycle_node.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <nav_2d_utils/parameters.hpp>
#include "teb_local_planner/robot_footprint_model.h"
#include <nav2_costmap_2d/footprint.hpp>

// Definitions
#define USE_ANALYTIC_JACOBI // if available for a specific edge, use analytic jacobi

namespace teb_local_planner
{
/**
 * @class TebConfig
 * @brief Config class for the teb_local_planner and its components.
 */
class TebConfig
{
public:
  using UniquePtr = std::unique_ptr<TebConfig>;
  
  std::string odom_topic; //!< Topic name of the odometry message, provided by the robot driver or simulator
  std::string map_frame; //!< Global planning frame
  std::string node_name; //!< node name used for parameter event callback

  RobotFootprintModelPtr robot_model; //!< 机器人碰撞模型（点/圆/线段/多圆/多边形）
  std::string model_name; //!< 足迹模型名称（point/circular/line/two_circles/polygon）
  double radius; //!< 圆形模型半径 [m]
  std::vector<double> line_start, line_end; //!< 线段模型起点/终点 [x, y]
  double front_offset, front_radius, rear_offset, rear_radius; //!< 双圆模型参数（前后圆心偏移与半径）
  std::string footprint_string; //!< 多边形足迹字符串（通常来自参数服务器）


  //! Trajectory related parameters
  struct Trajectory
  {
    double teb_autosize; //!< Enable automatic resizing of the trajectory w.r.t to the temporal resolution (recommended)
    double dt_ref; //!< Desired temporal resolution of the trajectory (should be in the magniture of the underlying control rate)
    double dt_hysteresis; //!< Hysteresis for automatic resizing depending on the current temporal resolution (dt): usually 10% of dt_ref
    int min_samples; //!< Minimum number of samples (should be always greater than 2)
    int max_samples; //!< Maximum number of samples; Warning: if too small the discretization/resolution might not be sufficient for the given robot model or obstacle avoidance does not work anymore.
    bool global_plan_overwrite_orientation; //!< Overwrite orientation of local subgoals provided by the global planner
    bool allow_init_with_backwards_motion; //!< If true, the underlying trajectories might be initialized with backwards motions in case the goal is behind the start within the local costmap (this is only recommended if the robot is equipped with rear sensors)
    double global_plan_viapoint_sep; //!< Min. separation between each two consecutive via-points extracted from the global plan (if negative: disabled)
    bool via_points_ordered; //!< If true, the planner adheres to the order of via-points in the storage container
    double max_global_plan_lookahead_dist; //!< Cumulative path length from global plan start: limits optimization subset; also caps sharp-corner search along the plan (if <=0: no path-length cap for corner search; bounded by local costmap for optimization)
    double global_plan_prune_distance; //!< Distance between robot and via_points of global plan which is used for pruning
    double global_plan_prune_max_accum_dist; //!< pruneGlobalPlan: max cumulative path length from plan start to search for prune point [m]; <=0 disables cap (search entire plan)
    double rough_global_plan_prune_distance; //!< Coarse-pass pruning distance threshold [m]; applied before the fine prune to remove obviously-passed points; <=0 skips coarse pass
    double rough_global_plan_prune_max_accum_dist; //!< Coarse-pass pruning: max cumulative path length to search [m]; <=0 searches the entire plan
    bool exact_arc_length; //!< If true, the planner uses the exact arc length in velocity, acceleration and turning rate computations [-> increased cpu time], otherwise the euclidean approximation is used.
    double force_reinit_new_goal_dist; //!< Reinitialize the trajectory if a previous goal is updated with a seperation of more than the specified value in meters (skip hot-starting)
    double force_reinit_new_goal_angular; //!< Reinitialize the trajectory if a previous goal is updated with an angular difference of more than the specified value in radians (skip hot-starting)
    int feasibility_check_no_poses; //!< Specify up to which pose (under the feasibility_check_lookahead_distance) on the predicted plan the feasibility should be checked each sampling interval; if -1, all poses up to feasibility_check_lookahead_distance are checked.
    double feasibility_check_lookahead_distance; //!< Specify up to which distance (and with an index below feasibility_check_no_poses) from the robot the feasibility should be checked each sampling interval; if -1, all poses up to feasibility_check_no_poses are checked.
    bool publish_feedback; //!< Publish planner feedback containing the full trajectory and a list of active obstacles (should be enabled only for evaluation or debugging purposes)
    double min_resolution_collision_check_angular; //! Min angular resolution used during the costmap collision check. If not respected, intermediate samples are added. [rad]
    int control_look_ahead_poses; //! Index of the pose used to extract the velocity command
    int theta_threshold; //!< 角点判定阈值 [deg]，用于识别路径急转角
    double corner_dist_threshold; //!< Distance threshold to consider a point a corner
    //!< 角点不可达时裁剪 global_plan：LETHAL_OBSTACLE 下允许裁剪的机器人到角点横向距离上限 [m]
    double cut_path_before_corner_lethal_dist; //!< LETHAL 障碍下角点裁剪横向阈值 [m]
    //!< 角点不可达时裁剪 global_plan：INSCRIBED_INFLATED_OBSTACLE 下允许裁剪的机器人到角点横向距离上限 [m]
    double cut_path_before_corner_inscribed_dist; //!< INSCRIBED 障碍下角点裁剪横向阈值 [m]
    //!< 在 global_plan 中搜索 last_corner_pose_ 的最大累积路径长度 [m]; <=0 表示不限制
    double prune_before_corner_distance; //!< 搜索角点的累计路径长度上限 [m]
    //!< 匹配到角点后，角点至路径终点剩余累积路径长度须大于该值才执行裁剪 [m]
    double prune_corner_residual_distance; //!< 角点到终点剩余长度需大于该值才裁剪 [m]
    //!< 角点有障碍时裁剪 global_plan：|linear.x| 须小于该阈值才执行裁剪 [m/s]
    double prune_before_corner_linear_x_threshold; //!< 仅当线速度绝对值高于该阈值时允许角点裁剪 [m/s]
    //!< 角点检测：true=机器人→角点射线采样+角点 footprint 检测；false=仅角点
    bool corner_approach_ray_check_enable;
    //!< 角点/射线路径 footprint 采样间距 [m]
    double corner_approach_check_sample_spacing;
    //!< 角点碰撞代价模式: both | lethal | inscribed
    std::string corner_approach_check_cost_mode;
    //!< 角点检测 footprint 四向扩展 [m]; <0 表示使用 obstacles.min_obstacle_dist
    double corner_approach_check_margin;
    //!< 是否发布角点/射线检测 footprint MarkerArray（teb_corner_approach_footprint_markers）
    bool corner_approach_footprint_marker_enable;
    //! transformGlobalPlan: when scorePose throws "Trajectory Hits Obstacle.", extend max plan length by this (m), capped by local costmap size
    double max_plan_length_extend_on_trajectory_obstacle_m; //!< 末端碰障时允许扩展的 transformed_plan 长度 [m]
    //!< transformGlobalPlan: 碰撞点距全局终点直线距离小于此值时，延长 max_plan_length 并前移 last_idx（m）
    double transformed_plan_collision_pose_to_end_distance; //!< 碰撞点到终点小于该距离时触发末端前移策略 [m]
    //!< transformGlobalPlan: max cumulative path length from plan[0] while searching for the pose closest to the robot [m]; <=0 = no limit (entire plan)
    double transform_global_plan_closest_search_max_accum_dist; //!< 搜索“距机器人最近路径点”的累计长度上限 [m]
    //!< transformGlobalPlan: 若末端路径点在局部代价地图上 footprint 碰撞，在全局路径(plan 系)上以该值为半宽做网格偏移搜索可调位姿 [m]；<=0 关闭微调
    double transform_global_plan_goal_occupied_tolerance; //!< 末端被占据时，平面偏移搜索半径 [m]
    //!< transformGlobalPlan: 末端位姿微调时的平面网格步长 [m]，与 SMAC Hybrid 等价的 goal_search_resolution
    double transform_global_plan_goal_search_resolution; //!< 末端偏移搜索网格步长 [m]
    //!< hasReverseSegmentInPlan: 参与倒车判定的路点段最小弦长 [m]；过短则向前扩窗至 i+2, i+3, ...
    double reverse_segment_min_segment_length_m; //!< 倒车判定的最小段弦长 [m]
    //!< hasReverseSegmentInPlan: 位移方向与 pose 航向夹角下限 [deg]，接近 180° 时判为倒车（如 150 表示 150°~180°）
    double reverse_segment_min_angle_deg; //!< 倒车判定最小夹角阈值 [deg]
    //!< hasReverseSegmentInPlan: 倒车路径弧长占判定区间总弧长的比例阈值 [0,1]
    double reverse_segment_arc_length_ratio; //!< 倒车弧长占比阈值
    //!< /backward_mode true→false: 连续非倒车帧防抖窗口时长 [s]
    double backward_check_duration; //!< backward_mode 退出防抖窗口时长 [s]
    //!< /backward_mode true→false: 窗口内连续 reverse_segment=false 帧数
    int backward_check_num; //!< 防抖窗口内需要的连续“非倒车”帧数
  } trajectory; //!< Trajectory related parameters

  //! Robot related parameters
  struct Robot
  {
    double base_max_vel_x; //!< Maximum translational velocity of the robot before speed limit is applied
    double base_max_vel_x_backwards; //!< Maximum translational velocity of the robot for driving backwards before speed limit is applied
    double base_max_vel_y; //!< Maximum strafing velocity of the robot (should be zero for non-holonomic robots!) before speed limit is applied
    double base_max_vel_theta; //!< Maximum angular velocity of the robot before speed limit is applied
    double max_vel_x; //!< Maximum translational velocity of the robot
    double max_vel_x_backwards; //!< Maximum translational velocity of the robot for driving backwards
    double max_vel_y; //!< Maximum strafing velocity of the robot (should be zero for non-holonomic robots!)
    double max_vel_theta; //!< Maximum angular velocity of the robot
    double acc_lim_x; //!< Maximum translational acceleration of the robot
    double acc_lim_y; //!< Maximum strafing acceleration of the robot
    double acc_lim_theta; //!< Maximum angular acceleration of the robot
    double min_turning_radius; //!< Minimum turning radius of a carlike robot (diff-drive robot: zero);
    double wheelbase; //!< The distance between the drive shaft and steering axle (only required for a carlike robot with 'cmd_angle_instead_rotvel' enabled); The value might be negative for back-wheeled robots!
    bool cmd_angle_instead_rotvel; //!< Substitute the rotational velocity in the commanded velocity message by the corresponding steering angle (check 'axles_distance')
    bool is_footprint_dynamic; //<! If true, updated the footprint before checking trajectory feasibility
    bool use_proportional_saturation; //<! If true, reduce all twists components (linear x and y, and angular z) proportionally if any exceed its corresponding bounds, instead of saturating each one individually
    double transform_tolerance = 0.5; //<! Tolerance when querying the TF Tree for a transformation (seconds)
    //!< 若大于 0：当 map_frame→基底坐标系 TF 最新消息时间戳早于当前时钟超过该值 [s] 时，控制器输出零速度；<=0 关闭
    double map_to_base_transform_max_age; //!< map->base TF 最大允许延迟 [s]，超时则抑制输出
    double safe_linear_speed_limit; //!< Maximum linear speed limit for the robot
  } robot; //!< Robot related parameters

  //! Goal tolerance related parameters
  struct GoalTolerance
  {
    double xy_goal_tolerance; //!< Allowed final euclidean distance to the goal position
    bool free_goal_vel; //!< Allow the robot's velocity to be nonzero (usally max_vel) for planning purposes
  } goal_tolerance; //!< Goal tolerance related parameters

  //! Obstacle related parameters
  struct Obstacles
  {
    double min_obstacle_dist; //!< Minimum desired separation from obstacles
    double inflation_dist; //!< buffer zone around obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
    double dynamic_obstacle_inflation_dist; //!< Buffer zone around predicted locations of dynamic obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
    bool include_dynamic_obstacles; //!< Specify whether the movement of dynamic obstacles should be predicted by a constant velocity model (this also effects homotopy class planning); If false, all obstacles are considered to be static.
    bool include_costmap_obstacles; //!< Specify whether the obstacles in the costmap should be taken into account directly
    double costmap_obstacles_behind_robot_dist; //!< Limit the occupied local costmap obstacles taken into account for planning behind the robot (specify distance in meters)
    int obstacle_poses_affected; //!< The obstacle position is attached to the closest pose on the trajectory to reduce computational effort, but take a number of neighbors into account as well
    bool legacy_obstacle_association; //!< If true, the old association strategy is used (for each obstacle, find the nearest TEB pose), otherwise the new one (for each teb pose, find only "relevant" obstacles).
    double obstacle_association_force_inclusion_factor; //!< The non-legacy obstacle association technique tries to connect only relevant obstacles with the discretized trajectory during optimization, all obstacles within a specifed distance are forced to be included (as a multiple of min_obstacle_dist), e.g. choose 2.0 in order to consider obstacles within a radius of 2.0*min_obstacle_dist.
    double obstacle_association_cutoff_factor; //!< See obstacle_association_force_inclusion_factor, but beyond a multiple of [value]*min_obstacle_dist all obstacles are ignored during optimization. obstacle_association_force_inclusion_factor is processed first.
    std::string costmap_converter_plugin; //!< Define a plugin name of the costmap_converter package (costmap cells are converted to points/lines/polygons)
    bool costmap_converter_spin_thread; //!< If \c true, the costmap converter invokes its callback queue in a different thread
    int costmap_converter_rate; //!< The rate that defines how often the costmap_converter plugin processes the current costmap (the value should not be much higher than the costmap update rate)
    double obstacle_proximity_ratio_max_vel; //!< Ratio of the maximum velocities used as an upper bound when reducing the speed due to the proximity to a static obstacles
    double obstacle_proximity_lower_bound; //!< Distance to a static obstacle for which the velocity should be lower
    double obstacle_proximity_upper_bound; //!< Distance to a static obstacle for which the velocity should be higher
    //!< 车身激光 + converter 多边形维护的滚动栅格，输出凸包膨胀障碍（map 系对齐）
    bool enable_vehicle_scan_grid; //!< 是否启用车身激光 + converter 多边形滚动栅格
    std::string vehicle_scan_topic; //!< 车身点云话题
    double vehicle_scan_grid_resolution; //!< 滚动栅格分辨率
    double vehicle_scan_grid_width; //!< 滚动栅格宽度
    double vehicle_scan_grid_height; //!< 滚动栅格高度
    int vehicle_scan_polygon_fill_threshold; //!< 多边形内已为 0 的格数超过此绝对值则整多边形填 0
    int vehicle_scan_max_stale_cycles; //!< 连续未刷新的占据格恢复为自由（周期数）
    double vehicle_scan_hull_inflation; //!< 凸包沿顶点外法向膨胀（m） 
    bool publish_vehicle_scan_grid; //!< 是否发布滚动栅格
    std::string vehicle_scan_grid_topic; //!< 滚动栅格话题
  } obstacles; //!< Obstacle related parameters
  //! WallLine related parameters
  struct WallLine
  {
    double min_wall_dist; //!< 进入贴边/中心距匹配用；侧隙模式下不作为优化目标
    double min_wall_direction; //!< buffer zone around obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
    double parallel_tolerance; //!< 路径与边线平行度阈值（通常为 cos 夹角）
    double distance_tolerance; //!< 路径与边线匹配的最大允许距离 [m]
    //!< 侧隙贴边（方案 A）：footprint 到墙线 guide 的侧隙 g，Pull 钉 g*，Push 在 g<g_push 时推远
    bool wall_side_clearance_mode; //!< true=侧隙 EdgeWallSideClearance；false=旧中心距 EdgeDistanceToWall
    double desired_side_clearance; //!< 目标侧隙 g* [m]
    double side_clearance_push; //!< Push 触发阈值 g_push，须 < g* [m]
    double side_clearance_attract_max; //!< (g-g*) 超过此值时 Pull 误差置 0；<=0 表示不截断 [m]
    //!< 调试：按间隔打印贴边代价与障碍代价；<=0 关闭 [s]
    double debug_wall_obstacle_cost_interval;
    //!< 贴墙距离项：路径点距轨迹起点（机器人）欧氏距离在 [0,R] 内时，信息权重系数由 min 线性过渡到 max；R<=0 表示关闭（恒为 max）
    double wall_line_dist_robot_weight_radius; //!< 距离分段权重半径 R（0~R 线性插值）[m]
    double wall_line_dist_weight_scale_at_robot; //!< 距离起点 0m 时的系数（通常较小，减轻起点附近贴边/倒退）
    double wall_line_dist_weight_scale_far;      //!< 距离 >= R 时的系数（通常为 1.0）
    //!< 机器人距贴边线 < min_wall_dist 时：weight *= clamp(robot_dist/min_wall_dist, lower, upper)
    double wall_line_dist_weight_scale_under_min_lower; //!< 小于 min_wall_dist 时比例缩放下限
    double wall_line_dist_weight_scale_under_min_upper; //!< 小于 min_wall_dist 时比例缩放上限
    //!< 机器人在 [min_wall_dist, distance_tolerance] 内：远端（distance_tolerance）处的 weight 系数
    double wall_line_dist_weight_scale_at_distance_tolerance; //!< 在 distance_tolerance 处的权重系数
    double edge_acc_lim_theta; //!< 贴边模式角加速度限制 [rad/s^2]
    double edge_max_vel_theta; //!< 贴边模式角速度上限 [rad/s]
    double edge_max_vel_x; //!< 贴边模式线速度上限 [m/s]
    double edge_weight_optimaltime; //!< weight_optimaltime parameter for edge-following mode
    double edge_min_obstacle_dist; //!< min_obstacle_dist parameter for edge-following mode
    double keep_wall_line_time; //!< time to keep the wall line when not detect usefull wall line
    std::string paired_mission_and_reference_path_topic; //!< Paired mission segment + edge reference paths
    std::string removed_plan_topic; //!< Remaining mission goals; front pose selects active reference pair
    double mission_segment_front_pose_match_threshold_m; //!< front_pose to mission segment max distance [m]
    double paths_near_edge_match_distance_threshold; //!< transformed_plan segment-to-segment max distance [m]
    double paths_near_edge_match_angle_threshold_deg; //!< max heading difference for parallel match [deg]
    int paths_near_edge_enter_hit_count; //!< Consecutive hits required to enter edge-following
    int paths_near_edge_exit_miss_count; //!< Consecutive misses required to exit edge-following
    double fusion_primary_lock_duration; //!< When WALL/CURB matches Reference, lock primary for this duration [s]
    double reference_match_max_angle_deg; //!< Primary vs Reference "close": max direction angle diff [deg]
    double reference_match_max_distance_m; //!< Primary vs Reference "close": max segment-to-segment distance [m]
    double reference_no_valid_path_timeout; //!< No valid Reference path: keep last segment until this timeout since last success [s]
    double new_vehicle_distance_threshold; //!< 车辆新建/更新关联距离阈值 [m]
    double erase_vehicle_distance_threshold; //!< 车辆擦除距离阈值 [m]
    double close_vehicle_distance_threshold; //!< 近邻车辆判定阈值 [m]
    //!< 贴边时车辆检测：机器人航向后方/前方距离 [m]，与墙法向内侧/机器人侧带宽 [m] 构成的平行条带（与航向组合为实际检测区）
    double vehicle_exit_corridor_rear_m; //!< 贴边退出检测区：机器人后向长度 [m]
    double vehicle_exit_corridor_front_m; //!< 贴边退出检测区：机器人前向长度 [m]
    double vehicle_exit_corridor_wall_inner_m;   //!< 墙线沿法向“内侧”扩展（与指向机器人的法向相反一侧）[m]
    double vehicle_exit_corridor_wall_robot_side_m; //!< 墙线沿法向朝机器人侧扩展 [m]
    double min_wall_line_length; //!< 候选墙线最小长度 [m]
    double min_path_line_length; //!< 输入路径最小长度 [m]
    double transform_path_line_length; //!< transformed_plan 截取用于贴边匹配的长度 [m]
    double static_layer_enable_delay; //!< 切换 static_layer 的延时 [s]
    std::string edge_footprint_vertices; //!< footprint vertices parameter for edge-following mode
    double wall_line_safety_offset; //!< Offset the wall line toward the wall side for safety margin [m]
    double wall_line_extension_distance; //!< Extend the wall line segment on both ends [m]
    double wall_line_lock_distance_threshold; //!< 相对锁定墙线的法向偏移阈值（两端点取 max），超过则解锁；忽略沿墙伸缩 [m]
    double wall_line_lock_angle_threshold; //!< 无向夹角变化阈值，超过则解锁 [deg]
    int wall_line_lock_min_stable_count; //!< Minimum consecutive stable frames before locking
    double wall_line_obstacle_filter_distance; //!< Base nominal: below effective value filters wall noise; above starts exit confirmation [m]
    double wall_line_obstacle_protrusion_base_distance; //!< If dist(robot, protrusion point) <= this, use nominal threshold [m]
    double wall_line_obstacle_filter_distance_scale; //!< Beyond base: add this * (d - base) to nominal threshold [m/m]
    double wall_line_obstacle_filter_distance_max; //!< Upper cap for distance-scaled threshold (filter + exit) [m]
    double wall_line_obstacle_along_wall_rear_margin; //!< 沿墙监测走廊：foot 点后方长度 [m]
    double wall_line_protrusion_exit_forward_max; //!< 沿墙监测走廊：foot 点前方长度 [m]
    double wall_line_protrusion_exit_lateral_max; //!< 沿墙监测走廊：墙→机器人侧法向宽度 [m]
    double monitor_corridor_path_heading_length; //!< 航向与墙夹角过大时，用路径定 along 方向的截取弧长 [m]
    double monitor_corridor_heading_angle_deg; //!< |wall.dir·robot_fwd| 低于 cos(此角) 时启用路径定方向 [deg]
    double wall_line_protrusion_exit_depth_min; //!< Exit check: base eff threshold vs robot-vertex dist (pen>eff counts); larger => stricter, less likely to exit edge mode [m]
    double wall_line_protrusion_exit_depth_max; //!< Exit check: max wall-normal protrusion toward robot counted [m], interval (eff, this]
    double obstacle_protrusion_reenter_distance; //!< Deprecated: 清除/重入判定已改用 base 监测走廊，保留参数兼容
    double obstacle_protrusion_timeout; //!< protruding_obstacles_ 记录超过此时间未刷新则删除 [s]
    int obstacle_protrusion_min_confirm_frames; //!< Deprecated: no longer used (protrusion recorded immediately on detection)
    double obstacle_protrusion_confirm_window; //!< Deprecated: no longer used (protrusion recorded immediately on detection)
    int obstacle_protrusion_max_stored; //!< Maximum number of protruding obstacles to store
    double wall_line_obstacle_clip_intersect_min_span; //!< Min along-wall span of polygon-wall segment intersection to enable clip [m]
    double wall_line_obstacle_clip_min_keep_area_sq; //!< If clipped polygon area below this, replace with minimal wall segment [m^2]
    double wall_line_obstacle_clip_vertex_dedupe_dist; //!< Merge polygon clip vertices closer than this [m]
    //!< 贴边时前向 2D 激光 protrusion 退出检测（与障碍物轮廓顶点判定一致）
    bool enable_front_scan_protrusion_check; //!< 是否启用 /front_scan 贴边 protrusion 退出检测
    std::string front_scan_topic; //!< 前向 2D 激光雷达话题（LaserScan）
    double front_scan_min_range_base_footprint; //!< 距离裁剪：costmap base_frame 下径向最近距离 [m]
    double front_scan_max_range_base_footprint; //!< 距离裁剪：costmap base_frame 下径向最远距离 [m]
    bool switch_static_layer; //!< Whether to toggle local costmap static_layer.enabled when switching parameter mode
    bool switch_local_footprint; //!< Whether to toggle local costmap footprint when switching parameter mode
    bool switch_global_footprint; //!< Whether to toggle global costmap footprint when switching parameter mode
  } wall_line; //!< Obstacle related parameters


  //! Optimization related parameters
  struct Optimization
  {
    int no_inner_iterations; //!< Number of solver iterations called in each outerloop iteration
    int no_outer_iterations; //!< Each outerloop iteration automatically resizes the trajectory and invokes the internal optimizer with no_inner_iterations

    bool optimization_activate; //!< Activate the optimization
    bool optimization_verbose; //!< Print verbose information

    double penalty_epsilon; //!< Add a small safety margin to penalty functions for hard-constraint approximations

    double weight_max_vel_x; //!< Optimization weight for satisfying the maximum allowed translational velocity
    double weight_max_vel_y; //!< Optimization weight for satisfying the maximum allowed strafing velocity (in use only for holonomic robots)
    double weight_max_vel_theta; //!< Optimization weight for satisfying the maximum allowed angular velocity
    double weight_acc_lim_x; //!< Optimization weight for satisfying the maximum allowed translational acceleration
    double weight_acc_lim_y; //!< Optimization weight for satisfying the maximum allowed strafing acceleration (in use only for holonomic robots)
    double weight_acc_lim_theta; //!< Optimization weight for satisfying the maximum allowed angular acceleration
    double weight_kinematics_nh; //!< Optimization weight for satisfying the non-holonomic kinematics
    double weight_kinematics_forward_drive; //!< Optimization weight for forcing the robot to choose only forward directions (positive transl. velocities, only diffdrive robot)
    double weight_kinematics_forward_drive_in_narrow_passages; //!< Reduced forward-drive penalty while in narrow passages or following reverse global plan segments
    double weight_kinematics_turning_radius; //!< Optimization weight for enforcing a minimum turning radius (carlike robots)
    double weight_optimaltime; //!< Optimization weight for contracting the trajectory w.r.t. transition time
    double weight_shortest_path; //!< Optimization weight for contracting the trajectory w.r.t. path length
    double weight_obstacle; //!< Optimization weight for satisfying a minimum separation from obstacles
    double weight_inflation; //!< Optimization weight for the inflation penalty (should be small)
    double weight_dynamic_obstacle; //!< Optimization weight for satisfying a minimum separation from dynamic obstacles
    double weight_dynamic_obstacle_inflation; //!< Optimization weight for the inflation penalty of dynamic obstacles (should be small)
    double weight_velocity_obstacle_ratio; //!< Optimization weight for satisfying a maximum allowed velocity with respect to the distance to a static obstacle
    double weight_viapoint; //!< Optimization weight for minimizing the distance to via-points
    double weight_prefer_rotdir; //!< Optimization weight for preferring a specific turning direction (-> currently only activated if an oscillation is detected, see 'oscillation_recovery'
    double weight_adapt_factor; //!< Some special weights (currently 'weight_obstacle') are repeatedly scaled by this factor in each outer TEB iteration (weight_new = weight_old*factor); Increasing weights iteratively instead of setting a huge value a-priori leads to better numerical conditions of the underlying optimization problem.
    double obstacle_cost_exponent; //!< Exponent for nonlinear obstacle cost (cost = linear_cost * obstacle_cost_exponent). Set to 1 to disable nonlinear cost (default)
    double weight_wall_line_dist; //!< 旧中心距贴边权重；侧隙模式下若 weight_wall_side_pull<=0 则回退用此值作 Pull
    double weight_wall_line_direction; //!< 贴边方向一致性代价权重
    double weight_wall_side_pull; //!< 侧隙 Pull 权重（g - g*）
    double weight_wall_side_push; //!< 侧隙 Push 权重（max(g_push - g, 0)）
  } optim; //!< Optimization related parameters


  struct HomotopyClasses
  {
    bool enable_homotopy_class_planning; //!< Activate homotopy class planning (Requires much more resources that simple planning, since multiple trajectories are optimized at once).
    bool enable_multithreading; //!< Activate multiple threading for planning multiple trajectories in parallel.
    bool simple_exploration; //!< If true, distinctive trajectories are explored using a simple left-right approach (pass each obstacle on the left or right side) for path generation, otherwise sample possible roadmaps randomly in a specified region between start and goal.
    int max_number_classes; //!< Specify the maximum number of allowed alternative homotopy classes (limits computational effort)
    int max_number_plans_in_current_class; //!< Specify the maximum number of trajectories to try that are in the same homotopy class as the current trajectory (helps avoid local minima)
    double selection_cost_hysteresis; //!< Specify how much trajectory cost must a new candidate have w.r.t. a previously selected trajectory in order to be selected (selection if new_cost < old_cost*factor).
    double selection_prefer_initial_plan; //!< Specify a cost reduction in the interval (0,1) for the trajectory in the equivalence class of the initial plan.
    double selection_obst_cost_scale; //!< Extra scaling of obstacle cost terms just for selecting the 'best' candidate.
    double selection_viapoint_cost_scale; //!< Extra scaling of via-point cost terms just for selecting the 'best' candidate.
    bool selection_alternative_time_cost; //!< If true, time cost is replaced by the total transition time.
    double selection_dropping_probability; //!< At each planning cycle, TEBs other than the current 'best' one will be randomly dropped with this probability. Prevents becoming 'fixated' on sub-optimal alternative homotopies.
    double switching_blocking_period; //!< Specify a time duration in seconds that needs to be expired before a switch to new equivalence class is allowed

    int roadmap_graph_no_samples; //! < Specify the number of samples generated for creating the roadmap graph, if simple_exploration is turend off.
    double roadmap_graph_area_width; //!< Random keypoints/waypoints are sampled in a rectangular region between start and goal. Specify the width of that region in meters.
    double roadmap_graph_area_length_scale; //!< The length of the rectangular region is determined by the distance between start and goal. This parameter further scales the distance such that the geometric center remains equal!
    double h_signature_prescaler; //!< Scale number of obstacle value in order to allow huge number of obstacles. Do not choose it extremly low, otherwise obstacles cannot be distinguished from each other (0.2<H<=1).
    double h_signature_threshold; //!< Two h-signatures are assumed to be equal, if both the difference of real parts and complex parts are below the specified threshold.

    double obstacle_keypoint_offset; //!< If simple_exploration is turned on, this parameter determines the distance on the left and right side of the obstacle at which a new keypoint will be cretead (in addition to min_obstacle_dist).
    double obstacle_heading_threshold; //!< Specify the value of the normalized scalar product between obstacle heading and goal heading in order to take them (obstacles) into account for exploration [0,1]

    bool viapoints_all_candidates; //!< If true, all trajectories of different topologies are attached to the current set of via-points, otherwise only the trajectory sharing the same one as the initial/global plan.

    bool visualize_hc_graph; //!< Visualize the graph that is created for exploring new homotopy classes.
    double visualize_with_time_as_z_axis_scale; //!< If this value is bigger than 0, the trajectory and obstacles are visualized in 3d using the time as the z-axis scaled by this value. Most useful for dynamic obstacles.
    bool delete_detours_backwards; //!< If enabled, the planner will discard the plans detouring backwards with respect to the best plan
    double detours_orientation_tolerance; //!< A plan is considered a detour if its start orientation differs more than this from the best plan
    double length_start_orientation_vector; //!< Length of the vector used to compute the start orientation of a plan
    double max_ratio_detours_duration_best_duration; //!< Detours are discarted if their execution time / the execution time of the best teb is > this
  } hcp;

  //! Recovery/backup related parameters
  struct Recovery
  {
    bool shrink_horizon_backup; //!< Allows the planner to shrink the horizon temporary (50%) in case of automatically detected issues.
    double shrink_horizon_min_duration; //!< Specify minimum duration for the reduced horizon in case an infeasible trajectory is detected.
    bool oscillation_recovery; //!< Try to detect and resolve oscillations between multiple solutions in the same equivalence class (robot frequently switches between left/right/forward/backwards)
    double oscillation_v_eps; //!< Threshold for the average normalized linear velocity: if oscillation_v_eps and oscillation_omega_eps are not exceeded both, a possible oscillation is detected
    double oscillation_omega_eps; //!< Threshold for the average normalized angular velocity: if oscillation_v_eps and oscillation_omega_eps are not exceeded both, a possible oscillation is detected
    double oscillation_recovery_min_duration; //!< Minumum duration [sec] for which the recovery mode is activated after an oscillation is detected.
    double oscillation_filter_duration; //!< Filter length/duration [sec] for the detection of oscillations
    bool divergence_detection_enable; //!< True to enable divergence detection.
    int divergence_detection_max_chi_squared; //!< Maximum acceptable Mahalanobis distance above which it is assumed that the optimization diverged.
  } recovery; //!< Parameters related to recovery and backup strategies

  //! In-place rotation related parameters
  struct Rotation
  {
    double linear_vel_threshold; //!< Threshold for linear velocity to enable in-place rotation [m/s]
    double angle_threshold; //!< Threshold for angle difference to trigger in-place rotation [rad]
    double rotate_to_heading_angular_vel; //!< Angular velocity for in-place rotation [rad/s]
    double max_angular_accel; //!< Maximum angular acceleration for in-place rotation [rad/s^2]
    double simulate_ahead_time; //!< Time to simulate ahead for collision checking during rotation [s] (deprecated, not used)
    double forward_lookahead_distance; //!< Forward lookahead distance to select target pose for rotation [m]
    //!< 原地转路径 footprint 碰障采样间距 [m]（沿 transformed_plan 弧长）
    double path_footprint_sample_spacing; //!< 沿路径做 footprint 碰撞采样的间距 [m]
    //!< 原地转路径 footprint 碰撞代价模式: both | lethal | inscribed
    std::string path_footprint_check_cost_mode;
    double rotate_min_angular_vel; //!< Minimum angular velocity for in-place rotation [rad/s]
    //!< >0：手动 blend(rad)，标称上界 wn*r/blend；<=0：由 wn、wm、max_angular_accel 与当前|ω|自动算 blend
    double decel_blend_start_rad; //!< 原地转减速 blend 角度阈值 [rad]（<=0 自动计算）
    double rotation_limit_duration; //!< Duration window to suppress repeated in-place rotation [s], <=0 disables
    double rotation_limit_distance; //!< Distance threshold to suppress repeated in-place rotation [m], <=0 disables
  } rotation; //!< Parameters related to in-place rotation


  /**
  * @brief Construct the TebConfig using default values.
  * @warning If the \b rosparam server or/and \b dynamic_reconfigure (rqt_reconfigure) node are used,
  *	     the default variables will be overwritten: \n
  *	     E.g. if \e base_local_planner is utilized as plugin for the navigation stack, the initialize() method will register a
  * 	     dynamic_reconfigure server. A subset (not all but most) of the parameters are considered for dynamic modifications.
  * 	     All parameters considered by the dynamic_reconfigure server (and their \b default values) are
  * 	     set in \e PROJECT_SRC/cfg/TebLocalPlannerReconfigure.cfg. \n
  * 	     In addition the rosparam server can be queried to get parameters e.g. defiend in a launch file.
  * 	     The plugin source (or a possible binary source) can call loadRosParamFromNodeHandle() to update the parameters.
  * 	     In \e summary, default parameters are loaded in the following order (the right one overrides the left ones): \n
  * 		<b>TebConfig Constructor defaults << dynamic_reconfigure defaults << rosparam server defaults</b>
  */
  TebConfig()
  {

    odom_topic = "odom";
    map_frame = "odom";
    

    // Trajectory
    trajectory.theta_threshold = 135;
    trajectory.corner_dist_threshold = 1.5;
    trajectory.cut_path_before_corner_lethal_dist = 1.0;
    trajectory.cut_path_before_corner_inscribed_dist = 1.5;
    trajectory.prune_before_corner_distance = 4.0;
    trajectory.prune_corner_residual_distance = 0.5;
    trajectory.prune_before_corner_linear_x_threshold = 0.1;
    trajectory.corner_approach_ray_check_enable = true;
    trajectory.corner_approach_check_sample_spacing = 0.2;
    trajectory.corner_approach_check_cost_mode = "both";
    trajectory.corner_approach_check_margin = -1.0;
    trajectory.corner_approach_footprint_marker_enable = false;
    trajectory.teb_autosize = true;
    trajectory.dt_ref = 0.3;
    trajectory.dt_hysteresis = 0.1;
    trajectory.min_samples = 3;
    trajectory.max_samples = 500;
    trajectory.global_plan_overwrite_orientation = true;
    trajectory.allow_init_with_backwards_motion = false;
    trajectory.global_plan_viapoint_sep = -1;
    trajectory.via_points_ordered = false;
    trajectory.max_global_plan_lookahead_dist = 1;
    trajectory.global_plan_prune_distance = 1;
    trajectory.global_plan_prune_max_accum_dist = 8.0;
    trajectory.rough_global_plan_prune_distance = 3.0;
    trajectory.rough_global_plan_prune_max_accum_dist = -1.0;
    trajectory.exact_arc_length = false;
    trajectory.force_reinit_new_goal_dist = 1;
    trajectory.force_reinit_new_goal_angular = 0.5 * M_PI;
    trajectory.feasibility_check_no_poses = 5;
    trajectory.feasibility_check_lookahead_distance = -1;
    trajectory.publish_feedback = false;
    trajectory.min_resolution_collision_check_angular = M_PI;
    trajectory.control_look_ahead_poses = 1;
    trajectory.max_plan_length_extend_on_trajectory_obstacle_m = 3.0;
    trajectory.transformed_plan_collision_pose_to_end_distance = 1.0;
    trajectory.transform_global_plan_closest_search_max_accum_dist = 0.0;
    trajectory.transform_global_plan_goal_occupied_tolerance = 1.0;
    trajectory.transform_global_plan_goal_search_resolution = 0.2;
    trajectory.reverse_segment_min_segment_length_m = 0.05;
    trajectory.reverse_segment_min_angle_deg = 150.0;
    trajectory.reverse_segment_arc_length_ratio = 0.3;
    trajectory.backward_check_duration = 1.0;
    trajectory.backward_check_num = 2;

    // Robot

    robot.max_vel_x = 0.4;
    robot.max_vel_x_backwards = 0.2;
    robot.max_vel_y = 0.0;
    robot.max_vel_theta = 0.3;
    robot.base_max_vel_x = robot.max_vel_x;
    robot.base_max_vel_x_backwards = robot.base_max_vel_x_backwards;
    robot.base_max_vel_y = robot.base_max_vel_y;
    robot.base_max_vel_theta = robot.base_max_vel_theta;
    robot.acc_lim_x = 0.5;
    robot.acc_lim_y = 0.5;
    robot.acc_lim_theta = 0.5;
    robot.min_turning_radius = 0;
    robot.wheelbase = 1.0;
    robot.cmd_angle_instead_rotvel = false;
    robot.is_footprint_dynamic = false;
    robot.use_proportional_saturation = false;
    robot.map_to_base_transform_max_age = 0.0;
    robot.safe_linear_speed_limit = 2.0;

    // GoalTolerance

    goal_tolerance.xy_goal_tolerance = 0.2;
    goal_tolerance.free_goal_vel = false;

    // Obstacles

    obstacles.min_obstacle_dist = 0.5;
    obstacles.inflation_dist = 0.6;
    obstacles.dynamic_obstacle_inflation_dist = 0.6;
    obstacles.include_dynamic_obstacles = true;
    obstacles.include_costmap_obstacles = true;
    obstacles.costmap_obstacles_behind_robot_dist = 1.5;
    obstacles.obstacle_poses_affected = 25;
    obstacles.legacy_obstacle_association = false;
    obstacles.obstacle_association_force_inclusion_factor = 1.5;
    obstacles.obstacle_association_cutoff_factor = 5;
    obstacles.costmap_converter_plugin = "";
    obstacles.costmap_converter_spin_thread = true;
    obstacles.costmap_converter_rate = 5;
    obstacles.obstacle_proximity_ratio_max_vel = 1;
    obstacles.obstacle_proximity_lower_bound = 0;
    obstacles.obstacle_proximity_upper_bound = 0.5;

    obstacles.enable_vehicle_scan_grid = false;
    obstacles.vehicle_scan_topic = "/vehicle_scan_points";
    obstacles.vehicle_scan_grid_resolution = 0.2;
    obstacles.vehicle_scan_grid_width = 10.0;
    obstacles.vehicle_scan_grid_height = 10.0;
    obstacles.vehicle_scan_polygon_fill_threshold = 10;
    obstacles.vehicle_scan_max_stale_cycles = 10;
    obstacles.vehicle_scan_hull_inflation = 1.5;
    obstacles.publish_vehicle_scan_grid = false;
    obstacles.vehicle_scan_grid_topic = "/teb_vehicle_scan_grid";

    wall_line.min_wall_dist = 0.4;
    wall_line.min_wall_direction = 0.0;
    wall_line.parallel_tolerance = 0.98;
    wall_line.distance_tolerance = 0.8;
    wall_line.wall_side_clearance_mode = true;
    wall_line.desired_side_clearance = 0.08;
    wall_line.side_clearance_push = 0.05;
    wall_line.side_clearance_attract_max = 0.35;
    wall_line.debug_wall_obstacle_cost_interval = 0.0;
    wall_line.edge_acc_lim_theta = 0.2;
    wall_line.edge_max_vel_theta = 0.3;
    wall_line.edge_max_vel_x = 0.5;
    wall_line.edge_weight_optimaltime = 10.0;
    wall_line.edge_min_obstacle_dist = 0.05;
    wall_line.keep_wall_line_time = 5.0;
    wall_line.paired_mission_and_reference_path_topic = "paired_mission_and_reference_path";
    wall_line.removed_plan_topic = "/removed_plan";
    wall_line.mission_segment_front_pose_match_threshold_m = 0.1;
    wall_line.paths_near_edge_match_distance_threshold = 0.6;
    wall_line.paths_near_edge_match_angle_threshold_deg = 20.0;
    wall_line.paths_near_edge_enter_hit_count = 2;
    wall_line.paths_near_edge_exit_miss_count = 4;
    wall_line.fusion_primary_lock_duration = 5.0;
    wall_line.reference_match_max_angle_deg = 10.0;
    wall_line.reference_match_max_distance_m = 0.5;
    wall_line.reference_no_valid_path_timeout = 5.0;
    wall_line.new_vehicle_distance_threshold = 5.0;
    wall_line.erase_vehicle_distance_threshold = 10.0;
    wall_line.close_vehicle_distance_threshold = 1.5;
    wall_line.vehicle_exit_corridor_rear_m = 2.0;
    wall_line.vehicle_exit_corridor_front_m = 6.0;
    wall_line.vehicle_exit_corridor_wall_inner_m = 0.5;
    wall_line.vehicle_exit_corridor_wall_robot_side_m = 2.0;
    wall_line.min_wall_line_length = 1.0;
    wall_line.min_path_line_length = 2.0;
    wall_line.transform_path_line_length = 2.0;
    wall_line.edge_footprint_vertices = "[[1.25, 0.5], [1.25, -0.5], [-0.65, -0.5], [-0.65, 0.5]]";
    wall_line.static_layer_enable_delay = 5.0;
    wall_line.wall_line_safety_offset = 0.03;
    wall_line.wall_line_extension_distance = 1.0;
    wall_line.wall_line_lock_distance_threshold = 0.08;
    wall_line.wall_line_lock_angle_threshold = 3.0;
    wall_line.wall_line_lock_min_stable_count = 5;
    wall_line.wall_line_obstacle_filter_distance = 0.5;
    wall_line.wall_line_obstacle_protrusion_base_distance = 1000.0;
    wall_line.wall_line_obstacle_filter_distance_scale = 0.0;
    wall_line.wall_line_obstacle_filter_distance_max = 0.5;
    wall_line.wall_line_obstacle_along_wall_rear_margin = 2.0;
    wall_line.wall_line_protrusion_exit_forward_max = 6.0;
    wall_line.wall_line_protrusion_exit_lateral_max = 1.0;
    wall_line.monitor_corridor_path_heading_length = 2.0;
    wall_line.monitor_corridor_heading_angle_deg = 45.0;
    wall_line.wall_line_protrusion_exit_depth_min = 0.3;
    wall_line.wall_line_protrusion_exit_depth_max = 2.0;
    wall_line.obstacle_protrusion_reenter_distance = 2.0;
    wall_line.obstacle_protrusion_timeout = 30.0;
    wall_line.obstacle_protrusion_min_confirm_frames = 3;
    wall_line.obstacle_protrusion_confirm_window = 2.0;
    wall_line.obstacle_protrusion_max_stored = 20;
    wall_line.wall_line_obstacle_clip_intersect_min_span = 0.05;
    wall_line.wall_line_obstacle_clip_min_keep_area_sq = 0.008;
    wall_line.wall_line_obstacle_clip_vertex_dedupe_dist = 0.02;
    wall_line.enable_front_scan_protrusion_check = true;
    wall_line.front_scan_topic = "/front_scan";
    wall_line.front_scan_min_range_base_footprint = 0.3;
    wall_line.front_scan_max_range_base_footprint = 5.0;
    wall_line.switch_static_layer = true;
    wall_line.switch_local_footprint = true;
    wall_line.switch_global_footprint = true;
    wall_line.wall_line_dist_robot_weight_radius = 0.0;
    wall_line.wall_line_dist_weight_scale_at_robot = 0.15;
    wall_line.wall_line_dist_weight_scale_far = 1.0;
    wall_line.wall_line_dist_weight_scale_under_min_lower = 0.1;
    wall_line.wall_line_dist_weight_scale_under_min_upper = 1.0;
    wall_line.wall_line_dist_weight_scale_at_distance_tolerance = 0.5;

    // Optimization

    optim.no_inner_iterations = 5;
    optim.no_outer_iterations = 4;
    optim.optimization_activate = true;
    optim.optimization_verbose = false;
    optim.penalty_epsilon = 0.05;
    optim.weight_max_vel_x = 2; //1
    optim.weight_max_vel_y = 2;
    optim.weight_max_vel_theta = 1;
    optim.weight_acc_lim_x = 1;
    optim.weight_acc_lim_y = 1;
    optim.weight_acc_lim_theta = 1;
    optim.weight_kinematics_nh = 1000;
    optim.weight_kinematics_forward_drive = 1;
    optim.weight_kinematics_forward_drive_in_narrow_passages = 0.0;
    optim.weight_kinematics_turning_radius = 1;
    optim.weight_optimaltime = 1;
    optim.weight_shortest_path = 0;
    optim.weight_obstacle = 50;
    optim.weight_inflation = 0.1;
    optim.weight_dynamic_obstacle = 50;
    optim.weight_dynamic_obstacle_inflation = 0.1;
    optim.weight_velocity_obstacle_ratio = 0;
    optim.weight_viapoint = 1;
    optim.weight_prefer_rotdir = 50;

    optim.weight_adapt_factor = 2.0;
    optim.obstacle_cost_exponent = 1.0;
    optim.weight_wall_line_dist = 1.0;
    optim.weight_wall_line_direction = 1.0;
    optim.weight_wall_side_pull = 80.0;
    optim.weight_wall_side_push = 200.0;

    // Homotopy Class Planner

    hcp.enable_homotopy_class_planning = true;
    hcp.enable_multithreading = true;
    hcp.simple_exploration = false;
    hcp.max_number_classes = 5;
    hcp.selection_cost_hysteresis = 1.0;
    hcp.selection_prefer_initial_plan = 0.95;
    hcp.selection_obst_cost_scale = 100.0;
    hcp.selection_viapoint_cost_scale = 1.0;
    hcp.selection_alternative_time_cost = false;
    hcp.selection_dropping_probability = 0.0;

    hcp.obstacle_keypoint_offset = 0.1;
    hcp.obstacle_heading_threshold = 0.45;
    hcp.roadmap_graph_no_samples = 15;
    hcp.roadmap_graph_area_width = 6; // [m]
    hcp.roadmap_graph_area_length_scale = 1.0;
    hcp.h_signature_prescaler = 1;
    hcp.h_signature_threshold = 0.1;
    hcp.switching_blocking_period = 0.0;

    hcp.viapoints_all_candidates = true;

    hcp.visualize_hc_graph = false;
    hcp.visualize_with_time_as_z_axis_scale = 0.0;
    hcp.delete_detours_backwards = true;
    hcp.detours_orientation_tolerance = M_PI / 2.0;
    hcp.length_start_orientation_vector = 0.4;
    hcp.max_ratio_detours_duration_best_duration = 3.0;

    // Recovery

    recovery.shrink_horizon_backup = true;
    recovery.shrink_horizon_min_duration = 10;
    recovery.oscillation_recovery = true;
    recovery.oscillation_v_eps = 0.1;
    recovery.oscillation_omega_eps = 0.1;
    recovery.oscillation_recovery_min_duration = 10;
    recovery.oscillation_filter_duration = 10;
    recovery.divergence_detection_enable = false;
    recovery.divergence_detection_max_chi_squared = 10;

    // Rotation
    rotation.linear_vel_threshold = 0.1;
    rotation.angle_threshold = 0.3875;  // 45 degrees
    rotation.rotate_to_heading_angular_vel = 0.35;
    rotation.max_angular_accel = 0.35;
    rotation.forward_lookahead_distance = 0.5;  // 0.5 meters forward
    rotation.path_footprint_sample_spacing = 0.2;
    rotation.path_footprint_check_cost_mode = "both";
    rotation.rotate_min_angular_vel = 0.05;
    rotation.decel_blend_start_rad = 0.0;
    rotation.rotation_limit_duration = 3.0;
    rotation.rotation_limit_distance = 0.2;
  }
  
  /**
   * @brief 声明 teb_local_planner 相关参数及默认值。
   */
  void declareParameters(const nav2_util::LifecycleNode::SharedPtr, const std::string name);

  /**
   * @brief 从参数服务器加载并覆盖当前配置。
   * @param nh 生命周期节点指针
   */
  void loadRosParamFromNodeHandle(const nav2_util::LifecycleNode::SharedPtr nh, const std::string name);
  
  /**
   * @brief 动态参数回调：接收参数变更并更新内部配置。
   * @param parameters 发生变化的参数列表
   */
  rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
  
  /**
   * @brief 校验参数合法性并输出告警。
   *
   * 当通过公开接口修改配置后，应调用该函数检查不合理组合。
   */
  void checkParameters() const;
  
  /**
   * @brief 检查是否使用了已废弃参数并输出提示。
   * @param nh 生命周期节点指针
   */
  void checkDeprecated(const nav2_util::LifecycleNode::SharedPtr nh, const std::string name) const;
  
  /**
   * @brief 返回配置互斥锁（用于线程安全访问）。
   */
  std::mutex& configMutex() {return config_mutex_;}

private:
  std::mutex config_mutex_; //!< Mutex for config accesses and changes
  rclcpp::Logger logger_{rclcpp::get_logger("TEBLocalPlanner")};
};
} // namespace teb_local_planner

#endif
