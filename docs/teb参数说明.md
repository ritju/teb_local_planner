# TEB Local Planner 参数说明

本文档根据用户提供的参数文本生成，覆盖原地转向、角点、贴边、轨迹、机器人运动学、障碍物、优化、同伦规划和恢复等参数。默认值是该文本中的值；若运行时通过 ROS 参数、环境变量或底盘配置覆盖，以实际加载值为准。

## 1. 参数读取与调参原则

- 长度单位为 **m**，时间为 **s**，线速度为 **m/s**，角速度为 **rad/s**，角加速度为 **rad/s²**；名称带 `_deg` 或注释标明 `[deg]` 的角度使用度。
- 布尔值控制功能开关；阈值类参数通常满足“越小越敏感/越早触发”，距离和速度上限则直接限制行为。
- `min_turning_radius > 0` 表示前轮转向模式；等于 `0` 表示差速/后轮差速模式。前轮转向时，转向角、转向角速度和转向角加速度优先由底盘配置或 `max_steering_*` 约束。
- 代价权重越大，优化器越倾向满足对应约束；权重过大可能造成求解变慢或与其他目标冲突。
- `both`、`lethal`、`inscribed` 是障碍物代价判定模式：分别表示两类代价都检查、仅致命障碍、仅内切障碍。

## 2. 关键参数关系

| 场景 | 主要参数 | 调整建议 |
|---|---|---|
| 原地转向触发 | `rotation_linear_vel_threshold`、`rotation_angle_threshold` | 线速度必须足够低且航向误差足够大才触发；频繁摇摆时增大 `rotation_limit_duration` 或减小触发范围。 |
| 角点裁剪 | `theta_threshold`、`prune_before_corner_distance`、`cut_path_before_corner_*` | 先识别角点，再按障碍代价裁剪路径；裁剪后至少保留 `prune_corner_residual_distance`。 |
| 贴边进入 | `min_wall_dist`、`distance_tolerance`、`parallel_tolerance` | 距离和方向同时满足才进入；增大容差会更容易贴边，也更容易误匹配。 |
| 贴边安全 | `edge_min_obstacle_dist`、`wall_side_clearance_*`、`weight_wall_side_*` | 先保证机器人轮廓与障碍物安全距离，再调整侧隙吸引/排斥权重。 |
| 前轮转向 | `min_turning_radius`、`max_steering_angle`、`max_steering_rate`、`max_steering_acceleration` | 先确认模式，再检查底盘覆盖参数和速度上限；低速是否允许打角由 `allow_steer_at_standstill` 决定。 |
| 动态障碍等待 | `path_wait_*` | `path_wait_timeout` 应大于检测器稳定时间；超时后才进入锁存/绕行。 |

## 3. 参数逐项说明

参数名带有前缀时，可按以下方式快速定位：`rotation_`/`rotate_` 为原地转向，`corner_`/`prune_`/`cut_path_` 为角点与路径裁剪，`wall_line_`/`edge_` 为贴边，`vehicle_` 为车辆识别，`path_wait_` 为路径走廊等待，`reverse_`/`backward_` 为后退，`steering_`/`front_steering_` 为前轮转向，`weight_` 为优化权重，`major_arc_`/`dual_arc_` 为优弧重试和失败恢复，`dyn_`/`feas_pred_` 为动态安全预测。

### 原地转向参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `rotation_linear_vel_threshold` | `0.35` | 允许原地转向的最大线速度 |
| `rotation_angle_threshold` | `0.3875` | 允许原地转向的最小角度 |
| `rotate_to_heading_angular_vel` | `0.4` | 旋转角速度 |
| `rotation_max_angular_accel` | `0.4` | 最大角加速度 |
| `rotation_forward_lookahead_distance` | `0.5` | 获取路径点方向的截断距离 |
| `rotation_rotate_min_angular_vel` | `0.2` | 终点处的角速度 |
| `rotation_limit_duration` | `30.0` | 转向点有效保持时间，防止反复旋转造成的摇摆 |
| `rotation_limit_distance` | `0.3` | 小于次距离，判断属于同一个转向点 |
| `rotation_decel_blend_start_rad` | `0.0` | 当角度小于此值时，开始减速 |
| `rotation_path_footprint_check_enable` | `false` | 原地转前是否做路径 footprint 扫掠；false 时只做原地转向扫掠 |
| `rotation_path_footprint_check_cost_mode` | `"lethal"` | 路径点与局部轮廓的代价计算模式，"both"、"lethal"、"inscribed" |

### 角点处理参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `theta_threshold` | `135` | 路径点与前后路径点角度小于该阈值，判定为角点 |
| `prune_before_corner_linear_x_threshold` | `0.2` | 角点被障碍物占用时，机器人背离角度的线速度大于该阈值，则截断全局路径lethal或inscribed长度 |
| `cut_path_before_corner_lethal_dist` | `3.0` | 角点被lethal障碍物占用时的，向前裁剪全局路径的距离 |
| `cut_path_before_corner_inscribed_dist` | `3.0` | 角点被inscribed障碍物占用时，向内裁剪全局路径的距离 |
| `prune_before_corner_distance` | `4.0` | 识别到角点后，向前截断全局路径的距离 |
| `prune_corner_residual_distance` | `0.5` | 截断全局路径的剩余路径长度最小值 |
| `corner_approach_check_margin` | `-1.0` | 角点处理参数，-1.0表示用min_obstacle_dist |
| `corner_approach_check_sample_spacing` | `0.2` | 角点处理参数，采样间隔 |
| `corner_approach_check_cost_mode` | `"lethal"` | 角点处理参数，代价计算模式，"both"、"lethal"、"inscribed" |
| `corner_approach_footprint_marker_enable` | `true` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |

### 贴边参数配置

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `min_wall_dist` | `0.15` | 机器人外轮廓最小贴边距离 |
| `distance_tolerance` | `1.0` | 路径与贴边线距离小于该阈值，允许贴边 |
| `parallel_tolerance` | `10.0` | 贴边线与路径点方向角度误差小于该阈值，允许贴边 |
| `edge_acc_lim_theta` | `0.2` | 贴边最大角加速度。前轮转向不使用 |
| `edge_max_vel_theta` | `0.2` | 贴边最大角速度。前轮转向不使用 |
| `edge_max_vel_x` | `0.5` | 贴边最大线速度 |
| `edge_weight_optimaltime` | `10.0` | 贴边优化时间权重 |
| `edge_min_obstacle_dist` | `0.1` | 贴边时距离障碍物最小距离 |
| `edge_footprint_vertices` | `"[[1.15, 0.6], [1.15, -0.6], [-0.68, -0.6], [-0.68, 0.6]]"` | 贴边轮廓顶点 |
| `min_wall_line_length` | `1.0` | 贴边特征长度大于该值，允许贴边 |
| `min_path_line_length` | `2.0` | 路径长度大于该值，允许贴边 |
| `transform_path_line_length` | `2.5` | 路径长度大于该值，允许贴边 |
| `keep_wall_line_time` | `5.0` | 贴边线保持时间 |
| `static_layer_enable_delay` | `5.0` | 静态层启用延迟 |
| `wall_line_dist_weight_scale_under_min_lower` | `0.8` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `wall_line_dist_weight_scale_under_min_upper` | `1.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `wall_line_dist_robot_weight_radius` | `2.0` | 机器人前向渐变权重距离，避免在机器人处权重过大，导致路径不平滑 |
| `wall_line_dist_weight_scale_at_robot` | `0.1` | 起点附近贴边权重 |
| `wall_line_dist_weight_scale_far` | `1.0` | 达到wall_line_dist_weight_scale_far距离附近贴边权重 |
| `wall_line_dist_weight_scale_at_distance_tolerance` | `0.1` | 机器人与贴边线距离增大而权重减小的比例 |
| `switch_static_layer` | `false` | 贴边时是否开关局部代价图静态图层 |
| `switch_local_footprint` | `true` | 贴边时是否调整局部轮廓为edge_footprint_vertices |
| `switch_global_footprint` | `true` | 贴边时是否调整全局轮廓为edge_footprint_vertices |
| `wall_line_safety_offset` | `0.00` | 贴边线偏移距离，裁剪障碍物轮廓时使用 |
| `wall_line_extension_distance` | `6.0` | 贴边线扩展距离 |
| `wall_line_lock_distance_threshold` | `0.08` | 多帧贴边线距离差小于该阈值，判定为锁定 |
| `wall_line_lock_angle_threshold` | `3.0` | 多帧贴边线角度差小于该阈值，判定为锁定 |
| `wall_line_lock_min_stable_count` | `10` | 多帧贴边线角度差小于该阈值，判定为锁定 |

### 贴边侧向间隙

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `wall_side_clearance_mode` | `true` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `desired_side_clearance` | `0.125` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `side_clearance_push` | `0.075` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `side_clearance_attract_max` | `0.4` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `weight_wall_side_pull` | `40.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_wall_side_push` | `200.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `debug_wall_obstacle_cost_interval` | `2.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `paired_mission_and_reference_path_topic` | `"paired_mission_and_reference_path"` | 配对路径和贴边参考线话题 |
| `removed_plan_topic` | `"removed_plan"` | Remove path topic，用于选择贴边参考线 |
| `paths_near_edge_match_distance_threshold` | `0.3` | 贴边线匹配距离阈值 |
| `paths_near_edge_match_angle_threshold_deg` | `10.0` | 贴边线匹配角度阈值 |
| `fusion_primary_lock_duration` | `5.0` | 融合贴边参考线和识别的贴边特征锁定时间 |
| `reference_match_max_angle_deg` | `10.0` | 融合贴边参考线和识别的贴边特征匹配角度阈值 |
| `reference_match_max_distance_m` | `0.5` | 融合贴边参考线和识别的贴边特征匹配距离阈值 |
| `reference_no_valid_path_timeout` | `5.0` | 融合贴边参考线和识别的贴边特征匹配时间阈值 |
| `mission_segment_front_pose_match_threshold_m` | `0.1` | 判断当前路径点首点与配对的路径点的距离 |
| `wall_line_obstacle_filter_distance` | `0.05` | 贴边线裁剪机器人一侧障碍物的距离阈值，正值裁剪，负值关闭 |

### 贴边障碍物裁剪与凸出检测

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `wall_line_obstacle_protrusion_base_distance` | `6.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `wall_line_obstacle_filter_distance_scale` | `0.1` | 裁剪距离放大系数 |
| `wall_line_obstacle_filter_distance_max` | `0.05` | 裁剪距离最大值，负值关闭 |
| `wall_line_obstacle_along_wall_rear_margin` | `0.5` | 障碍物轮廓在机器人后方时，计入的最大距离 |
| `wall_line_protrusion_exit_forward_max` | `5.0` | 障碍物轮廓在机器人前方时，计入的最大距离 |
| `wall_line_protrusion_exit_lateral_max` | `1.8` | 障碍物轮廓在机器人侧面时，计入的最大距离 |
| `wall_line_protrusion_exit_depth_min` | `0.5` | 障碍物轮廓在机器人侧超过这个距离时，退出贴边 |
| `wall_line_protrusion_exit_depth_max` | `1.8` | 障碍物轮廓在机器人小于该距离阈值，退出贴边 |
| `wall_line_obstacle_clip_intersect_min_span` | `0.01` | 与墙线交叠最小沿墙长度 |
| `wall_line_obstacle_clip_min_keep_area_sq` | `0.001` | 裁后面积下限 |
| `wall_line_obstacle_clip_vertex_dedupe_dist` | `0.02` | 顶点去重距离 |

### 障碍物轮廓裁剪参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `obstacle_protrusion_reenter_distance` | `5.0` | 凸出的障碍物轮廓与机器人距离小于该阈值，退出贴边 |
| `obstacle_protrusion_timeout` | `10.0` | 凸出的障碍物轮廓保存时间 |
| `obstacle_protrusion_min_confirm_frames` | `1` | 凸出的障碍物轮廓帧数超过该阈值，退出贴边 |
| `obstacle_protrusion_confirm_window` | `3.0` | 凸出的障碍物轮廓确认窗口时间 |
| `obstacle_protrusion_max_stored` | `100` | 凸出的障碍物轮廓队列长度 |

### 贴边车辆检测与退出条件

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `new_vehicle_distance_threshold` | `5.0` | 车辆距离机器人距离小于该阈值，允许判断是否退出贴边 |
| `erase_vehicle_distance_threshold` | `10.0` | 检测到的车辆与机器人距离大于该阈值，删除车辆 |
| `close_vehicle_distance_threshold` | `1.5` | 判断识别到的车辆是否是同一辆的距离阈值 |
| `vehicle_exit_corridor_wall_inner_m` | `0.5` | 贴边时，机器人考虑车辆在墙线远离机器人一侧的最大距离 |
| `vehicle_exit_corridor_wall_robot_side_m` | `2.0` | 贴边时，机器人考虑车辆在墙线靠近机器人一侧的最大距离 |
| `vehicle_exit_corridor_rear_m` | `1.5` | 贴边时，机器人考虑车辆在机器人后方的最大距离 |
| `vehicle_exit_corridor_front_m` | `6.0` | 贴边时，机器人考虑车辆在机器人前方的最大距离 |
| `enable_vehicle_scan_grid` | `false` | 是否开启车辆识别扩大障碍物轮廓 |
| `vehicle_scan_topic` | `"/vehicle_scan_points"` | 车辆点云话题 |
| `vehicle_scan_grid_resolution` | `0.2` | 维护车辆栅格地图分辨率 |
| `vehicle_scan_grid_width` | `10.0` | 维护车辆栅格地图宽度 |
| `vehicle_scan_grid_height` | `10.0` | 维护车辆栅格地图长度 |
| `vehicle_scan_polygon_fill_threshold` | `10` | 维护车辆栅格地图填充阈值 |
| `vehicle_scan_max_stale_cycles` | `10` | 维护车辆栅格地图最大过期周期 |
| `vehicle_scan_hull_inflation` | `1.5` | 维护车辆栅格地图凸包膨胀距离 |
| `publish_vehicle_scan_grid` | `false` | 是否发布维护车辆栅格地图 |
| `vehicle_scan_grid_topic` | `"/teb_vehicle_scan_grid"` | 维护车辆栅格地图话题 |
| `enable_front_scan_protrusion_check` | `true` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `front_scan_topic` | `"/front_scan"` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `front_scan_min_range_base_footprint` | `2.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `front_scan_max_range_base_footprint` | `10.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `monitor_corridor_path_heading_length` | `2.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `monitor_corridor_heading_angle_deg` | `60.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `teb_autosize` | `1.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `dt_ref` | `0.2` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `dt_hysteresis` | `0.1` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `max_samples` | `500` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `min_samples` | `3` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `global_plan_overwrite_orientation` | `true` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `allow_init_with_backwards_motion` | `false` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `max_global_plan_lookahead_dist` | `4.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `global_plan_viapoint_sep` | `0.25` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `rough_global_plan_prune_distance` | `5.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `rough_global_plan_prune_max_accum_dist` | `50.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `global_plan_prune_distance` | `1.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `global_plan_prune_max_accum_dist` | `8.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `transform_global_plan_closest_search_max_accum_dist` | `2.0` | 全局路径变换/终点搜索参数：限制局部规划窗口、终点占用搜索范围或行末安全位姿。 |
| `transform_global_plan_costmap_span_scale` | `2.0` | plan-length cap & row-end search = scale * max(costmap full x,y) [m]; <=0 disables |
| `transformed_plan_collision_pose_to_end_distance` | `4.0` | 全局路径变换/终点搜索参数：限制局部规划窗口、终点占用搜索范围或行末安全位姿。 |
| `transformed_plan_collision_pose_safe_distance_front` | `0.3` | fallback: min along-path gap if terminal is behind occupied [m] |
| `transformed_plan_collision_pose_safe_distance_back` | `1.0` | fallback: min along-path gap if terminal is past occupied [m] |
| `transformed_plan_row_terminal_arrive_distance` | `0.3` | arrived at row-terminal safe pose → prune remaining row [m] |
| `transform_global_plan_goal_occupied_tolerance` | `0.3` | 全局路径变换/终点搜索参数：限制局部规划窗口、终点占用搜索范围或行末安全位姿。 |
| `transform_global_plan_goal_search_resolution` | `0.1` | 全局路径变换/终点搜索参数：限制局部规划窗口、终点占用搜索范围或行末安全位姿。 |
| `prune_angle_threshold` | `90.0` | [deg] heading align for prune and current-row split |
| `exact_arc_length` | `false` | 是否使用精确弧长计算；开启后计算更准确但开销略高。 |
| `feasibility_check_no_poses` | `1000` | 轨迹可行性检查的开关或采样规模；数值越大检查范围/数量越大。 |
| `feasibility_check_lookahead_distance` | `10.0` | 轨迹可行性检查的开关或采样规模；数值越大检查范围/数量越大。 |
| `publish_feedback` | `false` | 是否发布对应调试/可视化信息。开启会增加通信和计算开销。 |
| `control_look_ahead_poses` | `4` | 控制输出从优化轨迹中向前取用的位姿数量。 |
| `is_footprint_dynamic` | `true` | 是否允许使用随轨迹姿态变化的机器人轮廓进行碰撞检查。 |
| `map_to_base_transform_max_age` | `1.0` | 允许使用的 map 到 base TF 最大年龄，超时则认为变换无效。 |
| `path_wait_enable` | `True` | 路径走廊动态障碍等待参数；控制前视距离、走廊宽度、等待超时和清空防抖。 |
| `path_wait_topic` | `/dynamic_obstacles` | 路径走廊动态障碍等待参数；控制前视距离、走廊宽度、等待超时和清空防抖。 |
| `path_wait_lookahead_dist` | `6.0` | 路径走廊动态障碍等待参数；控制前视距离、走廊宽度、等待超时和清空防抖。 |
| `path_wait_corridor_scale` | `1.0` | half_width = footprint_half_width + min_obstacle_dist * scale |
| `path_wait_timeout` | `6.0` | per-ID wait before latch/detour |
| `path_wait_clear_time` | `0.5` | corridor empty and per-ID leave debounce |
| `path_wait_msg_timeout` | `0.5` | 路径走廊动态障碍等待参数；控制前视距离、走廊宽度、等待超时和清空防抖。 |
| `publish_path_wait_debug` | `True` | 是否发布对应调试/可视化信息。开启会增加通信和计算开销。 |

### 允许后退参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `reverse_segment_min_segment_length_m` | `0.05` | 相邻路径点长度大于该阈值，允许后退 |
| `reverse_segment_min_angle_deg` | `150.0` | 相邻路径点角度大于该阈值，允许后退 |
| `reverse_segment_arc_length_ratio` | `0.3` | 倒车弧长占比阈值。代码读的是这个键名，与单元测试默认一致 |
| `backward_check_duration` | `1.0` | 退出后退模式，防抖时间 |
| `backward_check_num` | `2` | 退出后退模式，防抖次数 |

### 机器人基础运动学参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `min_turning_radius` | `0.0` | 最小转弯半径；大于 0 启用前轮转向，0 表示差速模式。 |
| `max_vel_x` | `1.0` | 机器人对应方向的速度上限（m/s 或 rad/s）；同时影响优化约束和输出限幅。 |

### 前轮转向模式参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `max_vel_theta` | `0.6` | 机器人对应方向的速度上限（m/s 或 rad/s）；同时影响优化约束和输出限幅。 |
| `acc_lim_x` | `0.25` | 对应方向的加速度上限；用于轨迹平滑和速度变化约束。 |
| `acc_lim_theta` | `0.6` | 对应方向的加速度上限；用于轨迹平滑和速度变化约束。 |
| `max_vel_x_backwards` | `0.5` | 机器人对应方向的速度上限（m/s 或 rad/s）；同时影响优化约束和输出限幅。 |
| `max_vel_y` | `0.0` | 机器人对应方向的速度上限（m/s 或 rad/s）；同时影响优化约束和输出限幅。 |
| `safe_linear_speed_limit` | `2.0` | 机器人对应方向的速度上限（m/s 或 rad/s）；同时影响优化约束和输出限幅。 |
| `cmd_angle_instead_rotvel` | `false` | 是否发布转向角命令而不是角速度命令。 |

### 前轮转向模式参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `front_steering_rear_drive` | `false` | 前轮转向运动学和输出限制参数，控制转角、转角变化率以及是否允许原地旋转。 |
| `front_steering_hard_limit` | `false` | 对下发指令做前轮转角、角速度、角加速度硬限幅；关闭后只保留优化软约束 |
| `include_edge_kinematics_carlike` | `false` | 前轮转向时是否加入 EdgeKinematicsCarlike；关闭后曲率只由转向边约束，非完整约束仍保留 |
| `auto_min_turning_radius` | `true` | 为 true 时由 wheelbase 和 max_steering_angle 计算半径，不改写 min_turning_radius |
| `rear_track` | `0.84` | 前轮转向运动学和输出限制参数，控制转角、转角变化率以及是否允许原地旋转。 |
| `max_steering_angle` | `1.0` | 前轮转向运动学和输出限制参数，控制转角、转角变化率以及是否允许原地旋转。 |
| `max_steering_rate` | `0.5` | 前轮转向运动学和输出限制参数，控制转角、转角变化率以及是否允许原地旋转。 |
| `max_steering_acceleration` | `1.0` | 前轮转向运动学和输出限制参数，控制转角、转角变化率以及是否允许原地旋转。 |
| `steering_min_speed` | `0.08` | 前轮转向运动学和输出限制参数，控制转角、转角变化率以及是否允许原地旋转。 |
| `steering_omega_epsilon` | `1.0e-3` | 前轮转向运动学和输出限制参数，控制转角、转角变化率以及是否允许原地旋转。 |
| `no_inplace_rotation` | `true` | 前轮转向运动学和输出限制参数，控制转角、转角变化率以及是否允许原地旋转。 |
| `weight_steering_kinematics` | `500.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_steering_rate` | `300.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_steering_acceleration` | `200.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `footprint_model` | — | 轮廓模型配置块；支持 point、circular、two_circles、line、polygon。 |
| `footprint_model.type` | `"polygon"` | 机器人碰撞轮廓模型及其几何参数；顶点按机器人坐标系给出。 |
| `footprint_model.vertices` | `"[[1.15, 0.7], [1.15, -0.7], [-0.68, -0.7], [-0.68, 0.7]]"` | 机器人碰撞轮廓模型及其几何参数；顶点按机器人坐标系给出。 |
| `free_goal_vel` | `false` | 到达目标时是否允许保留非零速度；关闭时通常会在目标处减速。 |
| `min_obstacle_dist` | `0.15` | 障碍物模型和碰撞代价参数，控制安全距离、膨胀范围及参与检查的障碍物数量。 |
| `inflation_dist` | `0.4` | 障碍物模型和碰撞代价参数，控制安全距离、膨胀范围及参与检查的障碍物数量。 |
| `include_costmap_obstacles` | `true` | 障碍物模型和碰撞代价参数，控制安全距离、膨胀范围及参与检查的障碍物数量。 |
| `costmap_obstacles_behind_robot_dist` | `1.5` | 障碍物模型和碰撞代价参数，控制安全距离、膨胀范围及参与检查的障碍物数量。 |
| `obstacle_poses_affected` | `30` | 障碍物模型和碰撞代价参数，控制安全距离、膨胀范围及参与检查的障碍物数量。 |
| `obstacle_proximity_upper_bound` | `0.5` | 障碍物模型和碰撞代价参数，控制安全距离、膨胀范围及参与检查的障碍物数量。 |
| `obstacle_proximity_lower_bound` | `0.01` | 障碍物模型和碰撞代价参数，控制安全距离、膨胀范围及参与检查的障碍物数量。 |
| `dynamic_obstacle_inflation_dist` | `0.7` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `include_dynamic_obstacles` | `false` | 障碍物模型和碰撞代价参数，控制安全距离、膨胀范围及参与检查的障碍物数量。 |
| `min_resolution_collision_check_angular` | `0.017` | 0.087 radians = 5 degrees |

### 优弧重试与停滞看门狗

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `major_arc_retry_enable` | `False` | 优弧重试/双弧失败恢复参数，用于在局部轨迹不可行或停滞时切换策略。 |
| `major_arc_retry_heading_threshold` | `0.0` | 优弧重试/双弧失败恢复参数，用于在局部轨迹不可行或停滞时切换策略。 |

### 优弧重试与停滞看门狗

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `major_arc_stagnation_timeout` | `3.0` | 优弧重试/双弧失败恢复参数，用于在局部轨迹不可行或停滞时切换策略。 |
| `major_arc_stagnation_yaw` | `0.2` | 优弧重试/双弧失败恢复参数，用于在局部轨迹不可行或停滞时切换策略。 |
| `major_arc_stagnation_xy` | `0.1` | 优弧重试/双弧失败恢复参数，用于在局部轨迹不可行或停滞时切换策略。 |

### 双弧失败恢复

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `dual_arc_reverse_enable` | `True` | 优弧重试/双弧失败恢复参数，用于在局部轨迹不可行或停滞时切换策略。 |
| `dual_arc_reverse_fail_num` | `5` | 优弧重试/双弧失败恢复参数，用于在局部轨迹不可行或停滞时切换策略。 |
| `dual_arc_reverse_min_hold` | `10.0` | 优弧重试/双弧失败恢复参数，用于在局部轨迹不可行或停滞时切换策略。 |
| `dual_arc_reverse_timeout` | `20.0` | 优弧重试/双弧失败恢复参数，用于在局部轨迹不可行或停滞时切换策略。 |
| `dual_arc_reverse_forward_vx` | `0.2` | 优弧重试/双弧失败恢复参数，用于在局部轨迹不可行或停滞时切换策略。 |
| `dual_arc_reverse_forward_num` | `10` | 优弧重试/双弧失败恢复参数，用于在局部轨迹不可行或停滞时切换策略。 |
| `dual_arc_reverse_publish_backward_mode` | `True` | 优弧重试/双弧失败恢复参数，用于在局部轨迹不可行或停滞时切换策略。 |

### 近视域动态安全与可行性检查

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `dynamic_safety_predictable_mode` | `false` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `dynamic_obstacles_as_static_edges` | `true` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `dynamic_obstacle_cache_time` | `0.5` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `dyn_gate_T_near` | `3.0` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `dyn_gate_v_x_max` | `5.0` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `dyn_gate_omega_z_max` | `1.0` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `dyn_gate_inflation` | `0.5` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `dyn_gate_overlap_dist` | `0.5` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `dyn_gate_roi_front` | `1.0` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `dyn_gate_roi_rear` | `1.0` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `dyn_gate_roi_left` | `1.0` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `dyn_gate_roi_right` | `1.0` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `dyn_gate_hold_time` | `0.5` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `delta_omega_scale` | `0.2` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `publish_near_horizon_debug` | `false` | 是否发布对应调试/可视化信息。开启会增加通信和计算开销。 |
| `feasibility_check_obstacle_contours` | `false` | 轨迹可行性检查的开关或采样规模；数值越大检查范围/数量越大。 |
| `feasibility_check_dynamic_prediction` | `false` | 轨迹可行性检查的开关或采样规模；数值越大检查范围/数量越大。 |
| `feas_contour_margin` | `0.0` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `feas_pred_T` | `3.0` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `feas_pred_R` | `3.0` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `feas_pred_gate_mode` | `"both_or"` | rel \| abs \| both_or |
| `feas_pred_delta_omega_abs` | `0.2` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `feas_pred_delta_omega_rel` | `-1.0` | <=0 → delta_omega_scale * max_vel_theta |
| `feas_pred_v_min` | `0.0` | 动态障碍物预测与近视域安全参数，控制预测时域、门控区域、膨胀和角速度变化阈值。 |
| `costmap_converter_plugin` | `"costmap_converter::CostmapToPolygonsDBSMCCH"` | 代价地图障碍物转换器配置。 |
| `costmap_converter_spin_thread` | `true` | 代价地图障碍物转换器配置。 |
| `no_inner_iterations` | `5` | TEB 优化器迭代或代价函数配置。 |
| `no_outer_iterations` | `4` | TEB 优化器迭代或代价函数配置。 |
| `optimization_activate` | `true` | TEB 优化器迭代或代价函数配置。 |
| `optimization_verbose` | `false` | TEB 优化器迭代或代价函数配置。 |
| `penalty_epsilon` | `0.05` | TEB 优化器迭代或代价函数配置。 |
| `obstacle_cost_exponent` | `1.0` | TEB 优化器迭代或代价函数配置。 |
| `weight_max_vel_x` | `0.1` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_max_vel_theta` | `0.1` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_acc_lim_x` | `1.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_acc_lim_theta` | `0.1` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |

### 优化权重

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `weight_max_vel_theta_front_steering` | `0.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_acc_lim_theta_front_steering` | `0.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_kinematics_nh` | `10000.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_kinematics_forward_drive` | `100.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_kinematics_forward_drive_in_narrow_passages` | `1.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_kinematics_turning_radius` | `1000.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |

### 优化权重

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `weight_optimaltime` | `2.0` | 急弯上给求解器缩短时间的理由，仍远小于转向权重 10000 |
| `weight_shortest_path` | `0.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_obstacle` | `100.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_inflation` | `1.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_dynamic_obstacle` | `50.0` | not in use yet |
| `weight_dynamic_obstacle_inflation` | `0.1` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_velocity_obstacle_ratio` | `0.2` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |

### 优化权重

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `weight_viapoint` | `20.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |

### 优化权重

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `weight_path_heading` | `25.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_wall_line_dist` | `20.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_adapt_factor` | `2.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `weight_prefer_rotdir` | `0.0` | 优化目标权重；增大后更强调该约束/目标，但可能挤压其他目标并增加求解难度。 |
| `enable_homotopy_class_planning` | `true` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `enable_multithreading` | `true` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `max_number_classes` | `5` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `max_number_plans_in_current_class` | `5` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `selection_cost_hysteresis` | `1.0` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `selection_prefer_initial_plan` | `2.0` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `selection_obst_cost_scale` | `2.0` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `selection_dropping_probability` | `0.05` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `selection_alternative_time_cost` | `false` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `roadmap_graph_no_samples` | `15` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `roadmap_graph_area_width` | `5.0` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `roadmap_graph_area_length_scale` | `1.0` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `h_signature_prescaler` | `1.0` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `h_signature_threshold` | `0.1` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `obstacle_heading_threshold` | `0.45` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `switching_blocking_period` | `0.0` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `viapoints_all_candidates` | `true` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `delete_detours_backwards` | `true` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `max_ratio_detours_duration_best_duration` | `3.0` | 该参数控制所属模块的阈值、开关或上限；调整后应结合对应调试话题验证。 |
| `visualize_hc_graph` | `false` | 是否发布对应调试/可视化信息。开启会增加通信和计算开销。 |
| `visualize_with_time_as_z_axis_scale` | `0.0` | 同伦类规划参数，用于生成、筛选和可视化多条候选路径。 |
| `shrink_horizon_backup` | `true` | 恢复行为参数，用于处理规划失败、振荡或局部停滞。 |
| `shrink_horizon_min_duration` | `10.0` | 恢复行为参数，用于处理规划失败、振荡或局部停滞。 |
| `oscillation_recovery` | `true` | 恢复行为参数，用于处理规划失败、振荡或局部停滞。 |
| `oscillation_v_eps` | `0.1` | 恢复行为参数，用于处理规划失败、振荡或局部停滞。 |
| `oscillation_omega_eps` | `0.1` | 恢复行为参数，用于处理规划失败、振荡或局部停滞。 |
| `oscillation_recovery_min_duration` | `10.0` | 恢复行为参数，用于处理规划失败、振荡或局部停滞。 |
| `oscillation_filter_duration` | `10.0` | 恢复行为参数，用于处理规划失败、振荡或局部停滞。 |

## 4. 调参与故障定位

1. **原地转向不触发**：检查线速度是否低于 `rotation_linear_vel_threshold`，航向误差是否达到 `rotation_angle_threshold`，以及前轮转向模式下是否启用了 `no_inplace_rotation`。
2. **原地转向来回摆动**：增大 `rotation_limit_duration`，适当增大 `rotation_limit_distance`，降低 `rotate_to_heading_angular_vel`，并确保 `rotation_decel_blend_start_rad` 合理。
3. **贴边抖动或贴不上墙**：先检查墙线话题和 TF 时效，再调整 `paths_near_edge_match_*`、`wall_line_lock_*`；随后调整 `edge_min_obstacle_dist` 和 `wall_line_dist_weight_*`。
4. **角点前过早停/裁剪过多**：减小 `prune_before_corner_distance` 或 `cut_path_before_corner_*`，并增大 `prune_corner_residual_distance` 以避免剩余路径过短。
5. **前轮转向不可行**：确认 `wheelbase`、`max_steering_angle`、`min_turning_radius` 一致，降低 `max_vel_x` 或提高转向角/角速度上限；检查 `weight_steering_kinematics` 是否过大导致其他约束无法满足。
6. **动态障碍一直等待**：检查 `path_wait_topic` 是否有消息、`path_wait_msg_timeout` 是否过小，以及 `path_wait_timeout` 是否合理；开启 `publish_path_wait_debug` 查看走廊和 ID 状态。
7. **修改后不生效**：确认参数位于 `FollowPath` 节点命名空间，检查启动文件实际加载的 YAML，并核对底盘 `CHASSIS_*` 配置是否覆盖 ROS 参数。

## 5. 配置一致性说明

用户提供的文本与仓库现有 `teb_local_planner/params/teb_params.yaml` 可能存在默认值差异。本文档刻意以用户提供文本为准；部署前请使用 `ros2 param dump` 或启动日志核对最终生效值。
