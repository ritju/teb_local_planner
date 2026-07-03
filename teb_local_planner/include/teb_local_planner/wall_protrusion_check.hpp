/*********************************************************************
 * Shared wall-line protrusion point check (obstacle vertices + front_scan)
 *********************************************************************/
#ifndef TEB_LOCAL_PLANNER_WALL_PROTRUSION_CHECK_HPP_
#define TEB_LOCAL_PLANNER_WALL_PROTRUSION_CHECK_HPP_

#include <teb_local_planner/teb_config.h>

#include <Eigen/Dense>
#include <vector>

namespace teb_local_planner
{

struct EffWallFrame
{
  Eigen::Vector2d start;
  Eigen::Vector2d dir;
  Eigen::Vector2d normal;
  double length{0.0};
  int robot_side_sign{0};
};

struct WallProtrusionHit
{
  bool exit_trigger{false};
  Eigen::Vector2d exit_best_point{Eigen::Vector2d::Zero()};
  double exit_best_pen{0.0};
};

/** @brief 墙线 Frenet 监测走廊（foot + along + normal） */
struct WallMonitorCorridor
{
  Eigen::Vector2d foot{Eigen::Vector2d::Zero()};
  Eigen::Vector2d along{Eigen::Vector2d::Zero()};
  Eigen::Vector2d normal{Eigen::Vector2d::Zero()};
  double rear_m{0.0};
  double forward_m{0.0};
  double lateral_inner_m{0.0};
  double lateral_robot_m{0.0};
  bool valid{false};
  bool used_path_heading{false};
};

inline double wallSignedDist(const Eigen::Vector2d& p, const EffWallFrame& wall)
{
  return (p - wall.start).dot(wall.normal);
}

inline int wallSideSignFromCoord(double coord, double eps = 1e-9)
{
  if (coord > eps) {
    return 1;
  }
  if (coord < -eps) {
    return -1;
  }
  return 0;
}

inline bool isOnSameWallSideAsRobot(
  double point_coord, int robot_side_sign, double eps = 1e-9)
{
  const int point_sign = wallSideSignFromCoord(point_coord, eps);
  if (robot_side_sign == 0 || point_sign == 0) {
    return true;
  }
  return point_sign == robot_side_sign;
}

Eigen::Vector2d footpointOnEffWall(
  const Eigen::Vector2d& position,
  const EffWallFrame& wall);

/**
 * @brief 从路径点序列向前累计弧长，提取用于定沿墙方向的 map 系点列
 */
std::vector<Eigen::Vector2d> extractPathPointsForHeading(
  const std::vector<Eigen::Vector2d>& path_points_map,
  double max_arc_length);

/**
 * @brief 计算沿墙正方向；|wall.dir·robot_fwd| 过小时用路径首尾方向 fallback
 */
bool computeAlongWallUnitDirection(
  const EffWallFrame& wall,
  double robot_yaw,
  const std::vector<Eigen::Vector2d>& path_points_map,
  double path_heading_length,
  double heading_cos_threshold,
  Eigen::Vector2d& along_out,
  bool& used_path_heading);

WallMonitorCorridor buildWallMonitorCorridor(
  const EffWallFrame& wall,
  const Eigen::Vector2d& robot_position,
  double robot_yaw,
  const std::vector<Eigen::Vector2d>& path_points_map,
  double rear_m,
  double forward_m,
  double lateral_inner_m,
  double lateral_robot_m,
  const TebConfig& config);

bool isPointInWallMonitorCorridor(
  const Eigen::Vector2d& point_map,
  const WallMonitorCorridor& corridor);

/**
 * @brief 监测走廊四顶点（map 系，用于 Marker）
 */
void wallMonitorCorridorCorners(
  const WallMonitorCorridor& corridor,
  Eigen::Vector2d corners_out[4]);

/**
 * @brief 单点 protrusion 退出判定（与 processWallFilteredPolygon 顶点逻辑一致）
 */
WallProtrusionHit checkWallProtrusionPoint(
  const Eigen::Vector2d& vertex_map,
  const EffWallFrame& wall,
  const Eigen::Vector2d& robot_position,
  double robot_yaw,
  const TebConfig& config,
  const WallMonitorCorridor* monitor_corridor = nullptr);

bool buildEffWallFrameFromSegment(
  const Eigen::Vector2d& segment_start,
  const Eigen::Vector2d& segment_end,
  const Eigen::Vector2d& robot_position,
  const TebConfig& config,
  EffWallFrame& wall_out);

}  // namespace teb_local_planner

#endif  // TEB_LOCAL_PLANNER_WALL_PROTRUSION_CHECK_HPP_
