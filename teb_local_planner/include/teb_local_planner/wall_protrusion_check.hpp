/*********************************************************************
 * Shared wall-line protrusion point check (obstacle vertices + front_scan)
 *********************************************************************/
#ifndef TEB_LOCAL_PLANNER_WALL_PROTRUSION_CHECK_HPP_
#define TEB_LOCAL_PLANNER_WALL_PROTRUSION_CHECK_HPP_

#include <teb_local_planner/teb_config.h>

#include <Eigen/Dense>

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

/**
 * @brief 单点 protrusion 退出判定（与 processWallFilteredPolygon 顶点逻辑一致）
 * @param vertex_map 地图/障碍坐标系下的点
 * @param p_base base 坐标系下的点（用于走廊过滤；无效时跳过走廊判定）
 * @param have_p_base 是否已有 base 坐标
 */
WallProtrusionHit checkWallProtrusionPoint(
  const Eigen::Vector2d& vertex_map,
  const Eigen::Vector2d& p_base,
  bool have_p_base,
  const EffWallFrame& wall,
  const Eigen::Vector2d& robot_position,
  const TebConfig& config);

}  // namespace teb_local_planner

#endif  // TEB_LOCAL_PLANNER_WALL_PROTRUSION_CHECK_HPP_
