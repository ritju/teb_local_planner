#include <teb_local_planner/wall_protrusion_check.hpp>

#include <algorithm>
#include <cmath>

namespace teb_local_planner
{

WallProtrusionHit checkWallProtrusionPoint(
  const Eigen::Vector2d& vertex_map,
  const Eigen::Vector2d& p_base,
  bool have_p_base,
  const EffWallFrame& wall,
  const Eigen::Vector2d& robot_position,
  const TebConfig& config)
{
  WallProtrusionHit hit;

  const double vertex_wall_coord = wallSignedDist(vertex_map, wall);
  if (!isOnSameWallSideAsRobot(vertex_wall_coord, wall.robot_side_sign)) {
    return hit;
  }

  if (have_p_base) {
    const double fwd_max = config.wall_line.wall_line_protrusion_exit_forward_max;
    const double lat_max = config.wall_line.wall_line_protrusion_exit_lateral_max;
    if (p_base.x() <= -config.wall_line.wall_line_obstacle_along_wall_rear_margin ||
      p_base.x() > fwd_max ||
      std::fabs(p_base.y()) > lat_max)
    {
      return hit;
    }
  } else {
    return hit;
  }

  const double pen = std::abs(vertex_wall_coord);
  const double base_d_exit = config.wall_line.wall_line_obstacle_protrusion_base_distance;
  const double scale_k_exit = config.wall_line.wall_line_protrusion_exit_depth_min;
  const double depth_max = config.wall_line.wall_line_protrusion_exit_depth_max;

  const double dist_robot_vertex = (vertex_map - robot_position).norm();
  double eff_v = config.wall_line.wall_line_protrusion_exit_depth_min;
  if (dist_robot_vertex > base_d_exit) {
    eff_v = std::min(
      eff_v + scale_k_exit * (dist_robot_vertex - base_d_exit),
      depth_max);
  }

  if (pen <= eff_v || pen > depth_max) {
    return hit;
  }

  hit.exit_trigger = true;
  hit.exit_best_pen = pen;
  hit.exit_best_point = vertex_map;
  return hit;
}

}  // namespace teb_local_planner
