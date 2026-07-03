#include <teb_local_planner/wall_protrusion_check.hpp>

#include <algorithm>
#include <cmath>

namespace teb_local_planner
{

Eigen::Vector2d footpointOnEffWall(
  const Eigen::Vector2d& position,
  const EffWallFrame& wall)
{
  const double along = (position - wall.start).dot(wall.dir);
  const double clamped_along = std::clamp(along, 0.0, wall.length);
  return wall.start + wall.dir * clamped_along;
}

std::vector<Eigen::Vector2d> extractPathPointsForHeading(
  const std::vector<Eigen::Vector2d>& path_points_map,
  double max_arc_length)
{
  std::vector<Eigen::Vector2d> extracted;
  if (path_points_map.empty() || max_arc_length <= 0.0) {
    return extracted;
  }

  extracted.push_back(path_points_map.front());
  if (path_points_map.size() == 1) {
    return extracted;
  }

  double accumulated = 0.0;
  for (std::size_t i = 1; i < path_points_map.size(); ++i) {
    accumulated += (path_points_map[i] - path_points_map[i - 1]).norm();
    extracted.push_back(path_points_map[i]);
    if (accumulated >= max_arc_length) {
      break;
    }
  }
  return extracted;
}

bool computeAlongWallUnitDirection(
  const EffWallFrame& wall,
  double robot_yaw,
  const std::vector<Eigen::Vector2d>& path_points_map,
  double path_heading_length,
  double heading_cos_threshold,
  Eigen::Vector2d& along_out,
  bool& used_path_heading)
{
  used_path_heading = false;
  const Eigen::Vector2d robot_fwd(std::cos(robot_yaw), std::sin(robot_yaw));
  const double wall_dot_fwd = wall.dir.dot(robot_fwd);

  if (std::abs(wall_dot_fwd) >= heading_cos_threshold) {
    along_out = (wall_dot_fwd >= 0.0) ? wall.dir : -wall.dir;
    return true;
  }

  const std::vector<Eigen::Vector2d> truncated =
    extractPathPointsForHeading(path_points_map, path_heading_length);
  if (truncated.size() >= 2) {
    along_out = truncated.back() - truncated.front();
    const double len = along_out.norm();
    if (len > 1e-6) {
      along_out /= len;
      if (along_out.dot(wall.dir) < 0.0) {
        along_out = -along_out;
      }
      used_path_heading = true;
      return true;
    }
  }

  along_out = (wall_dot_fwd >= 0.0) ? wall.dir : -wall.dir;
  return true;
}

WallMonitorCorridor buildWallMonitorCorridor(
  const EffWallFrame& wall,
  const Eigen::Vector2d& robot_position,
  double robot_yaw,
  const std::vector<Eigen::Vector2d>& path_points_map,
  double rear_m,
  double forward_m,
  double lateral_inner_m,
  double lateral_robot_m,
  const TebConfig& config)
{
  WallMonitorCorridor corridor;
  if (wall.length <= 1e-6) {
    return corridor;
  }

  corridor.foot = footpointOnEffWall(robot_position, wall);
  corridor.rear_m = rear_m;
  corridor.forward_m = forward_m;
  corridor.lateral_inner_m = lateral_inner_m;
  corridor.lateral_robot_m = lateral_robot_m;

  const double heading_cos_threshold = std::cos(
    config.wall_line.monitor_corridor_heading_angle_deg * M_PI / 180.0);

  if (!computeAlongWallUnitDirection(
      wall, robot_yaw, path_points_map,
      config.wall_line.monitor_corridor_path_heading_length,
      heading_cos_threshold,
      corridor.along,
      corridor.used_path_heading))
  {
    return corridor;
  }

  corridor.normal = wall.normal;
  if (wall.robot_side_sign < 0) {
    corridor.normal = -corridor.normal;
  } else if (wall.robot_side_sign == 0) {
    const Eigen::Vector2d to_robot = robot_position - corridor.foot;
    if (to_robot.dot(corridor.normal) < 0.0) {
      corridor.normal = -corridor.normal;
    }
  }

  corridor.valid = true;
  return corridor;
}

bool isPointInWallMonitorCorridor(
  const Eigen::Vector2d& point_map,
  const WallMonitorCorridor& corridor)
{
  if (!corridor.valid) {
    return false;
  }

  const Eigen::Vector2d rel = point_map - corridor.foot;
  const double s = rel.dot(corridor.along);
  const double d = rel.dot(corridor.normal);

  if (s <= -corridor.rear_m || s > corridor.forward_m) {
    return false;
  }
  if (d < -corridor.lateral_inner_m || d > corridor.lateral_robot_m) {
    return false;
  }
  return true;
}

void wallMonitorCorridorCorners(
  const WallMonitorCorridor& corridor,
  Eigen::Vector2d corners_out[4])
{
  const Eigen::Vector2d p0 =
    corridor.foot - corridor.along * corridor.rear_m -
    corridor.normal * corridor.lateral_inner_m;
  const Eigen::Vector2d p1 =
    corridor.foot + corridor.along * corridor.forward_m -
    corridor.normal * corridor.lateral_inner_m;
  const Eigen::Vector2d p2 =
    corridor.foot + corridor.along * corridor.forward_m +
    corridor.normal * corridor.lateral_robot_m;
  const Eigen::Vector2d p3 =
    corridor.foot - corridor.along * corridor.rear_m +
    corridor.normal * corridor.lateral_robot_m;
  corners_out[0] = p0;
  corners_out[1] = p1;
  corners_out[2] = p2;
  corners_out[3] = p3;
}

WallProtrusionHit checkWallProtrusionPoint(
  const Eigen::Vector2d& vertex_map,
  const EffWallFrame& wall,
  const Eigen::Vector2d& robot_position,
  double robot_yaw,
  const TebConfig& config,
  const WallMonitorCorridor* monitor_corridor)
{
  WallProtrusionHit hit;

  const double vertex_wall_coord = wallSignedDist(vertex_map, wall);
  if (!isOnSameWallSideAsRobot(vertex_wall_coord, wall.robot_side_sign)) {
    return hit;
  }

  const WallMonitorCorridor* corridor_ptr = monitor_corridor;
  WallMonitorCorridor fallback;
  if (corridor_ptr == nullptr || !corridor_ptr->valid) {
    fallback = buildWallMonitorCorridor(
      wall, robot_position, robot_yaw, {},
      config.wall_line.wall_line_obstacle_along_wall_rear_margin,
      config.wall_line.wall_line_protrusion_exit_forward_max,
      0.0,
      config.wall_line.wall_line_protrusion_exit_lateral_max,
      config);
    corridor_ptr = &fallback;
  }

  if (!isPointInWallMonitorCorridor(vertex_map, *corridor_ptr)) {
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

namespace
{

Eigen::Vector2d closestPointOnSegment(
  const Eigen::Vector2d& p,
  const Eigen::Vector2d& seg_start,
  const Eigen::Vector2d& seg_dir,
  double seg_length)
{
  const double along = (p - seg_start).dot(seg_dir);
  const double clamped_along = std::clamp(along, 0.0, seg_length);
  return seg_start + seg_dir * clamped_along;
}

Eigen::Vector2d towardRobotUnit(
  const Eigen::Vector2d& robot_pos,
  const Eigen::Vector2d& seg_start,
  const Eigen::Vector2d& seg_dir,
  double seg_length)
{
  const Eigen::Vector2d fixed_normal(-seg_dir.y(), seg_dir.x());
  const double robot_coord = (robot_pos - seg_start).dot(fixed_normal);
  if (std::abs(robot_coord) > 1e-6) {
    return (robot_coord > 0.0) ? fixed_normal.normalized() : (-fixed_normal).normalized();
  }
  const Eigen::Vector2d robot_on_wall =
    closestPointOnSegment(robot_pos, seg_start, seg_dir, seg_length);
  const Eigen::Vector2d wall_to_robot = robot_pos - robot_on_wall;
  const double wall_to_robot_dist = wall_to_robot.norm();
  if (wall_to_robot_dist > 1e-6) {
    return wall_to_robot / wall_to_robot_dist;
  }
  return fixed_normal.normalized();
}

}  // namespace

bool buildEffWallFrameFromSegment(
  const Eigen::Vector2d& segment_start,
  const Eigen::Vector2d& segment_end,
  const Eigen::Vector2d& robot_position,
  const TebConfig& config,
  EffWallFrame& wall_out)
{
  Eigen::Vector2d base_dir = segment_end - segment_start;
  const double base_len = base_dir.norm();
  if (base_len <= 1e-6) {
    return false;
  }
  base_dir /= base_len;

  const double extension = config.wall_line.wall_line_extension_distance;
  Eigen::Vector2d eff_start = segment_start - base_dir * extension;
  Eigen::Vector2d eff_end = segment_end + base_dir * extension;

  const Eigen::Vector2d toward_robot = towardRobotUnit(
    robot_position, segment_start, base_dir, base_len);
  const double offset = config.wall_line.wall_line_safety_offset;
  eff_start += toward_robot * offset;
  eff_end += toward_robot * offset;

  Eigen::Vector2d eff_dir = eff_end - eff_start;
  const double eff_len = eff_dir.norm();
  if (eff_len <= 1e-6) {
    return false;
  }
  eff_dir /= eff_len;

  wall_out.start = eff_start;
  wall_out.dir = eff_dir;
  wall_out.normal = Eigen::Vector2d(-eff_dir.y(), eff_dir.x());
  wall_out.length = eff_len;
  wall_out.robot_side_sign = wallSideSignFromCoord(
    wallSignedDist(robot_position, wall_out));
  return true;
}

}  // namespace teb_local_planner
