/*********************************************************************
 * teb_obstacle_update.cpp
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

#include "teb_local_planner/detail/costmap_segment_clip.hpp"
#include "teb_local_planner/wall_protrusion_check.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace teb_local_planner
{
using detail::computeCohenSutherlandOutCode;
using detail::clipWorldSegmentToAlignedRect;

namespace {
using teb_local_planner::EffWallFrame;
using teb_local_planner::WallProtrusionHit;
using teb_local_planner::checkWallProtrusionPoint;
using teb_local_planner::isOnSameWallSideAsRobot;
using teb_local_planner::wallSideSignFromCoord;
using teb_local_planner::wallSignedDist;

struct WallClipGeometryParams {
  double intersect_min_span{0.05};
  double min_keep_area_sq{0.008};
  double vertex_dedupe_dist{0.02};
};

WallClipGeometryParams wallClipGeometryParamsFromConfig(const TebConfig& config)
{
  
  WallClipGeometryParams params;
  params.intersect_min_span =
    config.wall_line.wall_line_obstacle_clip_intersect_min_span;
  params.min_keep_area_sq =
    config.wall_line.wall_line_obstacle_clip_min_keep_area_sq;
  params.vertex_dedupe_dist =
    config.wall_line.wall_line_obstacle_clip_vertex_dedupe_dist;
  return params;
}

Eigen::Vector2d wallClosestPointOnSegment(
  const Eigen::Vector2d& p,
  const Eigen::Vector2d& seg_start,
  const Eigen::Vector2d& seg_dir,
  double seg_length)
{
  
  const double along = (p - seg_start).dot(seg_dir);
  const double clamped_along = std::clamp(along, 0.0, seg_length);
  return seg_start + seg_dir * clamped_along;
}

// Positive depth means farther from the wall on the robot side.
double wallRobotSideDepth(const Eigen::Vector2d& p, const EffWallFrame& wall)
{
  
  return static_cast<double>(wall.robot_side_sign) * wallSignedDist(p, wall);
}

// Unit vector from the wall line toward the robot (for safety offset).
Eigen::Vector2d wallTowardRobotUnit(
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
    wallClosestPointOnSegment(robot_pos, seg_start, seg_dir, seg_length);
  const Eigen::Vector2d wall_to_robot = robot_pos - robot_on_wall;
  const double wall_to_robot_dist = wall_to_robot.norm();
  if (wall_to_robot_dist > 1e-6) {
    return wall_to_robot / wall_to_robot_dist;
  }
  return fixed_normal.normalized();
}

double wallAlongCoord(const Eigen::Vector2d& p, const EffWallFrame& wall)
{
  
  return (p - wall.start).dot(wall.dir);
}

Eigen::Vector2d wallPointAtT(const EffWallFrame& wall, double t)
{
  
  return wall.start + wall.dir * t;
}

bool intersectEdgeWithWallSegment(
  const Eigen::Vector2d& edge_start,
  const Eigen::Vector2d& edge_end,
  const EffWallFrame& wall,
  double& t_along_wall)
{
  
  const Eigen::Vector2d edge_vec = edge_end - edge_start;
  const Eigen::Vector2d origin_to_edge = edge_start - wall.start;
  const double denom = wall.dir.x() * edge_vec.y() - wall.dir.y() * edge_vec.x();
  if (std::abs(denom) < 1e-12) {
    return false;
  }
  const double edge_param =
    (wall.dir.x() * origin_to_edge.y() - wall.dir.y() * origin_to_edge.x()) / denom;
  const double wall_param =
    (edge_vec.x() * origin_to_edge.y() - edge_vec.y() * origin_to_edge.x()) / denom;
  if (edge_param < -1e-9 || edge_param > 1.0 + 1e-9) {
    return false;
  }
  if (wall_param < -1e-9 || wall_param > wall.length + 1e-9) {
    return false;
  }
  t_along_wall = std::clamp(wall_param, 0.0, wall.length);
  return true;
}

void collectWallSegmentIntersectionTs(
  const geometry_msgs::msg::Polygon& polygon,
  const EffWallFrame& wall,
  std::vector<double>& t_values)
{
  
  t_values.clear();
  if (polygon.points.empty()) {
    return;
  }
  const std::size_t vertex_count = polygon.points.size();
  for (std::size_t edge_index = 0; edge_index < vertex_count; ++edge_index) {
    const auto& point_a = polygon.points[edge_index];
    const auto& point_b = polygon.points[(edge_index + 1) % vertex_count];
    const Eigen::Vector2d edge_start(point_a.x, point_a.y);
    const Eigen::Vector2d edge_end(point_b.x, point_b.y);
    double t_intersect = 0.0;
    if (intersectEdgeWithWallSegment(edge_start, edge_end, wall, t_intersect)) {
      t_values.push_back(t_intersect);
    }
  }
  for (const auto& point : polygon.points) {
    const Eigen::Vector2d vertex(point.x, point.y);
    const double along = wallAlongCoord(vertex, wall);
    const double signed_distance = wallSignedDist(vertex, wall);
    if (along >= -1e-6 && along <= wall.length + 1e-6 && std::abs(signed_distance) < 0.03) {
      t_values.push_back(std::clamp(along, 0.0, wall.length));
    }
  }
}

bool wallSegmentIntersectionSpanSufficient(
  const std::vector<double>& t_values,
  double intersect_min_span)
{
  
  if (t_values.size() < 2) {
    return false;
  }
  const double t_min = *std::min_element(t_values.begin(), t_values.end());
  const double t_max = *std::max_element(t_values.begin(), t_values.end());
  return (t_max - t_min) >= intersect_min_span;
}

void dedupePolygonVertices(
  std::vector<Eigen::Vector2d>& vertices,
  double vertex_dedupe_dist)
{
  
  if (vertices.size() < 2) {
    return;
  }
  std::vector<Eigen::Vector2d> deduped;
  deduped.reserve(vertices.size());
  for (const auto& vertex : vertices) {
    if (deduped.empty() ||
      (deduped.back() - vertex).norm() > vertex_dedupe_dist)
    {
      deduped.push_back(vertex);
    }
  }
  if (deduped.size() >= 2 &&
    (deduped.front() - deduped.back()).norm() < vertex_dedupe_dist)
  {
    deduped.pop_back();
  }
  vertices.swap(deduped);
}

double polygonAreaAbs(const std::vector<Eigen::Vector2d>& vertices)
{
  
  if (vertices.size() < 3) {
    return 0.0;
  }
  double twice_area = 0.0;
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const std::size_t j = (i + 1) % vertices.size();
    twice_area += vertices[i].x() * vertices[j].y() - vertices[j].x() * vertices[i].y();
  }
  return std::abs(0.5 * twice_area);
}

void orderPolygonVerticesCCW(
  std::vector<Eigen::Vector2d>& vertices,
  double vertex_dedupe_dist)
{
  
  if (vertices.size() < 3) {
    return;
  }
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto& vertex : vertices) {
    centroid += vertex;
  }
  centroid /= static_cast<double>(vertices.size());
  std::sort(
    vertices.begin(), vertices.end(),
    [&centroid](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
      return std::atan2(a.y() - centroid.y(), a.x() - centroid.x()) <
             std::atan2(b.y() - centroid.y(), b.x() - centroid.x());
    });
  dedupePolygonVertices(vertices, vertex_dedupe_dist);
}

std::vector<Eigen::Vector2d> clipPolygonDeepRobotSide(
  const std::vector<Eigen::Vector2d>& input,
  const EffWallFrame& wall,
  double clip_depth,
  double vertex_dedupe_dist)
{
  
  std::vector<Eigen::Vector2d> output;
  if (input.empty()) {
    return output;
  }
  const double keep_boundary = clip_depth;
  const auto inside = [&](const Eigen::Vector2d& point) {
    return wallRobotSideDepth(point, wall) >= keep_boundary - 1e-9;
  };
  const std::size_t vertex_count = input.size();
  for (std::size_t i = 0; i < vertex_count; ++i) {
    const Eigen::Vector2d& p1 = input[i];
    const Eigen::Vector2d& p2 = input[(i + 1) % vertex_count];
    const double d1 = wallRobotSideDepth(p1, wall);
    const double d2 = wallRobotSideDepth(p2, wall);
    const bool in1 = inside(p1);
    const bool in2 = inside(p2);
    if (in1) {
      output.push_back(p1);
    }
    if (in1 != in2) {
      const double denom = d1 - d2;
      if (std::abs(denom) > 1e-12) {
        double alpha = d1 / denom;
        alpha = std::clamp(alpha, 0.0, 1.0);
        output.push_back(p1 + alpha * (p2 - p1));
      }
    }
  }
  dedupePolygonVertices(output, vertex_dedupe_dist);
  return output;
}

void appendWallSealBetweenIntersectionTs(
  std::vector<Eigen::Vector2d>& vertices,
  const EffWallFrame& wall,
  const std::vector<double>& wall_intersection_ts,
  const WallClipGeometryParams& clip_params)
{
  
  if (wall_intersection_ts.size() < 2 || vertices.empty()) {
    return;
  }
  const double t_min = *std::min_element(
    wall_intersection_ts.begin(), wall_intersection_ts.end());
  const double t_max = *std::max_element(
    wall_intersection_ts.begin(), wall_intersection_ts.end());
  if ((t_max - t_min) < clip_params.intersect_min_span) {
    return;
  }
  vertices.push_back(wallPointAtT(wall, t_min));
  vertices.push_back(wallPointAtT(wall, t_max));
  orderPolygonVerticesCCW(vertices, clip_params.vertex_dedupe_dist);
}

void buildMinimalWallRetainedSegment(
  const EffWallFrame& wall,
  double t_mid,
  double intersect_min_span,
  std::vector<Eigen::Vector2d>& vertices)
{
  
  const double half_span = 0.5 * intersect_min_span;
  const double t0 = std::clamp(t_mid - half_span, 0.0, wall.length);
  const double t1 = std::clamp(t_mid + half_span, 0.0, wall.length);
  vertices = {wallPointAtT(wall, t0), wallPointAtT(wall, t1)};
}

struct WallPolygonClipResult {
  std::vector<Eigen::Vector2d> vertices;
  bool use_processed_vertices{false};
  bool exit_trigger{false};
  Eigen::Vector2d exit_best_point{Eigen::Vector2d::Zero()};
  double exit_best_pen{0.0};
};

WallPolygonClipResult processWallFilteredPolygon(
  const geometry_msgs::msg::Polygon& polygon,
  const EffWallFrame& wall,
  const Eigen::Vector2d& robot_position,
  double robot_yaw,
  const TebConfig& config,
  const WallMonitorCorridor* monitor_corridor)
{
  
  WallPolygonClipResult result;
  const WallClipGeometryParams clip_params = wallClipGeometryParamsFromConfig(config);
  std::vector<Eigen::Vector2d> input_vertices;
  input_vertices.reserve(polygon.points.size());
  for (const auto& point : polygon.points) {
    input_vertices.emplace_back(point.x, point.y);
  }

  double max_robot_penetration = 0.0;
  Eigen::Vector2d max_penetration_point = input_vertices.empty() ?
    robot_position : input_vertices.front();

  for (const auto& point : polygon.points) {
    const Eigen::Vector2d vertex(point.x, point.y);
    const double vertex_wall_coord = wallSignedDist(vertex, wall);
    if (isOnSameWallSideAsRobot(vertex_wall_coord, wall.robot_side_sign)) {
      const double penetration = std::abs(vertex_wall_coord);
      if (penetration > max_robot_penetration) {
        max_robot_penetration = penetration;
        max_penetration_point = vertex;
      }
    }

    const WallProtrusionHit point_hit = checkWallProtrusionPoint(
      vertex, wall, robot_position, robot_yaw, config, monitor_corridor);
    if (point_hit.exit_trigger &&
      (!result.exit_trigger || point_hit.exit_best_pen > result.exit_best_pen))
    {
      result.exit_trigger = true;
      result.exit_best_pen = point_hit.exit_best_pen;
      result.exit_best_point = point_hit.exit_best_point;
    }
  }

  const double dist_robot_to_protrusion = (max_penetration_point - robot_position).norm();
  const double base_d = config.wall_line.wall_line_obstacle_protrusion_base_distance;
  const double scale_k = config.wall_line.wall_line_obstacle_filter_distance_scale;
  double eff_clip_depth = config.wall_line.wall_line_obstacle_filter_distance;
  if (dist_robot_to_protrusion > base_d) {
    eff_clip_depth = std::min(
      eff_clip_depth + scale_k * (dist_robot_to_protrusion - base_d),
      config.wall_line.wall_line_obstacle_filter_distance_max);
  }

  std::vector<double> wall_intersection_ts;
  collectWallSegmentIntersectionTs(polygon, wall, wall_intersection_ts);

  const bool intersects_wall_segment = wallSegmentIntersectionSpanSufficient(
    wall_intersection_ts, clip_params.intersect_min_span);
  const bool shallow_protrusion = max_robot_penetration < eff_clip_depth;

  if (intersects_wall_segment && shallow_protrusion) {
    std::vector<Eigen::Vector2d> clipped =
      clipPolygonDeepRobotSide(
      input_vertices, wall, eff_clip_depth, clip_params.vertex_dedupe_dist);
    appendWallSealBetweenIntersectionTs(clipped, wall, wall_intersection_ts, clip_params);
    orderPolygonVerticesCCW(clipped, clip_params.vertex_dedupe_dist);

    if (polygonAreaAbs(clipped) < clip_params.min_keep_area_sq) {
      const double t_mid = 0.5 * (
        *std::min_element(wall_intersection_ts.begin(), wall_intersection_ts.end()) +
        *std::max_element(wall_intersection_ts.begin(), wall_intersection_ts.end()));
      buildMinimalWallRetainedSegment(
        wall, t_mid, clip_params.intersect_min_span, clipped);
    }

    if (clipped.size() >= 3) {
      result.vertices = std::move(clipped);
      result.use_processed_vertices = true;
      return result;
    }
    if (clipped.size() == 2) {
      result.vertices = std::move(clipped);
      result.use_processed_vertices = true;
      return result;
    }
    const double t_mid = wall_intersection_ts.empty() ? 0.5 * wall.length :
      0.5 * (*std::min_element(wall_intersection_ts.begin(), wall_intersection_ts.end()) +
             *std::max_element(wall_intersection_ts.begin(), wall_intersection_ts.end()));
    buildMinimalWallRetainedSegment(
      wall, t_mid, clip_params.intersect_min_span, clipped);
    result.vertices = std::move(clipped);
    result.use_processed_vertices = true;
    return result;
  }

  return result;
}

void pushObstacleFromPolygonPoints(
  std::vector<teb_local_planner::ObstaclePtr>& obstacles,
  const std::vector<Eigen::Vector2d>& vertices)
{
  
  if (vertices.size() >= 3) {
    PolygonObstacle* polyobst = new PolygonObstacle;
    for (const auto& vertex : vertices) {
      polyobst->pushBackVertex(vertex.x(), vertex.y());
    }
    polyobst->finalizePolygon();
    obstacles.push_back(ObstaclePtr(polyobst));
  } else if (vertices.size() == 2) {
    obstacles.push_back(ObstaclePtr(new LineObstacle(
      vertices[0].x(), vertices[0].y(),
      vertices[1].x(), vertices[1].y())));
  } else if (vertices.size() == 1) {
    obstacles.push_back(ObstaclePtr(new PointObstacle(vertices[0].x(), vertices[0].y())));
  }
}

}  // namespace

 void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
 {
     
   // Add costmap obstacles if desired
   if (cfg_->obstacles.include_costmap_obstacles)
   {
     std::lock_guard<std::recursive_mutex> lock(*costmap_->getMutex());
 
     Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();
     
     for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
     {
       for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
       {
         if (costmap_->getCost(i,j) == nav2_costmap_2d::LETHAL_OBSTACLE)
         {
           Eigen::Vector2d obs;
           costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
             
           // check if obstacle is interesting (e.g. not far behind the robot)
           Eigen::Vector2d obs_dir = obs-robot_pose_.position();
           if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_->obstacles.costmap_obstacles_behind_robot_dist  )
             continue;
             
           obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
         }
       }
     }
   }
 }
 
void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
{
  
  if (!costmap_converter_)
    return;
    
  //Get obstacles from costmap converter（可能为空；车身栅格仍应刷新并发布）
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  std::string obstacles_frame = cfg_->map_frame;
  if (obstacles && !obstacles->header.frame_id.empty()) {
    obstacles_frame = obstacles->header.frame_id;
  }
  if (cfg_->obstacles.enable_vehicle_scan_grid) {
    updateVehicleScanOccupancyGrid(obstacles, obstacles_frame);
  }
  const bool run_front_scan_only = (!obstacles &&
    cfg_->wall_line.enable_front_scan_protrusion_check &&
    is_edge_following_mode_);
  if (!obstacles && !run_front_scan_only) {
    if (cfg_->obstacles.enable_vehicle_scan_grid) {
      appendVehicleScanInflatedHullObstacles();
    }
    if (cfg_->obstacles.enable_vehicle_scan_grid && cfg_->obstacles.publish_vehicle_scan_grid) {
      publishVehicleScanOccupancyGrid();
    }
    return;
  }

  // --- Wall line locking, extension, safety offset, and obstacle filtering ---
  bool use_wall_line_filter = false;
  Eigen::Vector2d eff_wall_start, eff_wall_end, eff_wall_dir, eff_wall_normal;
  double eff_wall_length = 0.0;

  if (is_edge_following_mode_ && wall_line_points_.size() >= 2)
  {
    Eigen::Vector2d cur_start = wall_line_points_[0];
    Eigen::Vector2d cur_end = wall_line_points_[1];
    Eigen::Vector2d cur_dir = cur_end - cur_start;
    double cur_len = cur_dir.norm();

    if (cur_len > 1e-6)
    {
      cur_dir /= cur_len;
      double cur_angle = std::atan2(cur_dir.y(), cur_dir.x());

      // --- Wall line locking ---
      // 法向/角度超阈值则解锁；仍锁定时将当前检测端点投影到锁定支撑线上，
      // 沿墙平移锁定段（方案 A），避免优化仍使用身后旧短段。
      if (wall_line_locked_)
      {
        Eigen::Vector2d locked_dir = locked_wall_end_ - locked_wall_start_;
        const double locked_len = locked_dir.norm();
        if (locked_len > 1e-6)
        {
          locked_dir /= locked_len;
          const Eigen::Vector2d locked_normal(-locked_dir.y(), locked_dir.x());
          const double lat_start =
            std::abs((cur_start - locked_wall_start_).dot(locked_normal));
          const double lat_end =
            std::abs((cur_end - locked_wall_start_).dot(locked_normal));
          const double lateral_change = std::max(lat_start, lat_end);

          const double locked_angle = std::atan2(locked_dir.y(), locked_dir.x());
          double angle_diff = cur_angle - locked_angle;
          while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
          while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
          angle_diff = std::abs(angle_diff);
          if (angle_diff > M_PI_2)
            angle_diff = M_PI - angle_diff;

          const double angle_threshold_rad =
            cfg_->wall_line.wall_line_lock_angle_threshold * M_PI / 180.0;
          if (lateral_change > cfg_->wall_line.wall_line_lock_distance_threshold ||
              angle_diff > angle_threshold_rad)
          {
            wall_line_locked_ = false;
            wall_line_stable_count_ = 0;
            RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000,
              "墙线解锁: 横向变化=%.3f m, 角度变化=%.3f rad",
              lateral_change, angle_diff);
          }
          else
          {
            // 沿墙重锚定：当前端点投影到锁定直线，法向位置保持在锁定支撑线上。
            const Eigen::Vector2d locked_origin = locked_wall_start_;
            double t0 = (cur_start - locked_origin).dot(locked_dir);
            double t1 = (cur_end - locked_origin).dot(locked_dir);
            if (t1 < t0) {
              std::swap(t0, t1);
            }
            // 退化时退回以机器人为中心、长度与当前检测相当的窗口。
            if (t1 - t0 < 1e-3) {
              const double t_robot =
                (robot_pose_.position() - locked_origin).dot(locked_dir);
              const double half = 0.5 * cur_len;
              t0 = t_robot - half;
              t1 = t_robot + half;
            }
            locked_wall_start_ = locked_origin + t0 * locked_dir;
            locked_wall_end_ = locked_origin + t1 * locked_dir;
            RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000,
              "墙线锁沿墙滑动: t=[%.2f, %.2f] 长度=%.2f m",
              t0, t1, t1 - t0);
          }
        }
      }

      if (!wall_line_locked_)
      {
        ++wall_line_stable_count_;
        if (wall_line_stable_count_ >= cfg_->wall_line.wall_line_lock_min_stable_count)
        {
          locked_wall_start_ = cur_start;
          locked_wall_end_ = cur_end;
          wall_line_locked_ = true;
          RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000,
            "墙线已锁定，稳定帧数: %d", wall_line_stable_count_);
        }
      }

      // Use locked wall line if available, otherwise current
      Eigen::Vector2d base_start = wall_line_locked_ ? locked_wall_start_ : cur_start;
      Eigen::Vector2d base_end = wall_line_locked_ ? locked_wall_end_ : cur_end;
      Eigen::Vector2d base_dir = base_end - base_start;
      double base_len = base_dir.norm();
      if (base_len > 1e-6)
      {
        base_dir /= base_len;

        // --- Extend wall line on both ends ---
        double ext = cfg_->wall_line.wall_line_extension_distance;
        eff_wall_start = base_start - base_dir * ext;
        eff_wall_end = base_end + base_dir * ext;

        // --- Safety offset: shift toward robot side for conservative distance ---
        const Eigen::Vector2d toward_robot_unit = wallTowardRobotUnit(
          robot_pose_.position(), base_start, base_dir, base_len);

        double offset = cfg_->wall_line.wall_line_safety_offset;
        eff_wall_start += toward_robot_unit * offset;
        eff_wall_end += toward_robot_unit * offset;

        eff_wall_dir = eff_wall_end - eff_wall_start;
        eff_wall_length = eff_wall_dir.norm();
        if (eff_wall_length > 1e-6)
        {
          eff_wall_dir /= eff_wall_length;
          eff_wall_normal = Eigen::Vector2d(-eff_wall_dir.y(), eff_wall_dir.x());
          use_wall_line_filter = true;
        }
      }
    }
  }
  else
  {
    if (wall_line_locked_)
    {
      wall_line_locked_ = false;
      wall_line_stable_count_ = 0;
    }
  }

  EffWallFrame eff_wall;
  if (use_wall_line_filter) {
    eff_wall.start = eff_wall_start;
    eff_wall.dir = eff_wall_dir;
    eff_wall.normal = eff_wall_normal;
    eff_wall.length = eff_wall_length;
    eff_wall.robot_side_sign = wallSideSignFromCoord(
      wallSignedDist(robot_pose_.position(), eff_wall));
  }

  // --- Add obstacles; edge mode clips shallow wall-band geometry instead of skipping ---
  if (obstacles) {
  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter_msgs::msg::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::msg::Polygon* polygon = &obstacle->polygon;

    WallPolygonClipResult wall_result;
    if (polygon->points.size() > 2) {
      wall_result = processWallFilteredPolygon(
        *polygon,
        eff_wall,
        robot_pose_.position(),
        robot_pose_.theta(),
        *cfg_,
        protrusion_monitor_corridor_.valid ? &protrusion_monitor_corridor_ : nullptr);

      if (wall_result.exit_trigger) {
        recordProtrusionDetection(
          wall_result.exit_best_point,
          wall_result.exit_best_pen,
          "updateObstacleContainerWithCostmapConverter");
      }
    }

    if (wall_result.use_processed_vertices && !wall_result.vertices.empty()) {
      pushObstacleFromPolygonPoints(obstacles_, wall_result.vertices);
    } else if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    {
      obstacles_.push_back(ObstaclePtr(new CircularObstacle(
        polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    }
    else if (polygon->points.size()==2) // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(
        polygon->points[0].x, polygon->points[0].y,
        polygon->points[1].x, polygon->points[1].y)));
    }
    else if (polygon->points.size()>2) // Real polygon
    {
      PolygonObstacle* polyobst = new PolygonObstacle;
      for (std::size_t j= 0; j < polygon->points.size(); ++j) {
        polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
      }
      polyobst->finalizePolygon();
      obstacles_.push_back(ObstaclePtr(polyobst));
    }

    // Set velocity, if obstacle is moving
    if(!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
  }
  }

  checkFrontScanProtrusionExit(
    eff_wall,
    robot_pose_.position(),
    obstacles_frame);

  if (cfg_->obstacles.enable_vehicle_scan_grid) {
    appendVehicleScanInflatedHullObstacles();
  }

  // --- Add the effective wall line as a wall-guide LineObstacle (side-clearance only) ---
  if (use_wall_line_filter)
  {
    ObstaclePtr wall_guide(new LineObstacle(
      eff_wall_start.x(), eff_wall_start.y(),
      eff_wall_end.x(), eff_wall_end.y()));
    wall_guide->setWallGuide(true);
    obstacles_.push_back(wall_guide);
  }

  // OccupancyGrid 仅在 costmap_converter 流程末尾发布（不在纯 costmap 点障碍路径后发布）
  if (cfg_->obstacles.enable_vehicle_scan_grid && cfg_->obstacles.publish_vehicle_scan_grid) {
    publishVehicleScanOccupancyGrid();
  }
}
 
 
 void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
 {
   
   // Add custom obstacles obtained via message
   std::lock_guard<std::mutex> l(custom_obst_mutex_);

   // TEB historically latches the last ObstacleArray forever. When a dynamic-obstacle
   // publisher stops, that leaves stale dynamic objects (and OmegaHold) active forever.
   // In predictable mode, expire the latched message if no new CB arrives in time.
   if (cfg_->obstacles.dynamic_safety_predictable_mode && custom_obstacle_msg_valid_)
   {
     const double timeout = std::max(cfg_->obstacles.dynamic_obstacle_cache_time,
                                     cfg_->obstacles.dyn_gate_hold_time);
     const double age = (clock_->now() - custom_obstacle_receive_time_).seconds();
     if (timeout > 0.0 && age > timeout)
     {
       RCLCPP_INFO_THROTTLE(
           logger_, *clock_, 2000,
           "[NearHorizon] custom obstacles expired (age=%.2fs > timeout=%.2fs); clearing latch",
           age, timeout);
       custom_obstacle_msg_.obstacles.clear();
       custom_obstacle_msg_valid_ = false;
     }
   }
 
   if (!custom_obstacle_msg_.obstacles.empty())
   {
     // We only use the global header to specify the obstacle coordinate system instead of individual ones
     Eigen::Affine3d obstacle_to_map_eig;
     try 
     {
       // Use TimePointZero to get the latest available transform, avoiding extrapolation errors
       geometry_msgs::msg::TransformStamped obstacle_to_map = tf_->lookupTransform(
                   cfg_->map_frame,
                   custom_obstacle_msg_.header.frame_id,
                   tf2::TimePointZero,
                   tf2::durationFromSec(0.5));
       obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
       //tf2::fromMsg(obstacle_to_map.transform, obstacle_to_map_eig);
     }
     catch (const tf2::ExtrapolationException& ex)
     {
       RCLCPP_WARN(logger_, "自定义障碍物 TF 查询失败 (ExtrapolationException): %s, 改用单位变换", ex.what());
       obstacle_to_map_eig.setIdentity();
     }
     catch (const tf2::TransformException& ex)
     {
       RCLCPP_ERROR(logger_, "自定义障碍物 TF 查询失败 (TransformException): %s, 改用单位变换", ex.what());
       obstacle_to_map_eig.setIdentity();
     }
     
     for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
     {
       if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0 ) // circle
       {
         Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
         obstacles_.push_back(ObstaclePtr(new CircularObstacle( (obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
       }
       else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 ) // point
       {
         Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
         obstacles_.push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
       }
       else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2 ) // line
       {
         Eigen::Vector3d line_start( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                     custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                     custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
         Eigen::Vector3d line_end( custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                   custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                   custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z );
         obstacles_.push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2),
                                                            (obstacle_to_map_eig * line_end).head(2) )));
       }
       else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
       {
         RCLCPP_INFO(logger_, "收到无效自定义障碍物，多边形顶点为空，跳过");
         continue;
       }
       else // polygon
       {
         PolygonObstacle* polyobst = new PolygonObstacle;
         for (size_t j=0; j<custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
         {
           Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                                custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                                custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z );
           polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
         }
         polyobst->finalizePolygon();
         obstacles_.push_back(ObstaclePtr(polyobst));
       }
 
       // Set velocity, if obstacle is moving
       if(!obstacles_.empty())
         obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
     }
   }
 }
 
 void TebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, double min_separation)
 {
   
   via_points_.clear();
   
   if (min_separation<=0)
     return;
   ViaPointContainer occupied_points;
   std::size_t prev_idx = 0;
   for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
   {
     // check separation to the previous via-point inserted
     if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
       continue;
     unsigned int mx = 0;
     unsigned int my = 0;
     if (costmap_->worldToMap(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y, mx, my) && costmap_->getCost(mx, my) >= nav2_costmap_2d::MAX_NON_OBSTACLE)
     {
       // add via-point
       via_points_.clear();
       break;
     }
     via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
     prev_idx = i;
   } 
 }

 void TebLocalPlannerROS::customObstacleCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obst_msg)
 {
   
   std::lock_guard<std::mutex> l(custom_obst_mutex_);
   custom_obstacle_msg_ = *obst_msg;
   custom_obstacle_receive_time_ = clock_->now();
   custom_obstacle_msg_valid_ = true;
 }

 void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::msg::Path::ConstSharedPtr via_points_msg)
 {
   
   RCLCPP_INFO_ONCE(logger_, "已收到自定义经由点 (via-points)，本条只打印一次");
   if (cfg_->trajectory.global_plan_viapoint_sep > 0)
   {
     RCLCPP_INFO(logger_, "经由点已由全局路径生成 (global_plan_viapoint_sep>0)，"
              "忽略自定义经由点");
     custom_via_points_active_ = false;
     return;
   }
 
   std::lock_guard<std::mutex> l(via_point_mutex_);
   via_points_.clear();
   for (const geometry_msgs::msg::PoseStamped& pose : via_points_msg->poses)
   {
     via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
   }
   custom_via_points_active_ = !via_points_.empty();
 }

}  // namespace teb_local_planner
