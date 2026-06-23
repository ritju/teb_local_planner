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

 #include "teb_local_planner/teb_local_planner_ros.h"

 //#include <tf_conversions/tf_eigen.h>
 #include <boost/algorithm/string.hpp>

 #include <nav2_costmap_2d/cost_values.hpp>
#include <rclcpp/logging.hpp>
#include "rcl_interfaces/srv/set_parameters.hpp"
#include <future>
#include <thread>
 #include <string>
 
 // pluginlib macros
 #include <pluginlib/class_list_macros.hpp>
 
 #include "g2o/core/sparse_optimizer.h"
 #include "g2o/core/block_solver.h"
 #include "g2o/core/factory.h"
 #include "g2o/core/optimization_algorithm_gauss_newton.h"
 #include "g2o/core/optimization_algorithm_levenberg.h"
 #include "g2o/solvers/csparse/linear_solver_csparse.h"
 #include "g2o/solvers/cholmod/linear_solver_cholmod.h"
 
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
 #include <algorithm>
#include <cmath>
#include <cctype>
#include <cstring>
#include <limits>
 
 using nav2_util::declare_parameter_if_not_declared;
 
 namespace teb_local_planner
 {
 
   const char* use_curb_or_wall = std::getenv("USE_CURB_OR_WALL");

namespace {
std::string normalizeEdgeModeEnv(const char* environment_string)
{
  if (environment_string == nullptr) {
    RCLCPP_WARN(rclcpp::get_logger("controller_server"), "USE_CURB_OR_WALL 环境变量为空");
    return "";
  }
  std::string normalized_characters;
  for (const char* character_pointer = environment_string; *character_pointer != '\0';
       ++character_pointer) {
    if (*character_pointer != ' ' && *character_pointer != '\t') {
      normalized_characters.push_back(static_cast<char>(
        std::tolower(static_cast<unsigned char>(*character_pointer))));
    }
  }
  return normalized_characters;
}

namespace {

rclcpp::Clock & edgeFollowingLogClock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

double pointToSegmentDistance2d(
  const Eigen::Vector2d& query_point,
  const Eigen::Vector2d& segment_start,
  const Eigen::Vector2d& segment_end)
{
  const Eigen::Vector2d segment_vector = segment_end - segment_start;
  const double segment_length_squared = segment_vector.squaredNorm();
  if (segment_length_squared < 1e-12) {
    return (query_point - segment_start).norm();
  }
  const double projected_parameter =
    (query_point - segment_start).dot(segment_vector) / segment_length_squared;
  const double clamped_parameter = std::max(0.0, std::min(1.0, projected_parameter));
  const Eigen::Vector2d projected_point = segment_start + clamped_parameter * segment_vector;
  return (query_point - projected_point).norm();
}

double segmentToSegmentDistance2d(
  const Eigen::Vector2d& first_segment_start,
  const Eigen::Vector2d& first_segment_end,
  const Eigen::Vector2d& second_segment_start,
  const Eigen::Vector2d& second_segment_end)
{
  const double first_to_second_start =
    pointToSegmentDistance2d(first_segment_start, second_segment_start, second_segment_end);
  const double first_to_second_end =
    pointToSegmentDistance2d(first_segment_end, second_segment_start, second_segment_end);
  const double second_to_first_start =
    pointToSegmentDistance2d(second_segment_start, first_segment_start, first_segment_end);
  const double second_to_first_end =
    pointToSegmentDistance2d(second_segment_end, first_segment_start, first_segment_end);
  return std::min(
    std::min(first_to_second_start, first_to_second_end),
    std::min(second_to_first_start, second_to_first_end));
}

}  // namespace

bool segmentsCloseForFusion(
  const Eigen::Vector2d& fusion_primary_segment_start,
  const Eigen::Vector2d& fusion_primary_segment_end,
  const Eigen::Vector2d& reference_segment_start,
  const Eigen::Vector2d& reference_segment_end,
  const double maximum_angle_degrees,
  const double maximum_segment_separation_meters)
{
  const Eigen::Vector2d primary_direction = fusion_primary_segment_end - fusion_primary_segment_start;
  const Eigen::Vector2d reference_direction = reference_segment_end - reference_segment_start;
  const double primary_segment_length = primary_direction.norm();
  const double reference_segment_length = reference_direction.norm();
  if (primary_segment_length < 1e-6 || reference_segment_length < 1e-6) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teb_local_planner"), edgeFollowingLogClock(), 2000,
                         "[segmentsCloseForFusion] Edge following:" 
                         "贴边参考线和墙线或路沿特征长度太短, 墙线或路沿特征长度: %.2f, 贴边参考线长度: %.2f", 
                         primary_segment_length, reference_segment_length);
    return false;
  }
  const double cosine_alignment =
    std::fabs(primary_direction.dot(reference_direction) / (primary_segment_length * reference_segment_length));
  const double angle_degrees =
    std::acos(std::min(1.0, cosine_alignment)) * 180.0 / M_PI;
  if (angle_degrees > maximum_angle_degrees) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teb_local_planner"), edgeFollowingLogClock(), 2000,
                         "[segmentsCloseForFusion] Edge following:"
                         "贴边参考线和墙线或路沿特征角度太大: %.2f, 最大允许角度: %.2f", 
                         angle_degrees, maximum_angle_degrees);
    return false;
  }
  const double segment_separation_meters = segmentToSegmentDistance2d(
    fusion_primary_segment_start,
    fusion_primary_segment_end,
    reference_segment_start,
    reference_segment_end);
  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("teb_local_planner"), edgeFollowingLogClock(), 2000,
                       "[segmentsCloseForFusion] Edge following:"
                       "贴边参考线与墙线或路沿特征距离: %.2f, 最大允许距离: %.2f",
                       segment_separation_meters,
                       maximum_segment_separation_meters);
  if (segment_separation_meters > maximum_segment_separation_meters) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teb_local_planner"), edgeFollowingLogClock(), 2000,
                         "[segmentsCloseForFusion] Edge following:"
                         "贴边参考线和墙线或路沿特征距离太大: %.2f, 最大允许距离: %.2f",
                         segment_separation_meters, maximum_segment_separation_meters);
    return false;
  }
  return true;
}

int computeCohenSutherlandOutCode(
  double x, double y,
  double xmin, double ymin, double xmax, double ymax)
{
  constexpr int LEFT = 1;
  constexpr int RIGHT = 2;
  constexpr int BOTTOM = 4;
  constexpr int TOP = 8;
  int code = 0;
  if (x < xmin) {
    code |= LEFT;
  } else if (x > xmax) {
    code |= RIGHT;
  }
  if (y < ymin) {
    code |= BOTTOM;
  } else if (y > ymax) {
    code |= TOP;
  }
  return code;
}

bool clipWorldSegmentToAlignedRect(
  double & x0,
  double & y0,
  double & x1,
  double & y1,
  double xmin,
  double ymin,
  double xmax,
  double ymax)
{
  constexpr double kEpsilonSeg = 1e-14;
  int out_code0 =
    computeCohenSutherlandOutCode(x0, y0, xmin, ymin, xmax, ymax);
  int out_code1 =
    computeCohenSutherlandOutCode(x1, y1, xmin, ymin, xmax, ymax);

  for (int iterations = 0; iterations < 64; ++iterations) {
    if (!(out_code0 | out_code1)) {
      const double dx = x1 - x0;
      const double dy = y1 - y0;
      return (dx * dx + dy * dy > kEpsilonSeg * kEpsilonSeg);
    }
    if (out_code0 & out_code1) {
      return false;
    }

    const int outside = out_code0 ? out_code0 : out_code1;
    double x_intersect = x0;
    double y_intersect = y0;
    constexpr int TOP = 8;
    constexpr int BOTTOM = 4;
    constexpr int RIGHT = 2;
    constexpr int LEFT = 1;

    if (outside & TOP) {
      const double denom = y1 - y0;
      if (std::fabs(denom) > kEpsilonSeg) {
        x_intersect = x0 + (x1 - x0) * (ymax - y0) / denom;
      } else {
        x_intersect = x0;
      }
      y_intersect = ymax;
    } else if (outside & BOTTOM) {
      const double denom = y1 - y0;
      if (std::fabs(denom) > kEpsilonSeg) {
        x_intersect = x0 + (x1 - x0) * (ymin - y0) / denom;
      } else {
        x_intersect = x0;
      }
      y_intersect = ymin;
    } else if (outside & RIGHT) {
      const double denom = x1 - x0;
      if (std::fabs(denom) > kEpsilonSeg) {
        y_intersect = y0 + (y1 - y0) * (xmax - x0) / denom;
      } else {
        y_intersect = y0;
      }
      x_intersect = xmax;
    } else if (outside & LEFT) {
      const double denom = x1 - x0;
      if (std::fabs(denom) > kEpsilonSeg) {
        y_intersect = y0 + (y1 - y0) * (xmin - x0) / denom;
      } else {
        y_intersect = y0;
      }
      x_intersect = xmin;
    }

    if (outside == out_code0) {
      x0 = x_intersect;
      y0 = y_intersect;
      out_code0 =
        computeCohenSutherlandOutCode(x0, y0, xmin, ymin, xmax, ymax);
    } else {
      x1 = x_intersect;
      y1 = y_intersect;
      out_code1 =
        computeCohenSutherlandOutCode(x1, y1, xmin, ymin, xmax, ymax);
    }
  }
  return false;
}

struct EffWallFrame {
  Eigen::Vector2d start;
  Eigen::Vector2d dir;
  Eigen::Vector2d normal;
  double length{0.0};
  int robot_side_sign{0};
};

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

// Signed coordinate of p relative to the infinite wall line through wall.start.
double wallSignedDist(const Eigen::Vector2d& p, const EffWallFrame& wall)
{
  return (p - wall.start).dot(wall.normal);
}

int wallSideSignFromCoord(double coord, double eps = 1e-9)
{
  if (coord > eps) {
    return 1;
  }
  if (coord < -eps) {
    return -1;
  }
  return 0;
}

bool isOnSameWallSideAsRobot(double point_coord, int robot_side_sign, double eps = 1e-9)
{
  const int point_sign = wallSideSignFromCoord(point_coord, eps);
  if (point_sign == 0 || robot_side_sign == 0) {
    return true;
  }
  return point_sign == robot_side_sign;
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
  bool have_tf_obstacles_to_base,
  const geometry_msgs::msg::TransformStamped& tf_base_from_obstacles,
  const std::string& obstacles_frame,
  const TebConfig& config)
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

  const double base_d_exit = config.wall_line.wall_line_obstacle_protrusion_base_distance;
  const double scale_k_exit = config.wall_line.wall_line_protrusion_exit_depth_min;
  const auto eff_protrusion_for_vertex_dist = [&](double dist_robot_vertex) -> double {
    double effective = config.wall_line.wall_line_protrusion_exit_depth_min;
    if (dist_robot_vertex > base_d_exit) {
      effective = std::min(
        effective + scale_k_exit * (dist_robot_vertex - base_d_exit),
        config.wall_line.wall_line_obstacle_filter_distance_max);
    }
    return effective;
  };

  const double fwd_max = config.wall_line.wall_line_protrusion_exit_forward_max;
  const double lat_max = config.wall_line.wall_line_protrusion_exit_lateral_max;
  const double depth_max = config.wall_line.wall_line_protrusion_exit_depth_max;

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

    if (have_tf_obstacles_to_base) {
      geometry_msgs::msg::PointStamped pin;
      pin.header.frame_id = obstacles_frame;
      pin.point.x = point.x;
      pin.point.y = point.y;
      pin.point.z = point.z;
      geometry_msgs::msg::PointStamped p_base;
      tf2::doTransform(pin, p_base, tf_base_from_obstacles);

      if (p_base.point.x <= -config.wall_line.wall_line_obstacle_along_wall_rear_margin ||
        p_base.point.x > fwd_max ||
        std::fabs(p_base.point.y) > lat_max)
      {
        continue;
      }
    } else {
      continue;
    }

    if (!isOnSameWallSideAsRobot(vertex_wall_coord, wall.robot_side_sign)) {
      continue;
    }
    const double pen = std::abs(vertex_wall_coord);
    const double dist_robot_vertex = (vertex - robot_position).norm();
    const double eff_v = eff_protrusion_for_vertex_dist(dist_robot_vertex);
    if (pen <= eff_v || pen > depth_max) {
      continue;
    }
    if (!result.exit_trigger || pen > result.exit_best_pen) {
      result.exit_trigger = true;
      result.exit_best_pen = pen;
      result.exit_best_point = vertex;
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

bool footprintAxisAlignedBboxSeparateFromCostmapRect(
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double map_x_min,
  double map_y_min,
  double map_x_max_ex,
  double map_y_max_ex)
{
  if (footprint.empty()) {
    return true;
  }
  double fminx = footprint[0].x;
  double fmaxx = footprint[0].x;
  double fminy = footprint[0].y;
  double fmaxy = footprint[0].y;
  for (unsigned int ii = 1; ii < footprint.size(); ++ii) {
    fminx = std::min(fminx, footprint[ii].x);
    fmaxx = std::max(fmaxx, footprint[ii].x);
    fminy = std::min(fminy, footprint[ii].y);
    fmaxy = std::max(fmaxy, footprint[ii].y);
  }
  constexpr double tol = 1e-9;
  return (
    fmaxx <= map_x_min + tol ||
    fminx >= map_x_max_ex - tol ||
    fmaxy <= map_y_min + tol ||
    fminy >= map_y_max_ex - tol);
}

bool clippedFootprintOutlineTouchesBlockingCost(
  const nav2_costmap_2d::Costmap2D & costmap_ref,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double clip_x_min,
  double clip_y_min,
  double clip_x_max,
  double clip_y_max)
{
  if (footprint.size() < 2) {
    return false;
  }
  if (!(clip_x_max > clip_x_min && clip_y_max > clip_y_min)) {
    return false;
  }
  const unsigned int sx = costmap_ref.getSizeInCellsX();
  const unsigned int sy = costmap_ref.getSizeInCellsY();
  const unsigned int n = static_cast<unsigned int>(footprint.size());
  for (unsigned int i = 0; i < n; ++i) {
    const unsigned int j = (i + 1) % n;
    double ax = footprint[i].x;
    double ay = footprint[i].y;
    double bx = footprint[j].x;
    double by = footprint[j].y;
    if (!clipWorldSegmentToAlignedRect(ax, ay, bx, by,
      clip_x_min, clip_y_min, clip_x_max, clip_y_max))
    {
      continue;
    }
    int gx0_int = 0;
    int gy0_int = 0;
    int gx1_int = 0;
    int gy1_int = 0;
    costmap_ref.worldToMapEnforceBounds(ax, ay, gx0_int, gy0_int);
    costmap_ref.worldToMapEnforceBounds(bx, by, gx1_int, gy1_int);
    dwb_critics::LineIterator line(gx0_int, gy0_int, gx1_int, gy1_int);
    for (; line.isValid(); line.advance()) {
      const int lx = line.getX();
      const int ly = line.getY();
      if (lx < 0 || ly < 0 || lx >= static_cast<int>(sx) || ly >= static_cast<int>(sy)) {
        continue;
      }
      const unsigned char cost = costmap_ref.getCost(
        static_cast<unsigned int>(lx), static_cast<unsigned int>(ly));
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::NO_INFORMATION)
      {
        return true;
      }
    }
  }
  return false;
}
}  // namespace
   
 
 TebLocalPlannerROS::TebLocalPlannerROS() 
     : costmap_ros_(nullptr), tf_(nullptr), cfg_(new TebConfig()), costmap_model_(nullptr), intra_proc_node_(nullptr),
                                            costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                            custom_via_points_active_(false), no_infeasible_plans_(0),
                                            last_preferred_rotdir_(RotType::none), initialized_(false),
                                            weight_via_point_(1.0),
                                            cfg_max_angular_vel_(0.6), cfg_max_vel_x_(0.5), cfg_max_angular_acc_(0.6), wall_line_update_time_(0),
                                            curb_line_update_time_(0), min_obstacle_dist_(0.5), wall_line_ptr_(nullptr), curb_line_subscriber_(nullptr),
                                            normal_weight_optimaltime_(2.0), normal_min_obstacle_dist_(0.2),
                                            normal_footprint_vertices_("[[1.25, 0.55], [1.25, -0.55], [-0.65, -0.55], [-0.65, 0.55]]"),
                                            edge_weight_optimaltime_(10.0), edge_min_obstacle_dist_(0.05), min_wall_line_length_(1.0),
                                            edge_footprint_vertices_("[[1.25, 0.5], [1.25, -0.5], [-0.65, -0.5], [-0.65, 0.5]]"),
                                            prune_angle_threshold_(1.57079632679),
                                            is_edge_following_mode_(false),
                                            control_duration_(0.2), 
                                            safe_linear_speed_limit_(2.0),
                                            keep_wall_line_time_(5.0),
                                            new_vehicle_distance_threshold_(5.0),
                                            erase_vehicle_distance_threshold_(10.0),
                                            has_speed_limit_(false)
 {
   // Initialize edge mode parameters from config defaults (will be overridden by parameters if available)
   edge_weight_optimaltime_ = cfg_->wall_line.edge_weight_optimaltime;
   edge_min_obstacle_dist_ = cfg_->wall_line.edge_min_obstacle_dist;
   edge_footprint_vertices_ = cfg_->wall_line.edge_footprint_vertices;
 }
 
 
 TebLocalPlannerROS::~TebLocalPlannerROS()
 {
 }
 
 void TebLocalPlannerROS::initialize(nav2_util::LifecycleNode::SharedPtr node)
 {
   // check if the plugin is already initialized
   if(!initialized_)
   {	
    // declare parameters (ros2-dashing)
    intra_proc_node_.reset( 
            new rclcpp::Node("costmap_converter", node->get_namespace(), 
              rclcpp::NodeOptions()));
    cfg_->declareParameters(node, name_);

    // get parameters of TebConfig via the nodehandle and override the default config
    cfg_->loadRosParamFromNodeHandle(node, name_);
    weight_wall_line_direction_ = cfg_->optim.weight_wall_line_direction;
    weight_wall_line_dist_ = cfg_->optim.weight_wall_line_dist;
    weight_via_point_ = cfg_->optim.weight_viapoint;
    cfg_max_angular_vel_ = cfg_->robot.max_vel_theta;
    cfg_max_vel_x_ = cfg_->robot.max_vel_x;
    cfg_max_angular_acc_= cfg_->robot.acc_lim_theta;
    min_obstacle_dist_ = cfg_->obstacles.min_obstacle_dist;
    
    // Save normal mode parameters
    normal_weight_optimaltime_ = cfg_->optim.weight_optimaltime;
    normal_min_obstacle_dist_ = cfg_->obstacles.min_obstacle_dist;
    // Try to get footprint vertices from parameter server
    std::string footprint_string;
    if (node->get_parameter(name_ + "." + "footprint_model.vertices", footprint_string)) {
      normal_footprint_vertices_ = footprint_string;
    }
    
    // Load edge-following mode parameters from config
    edge_weight_optimaltime_ = cfg_->wall_line.edge_weight_optimaltime;
    edge_min_obstacle_dist_ = cfg_->wall_line.edge_min_obstacle_dist;
    edge_footprint_vertices_ = cfg_->wall_line.edge_footprint_vertices;
    // via_sep_ = cfg_->trajectory.global_plan_viapoint_sep;
    RCLCPP_INFO(logger_, "max_global_plan_lookahead_dist %.2f, max_vel_x: %.2f! In initialize!", cfg_->trajectory.max_global_plan_lookahead_dist, cfg_->robot.max_vel_x);

    // reserve some memory for obstacles
    obstacles_.reserve(500);
        
    // create the planner instance
    if (cfg_->hcp.enable_homotopy_class_planning)
    {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(node, *cfg_.get(), &obstacles_, visualization_, &via_points_, &wall_line_points_));
      RCLCPP_INFO(logger_, "Parallel planning in distinctive topologies enabled.");
    }
    else
    {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(node, *cfg_.get(), &obstacles_, visualization_, &via_points_, &wall_line_points_));
      RCLCPP_INFO(logger_, "Parallel planning in distinctive topologies disabled.");
    }
    
    // init other variables
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    
    costmap_model_ = std::make_shared<dwb_critics::ObstacleFootprintCritic>();
    std::string costmap_model_name("costmap_model");
    costmap_model_->initialize(node, costmap_model_name, name_, costmap_ros_);

    // Initialize rotation collision checker
    rotation_collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  
    // Get controller frequency for control_duration
    double controller_frequency = 5.0;
    node->get_parameter("controller_frequency", controller_frequency);
    control_duration_ = 1.0 / controller_frequency;

    cfg_->map_frame = costmap_ros_->getGlobalFrameID(); // TODO

    //Initialize a costmap to polygon converter
    if (!cfg_->obstacles.costmap_converter_plugin.empty())
    {
      try
      {
        costmap_converter_ = costmap_converter_loader_.createSharedInstance(cfg_->obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(cfg_->obstacles.costmap_converter_plugin);
        RCLCPP_INFO(logger_, "library path : %s", costmap_converter_loader_.getClassLibraryPath(cfg_->obstacles.costmap_converter_plugin).c_str());
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
        boost::replace_all(converter_name, "::", "/");

        costmap_converter_->setOdomTopic(cfg_->odom_topic);
        
        // Pass the main (lifecycle) node directly to costmap_converter so it can read parameters from controller node
        // Parameters will be read directly from the node's namespace (e.g., controller_server.FollowPath.cluster_max_distance)
        costmap_converter_->initialize(node);
        costmap_converter_->setCostmap2D(costmap_);
        const auto rate = std::make_shared<rclcpp::Rate>((double)cfg_->obstacles.costmap_converter_rate);
        costmap_converter_->startWorker(rate, costmap_, cfg_->obstacles.costmap_converter_spin_thread);
        RCLCPP_INFO(logger_, "Costmap conversion plugin %s loaded.", cfg_->obstacles.costmap_converter_plugin.c_str());
      }
      catch(pluginlib::PluginlibException& ex)
      {
        RCLCPP_INFO(logger_,
                    "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
        costmap_converter_.reset();
      }
    }
    else {
      RCLCPP_INFO(logger_, "No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
    }
  
    
    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);

    // Add callback for dynamic parameters
    dyn_params_handler = node->add_on_set_parameters_callback(
      std::bind(&TebConfig::dynamicParametersCallback, std::ref(cfg_), std::placeholders::_1));

    // validate optimization footprint and costmap footprint
    validateFootprints(cfg_->robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_->obstacles.min_obstacle_dist);
        
    // setup callback for custom obstacles
    custom_obst_sub_ = node->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
                "obstacles", 
                rclcpp::SystemDefaultsQoS(),
                std::bind(&TebLocalPlannerROS::customObstacleCB, this, std::placeholders::_1));

    // setup callback for custom via-points
    via_points_sub_ = node->create_subscription<nav_msgs::msg::Path>(
                "via_points", 
                rclcpp::SystemDefaultsQoS(),
                std::bind(&TebLocalPlannerROS::customViaPointsCB, this, std::placeholders::_1));

    // setup callback for surrounding vehicle poses
    {
      rclcpp::QoS vehicle_qos(rclcpp::KeepLast(1));
      vehicle_qos.transient_local();
      vehicle_qos.reliable();
      vehicle_poses_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/vehicle_poses_around",
        vehicle_qos,
        std::bind(&TebLocalPlannerROS::vehiclePosesCallback, this, std::placeholders::_1));
      RCLCPP_INFO(logger_, "TEB 已订阅 /vehicle_poses_around 话题");
    }

    // setup callback for external speed limit (linear x)
    {
      rclcpp::QoS speed_limit_qos(rclcpp::KeepLast(1));
      speed_limit_qos.transient_local();
      speed_limit_qos.reliable();
      speed_limit_sub_ = node->create_subscription<std_msgs::msg::Float64>(
                "/nav/speed_limit",
                speed_limit_qos,
                std::bind(&TebLocalPlannerROS::speedLimitCallback, this, std::placeholders::_1));
      RCLCPP_INFO(logger_, "TEB 已订阅 /nav/speed_limit 话题");
    }
    {
      // 狭窄通道多边形（与 Smac 共用 /narrow_passages）
      rclcpp::QoS narrow_qos(rclcpp::KeepLast(1));
      narrow_qos.transient_local();
      narrow_qos.reliable();
      narrow_passages_sub_ = node->create_subscription<garage_utils_msgs::msg::Polygons>(
        "/narrow_passages",
        narrow_qos,
        std::bind(&TebLocalPlannerROS::narrowPassagesCallback, this, std::placeholders::_1));
      narrow_passages_marker_pub_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>("teb_narrow_passages_markers", 1);
      RCLCPP_INFO(
        logger_,
        "狭窄通道: TEB 已订阅 /narrow_passages，"
        "forward_drive 正常=%.2f，窄通道=%.2f，Marker 话题 teb_narrow_passages_markers",
        cfg_->optim.weight_kinematics_forward_drive,
        cfg_->optim.weight_kinematics_forward_drive_in_narrow_passages);
    }
    {
      rclcpp::QoS enable_backward_qos(rclcpp::KeepLast(1));
      enable_backward_qos.transient_local();
      enable_backward_qos.reliable();
      enable_backward_sub_ = node->create_subscription<std_msgs::msg::Bool>(
        "/enable_backward",
        enable_backward_qos,
        std::bind(&TebLocalPlannerROS::enableBackwardCallback, this, std::placeholders::_1));
      RCLCPP_INFO(logger_, "TEB 已订阅 /enable_backward，true 时强制倒车友好参数");
    }
    {
      rclcpp::QoS backward_mode_qos(rclcpp::KeepLast(1));
      backward_mode_qos.transient_local();
      backward_mode_qos.reliable();
      backward_mode_pub_ = node->create_publisher<std_msgs::msg::Bool>(
        "/backward_mode", backward_mode_qos);
      resetBackwardModePublicationState(true);
      RCLCPP_INFO(
        logger_,
        "TEB 已发布 /backward_mode（几何倒车检测），"
        "angle_check_num=%d backward_check_duration=%.2f backward_check_num=%d",
        cfg_->trajectory.reverse_segment_angle_check_num,
        cfg_->trajectory.backward_check_duration,
        cfg_->trajectory.backward_check_num);
    }
    // 备份正常模式参数，供狭窄通道 RAII 恢复
    normal_weight_kinematics_forward_drive_ = cfg_->optim.weight_kinematics_forward_drive;
    normal_delete_detours_backwards_ = cfg_->hcp.delete_detours_backwards;
    normal_allow_init_with_backwards_motion_ = cfg_->trajectory.allow_init_with_backwards_motion;
    {
      rclcpp::QoS vehicle_scan_qos(rclcpp::KeepLast(5));
      vehicle_scan_qos.best_effort();
      vehicle_scan_cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg_->obstacles.vehicle_scan_topic,
        vehicle_scan_qos,
        std::bind(&TebLocalPlannerROS::vehicleScanCloudCallback, this, std::placeholders::_1));
    }
    {
      // volatile + reliable：与 RViz / 默认 echo 的 QoS 易匹配
      rclcpp::QoS grid_qos(rclcpp::KeepLast(1));
      grid_qos.reliable();
      vehicle_scan_grid_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        cfg_->obstacles.vehicle_scan_grid_topic, grid_qos);
      RCLCPP_INFO(logger_, "TEB 创建 /teb_vehicle_scan_grid 话题");
    }
    // setup callback for line center points (WALL / CURB / Reference / fusion modes via USE_CURB_OR_WALL)
    if (cfg_->optim.weight_wall_line_dist > 0.0 && use_curb_or_wall != nullptr) {
      const std::string normalized_wall_or_curb_edge_mode_string =
        normalizeEdgeModeEnv(use_curb_or_wall);
      if (normalized_wall_or_curb_edge_mode_string.find("wall") != std::string::npos) {
        wall_line_ptr_ = std::make_shared<line_path_compare::LinePathCompare>(node);
        RCLCPP_INFO(logger_, "TEB 使用2D激光雷达检测墙线贴边");
      }
      if (normalized_wall_or_curb_edge_mode_string == "curb" ||
          (normalized_wall_or_curb_edge_mode_string.find("curb") != std::string::npos &&
            normalized_wall_or_curb_edge_mode_string.find("reference") != std::string::npos)) {
        curb_line_subscriber_ = node->create_subscription<nav_msgs::msg::Path>(
          "camera1/extracted_line_path",
          rclcpp::QoS{5}.best_effort(),
          std::bind(&TebLocalPlannerROS::curb_line_callback, this, std::placeholders::_1));
        RCLCPP_INFO(logger_, "TEB 使用深度相机提取墙线贴边，话题：/camera1/extracted_line_path");
      }
      if (normalized_wall_or_curb_edge_mode_string.find("reference") != std::string::npos) {
        rclcpp::QoS reference_paths_subscription_qos(rclcpp::KeepLast(10));
        reference_paths_subscription_qos.transient_local();
        reference_paths_subscription_qos.reliable();
        paired_mission_and_reference_path_sub_ =
          node->create_subscription<capella_ros_msg::msg::LaneCenterPaths>(
          cfg_->wall_line.paired_mission_and_reference_path_topic,
          reference_paths_subscription_qos,
          std::bind(
            &TebLocalPlannerROS::pairedMissionAndReferencePathCallback, this,
            std::placeholders::_1));
        RCLCPP_INFO(
          logger_, "TEB 订阅成对 mission/reference 路径，话题：%s (transient_local)",
          cfg_->wall_line.paired_mission_and_reference_path_topic.c_str());
        removed_plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
          cfg_->wall_line.removed_plan_topic,
          rclcpp::QoS(rclcpp::KeepLast(5)),
          std::bind(&TebLocalPlannerROS::removedPlanCallback, this, std::placeholders::_1));
        RCLCPP_INFO(
          logger_, "TEB 订阅 removed_plan，话题：%s",
          cfg_->wall_line.removed_plan_topic.c_str());
      }
    }

    
    // initialize failure detector (reuse controller_frequency from control_duration_ above)
    failure_detector_.setBufferLength(
      std::round(cfg_->recovery.oscillation_filter_duration * controller_frequency));

    // set initialized flag
    initialized_ = true;

    // This should be called since to prevent different time sources exception
    time_last_infeasible_plan_ = clock_->now();
    time_last_oscillation_ = clock_->now();
    RCLCPP_DEBUG(logger_, "teb_local_planner plugin initialized.");

    transformed_path = node->create_publisher<nav_msgs::msg::Path>("teb_transformed_path", 1);
    global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("teb_global_plan", 1);
    wall_line_marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("teb_selected_wall_line", 1);
    edge_distance_publisher_  = node->create_publisher<std_msgs::msg::Float32>("edge_distance", 1);
    RCLCPP_INFO(logger_, "TEB 创建发布话题 /teb_selected_wall_line、/edge_distance、/teb_transformed_path、/teb_global_plan");

    // Create persistent client for static_layer parameter updates
    static_layer_client_ = node->create_client<rcl_interfaces::srv::SetParameters>("/local_costmap/local_costmap/set_parameters");
    RCLCPP_INFO(logger_, "static_layer client created.");

    // Create persistent clients for footprint parameter updates
    local_footprint_client_ = node->create_client<rcl_interfaces::srv::SetParameters>("/local_costmap/local_costmap/set_parameters");
    global_footprint_client_ = node->create_client<rcl_interfaces::srv::SetParameters>("/global_costmap/global_costmap/set_parameters");
    RCLCPP_INFO(logger_, "Footprint clients created for local and global costmaps.");

    // Get current footprint values from local and global costmaps for restoration via service calls
    // Use background threads to avoid blocking initialization
    std::thread([this, node]() {
      auto local_get_params_client = node->create_client<rcl_interfaces::srv::GetParameters>("/local_costmap/local_costmap/get_parameters");
      
      // Wait for service to be available (retry for up to 30 seconds)
      int retry_count = 0;
      while (!local_get_params_client->wait_for_service(std::chrono::seconds(2)) && retry_count < 15) {
        RCLCPP_DEBUG(logger_, "Waiting for local costmap get_parameters service... (%d/15)", retry_count + 1);
        retry_count++;
      }
      
      if (retry_count >= 15) {
        RCLCPP_WARN(logger_, "Local costmap get_parameters service not available after 30 seconds.");
        return;
      }
      
      auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
      get_request->names.push_back("footprint");
      auto future = local_get_params_client->async_send_request(get_request);
      
      if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        auto response = future.get();
        if (!response->values.empty()) {
          if (response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
            local_costmap_footprint_ = response->values[0].string_value;
            RCLCPP_INFO(logger_, "Local costmap footprint stored: %s", local_costmap_footprint_.c_str());
          } else if (response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            const auto& local_footprint_array = response->values[0].double_array_value;
            local_costmap_footprint_ = "[";
            for (size_t i = 0; i < local_footprint_array.size(); i += 2) {
              if (i + 1 < local_footprint_array.size()) {
                local_costmap_footprint_ += "[" + std::to_string(local_footprint_array[i]) + ", " + std::to_string(local_footprint_array[i+1]) + "]";
                if (i + 2 < local_footprint_array.size()) local_costmap_footprint_ += ", ";
              }
            }
            local_costmap_footprint_ += "]";
            RCLCPP_INFO(logger_, "Local costmap footprint stored: %s", local_costmap_footprint_.c_str());
          } else {
            RCLCPP_WARN(logger_, "Local costmap footprint has unknown type: %d", response->values[0].type);
          }
        } else {
          RCLCPP_WARN(logger_, "Local costmap footprint not found in response.");
        }
      } else {
        RCLCPP_WARN(logger_, "Local costmap get_parameters service call timeout.");
      }
    }).detach();

    std::thread([this, node]() {
      auto global_get_params_client = node->create_client<rcl_interfaces::srv::GetParameters>("/global_costmap/global_costmap/get_parameters");
      
      // Wait for service to be available (retry for up to 30 seconds)
      int retry_count = 0;
      while (!global_get_params_client->wait_for_service(std::chrono::seconds(2)) && retry_count < 15) {
        RCLCPP_DEBUG(logger_, "Waiting for global costmap get_parameters service... (%d/15)", retry_count + 1);
        retry_count++;
      }
      
      if (retry_count >= 15) {
        RCLCPP_WARN(logger_, "Global costmap get_parameters service not available after 30 seconds.");
        return;
      }
      
      auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
      get_request->names.push_back("footprint");
      auto future = global_get_params_client->async_send_request(get_request);
      
      if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        auto response = future.get();
        if (!response->values.empty()) {
          if (response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
            global_costmap_footprint_ = response->values[0].string_value;
            RCLCPP_INFO(logger_, "Global costmap footprint stored: %s", global_costmap_footprint_.c_str());
          } else if (response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            const auto& global_footprint_array = response->values[0].double_array_value;
            global_costmap_footprint_ = "[";
            for (size_t i = 0; i < global_footprint_array.size(); i += 2) {
              if (i + 1 < global_footprint_array.size()) {
                global_costmap_footprint_ += "[" + std::to_string(global_footprint_array[i]) + ", " + std::to_string(global_footprint_array[i+1]) + "]";
                if (i + 2 < global_footprint_array.size()) global_costmap_footprint_ += ", ";
              }
            }
            global_costmap_footprint_ += "]";
            RCLCPP_INFO(logger_, "Global costmap footprint stored: %s", global_costmap_footprint_.c_str());
          } else {
            RCLCPP_WARN(logger_, "Global costmap footprint has unknown type: %d", response->values[0].type);
          }
        } else {
          RCLCPP_WARN(logger_, "Global costmap footprint not found in response.");
        }
      } else {
        RCLCPP_WARN(logger_, "Global costmap get_parameters service call timeout.");
      }
    }).detach();
   }
   else
   {
     RCLCPP_INFO(logger_, "teb_local_planner has already been initialized, doing nothing.");
   }
}

void TebLocalPlannerROS::configure(
     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
     std::string name,
     std::shared_ptr<tf2_ros::Buffer> tf,
     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
   nh_ = parent;
 
   auto node = nh_.lock();
   logger_ = node->get_logger();
   clock_ = node->get_clock();
 
   costmap_ros_ = costmap_ros;
   tf_ = tf;
   name_ = name;
 
   initialize(node);
   visualization_ = std::make_shared<TebVisualization>(node, *cfg_);
   visualization_->on_configure();
   planner_->setVisualization(visualization_);
   
   return;
 }
 
 void TebLocalPlannerROS::setPlan(const nav_msgs::msg::Path & orig_global_plan)
 {
   // check if plugin is initialized
   if(!initialized_)
   {
     RCLCPP_ERROR(logger_, "teb_local_planner has not been initialized, please call initialize() before using this planner");
     return;
   }
 
   // store the global plan
   global_plan_.clear();
   global_plan_.reserve(orig_global_plan.poses.size());
   origin_plan_.clear();
   origin_plan_.reserve(orig_global_plan.poses.size());
   for(const auto &in_pose :orig_global_plan.poses) {
     geometry_msgs::msg::PoseStamped out_pose;
     out_pose.pose = in_pose.pose;
     out_pose.header = orig_global_plan.header;
     global_plan_.push_back(out_pose);
     origin_plan_.push_back(out_pose);
   }
   if (!global_plan_.empty()) {
    RCLCPP_INFO(logger_, "TEB 设置全局路径，路径点数：%zu", global_plan_.size());
   } else {
    RCLCPP_WARN(logger_, "TEB 设置全局路径失败，路径点数为0");
   }
 
 
   // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
   // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.  
             
   return;
 }
 
 
 geometry_msgs::msg::TwistStamped TebLocalPlannerROS::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
   const geometry_msgs::msg::Twist &velocity, nav2_core::GoalChecker *goal_checker)
 {
   // check if plugin initialized
   if(!initialized_)
   {
     throw nav2_core::PlannerException(
       std::string("teb_local_planner has not been initialized, please call initialize() before using this planner")
     );
   }
 
   geometry_msgs::msg::TwistStamped cmd_vel;
   
   cmd_vel.header.stamp = clock_->now();
   cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
   cmd_vel.twist.linear.x = 0;
   cmd_vel.twist.linear.y = 0;
   cmd_vel.twist.angular.z = 0;
 
   // Update for the current goal checker's state
   geometry_msgs::msg::Pose pose_tolerance;
   geometry_msgs::msg::Twist vel_tolerance;
   if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
     RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
   } else {
     cfg_->goal_tolerance.xy_goal_tolerance = pose_tolerance.position.x;
   }
   
   // Get robot pose
   robot_pose_ = PoseSE2(pose.pose);
   geometry_msgs::msg::PoseStamped robot_pose;
   robot_pose.header = pose.header;
   robot_pose_.toPoseMsg(robot_pose.pose);
   
   // Get robot velocity
   robot_vel_ = velocity;

   if (cfg_->robot.map_to_base_transform_max_age > 0.0) {
     const std::string & base_frame = costmap_ros_->getBaseFrameID();
     try {
       geometry_msgs::msg::TransformStamped tf_map_to_base =
         tf_->lookupTransform(
           cfg_->map_frame,
           base_frame,
           tf2::TimePointZero,
           tf2::durationFromSec(std::max(0.0, cfg_->robot.transform_tolerance)));
       const rclcpp::Time tf_stamp(tf_map_to_base.header.stamp, clock_->get_clock_type());
       const double data_age_sec = std::max(0.0, (clock_->now() - tf_stamp).seconds());
       if (data_age_sec > cfg_->robot.map_to_base_transform_max_age) {
         RCLCPP_WARN_THROTTLE(
           logger_,
           *clock_,
           1000,
           "%s → %s 的 TF 数据滞后, 滞后 %.3f s (阈值 %.3f s), 输出零速度",
           cfg_->map_frame.c_str(),
           base_frame.c_str(),
           data_age_sec,
           cfg_->robot.map_to_base_transform_max_age);
         return cmd_vel;
       }
     } catch (const tf2::TransformException & ex) {
       RCLCPP_WARN_THROTTLE(
         logger_,
         *clock_,
         1000,
         "无法查询 %s → %s 变换: %s, 输出零速度",
         cfg_->map_frame.c_str(),
         base_frame.c_str(),
         ex.what());
       return cmd_vel;
     }
   }
   
  // prune global plan to cut off parts of the past (spatially before the robot)
  // Coarse pass: remove obviously-passed points using a larger distance threshold and an
  // optionally unlimited search range. This prevents the fine pass from skipping too far
  // ahead when the robot detours around a large obstacle.
  if (cfg_->trajectory.rough_global_plan_prune_distance > 0.0)
  {
  pruneGlobalPlan(robot_pose, global_plan_,
                  cfg_->trajectory.rough_global_plan_prune_distance,
                  cfg_->trajectory.rough_global_plan_prune_max_accum_dist);
  }
  // Fine pass: precise pruning with the normal (smaller) distance and limited search range.
  pruneGlobalPlan(robot_pose, global_plan_, cfg_->trajectory.global_plan_prune_distance,
                cfg_->trajectory.global_plan_prune_max_accum_dist);

  geometry_msgs::msg::PoseStamped corner_pose_global, corner_pose_robot;
  bool corner_found = false;
  size_t corner_index = 0;  // Index of corner in global_plan_

  double globle_plane_length = 0.0;
  for (size_t i = 1; i < global_plan_.size(); ++i) {
    const auto & pi0 = global_plan_.at(i - 1).pose.position;
    const auto & pi1 = global_plan_.at(i).pose.position;
    const double sdx = pi1.x - pi0.x;
    const double sdy = pi1.y - pi0.y;
    globle_plane_length += std::sqrt(sdx * sdx + sdy * sdy);
    if (globle_plane_length > cfg_->trajectory.max_global_plan_lookahead_dist + 1.0) {
      break;
    }
  }
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 5000, "TEB 全局路径长度：%.2f", globle_plane_length);
  if (globle_plane_length < cfg_->trajectory.max_global_plan_lookahead_dist)
  {
    corner_pose_global = robot_pose;
    corner_pose_robot = robot_pose;
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "TEB 全局路径长度小于最大全局路径长度，不进行角点检测");
  }
  else
  {
    double theta = cfg_->trajectory.theta_threshold;
    const double min_segment_length = 1e-6;  // Minimum segment length to avoid division by zero
    const double position_tolerance = 0.01;  // Tolerance for position comparison
    
    // Find the nearest corner point that satisfies the condition (within max_global_plan_lookahead_dist along the path).
    double cum_dist = 0.0;
    for (size_t i = 2; i + 2 < global_plan_.size(); ++i)
    {
      {
        const auto & pi0 = global_plan_.at(i - 1).pose.position;
        const auto & pi1 = global_plan_.at(i).pose.position;
        const double sdx = pi1.x - pi0.x;
        const double sdy = pi1.y - pi0.y;
        cum_dist += std::sqrt(sdx * sdx + sdy * sdy);
      }
      const double max_corner_plan_len = cfg_->trajectory.max_global_plan_lookahead_dist + 1.0;
      if (max_corner_plan_len > 0.0 && cum_dist > max_corner_plan_len) {
        break;
      }

      double x0 = global_plan_.at(i).pose.position.x;
      double y0 = global_plan_.at(i).pose.position.y;
      double x1 = global_plan_.at(i - 2).pose.position.x;
      double y1 = global_plan_.at(i - 2).pose.position.y;
      double x2 = global_plan_.at(i + 2).pose.position.x;
      double y2 = global_plan_.at(i + 2).pose.position.y;

      // Calculate triangle side lengths for geometric cosine theorem
      // Three points form a triangle: x1 -> x0 -> x2
      // Calculate the angle at x0 using cosine theorem: cos(θ) = (a² + b² - c²) / (2ab)
      double dx_a = x0 - x1;  // Side a: from x1 to x0
      double dy_a = y0 - y1;
      double dx_b = x2 - x0;  // Side b: from x0 to x2
      double dy_b = y2 - y0;
      double dx_c = x2 - x1;  // Side c: from x1 to x2 (opposite to angle at x0)
      double dy_c = y2 - y1;
      
      double length_a = std::sqrt(dx_a * dx_a + dy_a * dy_a);  // |x1 - x0|
      double length_b = std::sqrt(dx_b * dx_b + dy_b * dy_b);  // |x0 - x2|
      double length_c = std::sqrt(dx_c * dx_c + dy_c * dy_c);  // |x1 - x2|
      
      // Check for zero-length segments to avoid division by zero
      if (length_a < min_segment_length || length_b < min_segment_length)
      {
        continue;
      }
      
      // Geometric cosine theorem: cos(θ) = (a² + b² - c²) / (2ab)
      // The result is naturally constrained to [0, 180] degrees
      double a_sq = length_a * length_a;
      double b_sq = length_b * length_b;
      double c_sq = length_c * length_c;
      double cos_theta = (a_sq + b_sq - c_sq) / (2.0 * length_a * length_b);
      
      // Clamp cos_theta to valid range [-1, 1] to avoid NaN from acos due to numerical errors
      cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
      
      // Calculate the turning angle using arccosine (0-180 degrees)
      // For sharp corners, we want small angles (acute)
      double temp_theta = std::acos(cos_theta) * 180.0 / M_PI;      
      // Check if this point is different from last_corner_pose_ (using distance tolerance)
      bool is_different_from_last = true;
      if (last_corner_pose_.header.frame_id == global_plan_.at(0).header.frame_id)
      {
        double dx = global_plan_.at(i).pose.position.x - last_corner_pose_.pose.position.x;
        double dy = global_plan_.at(i).pose.position.y - last_corner_pose_.pose.position.y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < position_tolerance * position_tolerance)
        {
          is_different_from_last = false;
        }
      }
      
      if (temp_theta < theta && is_different_from_last)
      {
        theta = temp_theta;
        corner_pose_global = global_plan_.at(i);
        corner_index = i;
        corner_found = true;
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "Edge following: 找到最近的角点: %zu, 角度: %.2f", i, temp_theta);
        break;  // Find the nearest corner that satisfies the condition
      }
    }

    {
    std::lock_guard<std::mutex> l(speed_limit_mutex_);
    if (has_speed_limit_) {
      // Ensure non-negative limit and cap by robot's nominal base maximum
      const double limit = std::max(0.0, speed_limit_linear_x_);
      if (std::isfinite(limit) && speed_limit_linear_x_ > 0) {
        if (limit < safe_linear_speed_limit_ && cfg_->robot.max_vel_x != limit) {
          cfg_->robot.max_vel_x = std::min(limit, safe_linear_speed_limit_);
          RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Performing change speed limit !, max linear speed is: %.2f", cfg_->robot.max_vel_x);
        } else if (limit >= safe_linear_speed_limit_ && cfg_->robot.max_vel_x != safe_linear_speed_limit_) {
          cfg_->robot.max_vel_x = safe_linear_speed_limit_;
          RCLCPP_WARN(logger_, "Receive speed limit is greater than safe linear speed limit, set limit speed to safe linear speed limit: %.2f !", safe_linear_speed_limit_);
        }
      } else if (!std::isfinite(limit) || limit <= 0) {
        RCLCPP_WARN(logger_, "Speed limit is not finite or less than 0, current speed limit is: %.2f !", cfg_->robot.max_vel_x);
      }
    }
    }
    
    if (corner_found)
    {
      corner_pose_global.header.frame_id = global_plan_.at(0).header.frame_id;
      // Use TimePointZero to get the latest available transform, avoiding extrapolation errors
      try 
      {
        geometry_msgs::msg::TransformStamped transform = tf_->lookupTransform(
          corner_pose_global.header.frame_id, 
          costmap_ros_->getBaseFrameID(), 
          tf2::TimePointZero,
          tf2::durationFromSec(0.5));
        corner_pose_global.header.stamp = transform.header.stamp;
      } 
      catch (const tf2::ExtrapolationException& ex) 
      {
        // If TimePointZero fails with extrapolation, try to get latest common time
        try {
          rclcpp::Time latest_time;
          if (tf_->canTransform(corner_pose_global.header.frame_id, costmap_ros_->getBaseFrameID(), tf2::TimePointZero)) {
            geometry_msgs::msg::TransformStamped transform = tf_->lookupTransform(
              corner_pose_global.header.frame_id, 
              costmap_ros_->getBaseFrameID(), 
              tf2::TimePointZero);
            corner_pose_global.header.stamp = transform.header.stamp;
          } else {
            RCLCPP_WARN(logger_, "TF 查询失败 (ExtrapolationException): %s, 尝试使用最新可用时间", ex.what());
            corner_found = false;
          }
        } catch (const tf2::TransformException& ex2) {
          RCLCPP_ERROR(logger_, "TF 查询失败: %s", ex2.what());
          corner_found = false;
        }
      }
      catch (const tf2::TransformException &ex) 
      {
        RCLCPP_ERROR(logger_, "TF 查询失败: %s", ex.what());
        corner_found = false;  // Mark corner as invalid if TF fails
      }
      if (corner_found &&
          !nav2_util::transformPoseInTargetFrame(
            corner_pose_global, corner_pose_robot, *tf_,
            costmap_ros_->getBaseFrameID(), 0.5)) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "Failed to transform corner pose to base frame");
        corner_found = false;
      }
    }
    else
    {
      // No corner found, initialize corner poses
      corner_pose_global = robot_pose;
      corner_pose_robot = robot_pose;
    }
  }

  // RCLCPP_INFO(logger_, "max_global_plan_lookahead_dist %.2f, max_vel_x: %.2f!", cfg_->trajectory.max_global_plan_lookahead_dist, cfg_->robot.max_vel_x);

  // Record corner pose if it's close enough
  if (corner_found)
  {
    double corner_pose_dist = corner_pose_robot.pose.position.x * corner_pose_robot.pose.position.x + 
          corner_pose_robot.pose.position.y * corner_pose_robot.pose.position.y;
    if (corner_pose_robot.pose.position.x > 0 && corner_pose_robot.pose.position.x < 0.3 && corner_pose_dist < 0.25)
    {
      last_corner_pose_ = corner_pose_global;
    }
  }

  // Prune path before last corner using distance tolerance
  if (last_corner_pose_.header.frame_id == global_plan_.at(0).header.frame_id)
  {
    const double prune_position_tolerance = 0.05;  // Tolerance for finding corner in path
    const double max_search_dist = cfg_->trajectory.prune_before_corner_distance;
    const double min_residual_dist = cfg_->trajectory.prune_corner_residual_distance;
    const bool limit_search = (max_search_dist > 0.0);
    double accum_dist = 0.0;
    for (auto prune_before_last_corner = global_plan_.begin();
        prune_before_last_corner != global_plan_.end();
        ++prune_before_last_corner)
    {
      if (prune_before_last_corner != global_plan_.begin())
      {
        const auto prev_it = prune_before_last_corner - 1;
        const double ddx = prune_before_last_corner->pose.position.x - prev_it->pose.position.x;
        const double ddy = prune_before_last_corner->pose.position.y - prev_it->pose.position.y;
        accum_dist += std::sqrt(ddx * ddx + ddy * ddy);
      }
      if (limit_search && accum_dist >= max_search_dist)
      {
        break;
      }

      const double dx = prune_before_last_corner->pose.position.x - last_corner_pose_.pose.position.x;
      const double dy = prune_before_last_corner->pose.position.y - last_corner_pose_.pose.position.y;
      const double dist_sq = dx * dx + dy * dy;

      if (dist_sq < prune_position_tolerance * prune_position_tolerance)
      {
        double residual_dist = 0.0;
        for (auto it = prune_before_last_corner; it + 1 != global_plan_.end(); ++it)
        {
          const double rdx = (it + 1)->pose.position.x - it->pose.position.x;
          const double rdy = (it + 1)->pose.position.y - it->pose.position.y;
          residual_dist += std::sqrt(rdx * rdx + rdy * rdy);
          if (residual_dist > min_residual_dist)
          {
            global_plan_.erase(global_plan_.begin(), prune_before_last_corner);
            break;
          }
        }
        break;
      }
    }
  }

  // Apply external speed limit (if any) before planning


  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::msg::TransformStamped tf_plan_to_global;
  if (!transformGlobalPlan(global_plan_, robot_pose, *costmap_, cfg_->map_frame, cfg_->trajectory.max_global_plan_lookahead_dist,
                          transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    throw nav2_core::PlannerException(
      std::string("Could not transform the global plan to the frame of the controller")
    );
  }
  // Check if corner has obstacle and prune path before corner if needed
  bool corner_has_obstacle = false;
  bool cut_path_before_corner = false;
  double cut_path_before_corner_distance = 0.0;
  if (corner_found && corner_pose_robot.pose.position.x < cfg_->trajectory.max_global_plan_lookahead_dist &&
      (corner_pose_robot.pose.position.x > 0.3 || 
      (corner_pose_robot.pose.position.x < 0 && 
      std::fabs(velocity.linear.x) > cfg_->trajectory.prune_before_corner_linear_x_threshold)))
  {
    geometry_msgs::msg::PoseStamped local_corner_check_pose2d;
    tf2::doTransform(corner_pose_global, local_corner_check_pose2d, tf_plan_to_global);
    
    // Get robot pose in local costmap frame (from transformed_plan, which starts with robot pose)
    geometry_msgs::msg::Pose2D robot_pose_local;
    if (!transformed_plan.empty())
    {
      robot_pose_local.x = transformed_plan.front().pose.position.x;
      robot_pose_local.y = transformed_plan.front().pose.position.y;
      robot_pose_local.theta = tf2::getYaw(transformed_plan.front().pose.orientation);
    }
    else
    {
      // Fallback: robot is at origin in local costmap frame
      robot_pose_local.x = 0.0;
      robot_pose_local.y = 0.0;
      robot_pose_local.theta = 0.0;
    }
    
    // Calculate direction from robot to corner
    double dx = local_corner_check_pose2d.pose.position.x - robot_pose_local.x;
    double dy = local_corner_check_pose2d.pose.position.y - robot_pose_local.y;
    double direction_to_corner = std::atan2(dy, dx);
    
    geometry_msgs::msg::Pose2D corner_check_pose2d;
    corner_check_pose2d.x = local_corner_check_pose2d.pose.position.x;
    corner_check_pose2d.y = local_corner_check_pose2d.pose.position.y;
    corner_check_pose2d.theta = direction_to_corner;  // Use direction from robot to corner
    
    
    try
    {
      unsigned char corner_check_cost = costmap_model_->scorePose(corner_check_pose2d, dwb_critics::getOrientedFootprint(corner_check_pose2d, footprint_spec_));
      if (corner_check_cost == nav2_costmap_2d::LETHAL_OBSTACLE || corner_check_cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        if (corner_check_cost == nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          cut_path_before_corner_distance = cfg_->trajectory.cut_path_before_corner_lethal_dist;
        } else {
          cut_path_before_corner_distance = cfg_->trajectory.cut_path_before_corner_inscribed_dist;
          RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "检测到角点被INSCRIBED_INFLATED_OBSTACLE障碍物占用");
        }
        corner_has_obstacle = true;
      }
    }
    catch(const dwb_core::IllegalTrajectoryException& e)
    {
      if (!std::strcmp(e.what(), "Trajectory Hits Obstacle."))
      {
        corner_has_obstacle = true;
        cut_path_before_corner_distance = cfg_->trajectory.cut_path_before_corner_lethal_dist;
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "检测到角点被LETHAL_OBSTACLE障碍物占用");
      }
    }
    
    // If corner has obstacle, delete all path points before the corner in global_plan_
    if (corner_has_obstacle)
    {
      double robot_corner_distance = pow(corner_pose_robot.pose.position.x, 2) + pow(corner_pose_robot.pose.position.y, 2);
      if (corner_index > 0 &&
          corner_index < global_plan_.size() &&
          robot_corner_distance < std::max(pow(cfg_->trajectory.max_global_plan_lookahead_dist / 2.0, 2), 
                                           pow(cut_path_before_corner_distance, 2)) &&
          std::fabs(velocity.linear.x) > cfg_->trajectory.prune_before_corner_linear_x_threshold && 
          corner_pose_robot.pose.position.x < 0)
      {
        // Erase all points before the corner (keep the corner point itself)
        cut_path_before_corner = true;
        global_plan_.erase(global_plan_.begin(), global_plan_.begin() + corner_index + 1);
        RCLCPP_INFO(logger_, "角点被障碍物占用，删除角点前的所有路径点，裁剪距离: %.2f m", robot_corner_distance);
      }
      goto jump_prune_transformed_plan;
    }
    
    // If no obstacle, find corner in transformed_plan and prune after corner if needed
    const double corner_position_tolerance = 0.05;  // Tolerance for finding corner in transformed plan
    for (auto check_it = transformed_plan.begin(); 
        check_it != transformed_plan.end(); 
        ++check_it)
    {
      double dx = check_it->pose.position.x - local_corner_check_pose2d.pose.position.x;
      double dy = check_it->pose.position.y - local_corner_check_pose2d.pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      
      if (dist_sq < corner_position_tolerance * corner_position_tolerance)
      {
        auto front_differance = check_it - transformed_plan.begin();
        auto end_differance = transformed_plan.end() - check_it;
        // Use a reasonable threshold (10 points) instead of cfg_->max_samples which may not exist
        if (front_differance > cfg_->trajectory.min_samples && end_differance > cfg_->trajectory.min_samples)
        {
          transformed_plan.erase(check_it, transformed_plan.end());
        }
        break;
      }
    }
  }

  jump_prune_transformed_plan:
  if (use_curb_or_wall != nullptr && transformed_plan.size() > 1) {
     runEdgeFollowingPathUpdate(transformed_plan, robot_pose);
  }
 
  // update via-points container
  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_->trajectory.global_plan_viapoint_sep);
  // check if we should enter any backup mode and apply settings
  configureBackupModes(transformed_plan, goal_idx);

  // --- 狭窄通道 / enable_backward：更新 latch，判断是否启用倒车友好 TEB 参数 ---
  updateNarrowPassageLatch(robot_pose);
  bool enable_backward_cmd = false;
  bool enable_backward_cmd_received = false;
  {
    std::lock_guard<std::mutex> lock(enable_backward_mutex_);
    enable_backward_cmd = enable_backward_cmd_;
    enable_backward_cmd_received = enable_backward_cmd_received_;
  }
  const bool narrow_mode = narrowPolygonsAvailable() && latched_narrow_passage_;
  const bool reverse_segment = hasReverseSegmentInPlan(transformed_plan);
  updateBackwardModePublication(reverse_segment);
  const bool enable_narrow_teb = (enable_backward_cmd_received && enable_backward_cmd) ?
    true : (narrow_mode || reverse_segment);
  updateNarrowPassageTebSettings(enable_narrow_teb);
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000,
    "TEB 倒车模式: enable_backward=%s narrow_latch=%s reverse_seg=%s enable=%s forward_drive=%.2f",
    (enable_backward_cmd_received && enable_backward_cmd) ? "是" : "否",
    narrow_mode ? "是" : "否",
    reverse_segment ? "是" : "否",
    enable_narrow_teb ? "是" : "否",
    enable_narrow_teb ? cfg_->optim.weight_kinematics_forward_drive_in_narrow_passages :
      normal_weight_kinematics_forward_drive_);
    
  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    throw nav2_core::PlannerException(
      std::string("Transformed plan is empty. Cannot determine a local plan.")
    );
  }

  // Check if in-place rotation should be performed before TEB planning
  const bool use_inplace_rotation =
    shouldRotateInPlace(velocity, transformed_plan, robot_pose);
  if (use_inplace_rotation)
  {
    geometry_msgs::msg::TwistStamped rotation_cmd_vel;
    // Calculate angle difference to target heading using forward lookahead distance
    const geometry_msgs::msg::PoseStamped *target_pose = nullptr;
    double accumulated_distance = 0.0;
    
    for (size_t i = 1; i < transformed_plan.size(); ++i)
    {
      double dx = transformed_plan[i].pose.position.x - transformed_plan[i-1].pose.position.x;
      double dy = transformed_plan[i].pose.position.y - transformed_plan[i-1].pose.position.y;
      accumulated_distance += std::sqrt(dx * dx + dy * dy);
      
      if (accumulated_distance >= cfg_->rotation.forward_lookahead_distance)
      {
        target_pose = &transformed_plan[i];
        break;
      }
    }
    
    // If no pose found within lookahead distance, use the last pose
    if (target_pose == nullptr)
    {
      target_pose = &transformed_plan.back();
    }
    
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    double target_yaw = tf2::getYaw(target_pose->pose.orientation);
    double angular_distance_to_heading = target_yaw - robot_yaw;
    // Normalize to [-PI, PI]
    while (angular_distance_to_heading > M_PI) angular_distance_to_heading -= 2.0 * M_PI;
    while (angular_distance_to_heading < -M_PI) angular_distance_to_heading += 2.0 * M_PI;
    
    if (computeRotateToHeadingCommand(angular_distance_to_heading, robot_pose, velocity, rotation_cmd_vel))
    {
      const double v_xy = std::hypot(velocity.linear.x, velocity.linear.y);
      // RCLCPP_INFO_THROTTLE(
      //   logger_, *clock_, 500,
      //   "原地转: 执行中（跳过 TEB），cmd_w=%.3f rad/s，|dtheta|=%.3f rad，w_odom=%.3f，v_xy=%.3f，wn=%.3f，dt=%.3f",
      //   rotation_cmd_vel.twist.angular.z,
      //   std::abs(angular_distance_to_heading),
      //   velocity.angular.z,
      //   v_xy,
      //   cfg_->rotation.rotate_to_heading_angular_vel,
      //   control_duration_);
         // Publish global_plan_ for visualization/debugging
      if (global_plan_pub_ && !global_plan_.empty())
      {
        nav_msgs::msg::Path global_plan_msg;
        global_plan_msg.header.stamp = clock_->now();
        global_plan_msg.header.frame_id = global_plan_.front().header.frame_id;
        global_plan_msg.poses = global_plan_;
        global_plan_pub_->publish(global_plan_msg);
      }
      return rotation_cmd_vel;
    }
    // If rotation fails due to collision, continue with TEB planning
    // RCLCPP_INFO_THROTTLE(
    //   logger_, *clock_, 500,
    //   "原地转: computeRotateToHeadingCommand 失败，改用 TEB（|dtheta|=%.3f rad、w_odom=%.3f）",
    //   std::abs(angular_distance_to_heading),
    //   velocity.angular.z);
  }

   // Get current goal point (last point of the transformed plan)
   const geometry_msgs::msg::PoseStamped &goal_point = transformed_plan.back();
   robot_goal_.x() = goal_point.pose.position.x;
   robot_goal_.y() = goal_point.pose.position.y;
   if (cfg_->trajectory.global_plan_overwrite_orientation)
   {
     robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, goal_point, goal_idx, tf_plan_to_global);
     // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
     //transformed_plan.back().pose.orientation = tf::createQuaternionMsgFromYaw(robot_goal_.theta());
     tf2::Quaternion q;
     q.setRPY(0, 0, robot_goal_.theta());
     transformed_plan.back().pose.orientation = tf2::toMsg(q);
   }  
   else
   {
     robot_goal_.theta() = tf2::getYaw(goal_point.pose.orientation);
   }
 
   // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
   if (transformed_plan.size()==1) // plan only contains the goal
   {
     transformed_plan.insert(transformed_plan.begin(), geometry_msgs::msg::PoseStamped()); // insert start (not yet initialized)
   }
   //tf::poseTFToMsg(robot_pose, transformed_plan.front().pose); // update start;
   transformed_plan.front().pose = robot_pose.pose;
     
   // clear currently existing obstacles
   obstacles_.clear();
   
   // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
   if (costmap_converter_) {
     updateObstacleContainerWithCostmapConverter();
   } else {
     updateObstacleContainerWithCostmap();
   }
   
   // also consider custom obstacles (must be called after other updates, since the container is not cleared)
   updateObstacleContainerWithCustomObstacles();
   
     
   // Do not allow config changes during the following optimization step
   std::lock_guard<std::mutex> cfg_lock(cfg_->configMutex());
     
   // Now perform the actual planning
 //   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_->goal_tolerance.free_goal_vel); // straight line init
   // RCLCPP_INFO(logger_, "Weight_via_point: %.2f !", cfg_->optim.weight_viapoint);
   bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_->goal_tolerance.free_goal_vel);
   if (!success)
   {
     planner_->clearPlanner(); // force reinitialization for next time
     
     ++no_infeasible_plans_; // increase number of infeasible solutions in a row
     time_last_infeasible_plan_ = clock_->now();
     last_cmd_ = cmd_vel.twist;
     
     throw nav2_core::PlannerException(
       std::string("teb_local_planner was not able to obtain a local plan for the current setting.")
     );
   }
 
   
 
   // Check for divergence
   if (planner_->hasDiverged())
   {
     cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
 
     // Reset everything to start again with the initialization of new trajectories.
     planner_->clearPlanner();
     RCLCPP_WARN_THROTTLE(logger_, *(clock_), 1, "TebLocalPlannerROS: the trajectory has diverged. Resetting planner...");
 
     ++no_infeasible_plans_; // increase number of infeasible solutions in a row
     time_last_infeasible_plan_ = clock_->now();
     last_cmd_ = cmd_vel.twist;
     throw nav2_core::PlannerException(
       std::string("TebLocalPlannerROS: velocity command invalid (hasDiverged). Resetting planner...")
     );
   }
          
   // Check feasibility (but within the first few states only)
   if(cfg_->robot.is_footprint_dynamic)
   {
     // Get current footprint from local costmap (edge mode may modify costmap via set_parameters, which may be different from TEB cache)
     const std::vector<geometry_msgs::msg::Point> costmap_footprint = costmap_ros_->getRobotFootprint();
     if (costmap_footprint != footprint_spec_) {
       footprint_spec_ = costmap_footprint;
       nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
     }
   }
 
   bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_->trajectory.feasibility_check_no_poses, cfg_->trajectory.feasibility_check_lookahead_distance);
   if (!feasible)
   {
     cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
    
     // now we reset everything to start again with the initialization of new trajectories.
     planner_->clearPlanner();
 
     ++no_infeasible_plans_; // increase number of infeasible solutions in a row
     time_last_infeasible_plan_ = clock_->now();
     last_cmd_ = cmd_vel.twist;
     
     throw nav2_core::PlannerException(
       std::string("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...")
     );
   }
 
   // Get the velocity command for this sampling interval
   if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->trajectory.control_look_ahead_poses))
   {
     planner_->clearPlanner();
     ++no_infeasible_plans_; // increase number of infeasible solutions in a row
     time_last_infeasible_plan_ = clock_->now();
     last_cmd_ = cmd_vel.twist;
     
     throw nav2_core::PlannerException(
       std::string("TebLocalPlannerROS: velocity command invalid. Resetting planner...")
     );
   }
   
   // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
   saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->robot.max_vel_x, cfg_->robot.max_vel_y,
                    cfg_->robot.max_vel_theta, cfg_->robot.max_vel_x_backwards);
 
   // convert rot-vel to steering angle if desired (carlike robot).
   // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
   // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
   if (cfg_->robot.cmd_angle_instead_rotvel)
   {
     cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, cfg_->robot.wheelbase, 0.95*cfg_->robot.min_turning_radius);
     if (!std::isfinite(cmd_vel.twist.angular.z))
     {
       cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
       last_cmd_ = cmd_vel.twist;
       planner_->clearPlanner();
 
       ++no_infeasible_plans_; // increase number of infeasible solutions in a row
       time_last_infeasible_plan_ = clock_->now();
       
       throw nav2_core::PlannerException(
         std::string("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...")
       );
     }
   }
   
   // a feasible solution should be found, reset counter
   no_infeasible_plans_ = 0;
   
   // store last command (for recovery analysis etc.)
   last_cmd_ = cmd_vel.twist;
   
   // Now visualize everything    
   planner_->visualize();
   visualization_->publishObstacles(obstacles_);
   visualization_->publishViaPoints(via_points_);
   visualization_->publishGlobalPlan(global_plan_);

   if (global_plan_pub_ && !global_plan_.empty())
   {
     nav_msgs::msg::Path global_plan_msg;
     global_plan_msg.header.stamp = clock_->now();
     global_plan_msg.header.frame_id = global_plan_.front().header.frame_id;
     global_plan_msg.poses = global_plan_;
     global_plan_pub_->publish(global_plan_msg);
   }
   
   return cmd_vel;
 }
 
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
  if (!obstacles) {
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

      // --- Wall line locking: compare current vs locked ---
      if (wall_line_locked_)
      {
        Eigen::Vector2d locked_dir = locked_wall_end_ - locked_wall_start_;
        double locked_len = locked_dir.norm();
        double locked_angle = (locked_len > 1e-6)
          ? std::atan2(locked_dir.y() / locked_len, locked_dir.x() / locked_len) : 0.0;

        Eigen::Vector2d locked_mid = (locked_wall_start_ + locked_wall_end_) * 0.5;
        Eigen::Vector2d cur_mid = (cur_start + cur_end) * 0.5;
        double dist_change = (cur_mid - locked_mid).norm();
        double angle_diff = cur_angle - locked_angle;
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

        double angle_threshold_rad = cfg_->wall_line.wall_line_lock_angle_threshold * M_PI / 180.0;
        if (dist_change > cfg_->wall_line.wall_line_lock_distance_threshold ||
            std::abs(angle_diff) > angle_threshold_rad)
        {
          wall_line_locked_ = false;
          wall_line_stable_count_ = 0;
          RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000,
            "Wall line unlocked: dist_change=%.3f, angle_change=%.3f", dist_change, std::abs(angle_diff));
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
            "Wall line locked after %d stable frames", wall_line_stable_count_);
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

  geometry_msgs::msg::TransformStamped tf_base_from_obstacles;
  bool have_tf_obstacles_to_base = false;

  if (use_wall_line_filter && tf_ && costmap_ros_) {
    try {
      tf_base_from_obstacles = tf_->lookupTransform(
        costmap_ros_->getBaseFrameID(), obstacles_frame, tf2::TimePointZero);
      have_tf_obstacles_to_base = true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG_THROTTLE(
        logger_, *(clock_), 2000,
        "protrusion exit: TF %s->%s failed: %s",
        obstacles_frame.c_str(), costmap_ros_->getBaseFrameID().c_str(), ex.what());
    }
  }

  EffWallFrame eff_wall;
  bool have_eff_wall = false;
  if (use_wall_line_filter) {
    eff_wall.start = eff_wall_start;
    eff_wall.dir = eff_wall_dir;
    eff_wall.normal = eff_wall_normal;
    eff_wall.length = eff_wall_length;
    eff_wall.robot_side_sign = wallSideSignFromCoord(
      wallSignedDist(robot_pose_.position(), eff_wall));
    have_eff_wall = true;
  }

  // --- Add obstacles; edge mode clips shallow wall-band geometry instead of skipping ---
  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter_msgs::msg::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::msg::Polygon* polygon = &obstacle->polygon;

    WallPolygonClipResult wall_result;
    if (have_eff_wall && polygon->points.size() > 2) {
      wall_result = processWallFilteredPolygon(
        *polygon,
        eff_wall,
        robot_pose_.position(),
        have_tf_obstacles_to_base,
        tf_base_from_obstacles,
        obstacles_frame,
        *cfg_);

      if (wall_result.exit_trigger) {
        rclcpp::Time now = clock_->now();
        protrusion_detection_timestamps_.push_back(now);

        rclcpp::Duration window_duration = rclcpp::Duration::from_seconds(
          cfg_->wall_line.obstacle_protrusion_confirm_window);
        while (!protrusion_detection_timestamps_.empty() &&
          (now - protrusion_detection_timestamps_.front()) > window_duration)
        {
          protrusion_detection_timestamps_.erase(protrusion_detection_timestamps_.begin());
        }
        RCLCPP_INFO(logger_, "[updateObstacleContainerWithCostmapConverter] protrusion_detection_timestamps_.size(): %ld", protrusion_detection_timestamps_.size());
        if (static_cast<int>(protrusion_detection_timestamps_.size()) >=
          cfg_->wall_line.obstacle_protrusion_min_confirm_frames)
        {
          bool is_duplicate = false;
          for (const auto& existing : protruding_obstacles_) {
            if ((existing.position - wall_result.exit_best_point).norm() < 0.5) {
              is_duplicate = true;
              break;
            }
          }

          if (!is_duplicate) {
            while (static_cast<int>(protruding_obstacles_.size()) >=
              cfg_->wall_line.obstacle_protrusion_max_stored)
            {
              protruding_obstacles_.erase(protruding_obstacles_.begin());
            }

            ProtrudingObstacle protruding_obst;
            protruding_obst.position = wall_result.exit_best_point;
            protruding_obst.influence_radius = wall_result.exit_best_pen;
            protruding_obst.detection_time = now;
            protruding_obstacles_.push_back(protruding_obst);
            RCLCPP_INFO(
              logger_,
              "Protruding obstacle confirmed at (%.2f, %.2f), penetration=%.2f.",
              wall_result.exit_best_point.x(), wall_result.exit_best_point.y(),
              wall_result.exit_best_pen);
          }
          protrusion_detection_timestamps_.clear();
        }
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

  if (cfg_->obstacles.enable_vehicle_scan_grid) {
    appendVehicleScanInflatedHullObstacles();
  }

  // --- Add the effective wall line as a clean LineObstacle ---
  if (use_wall_line_filter)
  {
    obstacles_.push_back(ObstaclePtr(new LineObstacle(
      eff_wall_start.x(), eff_wall_start.y(),
      eff_wall_end.x(), eff_wall_end.y())));
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
       RCLCPP_WARN(logger_, "updateObstacleContainerWithCustomObstacles: ExtrapolationException: %s, using identity transform", ex.what());
       obstacle_to_map_eig.setIdentity();
     }
     catch (const tf2::TransformException& ex)
     {
       RCLCPP_ERROR(logger_, "updateObstacleContainerWithCustomObstacles: TransformException: %s, using identity transform", ex.what());
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
         RCLCPP_INFO(logger_, "Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
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
 
bool TebLocalPlannerROS::isVehicleInEdgeFollowingExitCorridor(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const Eigen::Vector2d& wall_w0,
  const Eigen::Vector2d& wall_w1,
  const geometry_msgs::msg::Pose& vehicle_pose) const
{
  const auto& wl = cfg_->wall_line;
  Eigen::Vector2d R(robot_pose.pose.position.x, robot_pose.pose.position.y);
  Eigen::Vector2d V(vehicle_pose.position.x, vehicle_pose.position.y);

  Eigen::Vector2d seg = wall_w1 - wall_w0;
  const double L = seg.norm();
  if (L < 1e-6) {
    return false;
  }
  const Eigen::Vector2d u = seg / L;
  Eigen::Vector2d n(-u.y(), u.x());
  if (n.dot(R - wall_w0) < 0.0) {
    n = -n;
  }

  const double yaw = tf2::getYaw(robot_pose.pose.orientation);
  const Eigen::Vector2d fwd(std::cos(yaw), std::sin(yaw));

  const Eigen::Vector2d d = V - R;
  const double longitudinal = d.dot(fwd);
  if (longitudinal < -wl.vehicle_exit_corridor_rear_m || longitudinal > wl.vehicle_exit_corridor_front_m) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 车辆不在检查区域前后范围内");
    return false;
  }

  const double lateral = (V - wall_w0).dot(n);
  if (lateral < -wl.vehicle_exit_corridor_wall_inner_m || lateral > wl.vehicle_exit_corridor_wall_robot_side_m) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 车辆不在检查区域内外范围内");
    return false;
  }
  return true;
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
           RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000,
             "Edge following: 车辆在贴边检查区域范围内，退出贴边模式");
         } else {
           RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000,
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
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "Edge following: 机器人附近有车辆，退出贴边模式");
    switchParameterMode(false);
    return;
   }

   // Check for protruding obstacles that prohibit entering edge-following mode
   bool has_protruding_obstacle_nearby = false;
   Eigen::Vector2d robot_pos(robot_pose.pose.position.x, robot_pose.pose.position.y);
   
   // Clean up expired obstacles and check for nearby ones
   rclcpp::Time current_time = clock_->now();
   auto it = protruding_obstacles_.begin();
   while (it != protruding_obstacles_.end())
   {
     double dist = (robot_pos - it->position).norm();
     double time_elapsed = (current_time - it->detection_time).seconds();
     
     // Check if obstacle is expired by timeout
     if (time_elapsed > cfg_->wall_line.obstacle_protrusion_timeout)
     {
       it = protruding_obstacles_.erase(it);
       continue;
     }
     
     // Check if robot is within re-enter distance + influence radius
     if (dist < cfg_->wall_line.obstacle_protrusion_reenter_distance)
     {
       has_protruding_obstacle_nearby = true;
       ++it;
     } else {
       // Robot is far away, remove this obstacle record
       it = protruding_obstacles_.erase(it);
     }
   }
   
   if (has_protruding_obstacle_nearby)
   {
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 机器人附近有障碍物，退出贴边模式");
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
        RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "Edge following: 机器人方向与路径方向夹角太大: %.2f，退出贴边模式", 
                            angle_diff * 180.0 / M_PI);
        switchParameterMode(false);
        return;
      }
    } else {
      if(wall_line_points_.size() > 0) wall_line_points_.clear();
      RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "Edge following: 路径长度太短，退出贴边模式");
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
      RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 墙线点数量: %ld !", wall_path.poses.size());
      
      // 提取墙线端点
      const auto& wall_start = wall_path.poses.front().pose.position;
      const auto& wall_end = wall_path.poses.back().pose.position;
      
      // 计算墙线方向向量
      const double dx_wall = wall_end.x - wall_start.x;
      const double dy_wall = wall_end.y - wall_start.y;
      const double wall_length = std::hypot(dx_wall, dy_wall);
      if(wall_length < min_wall_line_length_) continue; // 无效墙线
      RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 墙线长度: %.2f !", wall_length);
      
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
                           cos_theta * 180.0 / M_PI, avg_distance);
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
      }
    }

    // 5. 如果找到最佳匹配的墙线，更新配置
    if (found_valid_wall) 
    {
      if(wall_line_points_.size() > 0) wall_line_points_.clear();
      wall_line_points_.emplace_back(Eigen::Vector2d(best_wall_start.x, best_wall_start.y));
      wall_line_points_.emplace_back(Eigen::Vector2d(best_wall_end.x, best_wall_end.y));
      cfg_->optim.weight_wall_line_dist =
        computeWallLineDistWeightFromRobotDistance(best_robot_to_edge_distance);
      wall_line_update_time_ = clock_->now();
      std_msgs::msg::Float32 distance;
      distance.data = static_cast<float>(best_robot_to_edge_distance);
      edge_distance_publisher_->publish(distance);
      
      // Switch to edge-following mode parameters
      switchParameterMode(true);
      
      RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 选择最佳墙线距离: %.2f", min_distance);
      RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 最小障碍物距离: %.2f, 最优时间权重: %.2f, 墙线距离权重: %.2f", 
                           cfg_->obstacles.min_obstacle_dist, cfg_->optim.weight_optimaltime, 
                           cfg_->optim.weight_wall_line_dist);
      
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
      RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 没有找到合适的墙线，退出贴边模式，清空贴边配置");
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
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 路沿路径长度太短，退出贴边模式");
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
           RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000,
             "[updateCurbLineVec] Edge following: 车辆在贴边检查区域范围内，退出贴边模式");
         } else {
           RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000,
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
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 机器人附近有车辆，退出贴边模式");
    switchParameterMode(false);
    return;
   }

   // Check for protruding obstacles that prohibit entering edge-following mode
   bool has_protruding_obstacle_nearby = false;
   Eigen::Vector2d robot_pos(robot_pose.pose.position.x, robot_pose.pose.position.y);
   
   // Clean up expired obstacles and check for nearby ones
   rclcpp::Time current_time = clock_->now();
   auto it = protruding_obstacles_.begin();
   while (it != protruding_obstacles_.end())
   {
     double dist = (robot_pos - it->position).norm();
     double time_elapsed = (current_time - it->detection_time).seconds();
     
     // Check if obstacle is expired by timeout
     if (time_elapsed > cfg_->wall_line.obstacle_protrusion_timeout)
     {
       it = protruding_obstacles_.erase(it);
       continue;
     }
     
     // Check if robot is within re-enter distance + influence radius
     double reenter_threshold = cfg_->wall_line.obstacle_protrusion_reenter_distance + it->influence_radius;
     if (dist < reenter_threshold)
     {
       has_protruding_obstacle_nearby = true;
       ++it;
     }
     else
     {
       // Robot is far away, remove this obstacle record
       it = protruding_obstacles_.erase(it);
     }
   }
   
   if (has_protruding_obstacle_nearby)
   {
     RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 机器人附近有障碍物，退出贴边模式");
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
        RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 机器人方向与路径方向夹角太大: %.2f，退出贴边模式", 
                            angle_diff * 180.0 / M_PI);
        switchParameterMode(false);
        return;
      }
    } else {
      RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 墙线长度太短，退出贴边模式，清空贴边配置");
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
     if(wall_line_points_.size() > 0) wall_line_points_.clear();
     wall_line_points_.emplace_back(Eigen::Vector2d(best_wall_start.x, best_wall_start.y));
     wall_line_points_.emplace_back(Eigen::Vector2d(best_wall_end.x, best_wall_end.y));
     
     cfg_->optim.weight_wall_line_dist =
       computeWallLineDistWeightFromRobotDistance(best_robot_to_edge_distance);
     wall_line_update_time_ = clock_->now();
     std_msgs::msg::Float32 distance;
     distance.data = static_cast<float>(best_robot_to_edge_distance);
     edge_distance_publisher_->publish(distance);
     
     // Switch to edge-following mode parameters
     switchParameterMode(true);
     
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 选择最佳墙线距离: %.2f", min_distance);
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[updateCurbLineVec] Edge following: 最小障碍物距离: %.2f, 最优时间权重: %.2f, 墙线距离权重: %.2f", 
                          cfg_->obstacles.min_obstacle_dist, 
                          cfg_->optim.weight_optimaltime, cfg_->optim.weight_wall_line_dist);
     
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
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Edge following: 没有找到合适的墙线，退出贴边模式，清空贴边配置");
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
       
       
 bool TebLocalPlannerROS::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, std::vector<geometry_msgs::msg::PoseStamped>& global_plan, double dist_behind_robot, double max_prune_dist)
 {
   if (global_plan.empty())
     return true;
   
   try
   {
     // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
     // Use TimePointZero to get the latest available transform to avoid extrapolation errors
     // when global_pose.header.stamp is in the future (especially important for low-frequency transforms like map->odom at 10Hz)
     geometry_msgs::msg::PoseStamped robot;
     try {
       // Get transform using TimePointZero to avoid extrapolation
       geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(
                   global_plan.front().header.frame_id,
                   global_pose.header.frame_id,
                   tf2::TimePointZero,
                   tf2::durationFromSec(0.5));
       
       // Apply transform manually to avoid using global_pose.header.stamp
       tf2::doTransform(global_pose, robot, global_to_plan_transform);
     } catch (const tf2::ExtrapolationException& ex) {
       RCLCPP_WARN(logger_, "pruneGlobalPlan: ExtrapolationException in transform: %s, skipping pruning this cycle", ex.what());
       // If TimePointZero fails with extrapolation, skip pruning this cycle rather than using future time
       // This prevents further extrapolation errors
       return true;
     } catch (const tf2::TransformException& ex) {
       RCLCPP_WARN(logger_, "pruneGlobalPlan: TransformException: %s, skipping pruning this cycle", ex.what());
       return true;
     }
     
     double dist_thresh_sq = dist_behind_robot*dist_behind_robot;    
     // iterate plan until a pose close the robot is found
     std::vector<geometry_msgs::msg::PoseStamped>::iterator it = global_plan.begin();
     std::vector<geometry_msgs::msg::PoseStamped>::iterator erase_end = it;
     double accum_dist = 0;
     double min_dist_threshold = std::numeric_limits<double>::max();
     const bool limit_prune_search = (max_prune_dist > 0.0);
     while (it != global_plan.end() && (!limit_prune_search || accum_dist < max_prune_dist))
     {
       double dx = robot.pose.position.x - it->pose.position.x;
       double dy = robot.pose.position.y - it->pose.position.y;
       double dist_sq = dx * dx + dy * dy;
       if (it != global_plan.begin())
       {
         double ddx = it->pose.position.x - (it-1)->pose.position.x;
         double ddy = it->pose.position.y - (it-1)->pose.position.y;
         accum_dist += std::sqrt(ddx*ddx + ddy*ddy);
       }
       if (dist_sq < min_dist_threshold)
       {
         min_dist_threshold = dist_sq;
         erase_end = it;
         if (dist_sq < dist_thresh_sq)
         {
           break;
         }
       }
       if (dist_sq < min_dist_threshold)
       {
         // If close enough, additionally require plan pose orientation to align with robot orientation
         // If yaw misalignment is too large, keep searching forward until we find a close pose with aligned yaw
         double robot_yaw = tf2::getYaw(robot.pose.orientation);
         double pose_yaw = tf2::getYaw(it->pose.orientation);
         double yaw_diff = pose_yaw - robot_yaw;
         while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
         while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

         if (std::fabs(yaw_diff) <= prune_angle_threshold_)
         {
           erase_end = it;
           break;
         }
         // else: do not break, continue searching forward
       }
       ++it;
       // ++count_it;
     }
 
     if (erase_end == global_plan.end())
       return false;
     
     if (erase_end != global_plan.begin())
       global_plan.erase(global_plan.begin(), erase_end);
   }
   catch (const tf2::TransformException& ex)
   {
     RCLCPP_DEBUG(logger_, "Cannot prune path since no transform is available: %s\n", ex.what());
     return false;
   }
   return true;
 }

bool TebLocalPlannerROS::globalPlanPoseFootprintFreeInControllerFrame(
  const geometry_msgs::msg::PoseStamped& pose_plan_frame,
  const geometry_msgs::msg::TransformStamped& plan_to_global_transform,
  geometry_msgs::msg::PoseStamped* out_pose_global_frame) const
{
  geometry_msgs::msg::PoseStamped pose_global;
  tf2::doTransform(pose_plan_frame, pose_global, plan_to_global_transform);
  if (out_pose_global_frame) {
    *out_pose_global_frame = pose_global;
  }
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = pose_global.pose.position.x;
  pose2d.y = pose_global.pose.position.y;
  pose2d.theta = tf2::getYaw(pose_global.pose.orientation);

  const dwb_critics::Footprint oriented_footprint =
    dwb_critics::getOrientedFootprint(pose2d, footprint_spec_);
  return orientedFootprintNavigableOnPartialCostmap(pose2d, oriented_footprint);
}

bool TebLocalPlannerROS::orientedFootprintNavigableOnPartialCostmap(
  const geometry_msgs::msg::Pose2D & pose2d_controller_frame,
  const std::vector<geometry_msgs::msg::Point> & oriented_footprint) const
{
  if (costmap_ == nullptr || oriented_footprint.size() < 2) {
    return true;
  }

  const unsigned int sx_cells = costmap_->getSizeInCellsX();
  const unsigned int sy_cells = costmap_->getSizeInCellsY();
  if (sx_cells < 1 || sy_cells < 1) {
    return true;
  }

  const double res = costmap_->getResolution();
  const double map_x_min = costmap_->getOriginX();
  const double map_y_min = costmap_->getOriginY();
  const double map_x_max_ex = map_x_min + static_cast<double>(sx_cells) * res;
  const double map_y_max_ex = map_y_min + static_cast<double>(sy_cells) * res;

  if (footprintAxisAlignedBboxSeparateFromCostmapRect(
      oriented_footprint, map_x_min, map_y_min, map_x_max_ex, map_y_max_ex))
  {
    return true;
  }

  constexpr double kClipShrink = 1e-8;
  double clip_x_min = map_x_min + kClipShrink;
  double clip_y_min = map_y_min + kClipShrink;
  double clip_x_max = map_x_max_ex - kClipShrink;
  double clip_y_max = map_y_max_ex - kClipShrink;
  if (!(clip_x_max > clip_x_min && clip_y_max > clip_y_min)) {
    clip_x_min = map_x_min;
    clip_y_min = map_y_min;
    clip_x_max = map_x_max_ex - std::numeric_limits<double>::epsilon() *
      std::max(std::fabs(map_x_max_ex), 1.0);
    clip_y_max = map_y_max_ex - std::numeric_limits<double>::epsilon() *
      std::max(std::fabs(map_y_max_ex), 1.0);
    if (!(clip_x_max > clip_x_min && clip_y_max > clip_y_min)) {
      return true;
    }
  }

  unsigned int verts_in_grid = 0;
  for (const auto & p : oriented_footprint) {
    unsigned int mx_dummy = 0;
    unsigned int my_dummy = 0;
    if (costmap_->worldToMap(p.x, p.y, mx_dummy, my_dummy)) {
      ++verts_in_grid;
    }
  }

  const auto clipped_outline_ok = [&]()
    {
      return !clippedFootprintOutlineTouchesBlockingCost(
        *costmap_, oriented_footprint, clip_x_min, clip_y_min, clip_x_max,
        clip_y_max);
    };

  if (verts_in_grid == oriented_footprint.size()) {
    try {
      (void)costmap_model_->scorePose(pose2d_controller_frame, oriented_footprint);
      return true;
    } catch (const dwb_core::IllegalTrajectoryException & exc) {
      const char * em = exc.what();
      if (!std::strcmp(em, "Trajectory Hits Obstacle.") ||
        !std::strcmp(em, "Trajectory Hits Unknown Region."))
      {
        return false;
      }
      if (!std::strcmp(em, "Footprint Goes Off Grid.") ||
        !std::strcmp(em, "Trajectory Goes Off Grid."))
      {
        return clipped_outline_ok();
      }
      throw;
    }
  }

  return clipped_outline_ok();
}

bool TebLocalPlannerROS::adjustOccupiedGlobalPlanGoalInPlace(
  const geometry_msgs::msg::PoseStamped& original_last_pose_plan_frame,
  const geometry_msgs::msg::TransformStamped& plan_to_global_transform,
  geometry_msgs::msg::PoseStamped& last_pose_plan_frame_inout) const
{
  last_pose_plan_frame_inout = original_last_pose_plan_frame;
  if (globalPlanPoseFootprintFreeInControllerFrame(
      original_last_pose_plan_frame, plan_to_global_transform, nullptr))
  {
    return true;
  }
  const double tol = cfg_->trajectory.transform_global_plan_goal_occupied_tolerance;
  const double res = cfg_->trajectory.transform_global_plan_goal_search_resolution;
  if (tol <= 1e-9 || res <= 1e-9) {
    RCLCPP_WARN(
      logger_,
      "transformGlobalPlan: last pose occupied but goal search disabled "
      "(transform_global_plan_goal_occupied_tolerance / goal_search_resolution <= 0)");
    return false;
  }
  {
    geometry_msgs::msg::PoseStamped occupied_global;
    tf2::doTransform(original_last_pose_plan_frame, occupied_global, plan_to_global_transform);
    RCLCPP_INFO(
      logger_,
      "transformGlobalPlan: last path point footprint collides — starting plan-frame grid search "
      "(frame=%s, tolerance=%.3f m, resolution=%.3f m)",
      original_last_pose_plan_frame.header.frame_id.c_str(), tol, res);
    RCLCPP_INFO(
      logger_,
      "transformGlobalPlan: last path point before adjust — plan[%s]: x=%.3f y=%.3f yaw=%.3f | "
      "controller[%s]: x=%.3f y=%.3f yaw=%.3f",
      original_last_pose_plan_frame.header.frame_id.c_str(),
      original_last_pose_plan_frame.pose.position.x,
      original_last_pose_plan_frame.pose.position.y,
      tf2::getYaw(original_last_pose_plan_frame.pose.orientation),
      occupied_global.header.frame_id.c_str(),
      occupied_global.pose.position.x,
      occupied_global.pose.position.y,
      tf2::getYaw(occupied_global.pose.orientation));
  }
  double min_dist_sq = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::PoseStamped best = original_last_pose_plan_frame;
  bool found = false;
  for (double goal_search_x = -tol; goal_search_x < tol + 0.1; goal_search_x += res) {
    for (double goal_search_y = -tol; goal_search_y < tol + 0.1; goal_search_y += res) {
      geometry_msgs::msg::PoseStamped search_goal = original_last_pose_plan_frame;
      search_goal.pose.position.x += goal_search_x;
      search_goal.pose.position.y += goal_search_y;
      if (!globalPlanPoseFootprintFreeInControllerFrame(
          search_goal, plan_to_global_transform, nullptr))
      {
        continue;
      }
      const double dist_sq = goal_search_x * goal_search_x + goal_search_y * goal_search_y;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        best = search_goal;
        found = true;
      }
    }
  }
  if (!found) {
    RCLCPP_WARN(
      logger_,
      "transformGlobalPlan: last global plan pose occupied, no free pose within tolerance %.3f m "
      "(plan-frame grid search)",
      tol);
    return false;
  }
  {
    const double dx = best.pose.position.x - original_last_pose_plan_frame.pose.position.x;
    const double dy = best.pose.position.y - original_last_pose_plan_frame.pose.position.y;
    geometry_msgs::msg::PoseStamped best_global;
    tf2::doTransform(best, best_global, plan_to_global_transform);
    RCLCPP_INFO(
      logger_,
      "transformGlobalPlan: last path point after adjust — plan[%s]: x=%.3f y=%.3f yaw=%.3f | "
      "controller[%s]: x=%.3f y=%.3f yaw=%.3f",
      best.header.frame_id.c_str(),
      best.pose.position.x,
      best.pose.position.y,
      tf2::getYaw(best.pose.orientation),
      best_global.header.frame_id.c_str(),
      best_global.pose.position.x,
      best_global.pose.position.y,
      tf2::getYaw(best_global.pose.orientation));
    RCLCPP_INFO(
      logger_,
      "transformGlobalPlan: last path point adjust delta (plan frame): dx=%.3f m dy=%.3f m | "
      "horizontal offset=%.3f m",
      dx, dy, std::hypot(dx, dy));
  }
  last_pose_plan_frame_inout = best;
  return true;
}

namespace {

int computeLastIdxAfterExtension(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const int from_idx,
  const double extend_arc_m)
{
  const int goal_idx = static_cast<int>(plan.size()) - 1;
  if (from_idx >= goal_idx || extend_arc_m <= 1e-9) {
    return goal_idx;
  }
  double accum = 0.0;
  int new_last = from_idx;
  for (int k = from_idx; k < goal_idx; ++k) {
    accum += teb_local_planner::distance_points2d(
      plan[static_cast<size_t>(k)].pose.position,
      plan[static_cast<size_t>(k + 1)].pose.position);
    new_last = k + 1;
    if (accum >= extend_arc_m - 1e-9) {
      break;
    }
  }
  return new_last;
}

}  // namespace

bool TebLocalPlannerROS::transformGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
                   const geometry_msgs::msg::PoseStamped& global_pose, const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                   std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::msg::TransformStamped* tf_plan_to_global) const
 {
   // this method is a slightly modified version of base_local_planner/goal_functions.h
 
   const geometry_msgs::msg::PoseStamped& plan_pose = global_plan[0];
 
   transformed_plan.clear();
 
   try 
   {
     if (global_plan.empty())
     {
       RCLCPP_ERROR(logger_, "Received plan with zero length");
       *current_goal_idx = 0;
       return false;
     }
     
     geometry_msgs::msg::TransformStamped plan_to_global_transform;
     try {
       plan_to_global_transform = tf_->lookupTransform(
                   global_frame,
                   plan_pose.header.frame_id,
                   tf2::TimePointZero,
                   tf2::durationFromSec(0.5));
     } catch (const tf2::ExtrapolationException& ex) {
       RCLCPP_WARN(logger_, "transformGlobalPlan: ExtrapolationException in lookupTransform: %s", ex.what());
       RCLCPP_WARN(logger_, "transformGlobalPlan: Requested from %s to %s with TimePointZero, retrying with latest available time", 
                    plan_pose.header.frame_id.c_str(), global_frame.c_str());
       // Retry with TimePointZero but without specifying source time (uses latest available)
       try {
         plan_to_global_transform = tf_->lookupTransform(
                     global_frame,
                     plan_pose.header.frame_id,
                     tf2::TimePointZero);
       } catch (const tf2::TransformException& ex2) {
         RCLCPP_ERROR(logger_, "transformGlobalPlan: Failed to get transform after retry: %s", ex2.what());
         throw nav2_core::PlannerException(
           std::string("Could not transform the global plan to the frame of the controller: ") + ex2.what()
         );
       }
     } catch (const tf2::TransformException& ex) {
       RCLCPP_ERROR(logger_, "transformGlobalPlan: TransformException: %s", ex.what());
       throw nav2_core::PlannerException(
         std::string("Could not transform the global plan to the frame of the controller: ") + ex.what()
       );
     }
     
     geometry_msgs::msg::PoseStamped robot_pose;
     try {
       // Get transform using TimePointZero to avoid extrapolation
       geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(
                   plan_pose.header.frame_id,
                   global_pose.header.frame_id,
                   tf2::TimePointZero,
                   tf2::durationFromSec(0.5));
       
       // Apply transform manually to avoid using global_pose.header.stamp
       tf2::doTransform(global_pose, robot_pose, global_to_plan_transform);
     } catch (const tf2::ExtrapolationException& ex) {
       RCLCPP_WARN(logger_, "transformGlobalPlan: ExtrapolationException when transforming robot pose: %s, retrying", ex.what());
       // Retry with TimePointZero but without specifying source time
       try {
         geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(
                     plan_pose.header.frame_id,
                     global_pose.header.frame_id,
                     tf2::TimePointZero);
         tf2::doTransform(global_pose, robot_pose, global_to_plan_transform);
       } catch (const tf2::TransformException& ex2) {
         RCLCPP_ERROR(logger_, "transformGlobalPlan: Failed to transform robot pose after retry: %s", ex2.what());
         throw nav2_core::PlannerException(
           std::string("Could not transform the global plan to the frame of the controller: ") + ex2.what()
         );
       }
     } catch (const tf2::TransformException& ex) {
       RCLCPP_ERROR(logger_, "transformGlobalPlan: TransformException in robot pose transform: %s", ex.what());
       throw nav2_core::PlannerException(
         std::string("Could not transform the global plan to the frame of the controller: ") + ex.what()
       );
     }
 
     //we'll discard points on the plan that are outside the local costmap
     double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                      costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
     dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                            // located on the border of the local costmap
     
 
     int i = 0;
     double sq_dist_threshold = dist_threshold * dist_threshold;
     double sq_dist = 1e10;
     
    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    double accum_path_from_plan_start = 0.0;
    const double max_accum_closest_search =
      cfg_->trajectory.transform_global_plan_closest_search_max_accum_dist;
    for (int j = 0; j < static_cast<int>(global_plan.size()); ++j) {
      if (j > 0) {
        accum_path_from_plan_start += distance_points2d(
          global_plan[static_cast<size_t>(j - 1)].pose.position,
          global_plan[static_cast<size_t>(j)].pose.position);
      }
      if (max_accum_closest_search > 1e-9 &&
        accum_path_from_plan_start > max_accum_closest_search + 1e-9) {
        break;
      }
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist_threshold) {
        break;  // force stop if we have reached the costmap border
      }

      if (robot_reached && new_sq_dist > sq_dist) {
        break;
      }

      if (new_sq_dist < sq_dist) {  // find closest distance
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.25) {  // 2.5 cm to the robot; take the immediate local minima; if it's not the global
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
        }
      }
    }
 
    const int n_plan = static_cast<int>(global_plan.size());
    const int global_goal_idx = n_plan - 1;
    int last_idx = global_goal_idx;
    geometry_msgs::msg::PoseStamped effective_last_plan_pose = global_plan.back();
    auto planPoseAt = [&](int idx) -> const geometry_msgs::msg::PoseStamped & {
      if (idx == global_goal_idx) {
        return effective_last_plan_pose;
      }
      return global_plan[static_cast<size_t>(idx)];
    };

     geometry_msgs::msg::PoseStamped newer_pose;
     
     const int segment_start_idx = i;
     double arc_from_segment_start = 0.0;

     //now we'll transform until points are outside of our distance threshold
     cfg_->optim.weight_viapoint = weight_via_point_;
     while (i < n_plan && i <= last_idx &&
       (max_plan_length <= 0 || arc_from_segment_start <= max_plan_length))
     {
       tf2::doTransform(planPoseAt(i), newer_pose, plan_to_global_transform);
       transformed_plan.push_back(newer_pose);

       if (i > segment_start_idx && max_plan_length > 0) {
         arc_from_segment_start += distance_points2d(
           planPoseAt(i - 1).pose.position, planPoseAt(i).pose.position);
       }

       const bool pose_free = globalPlanPoseFootprintFreeInControllerFrame(
         planPoseAt(i), plan_to_global_transform, nullptr);
       if (!pose_free && n_plan > 0 && max_plan_length > 0) {
         const double remaining_budget = max_plan_length - arc_from_segment_start;
         const double near_end_thresh =
           cfg_->trajectory.transformed_plan_collision_pose_to_end_distance;
         if (near_end_thresh > 1e-9 && remaining_budget < near_end_thresh) {
           const double target_max_plan_length = arc_from_segment_start + near_end_thresh;
           max_plan_length = std::min(target_max_plan_length, costmap.getSizeInMetersX());
           const double extend_arc_along_plan = max_plan_length - arc_from_segment_start;
           const int new_last_idx = computeLastIdxAfterExtension(
             global_plan, i, extend_arc_along_plan);
           last_idx = std::max(last_idx, new_last_idx);
           last_idx = std::min(last_idx, global_goal_idx);
           cfg_->optim.weight_viapoint = 1.0;

           if (last_idx == global_goal_idx && i == global_goal_idx) {
             if (!transformed_plan.empty()) {
               transformed_plan.pop_back();
             }
             RCLCPP_WARN(
               logger_,
               "transformGlobalPlan: last path point footprint not free in local costmap; "
               "running plan-frame goal search (no PlannerException).");
             const bool adjusted_ok = adjustOccupiedGlobalPlanGoalInPlace(
               global_plan.back(), plan_to_global_transform, effective_last_plan_pose);
             if (!adjusted_ok) {
               RCLCPP_ERROR(
                 logger_,
                 "transformGlobalPlan: last goal search did not find a free pose; keeping last plan pose.");
             }
             tf2::doTransform(planPoseAt(global_goal_idx), newer_pose, plan_to_global_transform);
             transformed_plan.push_back(newer_pose);
             if (!globalPlanPoseFootprintFreeInControllerFrame(
                 planPoseAt(global_goal_idx), plan_to_global_transform, nullptr))
             {
               cfg_->optim.weight_viapoint = 1.0;
             }
           }
         }
       }
       ++i;
     }
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[transformGlobalPlan]: transformed look forward distance: %.3f", max_plan_length);
         
     // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
     // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
     if (transformed_plan.empty())
     {
       //如果最后一个目标点不可达，抛出异常
       geometry_msgs::msg::PoseStamped plan_local_pose;
       tf2::doTransform(effective_last_plan_pose, plan_local_pose, plan_to_global_transform);
       geometry_msgs::msg::Pose2D pose2d;
       pose2d.x = plan_local_pose.pose.position.x;
       pose2d.y = plan_local_pose.pose.position.y;
       pose2d.theta = tf2::getYaw(plan_local_pose.pose.orientation);
       double cost = 0;
       try
       {
         cost =  costmap_model_->scorePose(pose2d, dwb_critics::getOrientedFootprint(pose2d, footprint_spec_));
       }
       catch(const dwb_core::IllegalTrajectoryException& e)
       {
         if (!std::strcmp(e.what(), "Trajectory Hits Obstacle."))
         {
           RCLCPP_WARN(
             logger_,
             "transformGlobalPlan: empty transformed plan and last pose hits obstacle; "
             "running plan-frame goal search (no PlannerException).");
           (void)adjustOccupiedGlobalPlanGoalInPlace(
             global_plan.back(), plan_to_global_transform, effective_last_plan_pose);
         }
       }
       //如果最后一个目标点不可达，抛出异常
 
       tf2::doTransform(effective_last_plan_pose, newer_pose, plan_to_global_transform);
 
       transformed_plan.push_back(newer_pose);
       
       // Return the index of the current goal point (inside the distance threshold)
       if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
     }
     else
     {
       if (current_goal_idx) {
         *current_goal_idx = std::min(i - 1, last_idx);
       }
     }
     
     // Return the transformation from the global plan to the global planning frame if desired
     if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
   }
   catch(tf2::LookupException& ex)
   {
     RCLCPP_ERROR(logger_, "No Transform available Error: %s\n", ex.what());
     return false;
   }
   catch(tf2::ConnectivityException& ex)
   {
     RCLCPP_ERROR(logger_, "Connectivity Error: %s\n", ex.what());
     return false;
   }
   catch(tf2::ExtrapolationException& ex)
   {
     RCLCPP_ERROR(logger_, "Extrapolation Error: %s\n", ex.what());
     if (global_plan.size() > 0)
       RCLCPP_ERROR(logger_, "Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());
 
     return false;
   }
 
   return true;
 }
 
 
     
       
       
 double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan, const geometry_msgs::msg::PoseStamped& local_goal,
                     int current_goal_idx, const geometry_msgs::msg::TransformStamped& tf_plan_to_global, int moving_average_length) const
 {
   int n = (int)global_plan.size();
   
   // check if we are near the global goal already
   if (current_goal_idx > n-moving_average_length-2)
   {
     if (current_goal_idx >= n-1) // we've exactly reached the goal
     {
       return tf2::getYaw(local_goal.pose.orientation);
     }
     else
     {
 //      tf::Quaternion global_orientation;
 //      tf::quaternionMsgToTF(global_plan.back().pose.orientation, global_orientation);
 //      return  tf2::getYaw(tf_plan_to_global.getRotation() *  global_orientation );
 
         tf2::Quaternion global_orientation, tf_plan_to_global_orientation;
         tf2::fromMsg(global_plan.back().pose.orientation, global_orientation);
         tf2::fromMsg(tf_plan_to_global.transform.rotation, tf_plan_to_global_orientation);
 
         return tf2::getYaw(tf_plan_to_global_orientation * global_orientation);
     }     
   }
   
   // reduce number of poses taken into account if the desired number of poses is not available
   moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
   
   std::vector<double> candidates;
   geometry_msgs::msg::PoseStamped tf_pose_k = local_goal;
   geometry_msgs::msg::PoseStamped tf_pose_kp1;
   
   int range_end = current_goal_idx + moving_average_length;
   for (int i = current_goal_idx; i < range_end; ++i)
   {
     // Transform pose of the global plan to the planning frame
     const geometry_msgs::msg::PoseStamped& pose = global_plan.at(i+1);
     tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);
       
     // calculate yaw angle  
     candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
               tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
     
     if (i<range_end-1) 
       tf_pose_k = tf_pose_kp1;
   }
   return average_angles(candidates);
 }
       
       
 void TebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards) const
 {
   double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
   // Limit translational velocity for forward driving
   if (vx > max_vel_x)
     ratio_x = max_vel_x / vx;
   
   // limit strafing velocity
   if (vy > max_vel_y || vy < -max_vel_y)
     ratio_y = std::abs(max_vel_y / vy);
   
   // Limit angular velocity
   if (omega > max_vel_theta || omega < -max_vel_theta)
     ratio_omega = std::abs(max_vel_theta / omega);
   
   // Limit backwards velocity
   if (max_vel_x_backwards<=0)
   {
     RCLCPP_WARN_ONCE(
                 logger_,
                 "TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
   }
   else if (vx < -max_vel_x_backwards)
     ratio_x = - max_vel_x_backwards / vx;
 
   if (cfg_->robot.use_proportional_saturation)
   {
     double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
     vx *= ratio;
     vy *= ratio;
     omega *= ratio;
   }
   else
   {
     vx *= ratio_x;
     vy *= ratio_y;
     omega *= ratio_omega;
   }
 }
      
      
 double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
 {
   if (omega==0 || v==0)
     return 0;
     
   double radius = v/omega;
   
   if (fabs(radius) < min_turning_radius)
     radius = double(g2o::sign(radius)) * min_turning_radius; 
 
   return std::atan(wheelbase / radius);
 }
      
 
 void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
 {
     RCLCPP_WARN_EXPRESSION(
                 logger_, opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                   "The inscribed radius of the footprint specified for TEB optimization (%.2f) + min_obstacle_dist (%.2f) are smaller "
                   "than the inscribed radius of the robot's footprint in the costmap parameters (%.2f, including 'footprint_padding'). "
                   "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
 }
    
    
    
 void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,  int& goal_idx)
 {
     rclcpp::Time current_time = clock_->now();
     
     // reduced horizon backup mode
     if (cfg_->recovery.shrink_horizon_backup && 
         goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
        (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).seconds() < cfg_->recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
     {
         RCLCPP_INFO_EXPRESSION(
                     logger_,
                     no_infeasible_plans_==1,
                     "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_->recovery.shrink_horizon_min_duration);
 
 
         // Shorten horizon if requested
         // reduce to 50 percent:
         int horizon_reduction = goal_idx/2;
         
         if (no_infeasible_plans_ > 9)
         {
             RCLCPP_INFO_EXPRESSION(
                         logger_,
                         no_infeasible_plans_==10,
                         "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
             horizon_reduction /= 2;
         }
         
         // we have a small overhead here, since we already transformed 50% more of the trajectory.
         // But that's ok for now, since we do not need to make transformGlobalPlan more complex 
         // and a reduced horizon should occur just rarely.
         int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
         goal_idx -= horizon_reduction;
         if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
             transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
         else
             goal_idx += horizon_reduction; // this should not happen, but safety first ;-) 
     }
     
     
     // detect and resolve oscillations
     if (cfg_->recovery.oscillation_recovery)
     {
         double max_vel_theta;
         double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_->robot.max_vel_x : cfg_->robot.max_vel_x_backwards;
         if (cfg_->robot.min_turning_radius!=0 && max_vel_current>0)
             max_vel_theta = std::max( max_vel_current/std::abs(cfg_->robot.min_turning_radius),  cfg_->robot.max_vel_theta );
         else
             max_vel_theta = cfg_->robot.max_vel_theta;
         
         failure_detector_.update(last_cmd_, cfg_->robot.max_vel_x, cfg_->robot.max_vel_x_backwards, max_vel_theta,
                                cfg_->recovery.oscillation_v_eps, cfg_->recovery.oscillation_omega_eps);
         
         bool oscillating = failure_detector_.isOscillating();
         bool recently_oscillated = (clock_->now()-time_last_oscillation_).seconds() < cfg_->recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently
         
         if (oscillating)
         {
             if (!recently_oscillated)
             {
                 // save current turning direction
                 if (robot_vel_.angular.z > 0)
                     last_preferred_rotdir_ = RotType::left;
                 else
                     last_preferred_rotdir_ = RotType::right;
                 RCLCPP_INFO(logger_, "TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
             }
             time_last_oscillation_ = clock_->now();
             planner_->setPreferredTurningDir(last_preferred_rotdir_);
         }
         else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
         {
             last_preferred_rotdir_ = RotType::none;
             planner_->setPreferredTurningDir(last_preferred_rotdir_);
             RCLCPP_INFO(logger_, "TebLocalPlannerROS: oscillation recovery disabled/expired.");
         }
     }
 
 }
      
 
 void TebLocalPlannerROS::setSpeedLimit(
     const double & speed_limit, const bool & percentage)
 {
   if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
     // Restore default value
     cfg_->robot.max_vel_x = cfg_->robot.base_max_vel_x;
     cfg_->robot.base_max_vel_x_backwards = cfg_->robot.base_max_vel_x_backwards;
     cfg_->robot.base_max_vel_y = cfg_->robot.base_max_vel_y;
     cfg_->robot.base_max_vel_theta = cfg_->robot.base_max_vel_theta;
   } else {
     if (percentage) {
       // Speed limit is expressed in % from maximum speed of robot
       cfg_->robot.max_vel_x = cfg_->robot.base_max_vel_x * speed_limit / 100.0;
       cfg_->robot.base_max_vel_x_backwards = cfg_->robot.base_max_vel_x_backwards * speed_limit / 100.0;
       cfg_->robot.base_max_vel_y = cfg_->robot.base_max_vel_y * speed_limit / 100.0;
       cfg_->robot.base_max_vel_theta = cfg_->robot.base_max_vel_theta * speed_limit / 100.0;
     } else {
       // Speed limit is expressed in absolute value
       double max_speed_xy = std::max(
             std::max(cfg_->robot.base_max_vel_x,cfg_->robot.base_max_vel_x_backwards),cfg_->robot.base_max_vel_y);
       if (speed_limit < max_speed_xy) {
         // Handling components and angular velocity changes:
         // Max velocities are being changed in the same proportion
         // as absolute linear speed changed in order to preserve
         // robot moving trajectories to be the same after speed change.
         // G. Doisy: not sure if that's applicable to base_max_vel_x_backwards.
         const double ratio = speed_limit / max_speed_xy;
         cfg_->robot.max_vel_x = cfg_->robot.base_max_vel_x * ratio;
         cfg_->robot.base_max_vel_x_backwards = cfg_->robot.base_max_vel_x_backwards * ratio;
         cfg_->robot.base_max_vel_y = cfg_->robot.base_max_vel_y * ratio;
         cfg_->robot.base_max_vel_theta = cfg_->robot.base_max_vel_theta * ratio;
       }
     }
    }
  }

void TebLocalPlannerROS::speedLimitCallback(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> l(speed_limit_mutex_);
  speed_limit_linear_x_ = msg->data;
  has_speed_limit_ = true;
}

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

bool TebLocalPlannerROS::hasReverseSegmentInPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan) const
{
  if (plan.size() < 2) {
    return false;
  }
  const int required_hits = std::max(1, cfg_->trajectory.reverse_segment_angle_check_num);
  size_t reverse_hit_count = 0;
  // 倒车判定：相邻路点位移方向与 pose 方向夹角接近 180°（跳过过短段与首段）
  for (size_t i = 1; i + 1 < plan.size(); ++i) {
    const double dx = plan[i + 1].pose.position.x - plan[i].pose.position.x;
    const double dy = plan[i + 1].pose.position.y - plan[i].pose.position.y;
    const double segment_length = std::hypot(dx, dy);
    if (segment_length < cfg_->trajectory.reverse_segment_min_segment_length_m) {
      RCLCPP_DEBUG(
        logger_,
        "狭窄通道: 跳过过短段 plan[%zu]->[%zu], length=%.4f",
        i, i + 1, segment_length);
      continue;
    }
    const double yaw = tf2::getYaw(plan[i].pose.orientation);
    const double dot = dx * std::cos(yaw) + dy * std::sin(yaw);
    const double cos_angle = std::clamp(dot / segment_length, -1.0, 1.0);
    const double heading_segment_angle_deg = std::acos(cos_angle) * 180.0 / M_PI;
    if (heading_segment_angle_deg >= cfg_->trajectory.reverse_segment_min_angle_deg) {
      ++reverse_hit_count;
      RCLCPP_DEBUG(
        logger_,
        "狭窄通道: 倒车段候选 plan[%zu]->[%zu], angle=%.2f deg, hits=%zu/%d",
        i, i + 1, heading_segment_angle_deg, reverse_hit_count, required_hits);
      if (reverse_hit_count >= static_cast<size_t>(required_hits)) {
        return true;
      }
    }
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
    "/backward_mode published: %s",
    backward ? "true (backward)" : "false (forward)");
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
       RCLCPP_INFO(logger_, "Switching to edge-following mode parameters");
       cfg_->optim.weight_optimaltime = cfg_->wall_line.edge_weight_optimaltime;
       cfg_->obstacles.min_obstacle_dist = cfg_->wall_line.edge_min_obstacle_dist;
       cfg_->robot.acc_lim_theta = cfg_->wall_line.edge_acc_lim_theta;
       cfg_->robot.max_vel_theta = cfg_->wall_line.edge_max_vel_theta;

       // Set footprint vertices
       node->set_parameter(rclcpp::Parameter(name_ + "." + "footprint_model.vertices", cfg_->wall_line.edge_footprint_vertices));
       is_edge_following_mode_ = true;
     } else {
       // Switch to normal mode
       RCLCPP_INFO(logger_, "Switching to normal mode parameters");
       cfg_->optim.weight_optimaltime = normal_weight_optimaltime_;
       cfg_->obstacles.min_obstacle_dist = normal_min_obstacle_dist_;
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
        RCLCPP_INFO(logger_, "static_layer.enabled already at desired state %d, skipping service call.", desired_state ? 1 : 0);
      } else if (desired_state) {
        std::thread([this]() {
          auto start = std::chrono::steady_clock::now();
          auto delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(cfg_->wall_line.static_layer_enable_delay));

          while (std::chrono::steady_clock::now() - start < delay_ms) {
            if (!desired_static_layer_state_.load()) {
              RCLCPP_DEBUG(logger_, "Delayed static_layer enable cancelled: desired state changed");
              return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }

          if (desired_static_layer_state_.load()) {
            setStaticLayerEnabled(true, true);
          }
        }).detach();
      } else {
        setStaticLayerEnabled(false, false);
      }
    }
    
    if (cfg_->wall_line.switch_local_footprint) {
      bool desired_footprint_state = enable_edge_mode;
      desired_local_footprint_state_ = desired_footprint_state;

      if (desired_footprint_state == current_local_footprint_state_.load()) {
        RCLCPP_INFO(logger_, "Local footprint already at desired state %d, skipping service call.", desired_footprint_state ? 1 : 0);
      } else if (!desired_footprint_state) {
        std::thread([this]() {
          auto start = std::chrono::steady_clock::now();
          auto delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(cfg_->wall_line.static_layer_enable_delay));

          while (std::chrono::steady_clock::now() - start < delay_ms) {
            if (desired_local_footprint_state_.load()) {
              RCLCPP_INFO(logger_, "Delayed local footprint enable cancelled: desired state changed");
              return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }

          if (!desired_local_footprint_state_.load()) {
            setLocalFootprintEnabled(false, true);
          }
        }).detach();
      } else {
        setLocalFootprintEnabled(true, false);
      }
    }

    if (cfg_->wall_line.switch_global_footprint) {
      bool desired_footprint_state = enable_edge_mode;
      desired_global_footprint_state_ = desired_footprint_state;

      if (desired_footprint_state == current_global_footprint_state_.load()) {
        RCLCPP_INFO(logger_, "Global footprint already at desired state %d, skipping service call.", desired_footprint_state ? 1 : 0);
      } else if (!desired_footprint_state) {
        std::thread([this]() {
          auto start = std::chrono::steady_clock::now();
          auto delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(cfg_->wall_line.static_layer_enable_delay));

          while (std::chrono::steady_clock::now() - start < delay_ms) {
            if (desired_global_footprint_state_.load()) {
              RCLCPP_INFO(logger_, "Delayed global footprint enable cancelled: desired state changed");
              return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }

          if (!desired_global_footprint_state_.load()) {
            setGlobalFootprintEnabled(false, true);
          }
        }).detach();
      } else {
        setGlobalFootprintEnabled(true, false);
      }
    }
    // Delayed max_vel_x restoration when switching to normal mode
    if (!enable_edge_mode) {
      desired_normal_vel_x_restore_ = true;
      std::thread([this]() {
        auto start = std::chrono::steady_clock::now();
        auto delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(cfg_->wall_line.static_layer_enable_delay));

        while (std::chrono::steady_clock::now() - start < delay_ms) {
          if (!desired_normal_vel_x_restore_.load()) {
            RCLCPP_DEBUG(logger_, "Delayed max_vel_x restore cancelled: switched back to edge mode");
            return;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (desired_normal_vel_x_restore_.load()) {
          {
            std::lock_guard<std::mutex> l(speed_limit_mutex_);
            if (has_speed_limit_) {
              // Ensure non-negative limit and cap by robot's nominal base maximum
              const double limit = std::max(0.0, speed_limit_linear_x_);
              if (std::isfinite(limit) && speed_limit_linear_x_ > 0) {
                if (limit < safe_linear_speed_limit_ && cfg_->robot.max_vel_x != limit) {
                  cfg_->robot.max_vel_x = std::min(limit, safe_linear_speed_limit_);
                  RCLCPP_INFO(logger_, "Performing change speed limit in switch to normal mode!, max linear speed is: %.2f", cfg_->robot.max_vel_x);
                } else if (limit >= safe_linear_speed_limit_ && cfg_->robot.max_vel_x != safe_linear_speed_limit_) {
                  cfg_->robot.max_vel_x = safe_linear_speed_limit_;
                  RCLCPP_INFO(logger_, "Receive speed limit is greater than safe linear speed limit in switch to normal mode!, set limit speed to safe linear speed limit: %.2f !", safe_linear_speed_limit_);
                }
              } else if (!std::isfinite(limit) || limit <= 0) {
                cfg_->robot.max_vel_x = cfg_max_vel_x_;
                RCLCPP_INFO_THROTTLE(logger_, *(clock_), 5000, "Speed limit is not finite or less than 0 in switch to normal mode!, current speed limit is: %.2f !", cfg_->robot.max_vel_x);
              }
            } else {
              cfg_->robot.max_vel_x = cfg_max_vel_x_;
              RCLCPP_INFO(logger_, "max_vel_x is not set in switch to normal mode!, set limit speed to normal max_vel_x: %.2f !", cfg_max_vel_x_);
            }
           }
        }
      }).detach();
    } else {
      desired_normal_vel_x_restore_ = false;
    }
   } catch (const std::exception& ex) {
     RCLCPP_WARN(logger_, "Failed to switch parameter mode: %s", ex.what());
   }
 }

 void TebLocalPlannerROS::setStaticLayerEnabled(bool enabled, bool is_delayed)
 {
   if (!static_layer_client_) {
     RCLCPP_ERROR(logger_, "static_layer_client_ not initialized. Cannot set static_layer.enabled.");
     return;
   }

   // Check if the requested state matches the desired state
   // This prevents outdated timer callbacks from overriding newer state changes
   if (enabled != desired_static_layer_state_.load()) {
     RCLCPP_DEBUG(logger_, "Skipping setStaticLayerEnabled(%s): desired state is now %s",
                  enabled ? "true" : "false", desired_static_layer_state_.load() ? "true" : "false");
     return;
   }

   // Check service availability
   if (!static_layer_client_->wait_for_service(std::chrono::seconds(2))) {
     RCLCPP_ERROR(logger_, "Service /local_costmap/local_costmap/set_parameters not available.");
     return;
   }

   // Create request
   auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
   rcl_interfaces::msg::Parameter param_msg = rclcpp::Parameter("static_layer.enabled", enabled).to_parameter_msg();
   request->parameters.push_back(param_msg);

   RCLCPP_INFO(logger_, "Calling /local_costmap/local_costmap/set_parameters to set static_layer.enabled -> %s%s",
               enabled ? "true" : "false", is_delayed ? " (delayed)" : "");

   // Use a separate thread to avoid blocking the executor
   std::thread([this, request, enabled]() {
     auto future = static_layer_client_->async_send_request(request);
     try {
       auto response = future.get();
       bool all_successful = true;
       std::string failed_reason;
       for (const auto& result : response->results) {
         if (!result.successful) {
           all_successful = false;
           failed_reason = result.reason;
           break;
         }
       }
       if (all_successful) {
         RCLCPP_INFO(logger_, "Set /local_costmap/local_costmap static_layer.enabled -> %s, result: successful",
                     enabled ? "true" : "false");
         current_static_layer_state_.store(enabled);
       } else {
         RCLCPP_ERROR(logger_, "Set /local_costmap/local_costmap static_layer.enabled -> %s, result: failed. Reason: %s",
                      enabled ? "true" : "false", failed_reason.empty() ? "unknown" : failed_reason.c_str());
       }
     } catch (const std::exception& e) {
       RCLCPP_ERROR(logger_, "Failed to call set_parameters service for local_costmap: %s", e.what());
     }
   }).detach();
 }

 void TebLocalPlannerROS::setLocalFootprintEnabled(bool enable, bool is_delayed)
 {
   if (!local_footprint_client_) {
     RCLCPP_ERROR(logger_, "local_footprint_client_ not initialized. Cannot set footprint.");
     return;
   }

   // Check if the requested state matches the desired state
   // This prevents outdated timer callbacks from overriding newer state changes
   // enable=true means edge footprint, enable=false means normal footprint
   if (enable == current_local_footprint_state_.load()) {
     RCLCPP_INFO(logger_, "Skipping setLocalFootprintEnabled(%s): desired state is now %s",
                  enable ? "edge" : "normal", current_local_footprint_state_.load() ? "edge" : "normal");
     return;
   }

   // Check service availability
   if (!local_footprint_client_->wait_for_service(std::chrono::seconds(2))) {
     RCLCPP_ERROR(logger_, "Service /local_costmap/local_costmap/set_parameters not available.");
     return;
   }

   // Determine footprint to set
   std::string footprint_vertices = enable ? cfg_->wall_line.edge_footprint_vertices : local_costmap_footprint_;

   // Create request
   auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
   rcl_interfaces::msg::Parameter param_msg = rclcpp::Parameter("footprint", footprint_vertices).to_parameter_msg();
   request->parameters.push_back(param_msg);

   RCLCPP_INFO(logger_, "Calling /local_costmap/local_costmap/set_parameters to set footprint -> %s%s",
               footprint_vertices.c_str(), is_delayed ? " (delayed)" : "");

   // Use a separate thread to avoid blocking the executor
   std::thread([this, request, footprint_vertices, enable]() {
     auto future = local_footprint_client_->async_send_request(request);
     try {
       auto response = future.get();
       bool all_successful = true;
       std::string failed_reason;
       for (const auto& result : response->results) {
         if (!result.successful) {
           all_successful = false;
           failed_reason = result.reason;
           break;
         }
       }
       if (all_successful) {
         RCLCPP_INFO(logger_, "Set /local_costmap/local_costmap footprint -> %s, result: successful",
                     footprint_vertices.c_str());
         current_local_footprint_state_.store(enable);
       } else {
         RCLCPP_ERROR(logger_, "Set /local_costmap/local_costmap footprint -> %s, result: failed. Reason: %s",
                      footprint_vertices.c_str(), failed_reason.empty() ? "unknown" : failed_reason.c_str());
       }
     } catch (const std::exception& e) {
       RCLCPP_ERROR(logger_, "Failed to call set_parameters service for local_costmap: %s", e.what());
     }
   }).detach();
 }

 void TebLocalPlannerROS::setGlobalFootprintEnabled(bool enable, bool is_delayed)
 {
   if (!global_footprint_client_) {
     RCLCPP_ERROR(logger_, "global_footprint_client_ not initialized. Cannot set footprint.");
     return;
   }

   // Check if the requested state matches the desired state
   // This prevents outdated timer callbacks from overriding newer state changes
   // enable=true means edge footprint, enable=false means normal footprint
   if (enable == current_global_footprint_state_.load()) {
     RCLCPP_INFO(logger_, "Skipping setGlobalFootprintEnabled(%s): desired state is now %s",
                  enable ? "edge" : "normal", current_global_footprint_state_.load() ? "edge" : "normal");
     return;
   }

   // Check service availability
   if (!global_footprint_client_->wait_for_service(std::chrono::seconds(2))) {
     RCLCPP_ERROR(logger_, "Service /global_costmap/global_costmap/set_parameters not available.");
     return;
   }

   // Determine footprint to set
   std::string footprint_vertices = enable ? cfg_->wall_line.edge_footprint_vertices : global_costmap_footprint_;

   // Create request
   auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
   rcl_interfaces::msg::Parameter param_msg = rclcpp::Parameter("footprint", footprint_vertices).to_parameter_msg();
   request->parameters.push_back(param_msg);

   RCLCPP_INFO(logger_, "Calling /global_costmap/global_costmap/set_parameters to set footprint -> %s%s",
               footprint_vertices.c_str(), is_delayed ? " (delayed)" : "");

   // Use a separate thread to avoid blocking the executor
   std::thread([this, request, footprint_vertices, enable]() {
     auto future = global_footprint_client_->async_send_request(request);
     try {
       auto response = future.get();
       bool all_successful = true;
       std::string failed_reason;
       for (const auto& result : response->results) {
         if (!result.successful) {
           all_successful = false;
           failed_reason = result.reason;
           break;
         }
       }
       if (all_successful) {
         RCLCPP_INFO(logger_, "Set /global_costmap/global_costmap footprint -> %s, result: successful",
                     footprint_vertices.c_str());
         current_global_footprint_state_.store(enable);
       } else {
         RCLCPP_ERROR(logger_, "Set /global_costmap/global_costmap footprint -> %s, result: failed. Reason: %s",
                      footprint_vertices.c_str(), failed_reason.empty() ? "unknown" : failed_reason.c_str());
       }
     } catch (const std::exception& e) {
       RCLCPP_ERROR(logger_, "Failed to call set_parameters service for global_costmap: %s", e.what());
     }
   }).detach();
 }

 void TebLocalPlannerROS::customObstacleCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obst_msg)
 {
   std::lock_guard<std::mutex> l(custom_obst_mutex_);
   custom_obstacle_msg_ = *obst_msg;  
 }
 
 void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::msg::Path::ConstSharedPtr via_points_msg)
 {
   RCLCPP_INFO_ONCE(logger_, "Via-points received. This message is printed once.");
   if (cfg_->trajectory.global_plan_viapoint_sep > 0)
   {
     RCLCPP_INFO(logger_, "Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
              "Ignoring custom via-points.");
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

void TebLocalPlannerROS::vehiclePosesCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr msg)
{
  if (!msg || !initialized_ || !costmap_ros_ || !tf_) {
    return;
  }

  // 获取机器人在 map 坐标系下的当前位姿
  geometry_msgs::msg::PoseStamped robot_pose_map;
  try {
    geometry_msgs::msg::TransformStamped tf_base_to_map =
      tf_->lookupTransform(
        cfg_->map_frame,
        costmap_ros_->getBaseFrameID(),
        tf2::TimePointZero,
        tf2::durationFromSec(0.5));

    robot_pose_map.header.stamp = tf_base_to_map.header.stamp;
    robot_pose_map.header.frame_id = cfg_->map_frame;
    robot_pose_map.pose.position.x = tf_base_to_map.transform.translation.x;
    robot_pose_map.pose.position.y = tf_base_to_map.transform.translation.y;
    robot_pose_map.pose.position.z = tf_base_to_map.transform.translation.z;
    robot_pose_map.pose.orientation = tf_base_to_map.transform.rotation;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "vehiclePosesCallback: Failed to get robot pose in map frame: %s", ex.what());
    return;
  }

  const double new_vehicle_distance_threshold_sq = new_vehicle_distance_threshold_ * new_vehicle_distance_threshold_;

  // 先构造当前消息中、距离机器人小于阈值的候选车辆（位姿统一转换到 map 坐标系）
  std::vector<geometry_msgs::msg::PoseStamped> near_vehicles;
  near_vehicles.reserve(msg->poses.size());

  for (const auto &pose : msg->poses) {
    geometry_msgs::msg::PoseStamped vehicle_pose;
    vehicle_pose.header = msg->header;
    vehicle_pose.pose = pose;

    // 如有必要，将车辆位姿转换到 map 坐标系
    if (vehicle_pose.header.frame_id != cfg_->map_frame) {
      try {
        geometry_msgs::msg::TransformStamped tf_to_map =
          tf_->lookupTransform(
            cfg_->map_frame,
            vehicle_pose.header.frame_id,
            tf2::TimePointZero,
            tf2::durationFromSec(0.5));
        tf2::doTransform(vehicle_pose, vehicle_pose, tf_to_map);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
          logger_, *(clock_), 2000,
          "vehiclePosesCallback: Failed to transform vehicle pose to map frame: %s", ex.what());
        continue;
      }
    }

    const double dx = vehicle_pose.pose.position.x - robot_pose_map.pose.position.x;
    const double dy = vehicle_pose.pose.position.y - robot_pose_map.pose.position.y;
    const double dist_sq = dx * dx + dy * dy;

    if (dist_sq <= new_vehicle_distance_threshold_sq) {
      RCLCPP_INFO_THROTTLE(
        logger_, *(clock_), 2000,
        "vehiclePosesCallback: Found nearby vehicle at (%.2f, %.2f), distance to robot: %.2f m",
        vehicle_pose.pose.position.x, vehicle_pose.pose.position.y, std::sqrt(dist_sq));
      near_vehicles.emplace_back(vehicle_pose);
    }
  }

  // 更新全局车辆列表：
  // (1) 先根据机器人位置剔除 global_vehicle_poses_ 中距离超过阈值的车辆
  // (2) 再从 near_vehicles 中加入新的车辆，如果与现有车辆距离太近则不加入
  const double erase_vehicle_distance_threshold_sq = erase_vehicle_distance_threshold_ * erase_vehicle_distance_threshold_;
  {
    std::lock_guard<std::mutex> veh_lock(global_vehicle_poses_mutex_);

    // (1) 移除距离机器人超过阈值的已有车辆
    auto it = global_vehicle_poses_.begin();
    while (it != global_vehicle_poses_.end()) {
      const double dx = it->pose.position.x - robot_pose_map.pose.position.x;
      const double dy = it->pose.position.y - robot_pose_map.pose.position.y;
      const double dist_sq = dx * dx + dy * dy;

      if (dist_sq > erase_vehicle_distance_threshold_sq) {
        RCLCPP_INFO_THROTTLE(
          logger_, *(clock_), 2000,
          "vehiclePosesCallback: Removing vehicle at (%.2f, %.2f) from global list, distance to robot: %.2f m",
          it->pose.position.x, it->pose.position.y, std::sqrt(dist_sq));
        it = global_vehicle_poses_.erase(it);
      } else {
        ++it;
      }
    }

    // (2) 对每一个候选车辆，若与现有车辆“距离相近”则认为是同一辆车，不重复加入
    for (const auto &candidate : near_vehicles) {
      bool exists_similar = false;
      for (const auto &existing : global_vehicle_poses_) {
        const double dx = existing.pose.position.x - candidate.pose.position.x;
        const double dy = existing.pose.position.y - candidate.pose.position.y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq <= same_vehicle_threshold_sq_) {
          RCLCPP_INFO_THROTTLE(
            logger_, *(clock_), 2000,
            "vehiclePosesCallback: Candidate vehicle at (%.2f, %.2f) is similar to existing vehicle at (%.2f, %.2f), distance: %.2f m. Not adding to global list.",
            candidate.pose.position.x, candidate.pose.position.y,
            existing.pose.position.x, existing.pose.position.y,
            std::sqrt(dist_sq));
          exists_similar = true;
          break;
        }
      }

      if (!exists_similar) {
        global_vehicle_poses_.emplace_back(candidate);
      }
    }
  }
}
 
 // void TebLocalPlannerROS::rotation_sigh_callback(const std_msgs::msg::Bool &msg)
 // {
 //   std::lock_guard<std::mutex> cfg_via_sep_lock(cfg_->configMutex());
 //   if (msg.data)
 //   {
 //     cfg_->trajectory.global_plan_viapoint_sep = via_sep_;
 //   }
 //   else
 //   {
 //     cfg_->trajectory.global_plan_viapoint_sep = -0.1;
 //   }
   
 // }
 
 void TebLocalPlannerROS::activate() {
   visualization_->on_activate();

   // Reset static_layer state to default (enabled)
   current_static_layer_state_.store(true);
   desired_static_layer_state_.store(true);
   current_global_footprint_state_.store(false);
   desired_global_footprint_state_.store(false);
   current_local_footprint_state_.store(false);
   desired_local_footprint_state_.store(false);

   return;
 }
 void TebLocalPlannerROS::deactivate() {
   visualization_->on_deactivate();
   restoreNarrowPassageTebSettings();
   resetBackwardModePublicationState(true);

   return;
 }
 void TebLocalPlannerROS::cleanup() {
   visualization_->on_cleanup();
   costmap_converter_->stopWorker();

   // Cleanup static_layer client
   static_layer_client_.reset();
   backward_mode_pub_.reset();
   resetBackwardModePublicationState(false);

   return;
 }

namespace {
double pathLengthToIndex(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const size_t end_index)
{
  if (plan.empty()) {
    return 0.0;
  }
  const size_t last_seg = std::min(end_index, plan.size() - 1);
  double len = 0.0;
  for (size_t i = 0; i < last_seg; ++i) {
    const double dx = plan[i + 1].pose.position.x - plan[i].pose.position.x;
    const double dy = plan[i + 1].pose.position.y - plan[i].pose.position.y;
    len += std::hypot(dx, dy);
  }
  return len;
}

/** 弧长 dist 处：取已驶过的最后一个离散路径点下标 k（cum[k]≤dist），朝向用 plan[k]（无 slerp） */
size_t waypointIndexOwningArcLength(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const size_t end_index,
  const double dist)
{
  if (plan.empty() || end_index >= plan.size()) {
    return 0;
  }
  if (dist <= 0.0) {
    return 0;
  }
  double cum_at_k = 0.0;
  size_t k = 0;
  while (k + 1 <= end_index) {
    const double dx = plan[k + 1].pose.position.x - plan[k].pose.position.x;
    const double dy = plan[k + 1].pose.position.y - plan[k].pose.position.y;
    const double seg = std::hypot(dx, dy);
    if (cum_at_k + seg <= dist + 1e-9) {
      cum_at_k += seg;
      ++k;
    } else {
      break;
    }
  }
  return k;
}

bool interpolatePoseAlongPlanToIndex(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const size_t end_index,
  const double distance_along,
  geometry_msgs::msg::PoseStamped * out)
{
  if (!out || plan.empty() || end_index >= plan.size()) {
    return false;
  }
  if (distance_along <= 0.0) {
    *out = plan[0];
    return true;
  }
  const double total = pathLengthToIndex(plan, end_index);
  const double dist = std::min(std::max(0.0, distance_along), total);
  if (dist >= total - 1e-9) {
    *out = plan[end_index];
    return true;
  }
  double accumulated = 0.0;
  for (size_t i = 0; i < end_index; ++i) {
    const double ax = plan[i].pose.position.x;
    const double ay = plan[i].pose.position.y;
    const double bx = plan[i + 1].pose.position.x;
    const double by = plan[i + 1].pose.position.y;
    const double dx = bx - ax;
    const double dy = by - ay;
    const double seg_len = std::hypot(dx, dy);
    if (seg_len < 1e-9) {
      continue;
    }
    if (accumulated + seg_len >= dist - 1e-9) {
      const double t = (dist - accumulated) / seg_len;
      out->header = plan[i].header;
      out->pose.position.x = ax + t * dx;
      out->pose.position.y = ay + t * dy;
      out->pose.position.z =
        plan[i].pose.position.z + t * (plan[i + 1].pose.position.z - plan[i].pose.position.z);
      const size_t orient_idx = waypointIndexOwningArcLength(plan, end_index, dist);
      out->pose.orientation = plan[orient_idx].pose.orientation;
      return true;
    }
    accumulated += seg_len;
  }
  *out = plan[end_index];
  return true;
}
}  // namespace

bool TebLocalPlannerROS::orientedFootprintAnyVertexOutsideLocalCostmap(
  double x, double y, double theta,
  const std::vector<geometry_msgs::msg::Point> & footprint_poly) const
{
  if (!costmap_ || footprint_poly.size() < 2) {
    return true;
  }
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  unsigned int mx = 0, my = 0;
  for (const auto & p : footprint_poly) {
    const double wx = x + (p.x * c - p.y * s);
    const double wy = y + (p.x * s + p.y * c);
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      return true;
    }
  }
  return false;
}

bool TebLocalPlannerROS::isTransformedPlanFootprintSamplesCollisionFree(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const double sample_spacing_m) const
{
  if (!rotation_collision_checker_ || plan.empty()) {
    return true;
  }
  const size_t end_idx = plan.size() - 1;

  const std::vector<geometry_msgs::msg::Point> & footprint_poly =
    (footprint_spec_.size() >= 3) ? footprint_spec_ : costmap_ros_->getRobotFootprint();

  using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  using nav2_costmap_2d::LETHAL_OBSTACLE;

  const auto pose_footprint_clear = [&](double px, double py, double pyaw) -> bool {
    if (orientedFootprintAnyVertexOutsideLocalCostmap(px, py, pyaw, footprint_poly)) {
      return true;
    }
    const double fc = rotation_collision_checker_->footprintCostAtPose(
      px, py, pyaw, footprint_poly);
    if (fc == static_cast<double>(LETHAL_OBSTACLE) ||
        fc == static_cast<double>(INSCRIBED_INFLATED_OBSTACLE)) {
      return false;
    }
    return true;
  };

  const double total_len = pathLengthToIndex(plan, end_idx);
  double forward_cap = cfg_->trajectory.max_global_plan_lookahead_dist;
  if (forward_cap <= 1e-9) {
    forward_cap = total_len;
  }
  const double sample_len = std::min(total_len, forward_cap);
  double spacing = std::max(1e-3, sample_spacing_m);
  if (sample_len > 1e-9) {
    spacing = std::max(spacing, sample_len / 150.0);
  }

  std::vector<double> dists;
  for (double d = 0.0; d < sample_len + 1e-9; d += spacing) {
    dists.push_back(std::min(d, sample_len));
  }
  if (dists.empty()) {
    dists.push_back(0.0);
  }
  if (sample_len > 1e-9 &&
      std::abs(dists.back() - sample_len) > 1e-3) {
    dists.push_back(sample_len);
  }

  geometry_msgs::msg::PoseStamped sampled;
  for (const double d : dists) {
    if (!interpolatePoseAlongPlanToIndex(plan, end_idx, d, &sampled)) {
      continue;
    }
    const double yaw = tf2::getYaw(sampled.pose.orientation);
    if (!pose_footprint_clear(sampled.pose.position.x, sampled.pose.position.y, yaw)) {
      return false;
    }
  }
  return true;
}

 bool TebLocalPlannerROS::shouldRotateInPlace(
  const geometry_msgs::msg::Twist & velocity,
  const std::vector<geometry_msgs::msg::PoseStamped> & transformed_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Note: do NOT gate rotation on current angular velocity (intentionally removed)

  // Check if plan is not empty
  if (transformed_plan.empty())
  {
    // RCLCPP_INFO_THROTTLE(
    //   logger_, *clock_, 2000,
    //   "原地转: shouldRotateInPlace=false，transformed_plan 为空");
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
  const rclcpp::Time now = clock_->now();
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

  if (
    rotation_limit_duration > 0.0 &&
    rotation_limit_distance > 0.0 &&
    last_rotation_pose_time_.nanoseconds() > 0 &&
    !last_rotation_pose_.header.frame_id.empty() &&
    last_rotation_pose_.header.frame_id == target_pose->header.frame_id)
  {
    const double elapsed = (now - last_rotation_pose_time_).seconds();
    if (elapsed <= rotation_limit_duration)
    {
      const double dx = target_pose->pose.position.x - last_rotation_pose_.pose.position.x;
      const double dy = target_pose->pose.position.y - last_rotation_pose_.pose.position.y;
      const double dist_to_last_rotation_pose = std::hypot(dx, dy);
      if (dist_to_last_rotation_pose <= rotation_limit_distance)
      {
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, 500,
          "原地转: shouldRotateInPlace=false，rotation_limit 生效 (dt=%.2f <= %.2f, dist=%.3f <= %.3f)",
          elapsed, rotation_limit_duration, dist_to_last_rotation_pose, rotation_limit_distance);
        was_inplace_rotation_active_ = false;
        return false;
      }
    }
  }

  // Check if angle difference exceeds threshold
  if (std::abs(angular_distance) <= cfg_->rotation.angle_threshold)
  {
    // 仅在“原地转状态退出”的那一拍记录锚点，避免直行阶段反复刷新导致误限流。
    if (was_inplace_rotation_active_) {
      last_rotation_pose_ = *target_pose;
      last_rotation_pose_time_ = now;
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
  const bool path_footprint_clear = isTransformedPlanFootprintSamplesCollisionFree(
    transformed_plan, cfg_->rotation.path_footprint_sample_spacing);
  const bool collision_ok = rotate_in_place_clear && path_footprint_clear;
  if (!collision_ok) {
    was_inplace_rotation_active_ = false;
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 500,
      "原地转: shouldRotateInPlace=false，碰障预检未通过 |dtheta|=%.3f rad、w_odom=%.3f、v_xy=%.3f "
      "(原地扫掠=%s、路径采样=%s)",
      std::abs(angular_distance), velocity.angular.z, std::abs(velocity.linear.x),
      rotate_in_place_clear ? "通过" : "未通过",
      path_footprint_clear ? "通过" : "未通过");
  } else {
    was_inplace_rotation_active_ = true;
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 500,
      "原地转: shouldRotateInPlace=true，碰障预检通过 |dtheta|=%.3f rad、v_xy=%.3f、w_odom=%.3f、forward_lookahead=%.2f m",
      std::abs(angular_distance), std::abs(velocity.linear.x), velocity.angular.z,
      cfg_->rotation.forward_lookahead_distance);
    // 仅在确定执行原地旋转时记录对应 target_idx 的 pose，供后续时间/距离门控复用。
    // last_rotation_pose_ = *target_pose;
    // last_rotation_pose_time_ = now;
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
    return true;
  }

  const double dir = (omega_direction_sign >= 0.0) ? 1.0 : -1.0;
  const double yaw0 = tf2::getYaw(pose.pose.orientation);
  const std::vector<geometry_msgs::msg::Point> & footprint_poly =
    (footprint_spec_.size() >= 3) ? footprint_spec_ : costmap_ros_->getRobotFootprint();

  auto footprint_ok = [&](double y) -> bool {
    while (y > M_PI) y -= 2.0 * M_PI;
    while (y < -M_PI) y += 2.0 * M_PI;
    using namespace nav2_costmap_2d;  // NOLINT
    // 与 isTrajectoryFeasible 一致优先 footprint_spec_，避免 getRobotFootprint() 与贴边/动态轮廓不一致导致误报
    const double footprint_cost = rotation_collision_checker_->footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y, y, footprint_poly);
    if (footprint_cost == static_cast<double>(LETHAL_OBSTACLE) || footprint_cost == static_cast<double>(INSCRIBED_INFLATED_OBSTACLE)) {
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
  
  if (abs_angular_vel < 1e-6)
  {
    // No rotation, check current pose only
    using namespace nav2_costmap_2d;  // NOLINT
    double footprint_cost = rotation_collision_checker_->footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y,
      initial_yaw, costmap_ros_->getRobotFootprint());
    
    if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
      return false;
    }
    
    if (footprint_cost >= static_cast<double>(MAX_NON_OBSTACLE))
    {
      return false;
    }
    
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
    
    using namespace nav2_costmap_2d;  // NOLINT
    double footprint_cost = rotation_collision_checker_->footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y,
      current_yaw, costmap_ros_->getRobotFootprint());
    
    if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
      return false;  // Potential collision detected
    }
    
    if (footprint_cost >= static_cast<double>(MAX_NON_OBSTACLE))
    {
      return false;  // Collision detected
    }
    
    // If we've reached the target, break
    if (rotated_angle >= abs_angular_distance)
    {
      break;
    }
  }
  
  return true;  // No collision detected
}

void TebLocalPlannerROS::clearActiveReferencePair()
{
  active_pair_index_ = -1;
  has_active_reference_pair_ = false;
  active_mission_segment_ = nav_msgs::msg::Path();
  active_reference_path_ = nav_msgs::msg::Path();
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "未找到匹配的贴边参考路径，清空贴边参考路径");
}

void TebLocalPlannerROS::refreshActiveReferencePair()
{
  if (paired_mission_and_reference_path_cache_.paths.size() < 2 ||
    removed_plan_cache_.poses.empty())
  {
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "贴边参考路径缓存或多点路径为空，清空贴边参考路径");
    clearActiveReferencePair();
    return;
  }

  const geometry_msgs::msg::PoseStamped & front_pose = removed_plan_cache_.poses.front();
  const Eigen::Vector2d front_xy(front_pose.pose.position.x, front_pose.pose.position.y);
  int best_pair_index = -1;
  const double match_threshold_m = cfg_->wall_line.mission_segment_front_pose_match_threshold_m;

  for (std::size_t pair_index = 0;
    2 * pair_index + 1 < paired_mission_and_reference_path_cache_.paths.size();
    ++pair_index)
  {
    const auto & mission_segment = paired_mission_and_reference_path_cache_.paths[2 * pair_index];
    bool segment_matched = false;
    for (const auto & pose_stamped : mission_segment.poses) {
      const double dx = front_xy.x() - pose_stamped.pose.position.x;
      const double dy = front_xy.y() - pose_stamped.pose.position.y;
      if (std::hypot(dx, dy) < match_threshold_m) {
        segment_matched = true;
        break;
      }
    }
    if (segment_matched) {
      best_pair_index = static_cast<int>(pair_index);
      break;
    }
  }

  if (best_pair_index < 0) {
    clearActiveReferencePair();
    return;
  }

  const std::size_t mission_path_index = static_cast<std::size_t>(best_pair_index) * 2;
  active_pair_index_ = best_pair_index;
  active_mission_segment_ = paired_mission_and_reference_path_cache_.paths[mission_path_index];
  active_reference_path_ = paired_mission_and_reference_path_cache_.paths[mission_path_index + 1];
  has_active_reference_pair_ = true;
}

void TebLocalPlannerROS::pairedMissionAndReferencePathCallback(
  const capella_ros_msg::msg::LaneCenterPaths::ConstSharedPtr lane_center_paths_message)
{
  std::lock_guard<std::mutex> edge_mission_pair_mutex_lock(edge_mission_pair_mutex_);
  paired_mission_and_reference_path_cache_ = *lane_center_paths_message;
  if (paired_mission_and_reference_path_cache_.paths.empty()) {
    clearActiveReferencePair();
    RCLCPP_INFO(
      logger_,
      "Empty paired_mission_and_reference_path received: cleared active reference pair");
    return;
  }
  refreshActiveReferencePair();
}

void TebLocalPlannerROS::removedPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr removed_plan_message)
{
  std::lock_guard<std::mutex> edge_mission_pair_mutex_lock(edge_mission_pair_mutex_);
  removed_plan_cache_ = *removed_plan_message;
  refreshActiveReferencePair();
}

bool TebLocalPlannerROS::snapshotHasActiveReferencePair() const
{
  std::lock_guard<std::mutex> edge_mission_pair_mutex_lock(edge_mission_pair_mutex_);
  return has_active_reference_pair_;
}

nav_msgs::msg::Path TebLocalPlannerROS::snapshotActiveMissionSegment() const
{
  std::lock_guard<std::mutex> edge_mission_pair_mutex_lock(edge_mission_pair_mutex_);
  return active_mission_segment_;
}

std::vector<nav_msgs::msg::Path> TebLocalPlannerROS::snapshotActiveReferencePathList() const
{
  std::lock_guard<std::mutex> edge_mission_pair_mutex_lock(edge_mission_pair_mutex_);
  if (!has_active_reference_pair_) {
    return {};
  }
  return {active_reference_path_};
}

bool TebLocalPlannerROS::extractLineSegmentFromPath(
  const nav_msgs::msg::Path& path,
  Eigen::Vector2d& segment_start,
  Eigen::Vector2d& segment_end) const
{
  if (path.poses.size() < 2) {
    return false;
  }
  segment_start = Eigen::Vector2d(path.poses.front().pose.position.x, path.poses.front().pose.position.y);
  segment_end = Eigen::Vector2d(path.poses.back().pose.position.x, path.poses.back().pose.position.y);
  return (segment_end - segment_start).norm() > 1e-6;
}

double TebLocalPlannerROS::pointToSegmentDistance(
  const Eigen::Vector2d& query_point,
  const Eigen::Vector2d& segment_start,
  const Eigen::Vector2d& segment_end) const
{
  const Eigen::Vector2d segment_vector = segment_end - segment_start;
  const double segment_length_squared = segment_vector.squaredNorm();
  if (segment_length_squared < 1e-12) {
    return (query_point - segment_start).norm();
  }
  const double projected_parameter =
    (query_point - segment_start).dot(segment_vector) / segment_length_squared;
  const double clamped_parameter = std::max(0.0, std::min(1.0, projected_parameter));
  const Eigen::Vector2d projected_point = segment_start + clamped_parameter * segment_vector;
  return (query_point - projected_point).norm();
}

double TebLocalPlannerROS::segmentToSegmentDistance(
  const Eigen::Vector2d& first_segment_start,
  const Eigen::Vector2d& first_segment_end,
  const Eigen::Vector2d& second_segment_start,
  const Eigen::Vector2d& second_segment_end) const
{
  const double first_to_second_start =
    pointToSegmentDistance(first_segment_start, second_segment_start, second_segment_end);
  const double first_to_second_end =
    pointToSegmentDistance(first_segment_end, second_segment_start, second_segment_end);
  const double second_to_first_start =
    pointToSegmentDistance(second_segment_start, first_segment_start, first_segment_end);
  const double second_to_first_end =
    pointToSegmentDistance(second_segment_end, first_segment_start, first_segment_end);
  return std::min(
    std::min(first_to_second_start, first_to_second_end),
    std::min(second_to_first_start, second_to_first_end));
}

double TebLocalPlannerROS::segmentDirectionAngleDifferenceDeg(
  const Eigen::Vector2d& first_segment_start,
  const Eigen::Vector2d& first_segment_end,
  const Eigen::Vector2d& second_segment_start,
  const Eigen::Vector2d& second_segment_end) const
{
  const Eigen::Vector2d first_direction = first_segment_end - first_segment_start;
  const Eigen::Vector2d second_direction = second_segment_end - second_segment_start;
  if (first_direction.norm() < 1e-6 || second_direction.norm() < 1e-6) {
    return 180.0;
  }
  const double dot_product =
    first_direction.normalized().dot(second_direction.normalized());
  const double clamped_dot = std::max(-1.0, std::min(1.0, dot_product));
  return std::acos(std::abs(clamped_dot)) * 180.0 / M_PI;
}

bool TebLocalPlannerROS::shouldRunEdgeFollowingForTransformedPlan(
  const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan)
{
  nav_msgs::msg::Path transformed_path_segment;
  transformed_path_segment.header = transformed_plan.front().header;
  transformed_path_segment.poses.assign(transformed_plan.begin(), transformed_plan.end());
  Eigen::Vector2d transformed_segment_start;
  Eigen::Vector2d transformed_segment_end;
  if (!extractLineSegmentFromPath(
        transformed_path_segment,
        transformed_segment_start,
        transformed_segment_end))
  {
    paths_near_edge_hit_count_ = 0;
    paths_near_edge_miss_count_++;
    if (paths_near_edge_miss_count_ >= cfg_->wall_line.paths_near_edge_exit_miss_count) {
      paths_near_edge_active_ = false;
    }
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Edge following: 路径长度太短，退出贴边模式");
    return paths_near_edge_active_;
  }

  const double transformed_segment_length =
    (transformed_segment_end - transformed_segment_start).norm();
  if (transformed_segment_length < cfg_->wall_line.transform_path_line_length) {
    paths_near_edge_hit_count_ = 0;
    paths_near_edge_active_ = false;
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "Edge following: 转换路径长度 %.2f < 最小转换路径长度阈值 %.2f, 退出贴边模式",
      transformed_segment_length,
      cfg_->wall_line.transform_path_line_length);
    return false;
  }

  if (!snapshotHasActiveReferencePair()) {
    paths_near_edge_hit_count_ = 0;
    paths_near_edge_miss_count_++;
    if (paths_near_edge_miss_count_ >= cfg_->wall_line.paths_near_edge_exit_miss_count) {
      paths_near_edge_active_ = false;
    }
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "Edge following: removed_plan front_pose 未匹配到 mission 段, miss=%d",
      paths_near_edge_miss_count_);
    return paths_near_edge_active_;
  }

  const nav_msgs::msg::Path active_mission_segment = snapshotActiveMissionSegment();
  Eigen::Vector2d candidate_segment_start;
  Eigen::Vector2d candidate_segment_end;
  if (!extractLineSegmentFromPath(
      active_mission_segment,
      candidate_segment_start,
      candidate_segment_end))
  {
    paths_near_edge_hit_count_ = 0;
    paths_near_edge_miss_count_++;
    if (paths_near_edge_miss_count_ >= cfg_->wall_line.paths_near_edge_exit_miss_count) {
      paths_near_edge_active_ = false;
    }
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "Edge following: 激活 mission 段长度太短");
    return paths_near_edge_active_;
  }

  const double segment_distance =
    segmentToSegmentDistance(
      transformed_segment_start,
      transformed_segment_end,
      candidate_segment_start,
      candidate_segment_end);
  const double heading_difference_deg =
    segmentDirectionAngleDifferenceDeg(
      transformed_segment_start,
      transformed_segment_end,
      candidate_segment_start,
      candidate_segment_end);
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000,
    "Edge following: 当前路径与激活 mission 段距离: %.2f, 夹角: %.2f 度",
    segment_distance,
    heading_difference_deg);

  bool has_matching_paths_near_edge = false;
  bool has_close_but_not_parallel_candidate = false;
  if (segment_distance <= cfg_->wall_line.paths_near_edge_match_distance_threshold) {
    if (heading_difference_deg <= cfg_->wall_line.paths_near_edge_match_angle_threshold_deg) {
      has_matching_paths_near_edge = true;
    } else {
      has_close_but_not_parallel_candidate = true;
    }
  }

  if (has_close_but_not_parallel_candidate && !has_matching_paths_near_edge) {
    paths_near_edge_hit_count_ = 0;
    paths_near_edge_active_ = false;
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "Edge following: 距离匹配但夹角 > 阈值 %.2f 度, 退出贴边模式",
      cfg_->wall_line.paths_near_edge_match_angle_threshold_deg);
    return false;
  }

  if (has_matching_paths_near_edge) {
    paths_near_edge_hit_count_++;
    paths_near_edge_miss_count_ = 0;
    if (paths_near_edge_hit_count_ >= cfg_->wall_line.paths_near_edge_enter_hit_count) {
      paths_near_edge_active_ = true;
    }
  } else {
    paths_near_edge_hit_count_ = 0;
    paths_near_edge_miss_count_++;
    if (paths_near_edge_miss_count_ >= cfg_->wall_line.paths_near_edge_exit_miss_count) {
      paths_near_edge_active_ = false;
    }
  }
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "Edge following: 找到贴边参考线，继续贴边模式");
  return paths_near_edge_active_;
}

bool TebLocalPlannerROS::edgeFollowingEntryGuards(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const nav_msgs::msg::Path& input_path)
{
  if (input_path.poses.size() < 2) {
    if (wall_line_points_.size() > 0) {
      wall_line_points_.clear();
    }
    switchParameterMode(false);
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Edge following: 路径长度太短，退出贴边模式");
    return false;
  }

  bool has_near_vehicles = false;
  {
    std::lock_guard<std::mutex> global_vehicle_poses_mutex_lock(global_vehicle_poses_mutex_);
    const bool have_active_edge_segment = (wall_line_points_.size() >= 2);
    Eigen::Vector2d active_edge_segment_start;
    Eigen::Vector2d active_edge_segment_end;
    if (have_active_edge_segment) {
      active_edge_segment_start = wall_line_points_[0];
      active_edge_segment_end = wall_line_points_[1];
    }
    for (const auto& nearby_vehicle_pose : global_vehicle_poses_) {
      bool vehicle_in_edge_following_exit_corridor = false;
      if (have_active_edge_segment) {
        vehicle_in_edge_following_exit_corridor = isVehicleInEdgeFollowingExitCorridor(
          robot_pose, active_edge_segment_start, active_edge_segment_end, nearby_vehicle_pose.pose);
      } else {
        vehicle_in_edge_following_exit_corridor =
          distance_points2d(robot_pose.pose.position, nearby_vehicle_pose.pose.position) <
          cfg_->wall_line.close_vehicle_distance_threshold;
      }
      if (vehicle_in_edge_following_exit_corridor) {
        has_near_vehicles = true;
        break;
      }
    }
  }
  if (has_near_vehicles) {
    if (wall_line_points_.size() > 0) {
      wall_line_points_.clear();
    }
    switchParameterMode(false);
    fusion_primary_lock_until_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Edge following: 机器人附近有车辆，退出贴边模式");
    return false;
  }

  bool has_protruding_obstacle_nearby = false;
  const Eigen::Vector2d robot_position_planar(
    robot_pose.pose.position.x, robot_pose.pose.position.y);
  const rclcpp::Time current_clock_time = clock_->now();
  auto protruding_obstacle_iterator = protruding_obstacles_.begin();
  while (protruding_obstacle_iterator != protruding_obstacles_.end()) {
    const double distance_robot_to_obstacle =
      (robot_position_planar - protruding_obstacle_iterator->position).norm();
    const double seconds_since_obstacle_detection =
      (current_clock_time - protruding_obstacle_iterator->detection_time).seconds();
    if (seconds_since_obstacle_detection > cfg_->wall_line.obstacle_protrusion_timeout) {
      protruding_obstacle_iterator = protruding_obstacles_.erase(protruding_obstacle_iterator);
      continue;
    }
    const double obstacle_reenter_influence_threshold =
      cfg_->wall_line.obstacle_protrusion_reenter_distance +
      protruding_obstacle_iterator->influence_radius;
    if (distance_robot_to_obstacle < obstacle_reenter_influence_threshold) {
      has_protruding_obstacle_nearby = true;
      ++protruding_obstacle_iterator;
    } else {
      protruding_obstacle_iterator = protruding_obstacles_.erase(protruding_obstacle_iterator);
    }
  }
  if (has_protruding_obstacle_nearby) {
    if (wall_line_points_.size() > 0) {
      wall_line_points_.clear();
    }
    switchParameterMode(false);
    fusion_primary_lock_until_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Edge following: 机器人附近障碍物轮廓点凸出距离太大，退出贴边模式");
    return false;
  }
  return true;
}

bool TebLocalPlannerROS::trySelectWallSegmentFromCandidates(
  const std::vector<nav_msgs::msg::Path>& wall_candidates,
  const nav_msgs::msg::Path& input_path,
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const double parallel_tolerance_degrees,
  const double distance_tolerance_meters,
  Eigen::Vector2d& selected_edge_segment_start,
  Eigen::Vector2d& selected_edge_segment_end,
  double& minimum_average_distance_to_plan,
  double& robot_perpendicular_distance_to_edge_line)
{
  minimum_average_distance_to_plan = std::numeric_limits<double>::max();
  robot_perpendicular_distance_to_edge_line = std::numeric_limits<double>::max();
  const auto& input_path_start_position = input_path.poses.front().pose.position;
  const auto& input_path_end_position = input_path.poses.back().pose.position;
  const double input_path_delta_x = input_path_end_position.x - input_path_start_position.x;
  const double input_path_delta_y = input_path_end_position.y - input_path_start_position.y;
  const double input_path_segment_length = std::hypot(input_path_delta_x, input_path_delta_y);
  if (input_path_segment_length <= cfg_->wall_line.min_path_line_length) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectWallSegmentFromCandidates] Edge following:"
                        "路径长度太短: %.2f, 最小路径长度阈值: %.2f", 
                        input_path_segment_length, cfg_->wall_line.min_path_line_length);
    return false;
  }
  const double robot_yaw_radians = tf2::getYaw(robot_pose.pose.orientation);
  const double input_path_yaw_radians = std::atan2(input_path_delta_y, input_path_delta_x);
  double path_heading_minus_robot_heading = input_path_yaw_radians - robot_yaw_radians;
  while (path_heading_minus_robot_heading > M_PI) {
    path_heading_minus_robot_heading -= 2 * M_PI;
  }
  while (path_heading_minus_robot_heading < -M_PI) {
    path_heading_minus_robot_heading += 2 * M_PI;
  }
  if (std::fabs(path_heading_minus_robot_heading) > M_PI / 4) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectWallSegmentFromCandidates] Edge following:"
                        "机器人方向与路径方向夹角 %.2f 度,超过45度", 
                        path_heading_minus_robot_heading * 180.0 / M_PI);
    return false;
  }
  const double input_path_direction_x = input_path_delta_x / input_path_segment_length;
  const double input_path_direction_y = input_path_delta_y / input_path_segment_length;

  bool found_valid_wall_segment = false;
  double best_minimum_average_distance = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point best_candidate_wall_start_position;
  geometry_msgs::msg::Point best_candidate_wall_end_position;
  double best_robot_perpendicular_distance_to_wall = std::numeric_limits<double>::max();

  for (const auto& wall_candidate_path : wall_candidates) {
    if (wall_candidate_path.poses.size() < 2) {
      continue;
    }
    const auto& wall_segment_start_position = wall_candidate_path.poses.front().pose.position;
    const auto& wall_segment_end_position = wall_candidate_path.poses.back().pose.position;
    const double wall_segment_delta_x = wall_segment_end_position.x - wall_segment_start_position.x;
    const double wall_segment_delta_y = wall_segment_end_position.y - wall_segment_start_position.y;
    const double wall_segment_length = std::hypot(wall_segment_delta_x, wall_segment_delta_y);
    if (wall_segment_length < min_wall_line_length_) {
      RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectWallSegmentFromCandidates] Edge following:"
                        "墙线长度太短: %.2f, 最小墙线长度阈值: %.2f", 
                        wall_segment_length, min_wall_line_length_);
      continue;
    }
    const double wall_direction_x = wall_segment_delta_x / wall_segment_length;
    const double wall_direction_y = wall_segment_delta_y / wall_segment_length;
    const double direction_dot_product =
      input_path_direction_x * wall_direction_x + input_path_direction_y * wall_direction_y;
    const double absolute_cosine_parallelism = std::fabs(direction_dot_product);
    const double parallelism_cosine_threshold =
      std::cos(parallel_tolerance_degrees / 180.0 * M_PI);
    if (absolute_cosine_parallelism < parallelism_cosine_threshold) {
      RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectWallSegmentFromCandidates] Edge following:"
                         "墙线与路径不平行: %.2f 度, 平行度阈值: %.2f 度", 
                         absolute_cosine_parallelism * 180.0 / M_PI, parallel_tolerance_degrees);
      continue;
    }
    const double wall_line_implicit_x_coefficient = wall_segment_delta_y;
    const double wall_line_implicit_y_coefficient = -wall_segment_delta_x;
    const double wall_line_implicit_constant_term =
      wall_segment_end_position.x * wall_segment_start_position.y -
      wall_segment_start_position.x * wall_segment_end_position.y;
    const double line_equation_normalization = std::hypot(
      wall_line_implicit_x_coefficient, wall_line_implicit_y_coefficient);
    const double average_distance_path_to_wall_line =
      std::fabs(
        wall_line_implicit_x_coefficient * input_path_start_position.x +
        wall_line_implicit_y_coefficient * input_path_start_position.y +
        wall_line_implicit_constant_term) /
      (line_equation_normalization + 1e-18);
    const double robot_perpendicular_distance_to_wall_line =
      std::fabs(
        wall_line_implicit_x_coefficient * robot_pose.pose.position.x +
        wall_line_implicit_y_coefficient * robot_pose.pose.position.y +
        wall_line_implicit_constant_term) /
      (line_equation_normalization + 1e-18);
    if (average_distance_path_to_wall_line <= distance_tolerance_meters &&
        average_distance_path_to_wall_line < best_minimum_average_distance) {
      found_valid_wall_segment = true;
      best_minimum_average_distance = average_distance_path_to_wall_line;
      best_candidate_wall_start_position = wall_segment_start_position;
      best_candidate_wall_end_position = wall_segment_end_position;
      best_robot_perpendicular_distance_to_wall = robot_perpendicular_distance_to_wall_line;
    }
  }
  if (!found_valid_wall_segment) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[trySelectWallSegmentFromCandidates] Edge following:"
                         "没有找到合适的墙线");
    return false;
  }
  selected_edge_segment_start = Eigen::Vector2d(
    best_candidate_wall_start_position.x, best_candidate_wall_start_position.y);
  selected_edge_segment_end = Eigen::Vector2d(
    best_candidate_wall_end_position.x, best_candidate_wall_end_position.y);
  minimum_average_distance_to_plan = best_minimum_average_distance;
  robot_perpendicular_distance_to_edge_line = best_robot_perpendicular_distance_to_wall;
  RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[trySelectWallSegmentFromCandidates] Edge following:"
                      "选择的墙线起点: (%.2f, %.2f), 终点: (%.2f, %.2f), 平均距离: %.2f, 机器人到墙线距离: %.2f", 
                       selected_edge_segment_start.x(), selected_edge_segment_start.y(), 
                       selected_edge_segment_end.x(), selected_edge_segment_end.y(), 
                       minimum_average_distance_to_plan, robot_perpendicular_distance_to_edge_line);
  return true;
}

bool TebLocalPlannerROS::trySelectSegmentFromTwoPointPath(
  const nav_msgs::msg::Path& two_point_line_path,
  const nav_msgs::msg::Path& input_path,
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const double parallel_tolerance_degrees,
  const double distance_tolerance_meters,
  Eigen::Vector2d& selected_edge_segment_start,
  Eigen::Vector2d& selected_edge_segment_end,
  double& minimum_average_distance_to_plan,
  double& robot_perpendicular_distance_to_edge_line)
{
  minimum_average_distance_to_plan = std::numeric_limits<double>::max();
  robot_perpendicular_distance_to_edge_line = std::numeric_limits<double>::max();
  if (two_point_line_path.poses.size() < 2 || input_path.poses.size() < 2) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectSegmentFromTwoPointPath] Edge following:"
                         "两点线路径或输入路径数量小于2, 路径点数: %zu, 输入路径点数: %zu",
                         two_point_line_path.poses.size(), input_path.poses.size());
    return false;
  }
  const auto& input_path_start_position = input_path.poses.front().pose.position;
  const auto& input_path_end_position = input_path.poses.back().pose.position;
  const double input_path_delta_x = input_path_end_position.x - input_path_start_position.x;
  const double input_path_delta_y = input_path_end_position.y - input_path_start_position.y;
  const double input_path_segment_length = std::hypot(input_path_delta_x, input_path_delta_y);
  if (input_path_segment_length <= cfg_->wall_line.min_path_line_length) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectSegmentFromTwoPointPath] Edge following:"
                        "路径长度太短: %.2f, 最小路径长度阈值: %.2f", 
                        input_path_segment_length, cfg_->wall_line.min_path_line_length);
    return false;
  }
  const double robot_yaw_radians = tf2::getYaw(robot_pose.pose.orientation);
  const double input_path_yaw_radians = std::atan2(input_path_delta_y, input_path_delta_x);
  double path_heading_minus_robot_heading = input_path_yaw_radians - robot_yaw_radians;
  while (path_heading_minus_robot_heading > M_PI) {
    path_heading_minus_robot_heading -= 2 * M_PI;
  }
  while (path_heading_minus_robot_heading < -M_PI) {
    path_heading_minus_robot_heading += 2 * M_PI;
  }
  if (std::fabs(path_heading_minus_robot_heading) > M_PI / 4) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectSegmentFromTwoPointPath] Edge following:"
                         "机器人方向与路径方向夹角为 %.2f 度，超过45度", 
                         path_heading_minus_robot_heading * 180.0 / M_PI);
    return false;
  }
  const double input_path_direction_x = input_path_delta_x / input_path_segment_length;
  const double input_path_direction_y = input_path_delta_y / input_path_segment_length;

  const auto& edge_line_start_position = two_point_line_path.poses.front().pose.position;
  const auto& edge_line_end_position = two_point_line_path.poses.back().pose.position;
  const double edge_line_delta_x = edge_line_end_position.x - edge_line_start_position.x;
  const double edge_line_delta_y = edge_line_end_position.y - edge_line_start_position.y;
  const double edge_line_segment_length = std::hypot(edge_line_delta_x, edge_line_delta_y);
  if (edge_line_segment_length < min_wall_line_length_) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectSegmentFromTwoPointPath] Edge following:"
                         "墙线长度太短: %.2f, 最小墙线长度阈值: %.2f", 
                         edge_line_segment_length, min_wall_line_length_);
    return false;
  }
  const double edge_line_direction_x = edge_line_delta_x / edge_line_segment_length;
  const double edge_line_direction_y = edge_line_delta_y / edge_line_segment_length;
  const double direction_dot_product =
    input_path_direction_x * edge_line_direction_x +
    input_path_direction_y * edge_line_direction_y;
  const double absolute_cosine_parallelism = std::fabs(direction_dot_product);
  const double parallelism_cosine_threshold =
    std::fabs(std::cos(parallel_tolerance_degrees / 180.0 * M_PI));
  if (absolute_cosine_parallelism <= parallelism_cosine_threshold) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectSegmentFromTwoPointPath] Edge following:"
                         "墙线与路径不平行: %.2f 度, 平行度阈值: %.2f 度", 
                         absolute_cosine_parallelism * 180.0 / M_PI, parallel_tolerance_degrees);
    return false;
  }
  const double edge_line_implicit_x_coefficient = edge_line_delta_y;
  const double edge_line_implicit_y_coefficient = -edge_line_delta_x;
  const double edge_line_implicit_constant_term =
    edge_line_end_position.x * edge_line_start_position.y -
    edge_line_start_position.x * edge_line_end_position.y;
  const double line_equation_normalization =
    std::hypot(edge_line_implicit_x_coefficient, edge_line_implicit_y_coefficient);
  const double average_distance_path_to_edge_line =
    std::fabs(
      edge_line_implicit_x_coefficient * input_path_start_position.x +
      edge_line_implicit_y_coefficient * input_path_start_position.y +
      edge_line_implicit_constant_term) /
    (line_equation_normalization + 1e-18);
  const double robot_perpendicular_distance_to_edge_line_value =
    std::fabs(
      edge_line_implicit_x_coefficient * robot_pose.pose.position.x +
      edge_line_implicit_y_coefficient * robot_pose.pose.position.y +
      edge_line_implicit_constant_term) /
    (line_equation_normalization + 1e-18);
  if (average_distance_path_to_edge_line > distance_tolerance_meters) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectSegmentFromTwoPointPath] Edge following:"
                         "路径与墙线距离太大: %.2f, 距离阈值: %.2f",
                         average_distance_path_to_edge_line, distance_tolerance_meters);
    return false;
  }
  selected_edge_segment_start =
    Eigen::Vector2d(edge_line_start_position.x, edge_line_start_position.y);
  selected_edge_segment_end =
    Eigen::Vector2d(edge_line_end_position.x, edge_line_end_position.y);
  minimum_average_distance_to_plan = average_distance_path_to_edge_line;
  robot_perpendicular_distance_to_edge_line = robot_perpendicular_distance_to_edge_line_value;
  return true;
}

bool robotOnReferenceSegmentWithinDistance(
  const Eigen::Vector2d& segment_start,
  const Eigen::Vector2d& segment_end,
  const Eigen::Vector2d& robot_xy,
  double max_distance_m)
{
  const Eigen::Vector2d segment_vector = segment_end - segment_start;
  const double segment_length_squared = segment_vector.squaredNorm();
  if (segment_length_squared < 1e-12) {
    return false;
  }
  const double along_parameter =
    (robot_xy - segment_start).dot(segment_vector) / segment_length_squared;
  if (along_parameter < 0.0 || along_parameter > 1.0) {
    return false;
  }
  const Eigen::Vector2d closest_point_on_segment =
    segment_start + along_parameter * segment_vector;
  return (robot_xy - closest_point_on_segment).norm() <= max_distance_m;
}

bool TebLocalPlannerROS::trySelectBestReferencePathFromList(
  const std::vector<nav_msgs::msg::Path>& reference_path_candidates,
  const nav_msgs::msg::Path& input_path,
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const double parallel_tolerance_degrees,
  const double distance_tolerance_meters,
  Eigen::Vector2d& selected_edge_segment_start,
  Eigen::Vector2d& selected_edge_segment_end,
  double& minimum_average_distance_to_plan,
  double& robot_perpendicular_distance_to_edge_line)
{
  bool found_any_matching_reference_segment = false;
  double best_minimum_average_distance_to_plan = std::numeric_limits<double>::max();
  Eigen::Vector2d best_reference_segment_start;
  Eigen::Vector2d best_reference_segment_end;
  double best_robot_perpendicular_distance_to_edge_line = 0.0;

  for (const auto& raw_reference_path : reference_path_candidates) {
    if (raw_reference_path.poses.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        logger_, *(clock_), 2000,
        "[trySelectBestReferencePathFromList] Edge following: 贴边参考线点数小于2, 点数: %zu",
        raw_reference_path.poses.size());
      continue;
    }
    // TF：用 TimePointZero 取缓冲区内最新可用变换，避免 now() 略超前于已发布 TF 导致外推失败。
    // 几何 header.stamp：有变换时用返回变换的时间戳与 TF 一致；已在 map 时用当前时钟刷新陈旧 latched stamp。
    nav_msgs::msg::Path reference_path_in_map_frame;
    reference_path_in_map_frame.header.frame_id = cfg_->map_frame;
    try {
      geometry_msgs::msg::PoseStamped reference_pose_start_in_map = raw_reference_path.poses.front();
      geometry_msgs::msg::PoseStamped reference_pose_end_in_map = raw_reference_path.poses.back();
      rclcpp::Time reference_geometry_stamp_in_map;
      if (!raw_reference_path.header.frame_id.empty() &&
          raw_reference_path.header.frame_id != cfg_->map_frame) {
        geometry_msgs::msg::TransformStamped transform_map_from_reference_path_frame =
          tf_->lookupTransform(
            cfg_->map_frame,
            raw_reference_path.header.frame_id,
            tf2::TimePointZero,
            tf2::durationFromSec(0.5));
        tf2::doTransform(reference_pose_start_in_map, reference_pose_start_in_map,
          transform_map_from_reference_path_frame);
        tf2::doTransform(reference_pose_end_in_map, reference_pose_end_in_map,
          transform_map_from_reference_path_frame);
        reference_geometry_stamp_in_map =
          rclcpp::Time(transform_map_from_reference_path_frame.header.stamp);
      } else {
        reference_geometry_stamp_in_map = clock_->now();
      }
      reference_pose_start_in_map.header.frame_id = cfg_->map_frame;
      reference_pose_end_in_map.header.frame_id = cfg_->map_frame;
      reference_pose_start_in_map.header.stamp = reference_geometry_stamp_in_map;
      reference_pose_end_in_map.header.stamp = reference_geometry_stamp_in_map;
      reference_path_in_map_frame.header.stamp = reference_geometry_stamp_in_map;
      reference_path_in_map_frame.poses.clear();
      reference_path_in_map_frame.poses.push_back(reference_pose_start_in_map);
      reference_path_in_map_frame.poses.push_back(reference_pose_end_in_map);

      const Eigen::Vector2d segment_start_map(
        reference_pose_start_in_map.pose.position.x,
        reference_pose_start_in_map.pose.position.y);
      const Eigen::Vector2d segment_end_map(
        reference_pose_end_in_map.pose.position.x,
        reference_pose_end_in_map.pose.position.y);
      const Eigen::Vector2d robot_xy(
        robot_pose.pose.position.x, robot_pose.pose.position.y);
      if (!robotOnReferenceSegmentWithinDistance(
            segment_start_map, segment_end_map, robot_xy, distance_tolerance_meters))
      {
        continue;
      }
    } catch (const tf2::TransformException& transform_exception) {
      RCLCPP_INFO_THROTTLE(
        logger_, *(clock_), 2000, "[trySelectBestReferencePathFromList] Edge following: 贴边参考线TF跳过: %s", 
        transform_exception.what());
      continue;
    }
    Eigen::Vector2d candidate_segment_start;
    Eigen::Vector2d candidate_segment_end;
    double candidate_minimum_average_distance = 0.0;
    double candidate_robot_perpendicular_distance = 0.0;
    if (trySelectSegmentFromTwoPointPath(
          reference_path_in_map_frame,
          input_path,
          robot_pose,
          parallel_tolerance_degrees,
          distance_tolerance_meters,
          candidate_segment_start,
          candidate_segment_end,
          candidate_minimum_average_distance,
          candidate_robot_perpendicular_distance)) {
      if (candidate_minimum_average_distance < best_minimum_average_distance_to_plan) {
        RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[trySelectBestReferencePathFromList] Edge following:"
                            "选择最佳贴边参考线平均距离: %.2f", candidate_minimum_average_distance);
        best_minimum_average_distance_to_plan = candidate_minimum_average_distance;
        best_reference_segment_start = candidate_segment_start;
        best_reference_segment_end = candidate_segment_end;
        best_robot_perpendicular_distance_to_edge_line = candidate_robot_perpendicular_distance;
        found_any_matching_reference_segment = true;
      }
    }
  }
  if (!found_any_matching_reference_segment) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[trySelectBestReferencePathFromList] Edge following:"
                         "没有找到合适的贴边参考线");
    return false;
  }
  selected_edge_segment_start = best_reference_segment_start;
  selected_edge_segment_end = best_reference_segment_end;
  minimum_average_distance_to_plan = best_minimum_average_distance_to_plan;
  robot_perpendicular_distance_to_edge_line = best_robot_perpendicular_distance_to_edge_line;
  return true;
}

double TebLocalPlannerROS::computeWallLineDistWeightFromRobotDistance(
  const double robot_perpendicular_distance_to_edge_line) const
{
  const double base_weight = weight_wall_line_dist_;
  const double min_dist = cfg_->wall_line.min_wall_dist;
  const double tolerance = cfg_->wall_line.distance_tolerance;
  const double delta = robot_perpendicular_distance_to_edge_line - min_dist;
  constexpr double k_eps = 1e-5;

  if (delta < -k_eps) {
    const double scale_lower = cfg_->wall_line.wall_line_dist_weight_scale_under_min_lower;
    const double scale_upper = cfg_->wall_line.wall_line_dist_weight_scale_under_min_upper;
    const double clamp_lo = std::min(scale_lower, scale_upper);
    const double clamp_hi = std::max(scale_lower, scale_upper);
    if (min_dist < k_eps) {
      return base_weight * clamp_lo;
    }
    const double ratio = robot_perpendicular_distance_to_edge_line / min_dist;
    const double scale = std::max(clamp_lo, std::min(clamp_hi, ratio));
    return base_weight * scale;
  }

  if (delta <= k_eps) {
    return base_weight;
  }

  const double band = tolerance - min_dist;
  if (std::fabs(band) <= k_eps) {
    return base_weight;
  }

  const double scale_at_tolerance = cfg_->wall_line.wall_line_dist_weight_scale_at_distance_tolerance;
  const double weight_at_tolerance = base_weight * scale_at_tolerance;
  const double weight_at_min = base_weight;
  const double interpolated =
    (weight_at_tolerance - weight_at_min) / band * delta + weight_at_min;
  const double weight_lo = std::min(weight_at_min, weight_at_tolerance);
  const double weight_hi = std::max(weight_at_min, weight_at_tolerance);
  return std::min(weight_hi, std::max(weight_lo, interpolated));
}

void TebLocalPlannerROS::applyWallLineSegmentAndVisual(
  const Eigen::Vector2d& wall_line_segment_start,
  const Eigen::Vector2d& wall_line_segment_end,
  const nav_msgs::msg::Path& input_path,
  const double minimum_average_distance_to_plan,
  const double robot_perpendicular_distance_to_edge_line)
{
  if (wall_line_points_.size() > 0) {
    wall_line_points_.clear();
  }
  wall_line_points_.push_back(wall_line_segment_start);
  wall_line_points_.push_back(wall_line_segment_end);
  cfg_->optim.weight_wall_line_dist =
    computeWallLineDistWeightFromRobotDistance(robot_perpendicular_distance_to_edge_line);
  RCLCPP_INFO_THROTTLE(
    logger_, *(clock_), 2000,
    "[applyWallLineSegmentAndVisual] Edge following: 机器人到贴边线距离 %.3f m, 贴边权重: %.2f",
    robot_perpendicular_distance_to_edge_line, cfg_->optim.weight_wall_line_dist);
  (void)minimum_average_distance_to_plan;
  wall_line_update_time_ = clock_->now();
  std_msgs::msg::Float32 edge_distance_message;
  edge_distance_message.data = static_cast<float>(robot_perpendicular_distance_to_edge_line);
  edge_distance_publisher_->publish(edge_distance_message);
  switchParameterMode(true);

  visualization_msgs::msg::Marker wall_line_marker_message;
  wall_line_marker_message.ns = "teb_local_planner";
  wall_line_marker_message.id = 0;
  wall_line_marker_message.type = visualization_msgs::msg::Marker::LINE_LIST;
  wall_line_marker_message.action = visualization_msgs::msg::Marker::ADD;
  wall_line_marker_message.scale.x = 0.1f;
  wall_line_marker_message.color.r = 1.0f;
  wall_line_marker_message.color.g = 1.0f;
  wall_line_marker_message.color.b = 0.0f;
  wall_line_marker_message.color.a = 1.0f;
  geometry_msgs::msg::Point marker_line_start_point;
  geometry_msgs::msg::Point marker_line_end_point;
  marker_line_start_point.x = wall_line_segment_start.x();
  marker_line_start_point.y = wall_line_segment_start.y();
  marker_line_start_point.z = 0.0;
  marker_line_end_point.x = wall_line_segment_end.x();
  marker_line_end_point.y = wall_line_segment_end.y();
  marker_line_end_point.z = 0.0;
  wall_line_marker_message.points.push_back(marker_line_start_point);
  wall_line_marker_message.points.push_back(marker_line_end_point);
  wall_line_marker_message.header.stamp = clock_->now();
  wall_line_marker_message.header.frame_id = input_path.header.frame_id;
  wall_line_marker_publisher_->publish(wall_line_marker_message);
}

void TebLocalPlannerROS::mergeFusionPrimaryWithReference(
  const Eigen::Vector2d& fusion_primary_segment_start,
  const Eigen::Vector2d& fusion_primary_segment_end,
  const bool fusion_primary_segment_valid,
  const double fusion_primary_minimum_average_distance_to_plan,
  const double fusion_primary_robot_perpendicular_distance_to_edge,
  const Eigen::Vector2d& reference_segment_start,
  const Eigen::Vector2d& reference_segment_end,
  const bool reference_segment_valid,
  const double reference_minimum_average_distance_to_plan,
  const double reference_robot_perpendicular_distance_to_edge,
  const nav_msgs::msg::Path& input_path)
{
  const rclcpp::Time current_clock_time = clock_->now();
  const double fusion_primary_lock_duration_seconds =
    cfg_->wall_line.fusion_primary_lock_duration;
  const double reference_fusion_maximum_angle_degrees =
    cfg_->wall_line.reference_match_max_angle_deg;
  const double reference_fusion_maximum_segment_distance_meters =
    cfg_->wall_line.reference_match_max_distance_m;

  if (!fusion_primary_segment_valid) {
    fusion_primary_lock_until_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  }

  const bool primary_and_reference_segments_geometrically_close =
    fusion_primary_segment_valid && reference_segment_valid &&
    segmentsCloseForFusion(
      fusion_primary_segment_start,
      fusion_primary_segment_end,
      reference_segment_start,
      reference_segment_end,
      reference_fusion_maximum_angle_degrees,
      reference_fusion_maximum_segment_distance_meters);
  if (primary_and_reference_segments_geometrically_close) {
    fusion_primary_lock_until_ =
      current_clock_time +
      rclcpp::Duration::from_seconds(fusion_primary_lock_duration_seconds);
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, 
                         "[mergeFusionPrimaryWithReference] Edge following:"
                         "贴边参考线和墙线或路沿特征接近");
  }

  bool use_fusion_primary_wall_or_curb_segment = false;
  if (fusion_primary_segment_valid) {
    if (primary_and_reference_segments_geometrically_close) {
      use_fusion_primary_wall_or_curb_segment = true;
    } else if (
      fusion_primary_lock_until_.nanoseconds() != 0u &&
      current_clock_time < fusion_primary_lock_until_) {
      use_fusion_primary_wall_or_curb_segment = true;
    } else if (!reference_segment_valid) {
      use_fusion_primary_wall_or_curb_segment = true;
    }
  }

  if (use_fusion_primary_wall_or_curb_segment) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, 
                         "[mergeFusionPrimaryWithReference] Edge following:"
                         "使用提取的墙线或是路沿作为贴边路径");
    applyWallLineSegmentAndVisual(
      fusion_primary_segment_start,
      fusion_primary_segment_end,
      input_path,
      fusion_primary_minimum_average_distance_to_plan,
      fusion_primary_robot_perpendicular_distance_to_edge);
    reference_line_hold_.clear();
    reference_line_hold_.push_back(fusion_primary_segment_start);
    reference_line_hold_.push_back(fusion_primary_segment_end);
    reference_line_hold_minimum_average_distance_to_plan_ = fusion_primary_minimum_average_distance_to_plan;
    reference_line_hold_robot_perpendicular_distance_to_edge_line_ = fusion_primary_robot_perpendicular_distance_to_edge;
    reference_line_last_success_time_ = current_clock_time;
    return;
  }

  if (reference_segment_valid) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, 
                         "[mergeFusionPrimaryWithReference] Edge following:"
                         "使用贴边参考线作为贴边路径");
    applyWallLineSegmentAndVisual(
      reference_segment_start,
      reference_segment_end,
      input_path,
      reference_minimum_average_distance_to_plan,
      reference_robot_perpendicular_distance_to_edge);
    reference_line_hold_.clear();
    reference_line_hold_.push_back(reference_segment_start);
    reference_line_hold_.push_back(reference_segment_end);
    reference_line_hold_minimum_average_distance_to_plan_ = reference_minimum_average_distance_to_plan;
    reference_line_hold_robot_perpendicular_distance_to_edge_line_ = reference_robot_perpendicular_distance_to_edge;
    reference_line_last_success_time_ = current_clock_time;
    return;
  }

  const double reference_no_valid_path_timeout_seconds =
    cfg_->wall_line.reference_no_valid_path_timeout;
  if (reference_line_hold_.size() == 2 &&
      reference_line_last_success_time_.nanoseconds() != 0u &&
      (current_clock_time - reference_line_last_success_time_).seconds() <
        reference_no_valid_path_timeout_seconds) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, 
                         "[mergeFusionPrimaryWithReference] Edge following:"
                         "使用缓存的贴边参考线作为贴边路径");
    applyWallLineSegmentAndVisual(
      reference_line_hold_[0],
      reference_line_hold_[1],
      input_path,
      reference_line_hold_minimum_average_distance_to_plan_,
      reference_line_hold_robot_perpendicular_distance_to_edge_line_);
    return;
  }

  if (wall_line_points_.size() > 0) {
    wall_line_points_.clear();
  }
  reference_line_hold_.clear();
  switchParameterMode(false);
  RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, 
                       "[mergeFusionPrimaryWithReference] Edge following:"
                       "未找到合适的贴边路径，清空缓存，退出贴边模式");
}

void TebLocalPlannerROS::applyReferenceLineHoldFallbackOrExit(
  const nav_msgs::msg::Path& input_path)
{
  const rclcpp::Time current_clock_time = clock_->now();
  const double reference_no_valid_path_timeout_seconds =
    cfg_->wall_line.reference_no_valid_path_timeout;
  if (reference_line_hold_.size() == 2 &&
      reference_line_last_success_time_.nanoseconds() != 0u &&
      (current_clock_time - reference_line_last_success_time_).seconds() <
        reference_no_valid_path_timeout_seconds) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000,
      "[applyReferenceLineHoldFallbackOrExit] Edge following: 使用缓存的参考路径作为贴边路径"
      "参考路径起点: (%.2f, %.2f), 参考路径终点: (%.2f, %.2f), 参考路径最小平均距离: %.2f, 参考路径与机器人垂直距离: %.2f",
      reference_line_hold_[0].x(), reference_line_hold_[0].y(),
      reference_line_hold_[1].x(), reference_line_hold_[1].y(),
      reference_line_hold_minimum_average_distance_to_plan_,
      reference_line_hold_robot_perpendicular_distance_to_edge_line_);
    applyWallLineSegmentAndVisual(
      reference_line_hold_[0],
      reference_line_hold_[1],
      input_path,
      reference_line_hold_minimum_average_distance_to_plan_,
      reference_line_hold_robot_perpendicular_distance_to_edge_line_);
    return;
  }
  if (wall_line_points_.size() > 0) {
    wall_line_points_.clear();
  }
  reference_line_hold_.clear();
  switchParameterMode(false);
}

void TebLocalPlannerROS::updateReferenceLineVec(
  const std::vector<nav_msgs::msg::Path>& reference_path_list,
  nav_msgs::msg::Path& input_path,
  const double parallel_tolerance_degrees,
  const double distance_tolerance_meters,
  const geometry_msgs::msg::PoseStamped& robot_pose)
{
  if (!edgeFollowingEntryGuards(robot_pose, input_path)) {
    RCLCPP_WARN(logger_, "[updateReferenceLineVec] Edge following:"
                "进入贴边模式检查失败");
    return;
  }
  Eigen::Vector2d selected_reference_segment_start;
  Eigen::Vector2d selected_reference_segment_end;
  double selected_minimum_average_distance_to_plan = 0.0;
  double selected_robot_perpendicular_distance_to_edge_line = 0.0;
  const bool reference_segment_selection_succeeded = trySelectBestReferencePathFromList(
    reference_path_list,
    input_path,
    robot_pose,
    parallel_tolerance_degrees,
    distance_tolerance_meters,
    selected_reference_segment_start,
    selected_reference_segment_end,
    selected_minimum_average_distance_to_plan,
    selected_robot_perpendicular_distance_to_edge_line);
  const rclcpp::Time current_clock_time = clock_->now();
  if (reference_segment_selection_succeeded) {
    RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "[updateReferenceLineVec] Edge following: 选择参考路径起点: (%.2f, %.2f),"
                        "参考路径终点: (%.2f, %.2f), 参考路径最小平均距离: %.2f, 参考路径与机器人垂直距离: %.2f",
                        selected_reference_segment_start.x(), selected_reference_segment_start.y(), 
                        selected_reference_segment_end.x(), selected_reference_segment_end.y(), 
                        selected_minimum_average_distance_to_plan, selected_robot_perpendicular_distance_to_edge_line);
    applyWallLineSegmentAndVisual(
      selected_reference_segment_start,
      selected_reference_segment_end,
      input_path,
      selected_minimum_average_distance_to_plan,
      selected_robot_perpendicular_distance_to_edge_line);
    reference_line_hold_.clear();
    reference_line_hold_.push_back(selected_reference_segment_start);
    reference_line_hold_.push_back(selected_reference_segment_end);
    reference_line_hold_minimum_average_distance_to_plan_ = selected_minimum_average_distance_to_plan;
    reference_line_hold_robot_perpendicular_distance_to_edge_line_ = selected_robot_perpendicular_distance_to_edge_line;
    reference_line_last_success_time_ = current_clock_time;
    return;
  }
  applyReferenceLineHoldFallbackOrExit(input_path);
}

void TebLocalPlannerROS::runEdgeFollowingPathUpdate(
  std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
  const geometry_msgs::msg::PoseStamped& robot_pose)
{
  if (cfg_->optim.weight_wall_line_dist <= 0.0) {
    return;
  }
  const std::string normalized_edge_mode_string = normalizeEdgeModeEnv(use_curb_or_wall);
  if (normalized_edge_mode_string.empty()) {
    return;
  }

  double accumulated_distance_meters_along_transformed_plan = 0.0;
  unsigned int input_path_end_pose_index_in_transformed_plan =
    static_cast<unsigned int>(transformed_plan.size() - 1);
  for (unsigned int segment_end_index = 1; segment_end_index < transformed_plan.size();
       segment_end_index++) {
    const double segment_delta_x =
      transformed_plan[segment_end_index].pose.position.x -
      transformed_plan[segment_end_index - 1].pose.position.x;
    const double segment_delta_y =
      transformed_plan[segment_end_index].pose.position.y -
      transformed_plan[segment_end_index - 1].pose.position.y;
    accumulated_distance_meters_along_transformed_plan +=
      std::sqrt(segment_delta_x * segment_delta_x + segment_delta_y * segment_delta_y);
    if (accumulated_distance_meters_along_transformed_plan >
        cfg_->wall_line.transform_path_line_length) {
      input_path_end_pose_index_in_transformed_plan = segment_end_index;
      break;
    }
  }
  nav_msgs::msg::Path input_path;
  input_path.header = transformed_plan.at(0).header;
  input_path.poses =
    std::vector<geometry_msgs::msg::PoseStamped>(transformed_plan.begin(),
      transformed_plan.begin() +
        static_cast<std::ptrdiff_t>(input_path_end_pose_index_in_transformed_plan + 1));
  transformed_path->publish(input_path);

  const bool edge_mode_includes_wall =
    (normalized_edge_mode_string.find("wall") != std::string::npos);
  const bool edge_mode_includes_curb =
    (normalized_edge_mode_string.find("curb") != std::string::npos);
  const bool edge_mode_includes_reference =
    (normalized_edge_mode_string.find("reference") != std::string::npos);

  if (edge_mode_includes_reference) {
    if (!shouldRunEdgeFollowingForTransformedPlan(input_path.poses)) {
      RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "Edge following: 不能找到路径靠近边缘, 退出贴边模式");
      if (wall_line_points_.size() > 0) {
        wall_line_points_.clear();
      }
      reference_line_hold_.clear();
      switchParameterMode(false);
      return;
    }
  }

  // paired_mission_and_reference_path：上层在进入导航前发布（TRANSIENT_LOCAL），
  // 此处不得因超时而清空缓存；几何与 TF 使用在 trySelectBestReferencePathFromList 中按当前时间刷新。

  if (normalized_edge_mode_string == "wall") {
    if (!wall_line_ptr_) {
      return;
    }
    const std::vector<nav_msgs::msg::Path> path_from_wall_line =
      wall_line_ptr_->get_compare_result(input_path);
    updateWallLineVec(
      path_from_wall_line,
      input_path,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      robot_pose);
    return;
  }
  if (normalized_edge_mode_string == "curb") {
    {
      std::lock_guard<std::mutex> curb_line_mutex_lock(curb_line_mutex_);
      if (curb_line_path_.poses.size() > 0) {
        const auto seconds_since_curb_path_update =
          clock_->now() - curb_line_update_time_;
        if (seconds_since_curb_path_update.seconds() > keep_wall_line_time_) {
          curb_line_path_.poses.clear();
        }
      }
    }
    updateCurbLineVec(
      curb_line_path_,
      input_path,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      robot_pose);
    return;
  }
  if (normalized_edge_mode_string == "reference") {
    const std::vector<nav_msgs::msg::Path> active_reference_path_list =
      snapshotActiveReferencePathList();
    updateReferenceLineVec(
      active_reference_path_list,
      input_path,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      robot_pose);
    return;
  }
  if (edge_mode_includes_wall && edge_mode_includes_reference && !edge_mode_includes_curb) {
    if (!wall_line_ptr_) {
      std::vector<nav_msgs::msg::Path> active_reference_path_list =
        snapshotActiveReferencePathList();
      Eigen::Vector2d selected_reference_segment_start;
      Eigen::Vector2d selected_reference_segment_end;
      double reference_minimum_average_distance_to_plan = 0.0;
      double reference_robot_perpendicular_distance_to_edge_line = 0.0;
      if (!edgeFollowingEntryGuards(robot_pose, input_path)) {
        return;
      }
      const bool reference_segment_selection_succeeded = trySelectBestReferencePathFromList(
        active_reference_path_list,
        input_path,
        robot_pose,
        cfg_->wall_line.parallel_tolerance,
        cfg_->wall_line.distance_tolerance,
        selected_reference_segment_start,
        selected_reference_segment_end,
        reference_minimum_average_distance_to_plan,
        reference_robot_perpendicular_distance_to_edge_line);
      if (reference_segment_selection_succeeded) {
        applyWallLineSegmentAndVisual(
          selected_reference_segment_start,
          selected_reference_segment_end,
          input_path,
          reference_minimum_average_distance_to_plan,
          reference_robot_perpendicular_distance_to_edge_line);
        reference_line_hold_.clear();
        reference_line_hold_.push_back(selected_reference_segment_start);
        reference_line_hold_.push_back(selected_reference_segment_end);
        reference_line_hold_minimum_average_distance_to_plan_ = reference_minimum_average_distance_to_plan;
        reference_line_hold_robot_perpendicular_distance_to_edge_line_ = reference_robot_perpendicular_distance_to_edge_line;
        reference_line_last_success_time_ = clock_->now();
      } else {
        applyReferenceLineHoldFallbackOrExit(input_path);
      }
      return;
    }
    if (!edgeFollowingEntryGuards(robot_pose, input_path)) {
      return;
    }
    const std::vector<nav_msgs::msg::Path> wall_line_candidate_paths =
      wall_line_ptr_->get_compare_result(input_path);
    Eigen::Vector2d fusion_primary_segment_start;
    Eigen::Vector2d fusion_primary_segment_end;
    double fusion_primary_minimum_average_distance_to_plan = 0.0;
    double fusion_primary_robot_perpendicular_distance_to_edge = 0.0;
    const bool fusion_primary_wall_segment_valid = trySelectWallSegmentFromCandidates(
      wall_line_candidate_paths,
      input_path,
      robot_pose,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      fusion_primary_segment_start,
      fusion_primary_segment_end,
      fusion_primary_minimum_average_distance_to_plan,
      fusion_primary_robot_perpendicular_distance_to_edge);
    std::vector<nav_msgs::msg::Path> active_reference_path_list = snapshotActiveReferencePathList();
    Eigen::Vector2d fusion_reference_segment_start;
    Eigen::Vector2d fusion_reference_segment_end;
    double fusion_reference_minimum_average_distance_to_plan = 0.0;
    double fusion_reference_robot_perpendicular_distance_to_edge = 0.0;
    const bool fusion_reference_segment_valid = trySelectBestReferencePathFromList(
      active_reference_path_list,
      input_path,
      robot_pose,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      fusion_reference_segment_start,
      fusion_reference_segment_end,
      fusion_reference_minimum_average_distance_to_plan,
      fusion_reference_robot_perpendicular_distance_to_edge);
    mergeFusionPrimaryWithReference(
      fusion_primary_segment_start,
      fusion_primary_segment_end,
      fusion_primary_wall_segment_valid,
      fusion_primary_minimum_average_distance_to_plan,
      fusion_primary_robot_perpendicular_distance_to_edge,
      fusion_reference_segment_start,
      fusion_reference_segment_end,
      fusion_reference_segment_valid,
      fusion_reference_minimum_average_distance_to_plan,
      fusion_reference_robot_perpendicular_distance_to_edge,
      input_path);
    return;
  }
  if (edge_mode_includes_curb && edge_mode_includes_reference && !edge_mode_includes_wall) {
    {
      std::lock_guard<std::mutex> curb_line_mutex_lock(curb_line_mutex_);
      if (curb_line_path_.poses.size() > 0) {
        const auto seconds_since_curb_path_update =
          clock_->now() - curb_line_update_time_;
        if (seconds_since_curb_path_update.seconds() > keep_wall_line_time_) {
          curb_line_path_.poses.clear();
        }
      }
    }
    if (!edgeFollowingEntryGuards(robot_pose, input_path)) {
      RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000, "[runEdgeFollowingPathUpdate] Edge following:" 
                          "贴边检查失败, 退出贴边模式");
      return;
    }
    nav_msgs::msg::Path curb_line_path_snapshot;
    {
      std::lock_guard<std::mutex> curb_line_mutex_lock(curb_line_mutex_);
      curb_line_path_snapshot = curb_line_path_;
    }
    Eigen::Vector2d fusion_primary_curb_segment_start;
    Eigen::Vector2d fusion_primary_curb_segment_end;
    double fusion_primary_curb_minimum_average_distance_to_plan = 0.0;
    double fusion_primary_curb_robot_perpendicular_distance_to_edge = 0.0;
    const bool fusion_primary_curb_segment_valid = trySelectSegmentFromTwoPointPath(
      curb_line_path_snapshot,
      input_path,
      robot_pose,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      fusion_primary_curb_segment_start,
      fusion_primary_curb_segment_end,
      fusion_primary_curb_minimum_average_distance_to_plan,
      fusion_primary_curb_robot_perpendicular_distance_to_edge);
    std::vector<nav_msgs::msg::Path> active_reference_path_list = snapshotActiveReferencePathList();
    Eigen::Vector2d curb_fusion_reference_segment_start;
    Eigen::Vector2d curb_fusion_reference_segment_end;
    double curb_fusion_reference_minimum_average_distance_to_plan = 0.0;
    double curb_fusion_reference_robot_perpendicular_distance_to_edge = 0.0;
    const bool curb_fusion_reference_segment_valid = trySelectBestReferencePathFromList(
      active_reference_path_list,
      input_path,
      robot_pose,
      cfg_->wall_line.parallel_tolerance,
      cfg_->wall_line.distance_tolerance,
      curb_fusion_reference_segment_start,
      curb_fusion_reference_segment_end,
      curb_fusion_reference_minimum_average_distance_to_plan,
      curb_fusion_reference_robot_perpendicular_distance_to_edge);
    mergeFusionPrimaryWithReference(
      fusion_primary_curb_segment_start,
      fusion_primary_curb_segment_end,
      fusion_primary_curb_segment_valid,
      fusion_primary_curb_minimum_average_distance_to_plan,
      fusion_primary_curb_robot_perpendicular_distance_to_edge,
      curb_fusion_reference_segment_start,
      curb_fusion_reference_segment_end,
      curb_fusion_reference_segment_valid,
      curb_fusion_reference_minimum_average_distance_to_plan,
      curb_fusion_reference_robot_perpendicular_distance_to_edge,
      input_path);
    return;
  }

  RCLCPP_WARN_THROTTLE(
    logger_,
    *(clock_),
    5000,
    "USE_CURB_OR_WALL=\"%s\" (normalized \"%s\") is not a supported edge mode",
    use_curb_or_wall != nullptr ? use_curb_or_wall : "(null)",
    normalized_edge_mode_string.c_str());
}

 } // end namespace teb_local_planner
 
 // register this planner as a nav2_core::Controller plugin
 PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav2_core::Controller)
 