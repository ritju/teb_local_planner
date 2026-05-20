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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/
#ifndef EDGE_DISTANCE_TO_MULTI_CURVE_H_
#define EDGE_DISTANCE_TO_MULTI_CURVE_H_

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "capella_ros_msg/msg/multi_curve.hpp"
#include "capella_ros_msg/msg/curve_segment.hpp"
#include "capella_ros_msg/msg/line_segment.hpp"
#include "capella_ros_msg/msg/ellipse_arc_segment.hpp"

#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/misc.h"
#include "g2o/core/base_unary_edge.h"

namespace teb_local_planner
{

// ============================================================
// 距离工具函数（teb_local_planner 内其他类可直接调用）
// ============================================================

/// 点到线段的平方距离（比较时避免 sqrt）
inline double pointToSegmentDistSq(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b)
{
  const Eigen::Vector3d ab = b - a;
  const double len2 = ab.squaredNorm();
  if (len2 < 1e-12) return (p - a).squaredNorm();
  const double t = std::clamp((p - a).dot(ab) / len2, 0.0, 1.0);
  return (p - (a + t * ab)).squaredNorm();
}

/// 在无约束椭圆 x=a*cos(t), y=b*sin(t) 上求离 (px,py) 最近的参数 t
/// Eberly Newton 迭代，折叠到第一象限，通常不超过 8 次收敛
inline double ellipseNearestAngle(
    double a, double b,
    double px, double py,
    int max_iter = 8)
{
  const double sx = (px >= 0.0) ? 1.0 : -1.0;
  const double sy = (py >= 0.0) ? 1.0 : -1.0;
  px = std::abs(px);
  py = std::abs(py);

  double t = std::atan2(a * py, b * px);
  for (int i = 0; i < max_iter; ++i) {
    const double ct = std::cos(t), st = std::sin(t);
    const double fx = a * ct - px, fy = b * st - py;
    const double F  = -a * fx * st + b * fy * ct;
    const double dF = -a * (fx * ct - a * st * st)
                      + b * (fy * (-st) + b * ct * ct);
    if (std::abs(dF) < 1e-14) break;
    const double dt = F / dF;
    t -= dt;
    if (std::abs(dt) < 1e-9) break;
  }
  return std::atan2(sy * b * std::abs(std::sin(t)),
                    sx * a * std::abs(std::cos(t)));
}

/// 将 theta 夹入弧段角度范围（支持 ccw / cw）
inline double clampAngleToArc(
    double theta, double t_start, double t_end, bool is_ccw)
{
  // 从 t_start 出发沿行进方向计算进度（结果 >= 0）
  auto progress = [&](double ang) -> double {
    double d = ang - t_start;
    if (is_ccw) { while (d < 0.0) d += 2 * M_PI; }
    else        { while (d > 0.0) d -= 2 * M_PI; }
    return d;
  };
  const double span  = progress(t_end);
  const double delta = progress(theta);

  const bool out_of_range = is_ccw ? (delta < 0.0 || delta > span)
                                   : (delta > 0.0 || delta < span);
  if (!out_of_range) return theta;

  // 用最短弧长距离选择更近的端点，避免绕转后比较错误
  auto shortAngleDist = [](double a, double b) -> double {
    double d = std::abs(a - b);
    while (d > M_PI) d = std::abs(d - 2 * M_PI);
    return d;
  };
  return (shortAngleDist(theta, t_start) <= shortAngleDist(theta, t_end))
         ? t_start : t_end;
}

/// 点到 EllipseArcSegment 的最近距离
inline double distToEllipseArc(
    const Eigen::Vector3d& p,
    const capella_ros_msg::msg::EllipseArcSegment& seg)
{
  const auto& q = seg.frame.orientation;
  const auto& o = seg.frame.position;
  const Eigen::Matrix3d R =
      Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
  const Eigen::Vector3d p_local =
      R.transpose() * (p - Eigen::Vector3d(o.x, o.y, o.z));

  double theta = ellipseNearestAngle(seg.a, seg.b, p_local.x(), p_local.y());
  theta = clampAngleToArc(theta, seg.theta_start, seg.theta_end, seg.is_ccw);

  const Eigen::Vector3d closest_local(
      seg.a * std::cos(theta), seg.b * std::sin(theta), 0.0);
  return (p - (R * closest_local + Eigen::Vector3d(o.x, o.y, o.z))).norm();
}

/// 点到 MultiCurve 所有段的最小距离
/// 零动态分配；线段全程用 squaredNorm 比较，仅最优段才开根
inline double minDistToMultiCurve(
    const Eigen::Vector3d& p,
    const capella_ros_msg::msg::MultiCurve& curve)
{
  double best_sq  = std::numeric_limits<double>::max();
  double best_arc = std::numeric_limits<double>::max();

  for (const auto& seg : curve.segments) {
    if (seg.type == capella_ros_msg::msg::CurveSegment::LINE) {
      const auto& ls = seg.line_segment;
      const double dsq = pointToSegmentDistSq(
          p,
          Eigen::Vector3d(ls.start_point.x, ls.start_point.y, ls.start_point.z),
          Eigen::Vector3d(ls.end_point.x,   ls.end_point.y,   ls.end_point.z));
      if (dsq < best_sq) best_sq = dsq;

    } else if (seg.type == capella_ros_msg::msg::CurveSegment::ELLIPSE_ARC) {
      const double d = distToEllipseArc(p, seg.ellipse_arc_segment);
      if (d < best_arc) best_arc = d;
    }
  }

  const double best_line = (best_sq < std::numeric_limits<double>::max())
                           ? std::sqrt(best_sq)
                           : std::numeric_limits<double>::max();
  return std::min(best_line, best_arc);
}

// ============================================================
// g2o Edge
// ============================================================

/**
 * @class EdgeDistanceToMultiCurve
 * @brief 约束路径点到 MultiCurve 的最近距离；信息矩阵权重可按路径点与机器人
 *        （轨迹首点）距离线性缩放，见 distanceWeightScale().
 */
class EdgeDistanceToMultiCurve
  : public BaseTebUnaryEdge<1, const capella_ros_msg::msg::MultiCurve*, VertexPose>
{
public:
  /**
   * @brief 贴线距离项权重系数：在距离首点 [0,R] 内从 k_min 线性增至 k_max，
   *        d>=R 时恒为 k_max；R<=0 时关闭渐变，恒返回 k_max
   */
  static double distanceWeightScale(const TebConfig& cfg,
                                    double distance_from_robot_pose_m)
  {
    const double R     = cfg.wall_line.wall_line_dist_robot_weight_radius;
    const double k_min = cfg.wall_line.wall_line_dist_weight_scale_at_robot;
    const double k_max = cfg.wall_line.wall_line_dist_weight_scale_far;
    if (R <= 1e-9) return k_max;
    const double t = std::clamp(distance_from_robot_pose_m / R, 0.0, 1.0);
    return k_min + (k_max - k_min) * t;
  }

  EdgeDistanceToMultiCurve()
  {
    _measurement = nullptr;
  }

  /// Actual cost function
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_,
      "You must call setParameters() on EdgeDistanceToMultiCurve()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    if (_measurement && !_measurement->segments.empty()) {
      const Eigen::Vector3d p(bandpt->position().x(), bandpt->position().y(), 0.0);
      const double dist = minDistToMultiCurve(p, *_measurement);
      _error[0] = std::fabs(dist - cfg_->wall_line.min_wall_dist);
    } else {
      _error[0] = 0.0;
    }

    TEB_ASSERT_MSG(std::isfinite(_error[0]),
      "EdgeDistanceToMultiCurve::computeError() _error[0]=%f\n", _error[0]);
  }

  /**
   * @brief Set all parameters at once
   * @param cfg    TebConfig class
   * @param curve  指向 MultiCurve 消息的指针（生命周期由调用方保证）
   */
  void setParameters(const TebConfig& cfg,
                     const capella_ros_msg::msg::MultiCurve* curve)
  {
    cfg_         = &cfg;
    _measurement = curve;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace teb_local_planner

#endif  // EDGE_DISTANCE_TO_MULTI_CURVE_H_