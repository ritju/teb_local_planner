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

/// 点到线段的平方距离（2D，比较时避免 sqrt）
inline double pointToSegmentDistSq(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b)
{
  const Eigen::Vector2d ab = b - a;
  const double len2 = ab.squaredNorm();
  if (len2 < 1e-12) return (p - a).squaredNorm();
  const double t = std::clamp((p - a).dot(ab) / len2, 0.0, 1.0);
  return (p - (a + t * ab)).squaredNorm();
}

/// 在无约束椭圆 x=a*cos(t), y=b*sin(t) 上求离 (px,py) 最近的参数 t
/// Eberly Newton 迭代，折叠到第一象限
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

  auto shortAngleDist = [](double a, double b) -> double {
    double d = std::abs(a - b);
    while (d > M_PI) d = std::abs(d - 2 * M_PI);
    return d;
  };
  return (shortAngleDist(theta, t_start) <= shortAngleDist(theta, t_end))
         ? t_start : t_end;
}

/// 点到 EllipseArcSegment 的最近距离（2D）
inline double distToEllipseArc(
    const Eigen::Vector2d& p,
    const capella_ros_msg::msg::EllipseArcSegment& seg)
{
  const auto& q = seg.frame.orientation;
  const auto& o = seg.frame.position;

  // 从四元数提取 2D 旋转矩阵（只取 XY 平面部分）
  const Eigen::Matrix2d R =
    Eigen::Quaterniond(q.w, q.x, q.y, q.z)
        .toRotationMatrix()
        .topLeftCorner<2, 2>();

  const Eigen::Vector2d origin(o.x, o.y);
  const Eigen::Vector2d p_local = R.transpose() * (p - origin);

  double theta = ellipseNearestAngle(seg.a, seg.b, p_local.x(), p_local.y());
  theta = clampAngleToArc(theta, seg.theta_start, seg.theta_end, seg.is_ccw);

  const Eigen::Vector2d closest_local(
      seg.a * std::cos(theta), seg.b * std::sin(theta));
  return (p - (R * closest_local + origin)).norm();
}

/// 点到 MultiCurve 所有段的最小距离（2D）
/// 线段全程用 squaredNorm 比较，仅最优段才开根
inline double minDistToMultiCurve(
    const Eigen::Vector2d& p,
    const capella_ros_msg::msg::MultiCurve& curve)
{
  double best_sq  = std::numeric_limits<double>::max();
  double best_arc = std::numeric_limits<double>::max();

  for (const auto& seg : curve.segments) {
    if (seg.type == capella_ros_msg::msg::CurveSegment::LINE) {
      const auto& ls = seg.line_segment;
      const double dsq = pointToSegmentDistSq(
          p,
          Eigen::Vector2d(ls.start_point.x, ls.start_point.y),
          Eigen::Vector2d(ls.end_point.x,   ls.end_point.y));
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

class EdgeDistanceToMultiCurve
  : public BaseTebUnaryEdge<1, const capella_ros_msg::msg::MultiCurve*, VertexPose>
{
public:
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

  void computeError()
  {
    TEB_ASSERT_MSG(cfg_,
      "You must call setParameters() on EdgeDistanceToMultiCurve()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    if (_measurement && !_measurement->segments.empty()) {
      const Eigen::Vector2d p(bandpt->position().x(), bandpt->position().y());
      const double dist = minDistToMultiCurve(p, *_measurement);
      _error[0] = std::fabs(dist - cfg_->wall_line.min_wall_dist);
    } else {
      _error[0] = 0.0;
    }

    TEB_ASSERT_MSG(std::isfinite(_error[0]),
      "EdgeDistanceToMultiCurve::computeError() _error[0]=%f\n", _error[0]);
  }

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