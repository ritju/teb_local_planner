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

inline void sampleLineSegment(
    const capella_ros_msg::msg::LineSegment& seg,
    double step,
    std::vector<Eigen::Vector2d>& out)
{
  const Eigen::Vector2d sp(seg.start_point.x, seg.start_point.y);
  const Eigen::Vector2d ep(seg.end_point.x, seg.end_point.y);

  const Eigen::Vector2d diff = ep - sp;
  const double length = diff.norm();

  if (length < 1e-9)
  {
    out.push_back(sp);
    return;
  }

  const Eigen::Vector2d dir = diff / length;

  const int sample_num =
      std::max(1, static_cast<int>(std::ceil(length / step)));

  out.reserve(out.size() + sample_num + 1);

  for (int i = 0; i <= sample_num; ++i)
  {
    const double s = std::min(i * step, length);
    out.push_back(sp + dir * s);
  }
}

/**
 * ds/dθ
 */
inline double ellipseArcDsDTheta(
    double a,
    double b,
    double theta)
{
  const double s = a * std::sin(theta);
  const double c = b * std::cos(theta);

  return std::sqrt(s * s + c * c);
}

/**
 * 实时友好的椭圆弧采样
 * 核心思想：
 *   dθ = ds / (ds/dθ)
 * 即：
 *   θ += step / ds_dtheta
 */
inline void sampleEllipseArcSegment(
    const capella_ros_msg::msg::EllipseArcSegment& seg,
    double step,
    std::vector<Eigen::Vector2d>& out)
{
  const auto& q = seg.frame.orientation;
  const auto& o = seg.frame.position;

  const Eigen::Matrix2d R =
      Eigen::Quaterniond(q.w, q.x, q.y, q.z)
          .toRotationMatrix()
          .topLeftCorner<2,2>();

  const Eigen::Vector2d origin(o.x, o.y);

  const double a = seg.a;
  const double b = seg.b;

  double theta = seg.theta_start;
  double theta_end = seg.theta_end;

  // 1. 按 is_ccw 展开跨零边界（决定取哪段弧）
  if (seg.is_ccw && theta_end <= theta)
    theta_end += 2 * M_PI;

  if (!seg.is_ccw && theta_end >= theta)
    theta_end -= 2 * M_PI;

  // 2. 计算两端点世界坐标
  const Eigen::Vector2d p_theta_start =
      R * Eigen::Vector2d(a * std::cos(theta), b * std::sin(theta)) + origin;
  const Eigen::Vector2d p_theta_end =
      R * Eigen::Vector2d(a * std::cos(theta_end), b * std::sin(theta_end)) + origin;

  const Eigen::Vector2d seg_start(seg.start_point.x, seg.start_point.y);

  // 3. 若 theta 对应点离 seg.start_point 更远，则从 theta_end 端反向采样
  if ((p_theta_start - seg_start).squaredNorm() > (p_theta_end   - seg_start).squaredNorm())
  {
    std::swap(theta, theta_end);
  }

  // 方向
  const double sign = (theta_end >= theta) ? 1.0 : -1.0;

  // 防止死循环
  constexpr double kMinDTheta = 1e-5;

  // 预估采样数量（减少 realloc）
  const double approx_radius = std::max(a, b);
  const double approx_arc =
      std::abs(theta_end - theta) * approx_radius;

  const int reserve_num =
      std::max(8, static_cast<int>(approx_arc / step) + 2);

  out.reserve(out.size() + reserve_num);
  auto appendPoint = [&](double t)
  {
    const Eigen::Vector2d local(
        a * std::cos(t),
        b * std::sin(t));

    out.push_back(R * local + origin);
  };

  // 起点
  appendPoint(theta);

  constexpr int kMaxIter = 10000;
  int iter = 0;
  while (true)
  {
    const double ds_dtheta = ellipseArcDsDTheta(a, b, theta);

    // 避免极小值
    const double safe_ds_dtheta =
        std::max(ds_dtheta, 1e-6);

    double dtheta =
        sign * step / safe_ds_dtheta;

    // 防止极小步长
    if (std::abs(dtheta) < kMinDTheta)
      dtheta = sign * kMinDTheta;

    double next_theta = theta + dtheta;

    // 是否到终点
    bool reach_end = false;

    if (sign > 0.0)
    {
      if (next_theta >= theta_end)
      {
        next_theta = theta_end;
        reach_end = true;
      }
    }
    else
    {
      if (next_theta <= theta_end)
      {
        next_theta = theta_end;
        reach_end = true;
      }
    }

    appendPoint(next_theta);

    theta = next_theta;

    if (reach_end)
      break;
    
    if (++iter > kMaxIter)
      break;
  }
}

inline std::vector<Eigen::Vector2d> sampleMultiCurve(
    const capella_ros_msg::msg::MultiCurve& curve,
    double step = 0.1)
{
  std::vector<Eigen::Vector2d> all;

  for (const auto& seg : curve.segments)
  {
    std::vector<Eigen::Vector2d> pts;

    if (seg.type ==
        capella_ros_msg::msg::CurveSegment::LINE)
    {
      sampleLineSegment(
          seg.line_segment,
          step,
          pts);
    }
    else if (
        seg.type ==
        capella_ros_msg::msg::CurveSegment::ELLIPSE_ARC)
    {
      sampleEllipseArcSegment(
          seg.ellipse_arc_segment,
          step,
          pts);
    }
    else
    {
      continue;
    }

    // 去除重复点
    if (!all.empty() && !pts.empty())
      pts.erase(pts.begin());

    all.insert(
        all.end(),
        pts.begin(),
        pts.end());
  }

  return all;
}

/// 计算两组MultiCurve的质心距离
inline double multiCurveCentroidDist(
    const capella_ros_msg::msg::MultiCurve& c1,
    const capella_ros_msg::msg::MultiCurve& c2,
    double step = 0.1)
{
  const auto pts1 = sampleMultiCurve(c1, step);
  const auto pts2 = sampleMultiCurve(c2, step);

  if (pts1.empty() || pts2.empty())
    return std::numeric_limits<double>::max();

  Eigen::Vector2d cen1 =
      Eigen::Vector2d::Zero();

  Eigen::Vector2d cen2 =
      Eigen::Vector2d::Zero();

  for (const auto& p : pts1)
    cen1 += p;

  for (const auto& p : pts2)
    cen2 += p;

  cen1 /= static_cast<double>(pts1.size());
  cen2 /= static_cast<double>(pts2.size());

  return (cen1 - cen2).norm();
}

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

inline double ellipseNearestAngle(
    double a, double b,
    double px, double py,
    int max_iter = 8)
{
    if (a <= 0.0 || b <= 0.0)
        return 0.0;

    const double sx = (px >= 0.0) ? 1.0 : -1.0;
    const double sy = (py >= 0.0) ? 1.0 : -1.0;

    px = std::abs(px);
    py = std::abs(py);

    if (px < 1e-12 && py < 1e-12)
        return 0.0;

    double t = std::atan2(a * py, b * px);

    for (int i = 0; i < max_iter; ++i)
    {
        const double ct = std::cos(t);
        const double st = std::sin(t);

        const double ex = a * ct;
        const double ey = b * st;

        const double rx = ex - px;
        const double ry = ey - py;

        const double F =
            -a * st * rx +
             b * ct * ry;

        const double dF =
            a * a * st * st +
            b * b * ct * ct -
            a * ct * rx -
            b * st * ry;

        if (std::abs(dF) < 1e-14)
            break;

        const double dt = F / dF;
        t -= dt;
        if (std::abs(dt) < 1e-12)
          break;
    }

    double result = std::atan2(sy * std::sin(t), sx * std::cos(t));

    return result;
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

/// 计算点到 MultiCurve 最小距离，及返回的运行切线方向 dir,最近点 out_closet_pt 距离无穷大时未获取到
inline double minDistToMultiCurveWithDir(
    const Eigen::Vector2d& input_pt,
    Eigen::Vector2d& out_closet_pt, 
    Eigen::Vector2d& dir,
    const capella_ros_msg::msg::MultiCurve& multi_curve)
{
  double min_path_dist_sq = std::numeric_limits<double>::max();
  bool found = false;
  for (const auto& seg : multi_curve.segments) {
    if (seg.type == capella_ros_msg::msg::CurveSegment::LINE) {
      const auto& ls = seg.line_segment;
      const Eigen::Vector2d a(ls.start_point.x, ls.start_point.y);
      const Eigen::Vector2d b(ls.end_point.x,   ls.end_point.y);

      const Eigen::Vector2d ab = b - a;
      const double len2 = ab.squaredNorm();

      Eigen::Vector2d closest;
      if (len2 < 1e-12) {
        closest = a;
      } else {
        const double t = std::clamp((input_pt - a).dot(ab) / len2, 0.0, 1.0);
        closest = a + t * ab;
      }

      const double dsq = (input_pt - closest).squaredNorm();
      if (dsq < min_path_dist_sq) {
        min_path_dist_sq = dsq;
        out_closet_pt = closest;
        found = true;

        dir = (len2 > 1e-24) ? (ab / std::sqrt(len2))
                : Eigen::Vector2d(1.0, 0.0);
      }

    // ── 椭圆弧 ──────────────────────────────────────────────────────────────
    } else if (seg.type == capella_ros_msg::msg::CurveSegment::ELLIPSE_ARC) {
      const auto& eseg = seg.ellipse_arc_segment;
      const auto& q    = eseg.frame.orientation;
      const auto& o    = eseg.frame.position;

      const Eigen::Matrix2d R =
          Eigen::Quaterniond(q.w, q.x, q.y, q.z)
              .toRotationMatrix()
              .topLeftCorner<2, 2>();

      const Eigen::Vector2d origin(o.x, o.y);
      const Eigen::Vector2d p_local = R.transpose() * (input_pt - origin);

      double theta = ellipseNearestAngle(eseg.a, eseg.b, p_local.x(), p_local.y());
      theta = clampAngleToArc(theta, eseg.theta_start, eseg.theta_end, eseg.is_ccw);
      
      // 最近点(local)
      const Eigen::Vector2d closest_local(eseg.a * std::cos(theta), eseg.b * std::sin(theta));
      // 最近点(world)
      const Eigen::Vector2d closest = R * closest_local + origin;
      const double dsq = (input_pt - closest).squaredNorm();

      if (dsq < min_path_dist_sq) {
        min_path_dist_sq = dsq;
        out_closet_pt = closest;
        found = true;

        // 椭圆切线（局部坐标）：d/dθ (a·cosθ, b·sinθ) = (-a·sinθ, b·cosθ) 逆时针方向
        Eigen::Vector2d tangent_local(-eseg.a * std::sin(theta), eseg.b * std::cos(theta));
        if (!eseg.is_ccw) tangent_local = -tangent_local;
        Eigen::Vector2d tangent_world = R * tangent_local;

        // const Eigen::Vector2d theta_start_local(eseg.a * std::cos(eseg.theta_start), eseg.b * std::sin(eseg.theta_start));
        // const Eigen::Vector2d theta_end_local(eseg.a * std::cos(eseg.theta_end), eseg.b * std::sin(eseg.theta_end));

        // const Eigen::Vector2d theta_start_world = R * theta_start_local + origin;
        // const Eigen::Vector2d theta_end_world = R * theta_end_local + origin;

        // const Eigen::Vector2d real_start(eseg.start_point.x, eseg.start_point.y);

        // const double d_start_theta_start = (real_start - theta_start_world).squaredNorm();
        // const double d_start_theta_end = (real_start - theta_end_world).squaredNorm();

        // const bool theta_increasing_is_forward = d_start_theta_start < d_start_theta_end;
        // if (!theta_increasing_is_forward)
        // {
        //   tangent_world = -tangent_world;
        // }

        const double tlen = tangent_world.norm();
        dir = (tlen > 1e-12) ? (tangent_world / tlen) : Eigen::Vector2d(1.0, 0.0);
      }
    }
  }
  return found ? std::sqrt(min_path_dist_sq) : std::numeric_limits<double>::infinity();
};

/// 前后做切线延长
inline capella_ros_msg::msg::MultiCurve extendMultiCurve(const capella_ros_msg::msg::MultiCurve& input, double extend_length)
{
  capella_ros_msg::msg::MultiCurve output = input;
  if (input.segments.empty())
    return output;

  auto safeNormalize = [](const Eigen::Vector2d& v) -> Eigen::Vector2d
  {
    double n = v.norm();
    if (n < 1e-9)
      return Eigen::Vector2d(1.0, 0.0);
    return v / n;
  };
  
  auto makePoint = [](double x, double y) -> geometry_msgs::msg::Point
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    return p;
  };

  // 前延长
  {
    const auto& first_seg = input.segments.front();

    Eigen::Vector2d start_pt;
    Eigen::Vector2d tangent;

    if (first_seg.type == capella_ros_msg::msg::CurveSegment::LINE)
    {
      const auto& line = first_seg.line_segment;
      Eigen::Vector2d s(line.start_point.x, line.start_point.y);
      Eigen::Vector2d e(line.end_point.x, line.end_point.y);
      start_pt = s;
      tangent  = safeNormalize(e - s);
    }
    else if (first_seg.type == capella_ros_msg::msg::CurveSegment::ELLIPSE_ARC)
    {
      const auto& arc = first_seg.ellipse_arc_segment;
      const auto& q   = arc.frame.orientation;
      const Eigen::Matrix2d R = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix().topLeftCorner<2, 2>();

      Eigen::Vector2d local_tangent(-arc.a * std::sin(arc.theta_start), arc.b * std::cos(arc.theta_start));
      start_pt = Eigen::Vector2d(arc.start_point.x, arc.start_point.y);

      Eigen::Vector2d world_tangent = safeNormalize(R * local_tangent);
      tangent = arc.is_ccw ? world_tangent : -world_tangent;
    }
    else
    {
      tangent = Eigen::Vector2d(1.0, 0.0);
    }

    Eigen::Vector2d ext_start = start_pt - tangent * extend_length;
    capella_ros_msg::msg::CurveSegment seg;
    seg.type = capella_ros_msg::msg::CurveSegment::LINE;

    seg.line_segment.start_point = makePoint(ext_start.x(), ext_start.y());
    seg.line_segment.end_point = makePoint(start_pt.x(), start_pt.y());

    output.segments.insert(output.segments.begin(), seg);
  }

  // 后延长
  {
    const auto& last_seg = input.segments.back();

    Eigen::Vector2d end_pt;
    Eigen::Vector2d tangent;

    if (last_seg.type == capella_ros_msg::msg::CurveSegment::LINE)
    {
      const auto& line = last_seg.line_segment;
      Eigen::Vector2d s(line.start_point.x, line.start_point.y);
      Eigen::Vector2d e(line.end_point.x, line.end_point.y);
      end_pt  = e;
      tangent = safeNormalize(e - s);
    }
    else if (last_seg.type == capella_ros_msg::msg::CurveSegment::ELLIPSE_ARC)
    {
      const auto& arc = last_seg.ellipse_arc_segment;
      const auto& q   = arc.frame.orientation;
      const Eigen::Matrix2d R = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix().topLeftCorner<2, 2>();

      Eigen::Vector2d local_tangent(-arc.a * std::sin(arc.theta_end), arc.b * std::cos(arc.theta_end));
      end_pt  = Eigen::Vector2d(arc.end_point.x, arc.end_point.y);

      Eigen::Vector2d world_tangent = safeNormalize(R * local_tangent);
      tangent = arc.is_ccw ? world_tangent : -world_tangent;
    }
    else
    {
      tangent = Eigen::Vector2d(1.0, 0.0);
    }

    Eigen::Vector2d ext_end = end_pt + tangent * extend_length;

    capella_ros_msg::msg::CurveSegment seg;
    seg.type = capella_ros_msg::msg::CurveSegment::LINE;

    seg.line_segment.start_point = makePoint(end_pt.x(), end_pt.y());
    seg.line_segment.end_point = makePoint(ext_end.x(), ext_end.y());
    output.segments.push_back(seg);
  }

  return output;
};

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