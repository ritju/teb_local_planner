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
#include "capella_ros_msg/msg/b_spline_segment.hpp"

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

// ============================================================
// B 样条核心实现（de Boor + 解析导数 + Newton 最近点）
// ============================================================

/**
 * @brief 在节点向量中二分查找区间索引 k，满足 knots[k] <= u < knots[k+1]
 *
 * 边界处理：
 *   - u <= u_min  → k = degree
 *   - u >= u_max  → k = knots.size() - degree - 2   （最后有效区间）
 *
 * @param knots   节点向量
 * @param degree  B 样条次数
 * @param u       查询参数
 * @return        区间索引 k
 */
inline int bsplineFindSpan(
    const std::vector<float>& knots,
    int degree,
    double u)
{
  const int n  = static_cast<int>(knots.size()) - 1;
  const int lo = degree;
  const int hi = n - degree - 1;   // 最后一个有效区间左端索引

  if (u >= knots[hi + 1]) return hi;
  if (u <= knots[lo])     return lo;

  // 标准二分
  int left = lo, right = hi + 1;
  while (right - left > 1)
  {
    const int mid = (left + right) / 2;
    if (u < knots[mid]) right = mid;
    else                left  = mid;
  }
  return left;
}

/**
 * @brief de Boor 算法：同时求曲线点 C(u) 和解析一阶导 C'(u)
 *
 * 一阶导公式（NURBS Book §2.3）：
 *   C'(u) = degree * Σ_{j=k-degree+1}^{k}
 *               [(P_j - P_{j-1}) / (knots[j+degree] - knots[j])] * N_{j,degree-1}(u)
 *
 * 实现上等价于对差商控制点 Q_j = degree*(P_j - P_{j-1}) / denom_j
 * 做 (degree-1) 次 de Boor 递推。
 *
 * @param cps     控制点（取 XY）
 * @param knots   节点向量（float，内部转 double 计算）
 * @param degree  B 样条次数
 * @param k       find_span 返回的区间索引
 * @param u       求值参数
 * @param C       [out] 曲线点
 * @param dC      [out] 一阶导向量（未归一化）
 */
inline void deBoorWithDeriv(
    const std::vector<geometry_msgs::msg::Point>& cps,
    const std::vector<float>& knots,
    int degree,
    int k,
    double u,
    Eigen::Vector2d& C,
    Eigen::Vector2d& dC)
{
  const int n = static_cast<int>(cps.size()) - 1;

  // ── 求曲线点 C(u)：标准 de Boor ──────────────────────────────────────────
  std::vector<Eigen::Vector2d> d(degree + 1);
  for (int j = 0; j <= degree; ++j)
  {
    const int idx = std::clamp(k - degree + j, 0, n);
    d[j] = Eigen::Vector2d(cps[idx].x, cps[idx].y);
  }
  for (int r = 1; r <= degree; ++r)
  {
    for (int j = degree; j >= r; --j)
    {
      const int li = k - degree + j;
      const int ri = k + 1 - r + j;
      const double denom = knots[ri] - knots[li];
      const double alpha = (denom < 1e-12) ? 0.0 : (u - knots[li]) / denom;
      d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
    }
  }
  C = d[degree];

  // ── 求一阶导 C'(u)：对差商控制点做 (degree-1) 次 de Boor ────────────────
  if (degree < 1)
  {
    dC = Eigen::Vector2d::Zero();
    return;
  }

  // 差商控制点 Q_j = degree * (P_j - P_{j-1}) / (knots[j+degree] - knots[j])
  // j 的范围：[k-degree+1 .. k]，共 degree 个
  std::vector<Eigen::Vector2d> q(degree);
  for (int j = 0; j < degree; ++j)
  {
    const int pi  = k - degree + 1 + j;          // 对应原控制点索引 pi, pi-1
    const int idx1 = std::clamp(pi,     0, n);
    const int idx0 = std::clamp(pi - 1, 0, n);
    const double kl = knots[std::clamp(pi,           0, (int)knots.size()-1)];
    const double kr = knots[std::clamp(pi + degree,  0, (int)knots.size()-1)];
    const double denom = kr - kl;
    const Eigen::Vector2d dp(cps[idx1].x - cps[idx0].x,
                             cps[idx1].y - cps[idx0].y);
    if (denom < 1e-12)
    {
      q[j].setZero();
    }
    else
    {
      q[j] = (degree * dp) / denom;
    }
  }

  // 对 q[] 做 (degree-1) 次 de Boor 递推（用相同的 k 和 u）
  for (int r = 1; r <= degree - 1; ++r)
  {
    for (int j = degree - 1; j >= r; --j)
    {
      const int li = k - degree + 1 + j;
      const int ri = k + 2 - r + j;
      const double denom = knots[std::clamp(ri, 0, (int)knots.size()-1)]
                         - knots[std::clamp(li, 0, (int)knots.size()-1)];
      const double alpha = (denom < 1e-12) ? 0.0 : (u - knots[li]) / denom;
      q[j] = (1.0 - alpha) * q[j - 1] + alpha * q[j];
    }
  }
  dC = q[degree - 1];
}

/**
 * @brief 仅求曲线点（不需要导数时的简化接口，复用 deBoorWithDeriv）
 */
inline Eigen::Vector2d deBoor(
    const std::vector<geometry_msgs::msg::Point>& cps,
    const std::vector<float>& knots,
    int degree,
    double u)
{
  const int k = bsplineFindSpan(knots, degree, u);
  Eigen::Vector2d C, dC;
  deBoorWithDeriv(cps, knots, degree, k, u, C, dC);
  return C;
}

/**
 * @brief 等弧长采样 B 样条线段
 *
 * 流程：
 *   1. 粗采样（64段）估算总弧长
 *   2. 沿粗采样折线做等步长重采样，弦长插值得参数初值
 *   3. 用解析导 dC 做一步 Newton 修正（弧长参数化近似）
 */
inline void sampleBSplineSegment(
    const capella_ros_msg::msg::BSplineSegment& seg,
    double step,
    std::vector<Eigen::Vector2d>& out)
{
  if (seg.control_points.empty() || seg.knot_vector.empty())
    return;

  const int degree   = std::max(1, static_cast<int>(seg.order) - 1);
  const auto& knots  = seg.knot_vector;
  const auto& cps    = seg.control_points;

  const double u_min = knots[degree];
  const double u_max = knots[static_cast<int>(knots.size()) - 1 - degree];

  if (u_max - u_min < 1e-9)
  {
    out.push_back(deBoor(cps, knots, degree, u_min));
    return;
  }

  // 步骤 1：粗采样
  constexpr int kCoarse = 64;
  std::vector<double> coarse_u(kCoarse + 1);
  std::vector<Eigen::Vector2d> coarse_pts(kCoarse + 1);
  for (int i = 0; i <= kCoarse; ++i)
  {
    coarse_u[i]   = u_min + (u_max - u_min) * i / kCoarse;
    coarse_pts[i] = deBoor(cps, knots, degree, coarse_u[i]);
  }

  double total_len = 0.0;
  for (int i = 0; i < kCoarse; ++i)
    total_len += (coarse_pts[i + 1] - coarse_pts[i]).norm();

  const int reserve_num = std::max(1, static_cast<int>(std::ceil(total_len / step)));
  out.reserve(out.size() + reserve_num + 2);

  // 步骤 2+3：弦长插值初值 → Newton 一步修正 → deBoor 求精确点
  double target_s = 0.0;
  double accum_s  = 0.0;
  out.push_back(coarse_pts[0]);
  target_s += step;

  Eigen::Vector2d last_pt = coarse_pts[0];   // ← 记录上一个实际采样点

  for (int i = 0; i < kCoarse; ++i)
  {
    const double chord = (coarse_pts[i + 1] - coarse_pts[i]).norm();
    if (chord < 1e-12) continue;

    while (target_s <= accum_s + chord)
    {
      const double t_lin = (target_s - accum_s) / chord;
      double u = coarse_u[i] + t_lin * (coarse_u[i + 1] - coarse_u[i]);
      u = std::clamp(u, u_min, u_max);

      // Newton 修正：参考点换成上一个实际采样点
      {
        const int kk = bsplineFindSpan(knots, degree, u);
        Eigen::Vector2d Cu, dCu;
        deBoorWithDeriv(cps, knots, degree, kk, u, Cu, dCu);

        const double speed = dCu.norm();
        if (speed > 1e-9)
        {
          const double s_cur  = (Cu - last_pt).norm();  // ← 到上一采样点的距离
          const double du_corr = (step - s_cur) / speed; // ← 目标始终是走 step
          u = std::clamp(u + du_corr, u_min, u_max);
        }
        const Eigen::Vector2d final_pt = deBoor(cps, knots, degree, u);
        out.push_back(final_pt);
        last_pt = final_pt;   // ← 更新参考点
      }

      target_s += step;
    }

    accum_s += chord;
  }

  out.push_back(coarse_pts[kCoarse]);
}

/**
 * @brief 点到 B 样条线段的最近距离及最近点、解析切线方向
 *
 * 流程：
 *   1. 粗采样（64段）→ 找距离最小的索引 best_i
 *   2. Newton 迭代求精确最近参数 u*
 *      目标方程：f(u) = (C(u) - p) · C'(u) = 0
 *      牛顿步：  Δu = -f(u) / f'(u)
 *              f'(u) = ‖C'(u)‖² + (C(u)-p)·C''(u)
 *      C''(u) 用二阶中心差分近似（仅 Newton 收敛判断用，精度够）
 *   3. deBoor + deBoorWithDeriv 求最近点坐标和解析切线
 */
inline double distToBSplineWithDir(
    const Eigen::Vector2d& p,
    const capella_ros_msg::msg::BSplineSegment& seg,
    Eigen::Vector2d& out_closest,
    Eigen::Vector2d& out_dir)
{
  if (seg.control_points.empty() || seg.knot_vector.empty())
    return std::numeric_limits<double>::max();

  const int degree   = std::max(1, static_cast<int>(seg.order) - 1);
  const auto& knots  = seg.knot_vector;
  const auto& cps    = seg.control_points;

  const double u_min = knots[degree];
  const double u_max = knots[static_cast<int>(knots.size()) - 1 - degree];

  if (u_max - u_min < 1e-9)
  {
    out_closest = deBoor(cps, knots, degree, u_min);
    out_dir     = Eigen::Vector2d(1.0, 0.0);
    return (p - out_closest).norm();
  }

  // ── 步骤 1：粗采样找初值 ─────────────────────────────────────────────────
  constexpr int kCoarse = 64;
  double best_u   = u_min;
  double best_dsq = std::numeric_limits<double>::max();
  int    best_i   = 0;

  std::vector<double> us(kCoarse + 1);
  std::vector<Eigen::Vector2d> coarse_pts(kCoarse + 1);
  for (int i = 0; i <= kCoarse; ++i)
  {
    us[i]         = u_min + (u_max - u_min) * i / kCoarse;
    coarse_pts[i] = deBoor(cps, knots, degree, us[i]);
    const double dsq = (p - coarse_pts[i]).squaredNorm();
    if (dsq < best_dsq) { best_dsq = dsq; best_u = us[i]; best_i = i; }
  }

  // 缩窄搜索区间（粗采样相邻段）
  const double u_lo0 = (best_i > 0)       ? us[best_i - 1] : u_min;
  const double u_hi0 = (best_i < kCoarse) ? us[best_i + 1] : u_max;

  // ── 步骤 2：Newton 迭代求 u* ─────────────────────────────────────────────
  // f(u)  = (C(u) - p) · C'(u)  = 0
  // f'(u) = ‖C'(u)‖² + (C(u) - p) · C''(u)
  // C''(u) 用中心差分近似
  double u = best_u;
  constexpr int    kNewton  = 10;
  constexpr double kNewtonTol = 1e-7;
  constexpr double kDu2 = 1e-5;   // 二阶差分步长

  for (int iter = 0; iter < kNewton; ++iter)
  {
    const int kk = bsplineFindSpan(knots, degree, u);
    Eigen::Vector2d Cu, dCu;
    deBoorWithDeriv(cps, knots, degree, kk, u, Cu, dCu);

    const Eigen::Vector2d diff = Cu - p;
    const double f = diff.dot(dCu);

    // 二阶中心差分估算 C''(u)
    const double ua = std::clamp(u - kDu2, u_min, u_max);
    const double ub = std::clamp(u + kDu2, u_min, u_max);
    Eigen::Vector2d Ca, dCa, Cb, dCb;
    deBoorWithDeriv(cps, knots, degree, bsplineFindSpan(knots, degree, ua), ua, Ca, dCa);
    deBoorWithDeriv(cps, knots, degree, bsplineFindSpan(knots, degree, ub), ub, Cb, dCb);
    const Eigen::Vector2d d2Cu = (Cb - 2.0 * Cu + Ca) / (kDu2 * kDu2);

    const double fp = dCu.squaredNorm() + diff.dot(d2Cu);

    // 防止除零
    if (std::abs(fp) < 1e-14) break;

    const double du = -f / fp;
    u = std::clamp(u + du, u_lo0, u_hi0);

    if (std::abs(du) < kNewtonTol) break;
  }

  // ── 步骤 3：deBoor + 解析导数 → 最近点 + 切线 ───────────────────────────
  {
    const int kk = bsplineFindSpan(knots, degree, u);
    Eigen::Vector2d Cu, dCu;
    deBoorWithDeriv(cps, knots, degree, kk, u, Cu, dCu);

    out_closest = Cu;

    const double tlen = dCu.norm();
    out_dir = (tlen > 1e-12) ? (dCu / tlen) : Eigen::Vector2d(1.0, 0.0);
  }

  return (p - out_closest).norm();
}

/// 点到 B 样条段的最近距离（仅距离，无方向输出）
inline double distToBSpline(
    const Eigen::Vector2d& p,
    const capella_ros_msg::msg::BSplineSegment& seg)
{
  Eigen::Vector2d dummy_closest, dummy_dir;
  return distToBSplineWithDir(p, seg, dummy_closest, dummy_dir);
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
    else if (
        seg.type ==
        capella_ros_msg::msg::CurveSegment::B_SPLINE)
    {
      sampleBSplineSegment(
          seg.bspline_segment,
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
    } else if (seg.type == capella_ros_msg::msg::CurveSegment::B_SPLINE) {
      const double d = distToBSpline(p, seg.bspline_segment);
      if (d * d < best_sq) best_sq = d * d;
    }
  }

  const double best_line = (best_sq < std::numeric_limits<double>::max())
                           ? std::sqrt(best_sq)
                           : std::numeric_limits<double>::max();
  return std::min(best_line, best_arc);
}

/// 计算点到 MultiCurve 最小距离，及返回的运行切线方向 dir(单位向量),最近点 out_closet_pt 距离无穷大时未获取到
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
    // ── B 样条 ─────────────────────────────────────────────────────────────
    } else if (seg.type == capella_ros_msg::msg::CurveSegment::B_SPLINE) {
      Eigen::Vector2d closest, tangent_dir;
      const double d = distToBSplineWithDir(
          input_pt, seg.bspline_segment, closest, tangent_dir);
      const double dsq = d * d;
      if (dsq < min_path_dist_sq) {
        min_path_dist_sq = dsq;
        out_closet_pt    = closest;
        dir              = tangent_dir;
        found            = true;
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
    else if (first_seg.type == capella_ros_msg::msg::CurveSegment::B_SPLINE)
    {
      const auto& bsp = first_seg.bspline_segment;
      start_pt = Eigen::Vector2d(bsp.start_point.x, bsp.start_point.y);

      // 利用前两个控制点估算起点切线方向
      if (bsp.control_points.size() >= 2)
      {
        const Eigen::Vector2d cp0(bsp.control_points[0].x, bsp.control_points[0].y);
        const Eigen::Vector2d cp1(bsp.control_points[1].x, bsp.control_points[1].y);
        tangent = safeNormalize(cp1 - cp0);
      }
      else
      {
        tangent = Eigen::Vector2d(1.0, 0.0);
      }
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
    else if (last_seg.type == capella_ros_msg::msg::CurveSegment::B_SPLINE)
    {
      const auto& bsp = last_seg.bspline_segment;
      end_pt = Eigen::Vector2d(bsp.end_point.x, bsp.end_point.y);

      // 利用最后两个控制点估算终点切线方向
      const int ncp = static_cast<int>(bsp.control_points.size());
      if (ncp >= 2)
      {
        const Eigen::Vector2d cp_n1(bsp.control_points[ncp - 2].x, bsp.control_points[ncp - 2].y);
        const Eigen::Vector2d cp_n(bsp.control_points[ncp - 1].x,  bsp.control_points[ncp - 1].y);
        tangent = safeNormalize(cp_n - cp_n1);
      }
      else
      {
        tangent = Eigen::Vector2d(1.0, 0.0);
      }
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