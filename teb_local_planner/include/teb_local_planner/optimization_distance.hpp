#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace teb_local_planner
{
struct OptimizationDistance
{
  double signed_distance{0.0};
  Eigen::Vector2d escape_normal{1.0, 0.0};
};

namespace optimization_distance
{
using Points = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

inline double cross(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{return a.x() * b.y() - a.y() * b.x();}

// Canonical CCW hull. Non-convex input is conservatively enclosed, never
// interpreted as an exact convex penetration problem.
inline Points convexHull(Points points)
{
  for (const auto & p : points) {
    if (!p.allFinite()) {throw std::invalid_argument("Non-finite optimization geometry");}
  }
  std::sort(points.begin(), points.end(), [](const auto & a, const auto & b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  points.erase(std::unique(points.begin(), points.end(), [](const auto & a, const auto & b) {
    return a.x() == b.x() && a.y() == b.y();
  }), points.end());
  if (points.size() <= 2) {return points;}
  Points hull;
  for (const auto & p : points) {
    while (hull.size() >= 2 && cross(hull.back() - hull[hull.size()-2], p - hull.back()) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(p);
  }
  const size_t lower = hull.size();
  for (size_t i = points.size()-1; i-- > 0;) {
    const auto & p = points[i];
    while (hull.size() > lower && cross(hull.back() - hull[hull.size()-2], p - hull.back()) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(p);
  }
  hull.pop_back();
  return hull;
}

// A-B is the configuration-space collision polygon for translating body A.
// Origin inside A-B means overlap. Distance to its boundary is the exact
// translational penetration depth for convex shapes, including containment.
inline OptimizationDistance between(const Points & body, const Points & obstacle, double radius = 0.0)
{
  if (body.empty() || obstacle.empty() || !std::isfinite(radius) || radius < 0.0) {
    throw std::invalid_argument("Invalid optimization distance geometry");
  }
  Points differences;
  differences.reserve(body.size() * obstacle.size());
  for (const auto & a : body) {
    for (const auto & b : obstacle) {differences.push_back(a-b);}
  }
  const auto hull = convexHull(std::move(differences));
  if (hull.size() == 1) {
    const double d = hull.front().norm();
    OptimizationDistance result;
    result.signed_distance = d-radius;
    if (d > 1e-12) {result.escape_normal = hull.front()/d;}
    return result;
  }
  bool inside = hull.size() >= 3;
  double best = std::numeric_limits<double>::infinity();
  Eigen::Vector2d closest = Eigen::Vector2d::Zero();
  Eigen::Vector2d inward = Eigen::Vector2d::UnitX();
  const size_t edges = hull.size() == 2 ? 1 : hull.size();
  for (size_t i = 0; i < edges; ++i) {
    const auto & a = hull[i];
    const Eigen::Vector2d edge = hull[(i+1)%hull.size()]-a;
    const double length2 = edge.squaredNorm();
    if (length2 <= 1e-24) {continue;}
    if (cross(edge, -a) < -1e-12) {inside = false;}
    const double t = std::max(0.0, std::min(1.0, -a.dot(edge)/length2));
    const Eigen::Vector2d q = a+t*edge;
    if (q.squaredNorm() < best) {
      best = q.squaredNorm();
      closest = q;
      inward = Eigen::Vector2d(-edge.y(), edge.x()).normalized();
    }
  }
  OptimizationDistance result;
  const double d = std::sqrt(best);
  result.signed_distance = (inside ? -d : d)-radius;
  // Select a deterministic one-sided subgradient at contact / symmetric ties,
  // where central finite differences can incorrectly return zero.
  result.escape_normal = d > 1e-12 ? Eigen::Vector2d((inside ? -1.0 : 1.0)*closest/d) : inward;
  return result;
}
}  // namespace optimization_distance
}  // namespace teb_local_planner
