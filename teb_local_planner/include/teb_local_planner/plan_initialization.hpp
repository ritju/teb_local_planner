#ifndef TEB_PLAN_INITIALIZATION_HPP_
#define TEB_PLAN_INITIALIZATION_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

namespace teb_local_planner
{
// Reduce only the optimization seed, not the stored global path or collision
// checking. Preserve endpoints, sharp bends, accumulated heading changes and
// in-place orientation changes; autoResize still handles temporal resolution.
inline std::vector<geometry_msgs::msg::PoseStamped> sampleInitializationPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  double spacing, double max_angle)
{
  if (plan.size() < 3 || !std::isfinite(spacing) || spacing <= 0.0 ||
      !std::isfinite(max_angle) || max_angle <= 0.0) {return plan;}
  for (const auto & p : plan) {
    if (!std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y) ||
        !std::isfinite(tf2::getYaw(p.pose.orientation))) {return plan;}
  }
  const auto angle_diff = [](double a, double b) {
      return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
    };
  std::vector<geometry_msgs::msg::PoseStamped> result;
  result.reserve(plan.size());
  result.push_back(plan.front());
  double distance = 0.0;
  double accumulated_turn = 0.0;
  for (size_t i = 1; i + 1 < plan.size(); ++i) {
    const auto & a = plan[i - 1].pose.position;
    const auto & b = plan[i].pose.position;
    const auto & c = plan[i + 1].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double nx = c.x - b.x, ny = c.y - b.y;
    const double step = std::hypot(dx, dy);
    distance += step;
    const double turn = step > 1e-9 && std::hypot(nx, ny) > 1e-9 ?
      std::abs(std::atan2(dx * ny - dy * nx, dx * nx + dy * ny)) : 0.0;
    accumulated_turn += turn;
    const double yaw = tf2::getYaw(plan[i].pose.orientation);
    const bool stationary_turn = step <= 1e-9 &&
      angle_diff(yaw, tf2::getYaw(plan[i - 1].pose.orientation)) > 1e-6;
    const bool upcoming_stationary_turn = std::hypot(nx, ny) <= 1e-9 &&
      angle_diff(tf2::getYaw(plan[i + 1].pose.orientation), yaw) > 1e-6;
    if (distance >= spacing || accumulated_turn >= max_angle || turn >= max_angle ||
        angle_diff(yaw, tf2::getYaw(result.back().pose.orientation)) >= max_angle ||
        stationary_turn || upcoming_stationary_turn) {
      result.push_back(plan[i]);
      distance = accumulated_turn = 0.0;
    }
  }
  result.push_back(plan.back());
  return result;
}
}  // namespace teb_local_planner
#endif
