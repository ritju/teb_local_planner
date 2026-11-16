#pragma once

#include <algorithm>
#include <cmath>

namespace teb_local_planner
{

inline double occupiedGoalStoppingDistance(double speed, double deceleration, double response_time)
{
  const double v = std::max(0.0, speed);
  const double a = std::max(1e-6, deceleration);
  const double delay = std::max(0.0, response_time);
  return v * delay + v * v / (2.0 * a);
}

inline double occupiedGoalSafeSpeed(double available_distance, double deceleration,
  double response_time)
{
  const double d = std::max(0.0, available_distance);
  const double a = std::max(1e-6, deceleration);
  const double delay = std::max(0.0, response_time);
  return std::max(0.0, std::sqrt(a * a * delay * delay + 2.0 * a * d) - a * delay);
}

}  // namespace teb_local_planner
