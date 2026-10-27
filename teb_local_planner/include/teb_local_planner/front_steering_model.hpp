#ifndef FRONT_STEERING_MODEL_HPP_
#define FRONT_STEERING_MODEL_HPP_

#include "teb_local_planner/misc.h"
#include "teb_local_planner/pose_se2.h"
#include "teb_local_planner/teb_config.h"

#include <g2o/stuff/misc.h>

#include <algorithm>
#include <cmath>

namespace teb_local_planner
{

constexpr double kFrontSteeringWheelbase = 0.854;
constexpr double kFrontSteeringRearTrack = 0.84;
constexpr double kFrontSteeringMaxAngle = 1.0471975511965976;
constexpr double kFrontSteeringMaxRate = 0.5;
constexpr double kFrontSteeringMaxAcceleration = 1.0;
constexpr double kFrontSteeringMinSpeed = 0.03;
constexpr double kFrontSteeringOmegaEpsilon = 1e-3;

inline double clampDouble(double value, double lower, double upper)
{
  if (value < lower)
  {
    return lower;
  }
  if (value > upper)
  {
    return upper;
  }
  return value;
}

inline double minimumTurningRadius(double wheelbase, double max_steering_angle)
{
  if (!(wheelbase > 0.0) || !(max_steering_angle > 0.0) || max_steering_angle >= 1.5)
  {
    return 0.0;
  }
  const double tangent = std::tan(max_steering_angle);
  if (!(tangent > 0.0))
  {
    return 0.0;
  }
  return wheelbase / tangent;
}

inline double maxCurvature(double wheelbase, double max_steering_angle)
{
  const double radius = minimumTurningRadius(wheelbase, max_steering_angle);
  if (!(radius > 0.0))
  {
    return 0.0;
  }
  return 1.0 / radius;
}

inline double effectiveMinTurningRadius(const TebConfig::Robot & robot)
{
  if (robot.front_steering_rear_drive && robot.auto_min_turning_radius)
  {
    const double radius = minimumTurningRadius(robot.wheelbase, robot.max_steering_angle);
    if (radius > 0.0)
    {
      return radius;
    }
  }
  return robot.min_turning_radius;
}

struct SegmentKinematics
{
  double dt{0.0};
  double delta_theta{0.0};
  double longitudinal_displacement{0.0};
  double lateral_displacement{0.0};
  double velocity{0.0};
  double omega{0.0};
  double curvature{0.0};
  double steering_angle{0.0};
  bool stopped{false};
  bool inplace_rotation{false};
  bool valid{false};
};

inline bool computeSteeringSegment(
  const PoseSE2 & pose1, const PoseSE2 & pose2, double dt, double wheelbase, double min_speed,
  double omega_epsilon, SegmentKinematics & segment)
{
  segment = SegmentKinematics();
  segment.dt = dt;
  if (isTimeDiffDegenerate(dt) || !(wheelbase > 0.0) || !std::isfinite(wheelbase))
  {
    return false;
  }

  const double dx = pose2.x() - pose1.x();
  const double dy = pose2.y() - pose1.y();
  const double delta_theta = g2o::normalize_theta(pose2.theta() - pose1.theta());
  const double theta_bar = pose1.theta() + 0.5 * delta_theta;
  const double cosine = std::cos(theta_bar);
  const double sine = std::sin(theta_bar);
  segment.delta_theta = delta_theta;
  segment.longitudinal_displacement = cosine * dx + sine * dy;
  segment.lateral_displacement = -sine * dx + cosine * dy;
  segment.velocity = segment.longitudinal_displacement / dt;
  segment.omega = delta_theta / dt;

  const double speed_gate = std::max(0.0, min_speed);
  if (std::abs(segment.velocity) <= speed_gate)
  {
    segment.stopped = true;
    segment.curvature = 0.0;
    segment.steering_angle = 0.0;
    segment.inplace_rotation = std::abs(segment.omega) > std::max(0.0, omega_epsilon);
  }
  else
  {
    segment.stopped = false;
    segment.inplace_rotation = false;
    segment.curvature = segment.omega / segment.velocity;
    segment.steering_angle = std::atan(wheelbase * segment.curvature);
  }
  segment.valid = std::isfinite(segment.velocity) && std::isfinite(segment.omega) &&
    std::isfinite(segment.steering_angle);
  return segment.valid;
}

inline PoseSE2 poseAfterLongitudinalSegment(
  const PoseSE2 & start, double longitudinal, double steering_angle, double wheelbase)
{
  const double delta_theta =
    (wheelbase > 0.0) ? longitudinal * std::tan(steering_angle) / wheelbase : 0.0;
  const double theta_bar = start.theta() + 0.5 * delta_theta;
  return PoseSE2(
    start.x() + longitudinal * std::cos(theta_bar),
    start.y() + longitudinal * std::sin(theta_bar),
    g2o::normalize_theta(start.theta() + delta_theta));
}

inline double steeringRate(const SegmentKinematics & first, const SegmentKinematics & second)
{
  const double average_dt = 0.5 * (first.dt + second.dt);
  if (isTimeDiffDegenerate(average_dt))
  {
    return 0.0;
  }
  return (second.steering_angle - first.steering_angle) / average_dt;
}

inline double steeringAcceleration(
  const SegmentKinematics & first, const SegmentKinematics & second, const SegmentKinematics & third)
{
  const double first_average = 0.5 * (first.dt + second.dt);
  const double second_average = 0.5 * (second.dt + third.dt);
  const double acceleration_dt = 0.5 * (first_average + second_average);
  if (isTimeDiffDegenerate(acceleration_dt))
  {
    return 0.0;
  }
  return (steeringRate(second, third) - steeringRate(first, second)) / acceleration_dt;
}

struct FrontSteeringCommandState
{
  bool valid{false};
  double steering_angle{0.0};
  double steering_rate{0.0};
};

struct FrontSteeringCommand
{
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
  double steering_angle{0.0};
  double steering_rate{0.0};
  double rear_left{0.0};
  double rear_right{0.0};
};

inline FrontSteeringCommand projectFrontSteeringCommand(
  double vx, double vy, double omega, double dt_ctrl, const TebConfig::Robot & robot,
  const FrontSteeringCommandState & previous)
{
  (void)vy;
  FrontSteeringCommand command;
  command.vy = 0.0;
  const double wheelbase = robot.wheelbase;
  const double delta_max = clampDouble(robot.max_steering_angle, 0.0, 1.5);
  const double kappa = maxCurvature(wheelbase, delta_max);
  const double min_speed = std::max(0.0, robot.steering_min_speed);

  if (!(wheelbase > 0.0) || !(kappa > 0.0))
  {
    command.vx = vx;
    command.omega = 0.0;
    return command;
  }

  if (robot.no_inplace_rotation && std::abs(vx) <= min_speed)
  {
    omega = 0.0;
  }
  else
  {
    double omega_limit = std::abs(vx) * kappa;
    if (robot.max_vel_theta > 0.0)
    {
      omega_limit = std::min(omega_limit, robot.max_vel_theta);
    }
    omega = clampDouble(omega, -omega_limit, omega_limit);
  }

  double delta = 0.0;
  if (std::abs(vx) > min_speed)
  {
    delta = std::atan(wheelbase * omega / vx);
  }
  delta = clampDouble(delta, -delta_max, delta_max);

  double rate = 0.0;
  if (previous.valid && dt_ctrl > 1e-4)
  {
    const double max_step = std::max(0.0, robot.max_steering_rate) * dt_ctrl;
    double limited = clampDouble(delta, previous.steering_angle - max_step, previous.steering_angle + max_step);
    double raw_rate = (limited - previous.steering_angle) / dt_ctrl;
    const double rate_lo = previous.steering_rate - robot.max_steering_acceleration * dt_ctrl;
    const double rate_hi = previous.steering_rate + robot.max_steering_acceleration * dt_ctrl;
    rate = clampDouble(raw_rate, rate_lo, rate_hi);
    rate = clampDouble(rate, -robot.max_steering_rate, robot.max_steering_rate);
    delta = previous.steering_angle + rate * dt_ctrl;
    delta = clampDouble(delta, -delta_max, delta_max);
    if (std::abs(vx) > min_speed)
    {
      omega = vx * std::tan(delta) / wheelbase;
      double omega_limit = std::abs(vx) * kappa;
      if (robot.max_vel_theta > 0.0)
      {
        omega_limit = std::min(omega_limit, robot.max_vel_theta);
      }
      omega = clampDouble(omega, -omega_limit, omega_limit);
      delta = std::atan(wheelbase * omega / vx);
      rate = (delta - previous.steering_angle) / dt_ctrl;
    }
    else
    {
      omega = 0.0;
      rate = (delta - previous.steering_angle) / dt_ctrl;
    }
  }

  command.vx = vx;
  command.omega = omega;
  command.steering_angle = delta;
  command.steering_rate = rate;
  command.rear_left = vx - 0.5 * robot.rear_track * omega;
  command.rear_right = vx + 0.5 * robot.rear_track * omega;
  return command;
}

}  // namespace teb_local_planner

#endif
